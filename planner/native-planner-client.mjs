import { spawn } from 'node:child_process';
import path from 'node:path';

export function defaultPlannerProcess({
  projectRoot,
  requestTimeoutMilliseconds = 10_000,
  environment = process.env,
  platform = process.platform,
} = {}) {
  const root = path.resolve(projectRoot ?? '.');
  const virtualEnvironmentPython = platform === 'win32'
    ? path.join(root, '.venv', 'Scripts', 'python.exe')
    : path.join(root, '.venv', 'bin', 'python');
  return {
    command: environment.AXIS_PLANNER_PYTHON || virtualEnvironmentPython,
    args: ['-m', 'axis_planner.service'],
    cwd: root,
    requestTimeoutMilliseconds,
  };
}

export class NativePlannerClient {
  constructor({
    command,
    args = [],
    cwd,
    requestTimeoutMilliseconds = 10_000,
  }) {
    this.command = command;
    this.args = [...args];
    this.cwd = cwd;
    this.requestTimeoutMilliseconds = requestTimeoutMilliseconds;
    this.child = null;
    this.nextId = 0;
    this.pending = new Map();
    this.stdoutBuffer = '';
    this.stderrBuffer = '';
    this.closed = false;
  }

  async start() {
    if (this.child && !this.closed) return this.request('health');
    this.closed = false;
    this.child = spawn(this.command, this.args, {
      cwd: this.cwd,
      stdio: ['pipe', 'pipe', 'pipe'],
      windowsHide: true,
    });
    this.child.stdout.setEncoding('utf8');
    this.child.stderr.setEncoding('utf8');
    this.child.stdout.on('data', (chunk) => this.#consumeStdout(chunk));
    this.child.stderr.on('data', (chunk) => {
      this.stderrBuffer = `${this.stderrBuffer}${chunk}`.slice(-12_000);
    });
    this.child.on('error', (error) => this.#failAll(error));
    this.child.on('exit', (code, signal) => {
      const suffix = this.stderrBuffer ? `\n${this.stderrBuffer}` : '';
      this.#failAll(
        new Error(`Native planner exited with code ${code}, signal ${signal}${suffix}`),
      );
    });
    return this.request('health');
  }

  request(method, params = {}) {
    if (!this.child || this.closed || !this.child.stdin.writable) {
      return Promise.reject(new Error('Native planner process is not running'));
    }
    const id = ++this.nextId;
    const request = JSON.stringify({ id, method, params });
    return new Promise((resolve, reject) => {
      const timeout = setTimeout(() => {
        this.pending.delete(id);
        reject(new Error(`Native planner request timed out: ${method}`));
      }, this.requestTimeoutMilliseconds);
      this.pending.set(id, { resolve, reject, timeout, method });
      this.child.stdin.write(`${request}\n`, (error) => {
        if (!error) return;
        clearTimeout(timeout);
        this.pending.delete(id);
        reject(error);
      });
    });
  }

  async loadModel(modelPath) {
    return this.request('load_model', {
      path: path.resolve(modelPath),
    });
  }

  async mirrorSnapshot(snapshot, bodyNames = []) {
    if (!snapshot?.simulation) {
      throw new Error('WorldSnapshot does not contain full simulation state');
    }
    return this.request('mirror_state', {
      simulation: snapshot.simulation,
      bodyNames,
    });
  }

  async describeGeometry({
    bodyNames = [],
    jointNames = [],
  } = {}) {
    return this.request('describe_geometry', {
      bodyNames,
      jointNames,
    });
  }

  async solveIk({
    bodyName,
    jointNames,
    targetPosition,
    targetQuaternion = null,
    seed = null,
    maximumIterations,
    positionTolerance,
    orientationTolerance,
    damping,
    maximumJointStep,
  }) {
    return this.request('solve_ik', {
      bodyName,
      jointNames,
      targetPosition,
      ...(targetQuaternion == null ? {} : { targetQuaternion }),
      ...(seed == null ? {} : { seed }),
      ...(maximumIterations == null ? {} : { maximumIterations }),
      ...(positionTolerance == null ? {} : { positionTolerance }),
      ...(orientationTolerance == null ? {} : { orientationTolerance }),
      ...(damping == null ? {} : { damping }),
      ...(maximumJointStep == null ? {} : { maximumJointStep }),
    });
  }

  async validatePath({
    jointNames,
    start,
    goal,
    maximumJointStep,
    allowedBodyNames = [],
  }) {
    return this.request('validate_path', {
      jointNames,
      start,
      goal,
      ...(maximumJointStep == null ? {} : { maximumJointStep }),
      allowedBodyNames,
    });
  }

  async checkConfiguration({
    jointNames,
    joints,
    allowedBodyNames = [],
  }) {
    return this.request('check_configuration', {
      jointNames,
      joints,
      allowedBodyNames,
    });
  }

  async planPath({
    jointNames,
    start,
    goal,
    maximumJointStep,
    allowedBodyNames = [],
    rrtStep,
    maximumIterations,
    goalBias,
    randomSeed,
  }) {
    return this.request('plan_path', {
      jointNames,
      start,
      goal,
      ...(maximumJointStep == null ? {} : { maximumJointStep }),
      allowedBodyNames,
      ...(rrtStep == null ? {} : { rrtStep }),
      ...(maximumIterations == null ? {} : { maximumIterations }),
      ...(goalBias == null ? {} : { goalBias }),
      ...(randomSeed == null ? {} : { randomSeed }),
    });
  }

  close() {
    if (!this.child || this.closed) return;
    this.closed = true;
    this.child.stdin.end();
    this.child.kill();
    this.#failAll(new Error('Native planner client closed'));
  }

  #consumeStdout(chunk) {
    this.stdoutBuffer += chunk;
    while (this.stdoutBuffer.includes('\n')) {
      const newline = this.stdoutBuffer.indexOf('\n');
      const line = this.stdoutBuffer.slice(0, newline);
      this.stdoutBuffer = this.stdoutBuffer.slice(newline + 1);
      if (!line.trim()) continue;
      let response;
      try {
        response = JSON.parse(line);
      } catch (error) {
        this.#failAll(
          new Error(`Native planner returned invalid JSON: ${error.message}\n${line}`),
        );
        return;
      }
      const pending = this.pending.get(Number(response.id));
      if (!pending) continue;
      clearTimeout(pending.timeout);
      this.pending.delete(Number(response.id));
      if (response.ok) {
        pending.resolve(response.result);
      } else {
        pending.reject(
          new Error(
            `${response.error?.type ?? 'PlannerError'}: `
            + `${response.error?.message ?? 'Unknown planner error'}`,
          ),
        );
      }
    }
  }

  #failAll(error) {
    this.closed = true;
    for (const pending of this.pending.values()) {
      clearTimeout(pending.timeout);
      pending.reject(error);
    }
    this.pending.clear();
  }
}
