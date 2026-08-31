import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import {
  NativePlannerClient,
  defaultPlannerProcess,
} from '../planner/native-planner-client.mjs';

test('loads and mirrors a model through the native planner process', async (context) => {
  const processConfig = defaultPlannerProcess({
    projectRoot: path.resolve('.'),
  });
  await fs.access(processConfig.command);
  const temporaryRoot = await fs.mkdtemp(path.join(os.tmpdir(), 'axis-planner-'));
  context.after(async () => {
    await fs.rm(temporaryRoot, { recursive: true, force: true });
  });
  const modelPath = path.join(temporaryRoot, 'scene.xml');
  await fs.writeFile(modelPath, `
<mujoco model="client_test">
  <worldbody>
    <body name="item">
      <freejoint/>
      <geom type="sphere" size=".1"/>
    </body>
  </worldbody>
</mujoco>`);
  const client = new NativePlannerClient(processConfig);
  context.after(() => client.close());

  await client.start();
  const summary = await client.loadModel(modelPath);
  const mirrored = await client.mirrorSnapshot({
    simulation: {
      qpos: [0, 0, 0, 1, 0, 0, 0],
      qvel: [0, 0, 0, 0, 0, 0],
      ctrl: [],
    },
  }, ['item']);

  assert.equal(summary.nq, 7);
  assert.deepEqual(mirrored.bodies.item.position, [0, 0, 0]);
  assert.equal(mirrored.summary.model_name.startsWith('client_test'), true);
});

test('solves IK and validates a path through the native client', async (context) => {
  const processConfig = defaultPlannerProcess({
    projectRoot: path.resolve('.'),
  });
  const temporaryRoot = await fs.mkdtemp(path.join(os.tmpdir(), 'axis-planner-'));
  context.after(async () => {
    await fs.rm(temporaryRoot, { recursive: true, force: true });
  });
  const modelPath = path.join(temporaryRoot, 'planning.xml');
  await fs.writeFile(modelPath, `
<mujoco model="client_planning_test">
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="robot">
      <joint name="slide" type="slide" axis="1 0 0" range="0 1"/>
      <geom type="sphere" size=".2"/>
    </body>
    <body name="obstacle" pos="1 0 0">
      <geom type="sphere" size=".2"/>
    </body>
  </worldbody>
</mujoco>`);
  const client = new NativePlannerClient(processConfig);
  context.after(() => client.close());
  await client.start();
  await client.loadModel(modelPath);
  await client.mirrorSnapshot({
    simulation: {
      qpos: [0],
      qvel: [0],
      ctrl: [],
    },
  });

  const ik = await client.solveIk({
    bodyName: 'robot',
    jointNames: ['slide'],
    targetPosition: [0.5, 0, 0],
  });
  const pathResult = await client.validatePath({
    jointNames: ['slide'],
    start: [0],
    goal: [1],
    maximumJointStep: 0.05,
  });
  const planned = await client.planPath({
    jointNames: ['slide'],
    start: [0],
    goal: [0.3],
    maximumJointStep: 0.05,
  });
  const geometry = await client.describeGeometry({
    bodyNames: ['robot'],
    jointNames: ['slide'],
  });
  const configuration = await client.checkConfiguration({
    jointNames: ['slide'],
    joints: [0.7],
  });

  assert.equal(ik.success, true);
  assert.ok(Math.abs(ik.joints[0] - 0.5) < 0.003);
  assert.equal(pathResult.valid, false);
  assert.equal(pathResult.reason, 'collision');
  assert.equal(planned.success, true);
  assert.equal(planned.method, 'direct');
  assert.equal(geometry.joints.slide.type, 'slide');
  assert.equal(geometry.bodies.robot.geoms.length, 1);
  assert.ok(configuration.collisions.length >= 1);
});

test('uses environment override before platform-specific planner Python', () => {
  const processConfig = defaultPlannerProcess({
    projectRoot: path.resolve('.'),
    environment: {
      AXIS_PLANNER_PYTHON: '/custom/python',
    },
    platform: 'linux',
  });

  assert.equal(processConfig.command, '/custom/python');
});

test('selects the virtualenv Python layout for each platform', () => {
  const root = path.resolve('project-root');

  assert.equal(
    defaultPlannerProcess({
      projectRoot: root,
      environment: {},
      platform: 'win32',
    }).command,
    path.join(root, '.venv', 'Scripts', 'python.exe'),
  );
  assert.equal(
    defaultPlannerProcess({
      projectRoot: root,
      environment: {},
      platform: 'linux',
    }).command,
    path.join(root, '.venv', 'bin', 'python'),
  );
});

test('rejects requests after the planner process exits', async () => {
  const client = new NativePlannerClient({
    command: process.execPath,
    args: ['-e', 'process.exit(0)'],
    cwd: path.resolve('.'),
    requestTimeoutMilliseconds: 1000,
  });

  await assert.rejects(client.start(), /exit|closed|timeout/i);
  await assert.rejects(client.request('health'), /running/i);
});
