import test from 'node:test';
import assert from 'node:assert/strict';
import { executeStagePlan } from '../execution/chunk-supervisor.mjs';
import { createStagePlanContract } from '../world/plan-contract.mjs';
import { createWorldSnapshot } from '../world/world-snapshot.mjs';

const observationAt = ({
  qpos = [0, 0, 0],
  sequence = 1,
  capturedAt = 1000,
  objectX = 0.5,
  objectZ = 0.1,
  gripperState = 'open',
} = {}) => ({
  sequence,
  capturedAt,
  modelId: 'model-a',
  simulationTime: sequence / 10,
  checkerStage: 'pick',
  robot: {
    qpos,
    qvel: [0, 0, 0],
    endEffectorPose: {
      position: [0.2, 0.5, 0.3],
      quaternion: [0, 0, 0, 1],
    },
    gripperState,
  },
  objects: {
    item: {
      position: [objectX, 0.2, objectZ],
      quaternion: [0, 0, 0, 1],
      velocity: [],
    },
  },
  articulatedJoints: {},
  contacts: { count: 0, pairs: [], accessor: 'count_only' },
});

const toSnapshot = (observation) => createWorldSnapshot(observation, {
  sequence: observation.sequence,
  capturedAt: observation.capturedAt,
});

const makePlan = () => {
  const source = toSnapshot(observationAt());
  return createStagePlanContract({
    planId: 'pick-plan-1',
    sourceSnapshot: source,
    stage: 'pick',
    candidateId: 'top-1',
    chunks: [{
      id: 'approach',
      jointTargets: [
        [0.1, 0.1, 0.1],
        [0.2, 0.2, 0.2],
      ],
    }],
    retreatPolicy: {
      type: 'reverse_executed_prefix',
      safeStartJoints: source.robot.qpos,
    },
    validForMilliseconds: 10_000,
  });
};

test('executes short targets and validates progress against each expected waypoint', async () => {
  const calls = [];
  let sequence = 1;
  let qpos = [0, 0, 0];
  const bridge = {
    async observe() {
      return observationAt({ qpos, sequence, capturedAt: 1000 + sequence * 10 });
    },
    async executeChunk(command) {
      calls.push(command);
      qpos = [...command.jointTargets[0]];
      sequence += 1;
      return {
        completed: true,
        executedTargets: [qpos],
        observation: observationAt({
          qpos,
          sequence,
          capturedAt: 1000 + sequence * 10,
        }),
      };
    },
  };

  const result = await executeStagePlan({
    plan: makePlan(),
    bridge,
    snapshotFromObservation: toSnapshot,
  });

  assert.equal(result.status, 'completed');
  assert.deepEqual(result.executedTargets, [
    [0.1, 0.1, 0.1],
    [0.2, 0.2, 0.2],
  ]);
  assert.equal(calls.length, 2);
  assert.ok(calls.every((call) => call.jointTargets.length === 1));
});

test('retreats through only the completed target prefix after a stall', async () => {
  const commanded = [];
  const events = [];
  let sequence = 1;
  let qpos = [0, 0, 0];
  let forwardCalls = 0;
  const bridge = {
    async observe() {
      return observationAt({ qpos, sequence, capturedAt: 1000 + sequence * 10 });
    },
    async executeChunk(command) {
      const target = [...command.jointTargets[0]];
      commanded.push(target);
      sequence += 1;
      if (command.mode !== 'retreat') {
        forwardCalls += 1;
        if (forwardCalls === 2) {
          qpos = [0.15, 0.15, 0.15];
          return {
            completed: false,
            stopReason: 'TARGET_STALLED',
            executedTargets: [],
            observation: observationAt({
              qpos,
              sequence,
              capturedAt: 1000 + sequence * 10,
            }),
          };
        }
      }
      qpos = target;
      return {
        completed: true,
        executedTargets: [target],
        observation: observationAt({
          qpos,
          sequence,
          capturedAt: 1000 + sequence * 10,
        }),
      };
    },
  };

  const result = await executeStagePlan({
    plan: makePlan(),
    bridge,
    snapshotFromObservation: toSnapshot,
    onEvent: (event) => events.push(event),
  });

  assert.equal(result.status, 'replan');
  assert.equal(result.reason, 'TARGET_STALLED');
  assert.deepEqual(result.executedTargets, [[0.1, 0.1, 0.1]]);
  assert.deepEqual(commanded, [
    [0.1, 0.1, 0.1],
    [0.2, 0.2, 0.2],
    [0.1, 0.1, 0.1],
    [0, 0, 0],
  ]);
  assert.ok(events.some((event) => event.type === 'RECOVERY_STARTED'));
  assert.ok(events.some((event) => event.type === 'RECOVERY_COMPLETED'));
});

test('rejects a stale plan before issuing any motion command', async () => {
  let commandCount = 0;
  const bridge = {
    async observe() {
      return observationAt({
        qpos: [0, 0, 0],
        objectX: 0.7,
        sequence: 2,
        capturedAt: 1100,
      });
    },
    async executeChunk() {
      commandCount += 1;
      throw new Error('must not execute');
    },
  };

  const result = await executeStagePlan({
    plan: makePlan(),
    bridge,
    snapshotFromObservation: toSnapshot,
  });

  assert.equal(result.status, 'replan');
  assert.equal(result.reason, 'OBJECT_STATE_CHANGED');
  assert.equal(commandCount, 0);
});

test('emergency abort stops without starting an automatic retreat', async () => {
  const modes = [];
  const bridge = {
    async observe() {
      return observationAt();
    },
    async executeChunk(command) {
      modes.push(command.mode);
      throw new Error('AUTONOMY_ABORTED');
    },
  };

  const result = await executeStagePlan({
    plan: makePlan(),
    bridge,
    snapshotFromObservation: toSnapshot,
  });

  assert.equal(result.status, 'aborted');
  assert.deepEqual(modes, ['forward']);
});

test('runs chunk actions only after reaching the chunk and applies owned-state scope', async () => {
  const source = toSnapshot(observationAt());
  const plan = createStagePlanContract({
    planId: 'pick-actions',
    sourceSnapshot: source,
    stage: 'pick',
    candidateId: 'top',
    chunks: [
      {
        id: 'grasp',
        jointTargets: [[0.1, 0.1, 0.1]],
        actionAfter: 'close_gripper',
      },
      {
        id: 'micro-lift',
        jointTargets: [[0.2, 0.2, 0.2]],
        actionAfter: 'verify_temporal_grasp',
        executionScope: {
          allowGripperStateChange: true,
          objects: [],
        },
      },
    ],
    retreatPolicy: {
      type: 'reverse_executed_prefix',
      safeStartJoints: source.robot.qpos,
    },
    validForMilliseconds: 10_000,
  });
  const order = [];
  let sequence = 1;
  let qpos = [0, 0, 0];
  let gripperState = 'open';
  let objectZ = 0.1;
  const bridge = {
    async observe() {
      return observationAt({
        qpos,
        sequence,
        capturedAt: 1000 + sequence * 10,
        gripperState,
        objectZ,
      });
    },
    async executeChunk(command) {
      qpos = [...command.jointTargets[0]];
      if (qpos[0] > 0.15) objectZ = 0.13;
      sequence += 1;
      order.push(`move:${qpos[0]}`);
      return {
        completed: true,
        observation: await this.observe(),
      };
    },
  };

  const result = await executeStagePlan({
    plan,
    bridge,
    snapshotFromObservation: toSnapshot,
    actionHandler: async ({ action }) => {
      order.push(`action:${action}`);
      if (action === 'close_gripper') gripperState = 'closed';
      return { completed: true };
    },
  });

  assert.equal(result.status, 'completed');
  assert.deepEqual(order, [
    'move:0.1',
    'action:close_gripper',
    'move:0.2',
    'action:verify_temporal_grasp',
  ]);
});
