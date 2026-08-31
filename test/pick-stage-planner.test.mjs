import test from 'node:test';
import assert from 'node:assert/strict';
import { planPickAndLiftStage } from '../planning/pick-stage-planner.mjs';

const geometry = {
  bodies: {
    item: {
      position: [0.5, 0, 0.5],
      quaternion: [0, 0, 0, 1],
      geoms: [{
        id: 10,
        type: 7,
        contype: 1,
        position: [0.5, 0, 0.5],
        axes: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        size: [0.02, 0.03, 0.1],
        rbound: 0.11,
      }],
    },
    hand: {
      position: [0, 0, 1],
      quaternion: [0, 0, 0, 1],
      geoms: [],
    },
    left: {
      position: [-0.04, 0, 0.9],
      quaternion: [0, 0, 0, 1],
      geoms: [{
        id: 20,
        type: 6,
        contype: 1,
        position: [-0.04, 0, 0.9],
        axes: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        size: [0.01, 0.02, 0.03],
        rbound: 0.04,
      }],
    },
    right: {
      position: [0.04, 0, 0.9],
      quaternion: [0, 0, 0, 1],
      geoms: [{
        id: 21,
        type: 6,
        contype: 1,
        position: [0.04, 0, 0.9],
        axes: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        size: [0.01, 0.02, 0.03],
        rbound: 0.04,
      }],
    },
  },
  joints: {},
};

test('plans pregrasp, grasp and micro-lift from live geometry', async () => {
  const ikCalls = [];
  const pathCalls = [];
  const solutions = [
    [0.1, 0.1],
    [0.2, 0.2],
    [0.3, 0.3],
  ];
  const planner = {
    async solveIk(request) {
      ikCalls.push(request);
      return {
        success: true,
        joints: solutions[ikCalls.length - 1],
        position_error: 0.001,
        orientation_error: 0.002,
        iterations: 10,
      };
    },
    async planPath(request) {
      pathCalls.push(request);
      return {
        success: true,
        method: 'direct',
        path: [request.start, request.goal],
        reason: null,
      };
    },
  };

  const result = await planPickAndLiftStage({
    planner,
    geometry,
    objectBodyName: 'item',
    handBodyName: 'hand',
    leftFingerBodyName: 'left',
    rightFingerBodyName: 'right',
    robotJointNames: ['j1', 'j2'],
    startJoints: [0, 0],
  });

  assert.equal(result.ready, true);
  assert.deepEqual(result.chunks.map((chunk) => chunk.id), [
    'object_pregrasp',
    'object_grasp',
    'object_micro_lift',
  ]);
  assert.equal(result.chunks[1].actionAfter, 'close_gripper');
  assert.equal(result.chunks[2].actionAfter, 'verify_temporal_grasp');
  assert.deepEqual(ikCalls.map((call) => call.seed), [
    [0, 0],
    [0.1, 0.1],
    [0.2, 0.2],
  ]);
  assert.deepEqual(pathCalls.map((call) => call.start), [
    [0, 0],
    [0.1, 0.1],
    [0.2, 0.2],
  ]);
  assert.deepEqual(pathCalls[1].allowedBodyNames, ['item']);
  assert.deepEqual(pathCalls[2].allowedBodyNames, ['item']);
  assert.ok(result.targets.pregraspPadPosition[2] > result.targets.graspPadPosition[2]);
  assert.ok(result.targets.liftPadPosition[2] > result.targets.graspPadPosition[2]);
});

test('stops before path planning when a pose has no IK solution', async () => {
  let pathCalls = 0;
  const planner = {
    async solveIk() {
      return {
        success: false,
        joints: [],
        position_error: 0.2,
        orientation_error: 0.4,
        iterations: 240,
      };
    },
    async planPath() {
      pathCalls += 1;
      throw new Error('must not plan');
    },
  };

  const result = await planPickAndLiftStage({
    planner,
    geometry,
    objectBodyName: 'item',
    handBodyName: 'hand',
    leftFingerBodyName: 'left',
    rightFingerBodyName: 'right',
    robotJointNames: ['j1', 'j2'],
    startJoints: [0, 0],
  });

  assert.equal(result.ready, false);
  assert.equal(result.failure.reason, 'IK_FAILED');
  assert.equal(result.failure.poseId, 'object_pregrasp');
  assert.equal(pathCalls, 0);
});
