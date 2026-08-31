import test from 'node:test';
import assert from 'node:assert/strict';
import { createWorldSnapshot } from '../world/world-snapshot.mjs';
import {
  createExecutedPrefixRetreat,
  createStagePlanContract,
  validateExecutionProgress,
  validatePlanStart,
} from '../world/plan-contract.mjs';

const makeObservation = () => ({
  modelId: 'model-a',
  simulationTime: 2,
  checkerStage: 'pick',
  robot: {
    qpos: [0, 0.1, 0.2],
    qvel: [0, 0, 0],
    endEffectorPose: {
      position: [0.2, 0.5, 0.3],
      quaternion: [0, 0, 0, 1],
    },
    gripperState: 'open',
  },
  objects: {
    item: {
      position: [0.5, 0.2, 0.1],
      quaternion: [0, 0, 0, 1],
      velocity: [0, 0, 0, 0, 0, 0],
    },
  },
  articulatedJoints: { door: 1.1 },
  contacts: { count: 0, pairs: [], accessor: 'count_only' },
});

const snapshot = (observation, sequence = 1, capturedAt = 1000) =>
  createWorldSnapshot(observation, { sequence, capturedAt });

const planFrom = (source) => createStagePlanContract({
  planId: 'plan-1',
  sourceSnapshot: source,
  stage: 'pick',
  candidateId: 'grasp-top-1',
  chunks: [
    { id: 'chunk-1', jointTargets: [[0, 0.1, 0.2], [0.02, 0.12, 0.22]] },
  ],
  retreatPolicy: {
    type: 'reverse_executed_prefix',
    safeStartJoints: source.robot.qpos,
  },
  validForMilliseconds: 1500,
});

test('accepts a plan start while live state remains inside its source envelope', () => {
  const source = snapshot(makeObservation());
  const liveObservation = makeObservation();
  liveObservation.robot.qpos[0] += 0.02;
  liveObservation.objects.item.position[0] += 0.003;
  const live = snapshot(liveObservation, 2, 1200);

  const result = validatePlanStart(planFrom(source), live);

  assert.equal(result.valid, true);
  assert.deepEqual(result.reasons, []);
});

test('invalidates a plan after excessive robot joint drift', () => {
  const source = snapshot(makeObservation());
  const liveObservation = makeObservation();
  liveObservation.robot.qpos[1] += 0.2;

  const result = validatePlanStart(
    planFrom(source),
    snapshot(liveObservation, 2, 1100),
  );

  assert.equal(result.valid, false);
  assert.ok(result.reasons.some((reason) => reason.code === 'ROBOT_STATE_CHANGED'));
});

test('invalidates a plan after the target object moves', () => {
  const source = snapshot(makeObservation());
  const liveObservation = makeObservation();
  liveObservation.objects.item.position[2] += 0.03;

  const result = validatePlanStart(
    planFrom(source),
    snapshot(liveObservation, 2, 1100),
  );

  assert.equal(result.valid, false);
  assert.ok(result.reasons.some((reason) => reason.code === 'OBJECT_STATE_CHANGED'));
});

test('invalidates a plan after articulation or checker stage changes', () => {
  const source = snapshot(makeObservation());
  const liveObservation = makeObservation();
  liveObservation.articulatedJoints.door += 0.1;
  liveObservation.checkerStage = 'place';

  const result = validatePlanStart(
    planFrom(source),
    snapshot(liveObservation, 2, 1100),
  );

  assert.equal(result.valid, false);
  assert.ok(result.reasons.some((reason) => reason.code === 'ARTICULATION_CHANGED'));
  assert.ok(result.reasons.some((reason) => reason.code === 'CHECKER_STAGE_CHANGED'));
});

test('invalidates a plan when its model changes or its validity deadline expires', () => {
  const source = snapshot(makeObservation());
  const otherModel = makeObservation();
  otherModel.modelId = 'model-b';

  const modelResult = validatePlanStart(
    planFrom(source),
    snapshot(otherModel, 2, 1100),
  );
  const expiredResult = validatePlanStart(
    planFrom(source),
    snapshot(makeObservation(), 3, 2600),
  );

  assert.ok(modelResult.reasons.some((reason) => reason.code === 'MODEL_CHANGED'));
  assert.ok(expiredResult.reasons.some((reason) => reason.code === 'PLAN_EXPIRED'));
});

test('requires every executable plan to declare reverse-executed-prefix retreat', () => {
  const source = snapshot(makeObservation());

  assert.throws(
    () => createStagePlanContract({
      planId: 'unsafe-plan',
      sourceSnapshot: source,
      stage: 'pick',
      candidateId: 'candidate',
      chunks: [{ id: 'forward', jointTargets: [[0, 0.1, 0.2]] }],
      retreatPolicy: null,
    }),
    /retreat/i,
  );
});

test('permits state changes explicitly owned by the active manipulation primitive', () => {
  const source = snapshot(makeObservation());
  const liveObservation = makeObservation();
  liveObservation.robot.gripperState = 'closed';
  liveObservation.articulatedJoints.door += 0.4;
  const plan = createStagePlanContract({
    planId: 'door-plan',
    sourceSnapshot: source,
    stage: 'open',
    candidateId: 'handle-grasp',
    chunks: [{ id: 'pull', jointTargets: [[0, 0.1, 0.2]] }],
    retreatPolicy: {
      type: 'reverse_executed_prefix',
      safeStartJoints: source.robot.qpos,
    },
    validityScope: {
      allowGripperStateChange: true,
      articulatedJoints: [],
    },
  });

  const result = validatePlanStart(
    plan,
    snapshot(liveObservation, 2, 1100),
  );

  assert.equal(result.valid, true);
});

test('validates execution against the expected waypoint instead of source qpos', () => {
  const source = snapshot(makeObservation());
  const plan = planFrom(source);
  const liveObservation = makeObservation();
  liveObservation.robot.qpos = [0.31, 0.42, 0.53];
  liveObservation.robot.endEffectorPose.position = [0.5, 0.6, 0.7];
  const live = snapshot(liveObservation, 2, 1100);

  const result = validateExecutionProgress({
    plan,
    liveSnapshot: live,
    expectedRobotState: {
      qpos: [0.30, 0.41, 0.52],
      endEffectorPose: {
        position: [0.5, 0.6, 0.7],
      },
    },
  });

  assert.equal(result.valid, true);
  assert.equal(
    result.reasons.some((reason) => reason.code === 'ROBOT_STATE_CHANGED'),
    false,
  );
});

test('stops execution when live robot drifts from the current expected waypoint', () => {
  const source = snapshot(makeObservation());
  const liveObservation = makeObservation();
  liveObservation.robot.qpos = [0.4, 0.5, 0.6];

  const result = validateExecutionProgress({
    plan: planFrom(source),
    liveSnapshot: snapshot(liveObservation, 2, 1100),
    expectedRobotState: {
      qpos: [0.1, 0.2, 0.3],
    },
  });

  assert.equal(result.valid, false);
  assert.ok(result.reasons.some((reason) => reason.code === 'ROBOT_TRACKING_ERROR'));
});

test('permits object and gripper changes only when the active primitive owns them', () => {
  const source = snapshot(makeObservation());
  const liveObservation = makeObservation();
  liveObservation.robot.gripperState = 'closed';
  liveObservation.objects.item.position[2] += 0.04;
  const live = snapshot(liveObservation, 2, 1100);

  const blocked = validateExecutionProgress({
    plan: planFrom(source),
    liveSnapshot: live,
    expectedRobotState: { qpos: live.robot.qpos },
  });
  const owned = validateExecutionProgress({
    plan: planFrom(source),
    liveSnapshot: live,
    expectedRobotState: { qpos: live.robot.qpos },
    executionScope: {
      allowGripperStateChange: true,
      objects: [],
    },
  });

  assert.equal(blocked.valid, false);
  assert.equal(owned.valid, true);
});

test('builds retreat only from the exact target prefix actually executed', () => {
  const source = snapshot(makeObservation());
  const plan = createStagePlanContract({
    planId: 'prefix-plan',
    sourceSnapshot: source,
    stage: 'pick',
    candidateId: 'candidate',
    chunks: [
      {
        id: 'forward',
        jointTargets: [
          [0.02, 0.12, 0.22],
          [0.04, 0.14, 0.24],
          [0.06, 0.16, 0.26],
        ],
      },
    ],
    retreatPolicy: {
      type: 'reverse_executed_prefix',
      safeStartJoints: source.robot.qpos,
    },
  });
  const recoveryObservation = makeObservation();
  recoveryObservation.robot.qpos = [0.041, 0.141, 0.241];
  const recoverySnapshot = snapshot(recoveryObservation, 2, 1200);

  const retreat = createExecutedPrefixRetreat({
    plan,
    executedTargets: [
      [0.02, 0.12, 0.22],
      [0.04, 0.14, 0.24],
    ],
    recoverySnapshot,
  });

  assert.deepEqual(retreat.jointTargets, [
    recoverySnapshot.robot.qpos,
    [0.04, 0.14, 0.24],
    [0.02, 0.12, 0.22],
    source.robot.qpos,
  ]);
  assert.equal(retreat.sourceSnapshotId, recoverySnapshot.snapshotId);
  assert.equal(retreat.sourcePlanId, plan.planId);
});

test('rejects retreat targets that were not the executed prefix of the plan', () => {
  const source = snapshot(makeObservation());
  const plan = planFrom(source);

  assert.throws(
    () => createExecutedPrefixRetreat({
      plan,
      executedTargets: [[0.99, 0.99, 0.99]],
      recoverySnapshot: snapshot(makeObservation(), 2, 1200),
    }),
    /executed prefix/i,
  );
});
