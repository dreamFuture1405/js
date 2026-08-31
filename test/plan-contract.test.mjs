import test from 'node:test';
import assert from 'node:assert/strict';
import { createWorldSnapshot } from '../world/world-snapshot.mjs';
import {
  createStagePlanContract,
  evaluatePlanValidity,
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
  retreatPlan: {
    chunks: [{ id: 'retreat-1', jointTargets: [[0.02, 0.12, 0.22], [0, 0.1, 0.2]] }],
  },
  validForMilliseconds: 1500,
});

test('accepts a plan while the live state remains inside its validity envelope', () => {
  const source = snapshot(makeObservation());
  const liveObservation = makeObservation();
  liveObservation.robot.qpos[0] += 0.02;
  liveObservation.objects.item.position[0] += 0.003;
  const live = snapshot(liveObservation, 2, 1200);

  const result = evaluatePlanValidity(planFrom(source), live);

  assert.equal(result.valid, true);
  assert.deepEqual(result.reasons, []);
});

test('invalidates a plan after excessive robot joint drift', () => {
  const source = snapshot(makeObservation());
  const liveObservation = makeObservation();
  liveObservation.robot.qpos[1] += 0.2;

  const result = evaluatePlanValidity(
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

  const result = evaluatePlanValidity(
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

  const result = evaluatePlanValidity(
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

  const modelResult = evaluatePlanValidity(
    planFrom(source),
    snapshot(otherModel, 2, 1100),
  );
  const expiredResult = evaluatePlanValidity(
    planFrom(source),
    snapshot(makeObservation(), 3, 2600),
  );

  assert.ok(modelResult.reasons.some((reason) => reason.code === 'MODEL_CHANGED'));
  assert.ok(expiredResult.reasons.some((reason) => reason.code === 'PLAN_EXPIRED'));
});

test('requires every executable plan to include an automatic retreat path', () => {
  const source = snapshot(makeObservation());

  assert.throws(
    () => createStagePlanContract({
      planId: 'unsafe-plan',
      sourceSnapshot: source,
      stage: 'pick',
      candidateId: 'candidate',
      chunks: [{ id: 'forward', jointTargets: [[0, 0.1, 0.2]] }],
      retreatPlan: null,
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
    retreatPlan: {
      chunks: [{ id: 'retreat', jointTargets: [[0, 0.1, 0.2]] }],
    },
    validityScope: {
      allowGripperStateChange: true,
      articulatedJoints: [],
    },
  });

  const result = evaluatePlanValidity(
    plan,
    snapshot(liveObservation, 2, 1100),
  );

  assert.equal(result.valid, true);
});
