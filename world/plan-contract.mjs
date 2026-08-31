import { deepFreeze, snapshotStateSignature } from './world-snapshot.mjs';

const DEFAULT_TOLERANCES = Object.freeze({
  robotJointRadians: 0.08,
  endEffectorPositionMeters: 0.035,
  objectPositionMeters: 0.012,
  objectOrientationRadians: 0.18,
  articulationRadians: 0.04,
});

const maximumAbsoluteDifference = (left = [], right = []) => {
  if (left.length !== right.length) return Number.POSITIVE_INFINITY;
  return Math.max(
    0,
    ...left.map((value, index) => Math.abs(Number(value) - Number(right[index]))),
  );
};

const positionDistance = (left = [], right = []) =>
  Math.hypot(
    Number(left[0]) - Number(right[0]),
    Number(left[1]) - Number(right[1]),
    Number(left[2]) - Number(right[2]),
  );

const quaternionDistance = (left = [], right = []) => {
  const dot = Math.abs(
    Number(left[0]) * Number(right[0])
    + Number(left[1]) * Number(right[1])
    + Number(left[2]) * Number(right[2])
    + Number(left[3]) * Number(right[3]),
  );
  return 2 * Math.acos(Math.max(-1, Math.min(1, dot)));
};

const nonEmptyChunks = (chunks) =>
  Array.isArray(chunks)
  && chunks.length > 0
  && chunks.every((chunk) => (
    typeof chunk?.id === 'string'
    && Array.isArray(chunk.jointTargets)
    && chunk.jointTargets.length > 0
  ));

export function createStagePlanContract({
  planId,
  sourceSnapshot,
  stage,
  candidateId,
  chunks,
  retreatPlan,
  validityGuards = {},
  validityScope = {},
  validForMilliseconds = 1200,
  metadata = {},
}) {
  if (!sourceSnapshot?.snapshotId) throw new Error('Source snapshot is required');
  if (!String(planId ?? '')) throw new Error('Plan id is required');
  if (!String(stage ?? '')) throw new Error('Plan stage is required');
  if (!String(candidateId ?? '')) throw new Error('Candidate id is required');
  if (!nonEmptyChunks(chunks)) throw new Error('Executable plan requires non-empty chunks');
  if (!retreatPlan || !nonEmptyChunks(retreatPlan.chunks)) {
    throw new Error('Executable plan requires an automatic retreat path');
  }
  const duration = Math.max(100, Number(validForMilliseconds) || 1200);
  return deepFreeze({
    version: 1,
    planId: String(planId),
    sourceSnapshotId: sourceSnapshot.snapshotId,
    sourceStateSignature: snapshotStateSignature(sourceSnapshot),
    sourceSnapshot,
    stage: String(stage),
    candidateId: String(candidateId),
    createdAt: sourceSnapshot.capturedAt,
    expiresAt: sourceSnapshot.capturedAt + duration,
    validityGuards: {
      ...DEFAULT_TOLERANCES,
      ...validityGuards,
    },
    validityScope: {
      robot: validityScope.robot !== false,
      allowGripperStateChange: Boolean(validityScope.allowGripperStateChange),
      objects: Array.isArray(validityScope.objects)
        ? [...validityScope.objects]
        : Object.keys(sourceSnapshot.objects),
      articulatedJoints: Array.isArray(validityScope.articulatedJoints)
        ? [...validityScope.articulatedJoints]
        : Object.keys(sourceSnapshot.articulatedJoints),
    },
    chunks,
    retreatPlan,
    metadata,
  });
}

export function evaluatePlanValidity(plan, liveSnapshot) {
  const source = plan?.sourceSnapshot;
  const guards = {
    ...DEFAULT_TOLERANCES,
    ...(plan?.validityGuards ?? {}),
  };
  const reasons = [];
  const addReason = (code, detail = {}) => reasons.push({ code, ...detail });

  if (!source || !liveSnapshot) {
    addReason('SNAPSHOT_MISSING');
    return { valid: false, reasons };
  }
  if (source.modelId !== liveSnapshot.modelId) {
    addReason('MODEL_CHANGED', {
      expected: source.modelId,
      actual: liveSnapshot.modelId,
    });
  }
  if (liveSnapshot.capturedAt > Number(plan.expiresAt)) {
    addReason('PLAN_EXPIRED', {
      expiresAt: plan.expiresAt,
      capturedAt: liveSnapshot.capturedAt,
    });
  }
  if (source.checkerStage !== liveSnapshot.checkerStage) {
    addReason('CHECKER_STAGE_CHANGED', {
      expected: source.checkerStage,
      actual: liveSnapshot.checkerStage,
    });
  }
  if (plan.validityScope?.robot !== false) {
    const robotJointDrift = maximumAbsoluteDifference(
      source.robot.qpos,
      liveSnapshot.robot.qpos,
    );
    const endEffectorDrift = positionDistance(
      source.robot.endEffectorPose.position,
      liveSnapshot.robot.endEffectorPose.position,
    );
    if (
      robotJointDrift > guards.robotJointRadians
      || endEffectorDrift > guards.endEffectorPositionMeters
    ) {
      addReason('ROBOT_STATE_CHANGED', {
        robotJointDrift,
        endEffectorDrift,
      });
    }
  }
  if (
    !plan.validityScope?.allowGripperStateChange
    && source.robot.gripperState !== liveSnapshot.robot.gripperState
  ) {
    addReason('GRIPPER_STATE_CHANGED', {
      expected: source.robot.gripperState,
      actual: liveSnapshot.robot.gripperState,
    });
  }
  const trackedObjects = plan.validityScope?.objects ?? Object.keys(source.objects);
  for (const name of trackedObjects) {
    const expected = source.objects[name];
    if (!expected) continue;
    const actual = liveSnapshot.objects[name];
    if (!actual) {
      addReason('OBJECT_MISSING', { objectName: name });
      continue;
    }
    const translation = positionDistance(expected.position, actual.position);
    const rotation = quaternionDistance(expected.quaternion, actual.quaternion);
    if (
      translation > guards.objectPositionMeters
      || rotation > guards.objectOrientationRadians
    ) {
      addReason('OBJECT_STATE_CHANGED', {
        objectName: name,
        translation,
        rotation,
      });
    }
  }
  const trackedArticulations = plan.validityScope?.articulatedJoints
    ?? Object.keys(source.articulatedJoints);
  for (const name of trackedArticulations) {
    const expected = source.articulatedJoints[name];
    if (!Number.isFinite(expected)) continue;
    const actual = liveSnapshot.articulatedJoints[name];
    if (
      !Number.isFinite(actual)
      || Math.abs(Number(expected) - Number(actual)) > guards.articulationRadians
    ) {
      addReason('ARTICULATION_CHANGED', {
        jointName: name,
        expected,
        actual: actual ?? null,
      });
    }
  }
  return {
    valid: reasons.length === 0,
    reasons,
    sourceSnapshotId: source.snapshotId,
    liveSnapshotId: liveSnapshot.snapshotId,
  };
}

export { DEFAULT_TOLERANCES };
