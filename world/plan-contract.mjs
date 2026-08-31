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

const positionDistance = (left = [], right = []) => {
  if (left.length !== 3 || right.length !== 3) return Number.POSITIVE_INFINITY;
  return Math.hypot(
    Number(left[0]) - Number(right[0]),
    Number(left[1]) - Number(right[1]),
    Number(left[2]) - Number(right[2]),
  );
};

const quaternionDistance = (left = [], right = []) => {
  if (left.length !== 4 || right.length !== 4) return Number.POSITIVE_INFINITY;
  const dot = Math.abs(
    Number(left[0]) * Number(right[0])
    + Number(left[1]) * Number(right[1])
    + Number(left[2]) * Number(right[2])
    + Number(left[3]) * Number(right[3]),
  );
  return 2 * Math.acos(Math.max(-1, Math.min(1, dot)));
};

const numericVector = (values, expectedLength, label) => {
  if (!Array.isArray(values) || values.length !== expectedLength) {
    throw new Error(`${label} must contain ${expectedLength} joints`);
  }
  const vector = values.map(Number);
  if (!vector.every(Number.isFinite)) {
    throw new Error(`${label} must contain only finite numbers`);
  }
  return vector;
};

const normalizeChunks = (chunks, jointCount) => {
  if (!Array.isArray(chunks) || chunks.length === 0) {
    throw new Error('Executable plan requires non-empty chunks');
  }
  return chunks.map((chunk, chunkIndex) => {
    if (
      !String(chunk?.id ?? '')
      || !Array.isArray(chunk.jointTargets)
      || chunk.jointTargets.length === 0
    ) {
      throw new Error('Executable plan requires non-empty chunks');
    }
    return {
      ...chunk,
      id: String(chunk.id),
      jointTargets: chunk.jointTargets.map((target, targetIndex) => numericVector(
        target,
        jointCount,
        `Chunk ${chunkIndex} target ${targetIndex}`,
      )),
    };
  });
};

const normalizeRetreatPolicy = (retreatPolicy, jointCount) => {
  if (retreatPolicy?.type !== 'reverse_executed_prefix') {
    throw new Error(
      'Executable plan requires a reverse-executed-prefix automatic retreat policy',
    );
  }
  return {
    type: 'reverse_executed_prefix',
    safeStartJoints: numericVector(
      retreatPolicy.safeStartJoints,
      jointCount,
      'Retreat safe start',
    ),
  };
};

const vectorsEqual = (left, right) =>
  left.length === right.length
  && left.every((value, index) => Object.is(Number(value), Number(right[index])));

const withoutAdjacentDuplicates = (targets) => targets.filter(
  (target, index) => index === 0 || !vectorsEqual(target, targets[index - 1]),
);

const validationContext = (plan, liveSnapshot) => ({
  source: plan?.sourceSnapshot,
  liveSnapshot,
  guards: {
    ...DEFAULT_TOLERANCES,
    ...(plan?.validityGuards ?? {}),
  },
  reasons: [],
});

const validateWorldState = (plan, context, executionScope = null) => {
  const {
    source,
    liveSnapshot,
    guards,
    reasons,
  } = context;
  const addReason = (code, detail = {}) => reasons.push({ code, ...detail });
  const scope = {
    ...(plan?.validityScope ?? {}),
    ...(executionScope ?? {}),
  };

  if (!source || !liveSnapshot) {
    addReason('SNAPSHOT_MISSING');
    return;
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
  if (
    !scope.allowGripperStateChange
    && source.robot.gripperState !== liveSnapshot.robot.gripperState
  ) {
    addReason('GRIPPER_STATE_CHANGED', {
      expected: source.robot.gripperState,
      actual: liveSnapshot.robot.gripperState,
    });
  }
  const trackedObjects = scope.objects ?? Object.keys(source.objects);
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
  const trackedArticulations = scope.articulatedJoints
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
};

const validationResult = (context) => ({
  valid: context.reasons.length === 0,
  reasons: context.reasons,
  sourceSnapshotId: context.source?.snapshotId ?? null,
  liveSnapshotId: context.liveSnapshot?.snapshotId ?? null,
});

export function createStagePlanContract({
  planId,
  sourceSnapshot,
  stage,
  candidateId,
  chunks,
  retreatPolicy,
  validityGuards = {},
  validityScope = {},
  validForMilliseconds = 1200,
  metadata = {},
}) {
  if (!sourceSnapshot?.snapshotId) throw new Error('Source snapshot is required');
  if (!String(planId ?? '')) throw new Error('Plan id is required');
  if (!String(stage ?? '')) throw new Error('Plan stage is required');
  if (!String(candidateId ?? '')) throw new Error('Candidate id is required');
  const jointCount = sourceSnapshot.robot?.qpos?.length ?? 0;
  if (jointCount === 0) throw new Error('Source snapshot robot joints are required');
  const normalizedChunks = normalizeChunks(chunks, jointCount);
  const normalizedRetreatPolicy = normalizeRetreatPolicy(retreatPolicy, jointCount);
  const duration = Math.max(100, Number(validForMilliseconds) || 1200);
  return deepFreeze({
    version: 2,
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
    chunks: normalizedChunks,
    executionTargets: normalizedChunks.flatMap((chunk) => chunk.jointTargets),
    retreatPolicy: normalizedRetreatPolicy,
    metadata,
  });
}

export function validatePlanStart(plan, liveSnapshot) {
  const context = validationContext(plan, liveSnapshot);
  validateWorldState(plan, context);
  if (
    context.source
    && liveSnapshot
    && plan.validityScope?.robot !== false
  ) {
    const robotJointDrift = maximumAbsoluteDifference(
      context.source.robot.qpos,
      liveSnapshot.robot.qpos,
    );
    const endEffectorDrift = positionDistance(
      context.source.robot.endEffectorPose.position,
      liveSnapshot.robot.endEffectorPose.position,
    );
    if (
      robotJointDrift > context.guards.robotJointRadians
      || endEffectorDrift > context.guards.endEffectorPositionMeters
    ) {
      context.reasons.push({
        code: 'ROBOT_STATE_CHANGED',
        robotJointDrift,
        endEffectorDrift,
      });
    }
  }
  return validationResult(context);
}

export function validateExecutionProgress({
  plan,
  liveSnapshot,
  expectedRobotState,
  executionScope = null,
}) {
  const context = validationContext(plan, liveSnapshot);
  validateWorldState(plan, context, executionScope);
  if (
    context.source
    && liveSnapshot
    && plan.validityScope?.robot !== false
  ) {
    if (!Array.isArray(expectedRobotState?.qpos)) {
      context.reasons.push({ code: 'EXPECTED_ROBOT_STATE_MISSING' });
      return validationResult(context);
    }
    const robotJointDrift = maximumAbsoluteDifference(
      expectedRobotState.qpos,
      liveSnapshot.robot.qpos,
    );
    const expectedEndEffectorPosition = expectedRobotState
      .endEffectorPose?.position;
    const endEffectorDrift = Array.isArray(expectedEndEffectorPosition)
      ? positionDistance(
        expectedEndEffectorPosition,
        liveSnapshot.robot.endEffectorPose.position,
      )
      : 0;
    if (
      robotJointDrift > context.guards.robotJointRadians
      || endEffectorDrift > context.guards.endEffectorPositionMeters
    ) {
      context.reasons.push({
        code: 'ROBOT_TRACKING_ERROR',
        robotJointDrift,
        endEffectorDrift,
      });
    }
  }
  return validationResult(context);
}

export function createExecutedPrefixRetreat({
  plan,
  executedTargets,
  recoverySnapshot,
  validForMilliseconds = 1200,
}) {
  if (!plan?.planId || plan.retreatPolicy?.type !== 'reverse_executed_prefix') {
    throw new Error('A plan with reverse executed prefix retreat is required');
  }
  if (!recoverySnapshot?.snapshotId) {
    throw new Error('Recovery snapshot is required');
  }
  if (plan.sourceSnapshot?.modelId !== recoverySnapshot.modelId) {
    throw new Error('Recovery snapshot model must match the source plan');
  }
  if (!Array.isArray(executedTargets)) {
    throw new Error('Executed prefix targets are required');
  }
  const plannedTargets = plan.executionTargets
    ?? plan.chunks.flatMap((chunk) => chunk.jointTargets);
  const jointCount = plan.sourceSnapshot.robot.qpos.length;
  const normalizedExecutedTargets = executedTargets.map((target, index) =>
    numericVector(target, jointCount, `Executed prefix target ${index}`));
  const isExactPrefix = normalizedExecutedTargets.length <= plannedTargets.length
    && normalizedExecutedTargets.every(
      (target, index) => vectorsEqual(target, plannedTargets[index]),
    );
  if (!isExactPrefix) {
    throw new Error('Retreat can only reverse the exact executed prefix of the plan');
  }
  const liveJoints = numericVector(
    recoverySnapshot.robot?.qpos,
    jointCount,
    'Recovery snapshot robot state',
  );
  const safeStartJoints = numericVector(
    plan.retreatPolicy.safeStartJoints,
    jointCount,
    'Retreat safe start',
  );
  const jointTargets = withoutAdjacentDuplicates([
    liveJoints,
    ...normalizedExecutedTargets.toReversed(),
    safeStartJoints,
  ]);
  const duration = Math.max(100, Number(validForMilliseconds) || 1200);
  return deepFreeze({
    version: 1,
    retreatId: `retreat:${plan.planId}:${recoverySnapshot.snapshotId}`,
    sourcePlanId: plan.planId,
    sourceSnapshotId: recoverySnapshot.snapshotId,
    sourceStateSignature: snapshotStateSignature(recoverySnapshot),
    sourceSnapshot: recoverySnapshot,
    originalPlanSnapshotId: plan.sourceSnapshotId,
    stage: plan.stage,
    candidateId: plan.candidateId,
    createdAt: recoverySnapshot.capturedAt,
    expiresAt: recoverySnapshot.capturedAt + duration,
    jointTargets,
    validityGuards: {
      ...DEFAULT_TOLERANCES,
      ...plan.validityGuards,
    },
    validityScope: {
      ...plan.validityScope,
      robot: true,
    },
    metadata: {
      recoveryType: 'reverse_executed_prefix',
      executedTargetCount: normalizedExecutedTargets.length,
    },
  });
}

// Compatibility for M0/M1 callers. Runtime execution must use
// validateExecutionProgress once the first target starts moving.
export const evaluatePlanValidity = validatePlanStart;

export { DEFAULT_TOLERANCES };
