import {
  createExecutedPrefixRetreat,
  validateExecutionProgress,
  validatePlanStart,
} from '../world/plan-contract.mjs';

const firstReason = (validation, fallback) =>
  validation.reasons[0]?.code ?? fallback;

const isAbort = (error) =>
  /AUTONOMY_ABORTED|aborted/i.test(error?.message ?? String(error));

const emit = async (onEvent, event) => {
  await onEvent?.({
    at: Date.now(),
    ...event,
  });
};

const observationSnapshot = async ({
  bridge,
  observation,
  snapshotFromObservation,
}) => {
  const liveObservation = observation ?? await bridge.observe();
  return snapshotFromObservation(liveObservation);
};

const executeRetreat = async ({
  plan,
  bridge,
  executedTargets,
  recoverySnapshot,
  snapshotFromObservation,
  onEvent,
  tracking,
}) => {
  const retreat = createExecutedPrefixRetreat({
    plan,
    executedTargets,
    recoverySnapshot,
    validForMilliseconds: tracking.retreatValidForMilliseconds,
  });
  const startValidity = validatePlanStart(retreat, recoverySnapshot);
  if (!startValidity.valid) {
    return {
      completed: false,
      reason: firstReason(startValidity, 'RETREAT_START_INVALID'),
      retreat,
      snapshot: recoverySnapshot,
    };
  }
  let latestSnapshot = recoverySnapshot;
  const targets = retreat.jointTargets.slice(1);
  for (let index = 0; index < targets.length; index += 1) {
    const target = targets[index];
    await emit(onEvent, {
      type: 'RECOVERY_TARGET_STARTED',
      planId: plan.planId,
      retreatId: retreat.retreatId,
      targetIndex: index + 1,
      targetCount: targets.length,
    });
    let result;
    try {
      result = await bridge.executeChunk({
        planId: retreat.retreatId,
        snapshotId: retreat.sourceSnapshotId,
        chunkId: `retreat-${index + 1}`,
        jointTargets: [target],
        mode: 'retreat',
        ...tracking.bridgeOptions,
      });
    } catch (error) {
      if (isAbort(error)) {
        return {
          completed: false,
          aborted: true,
          reason: 'AUTONOMY_ABORTED',
          retreat,
          snapshot: latestSnapshot,
        };
      }
      return {
        completed: false,
        reason: 'RETREAT_EXECUTION_ERROR',
        error,
        retreat,
        snapshot: latestSnapshot,
      };
    }
    latestSnapshot = await observationSnapshot({
      bridge,
      observation: result?.observation,
      snapshotFromObservation,
    });
    if (!result?.completed) {
      return {
        completed: false,
        reason: result?.stopReason ?? 'RETREAT_TARGET_FAILED',
        retreat,
        snapshot: latestSnapshot,
      };
    }
    const progressValidity = validateExecutionProgress({
      plan: retreat,
      liveSnapshot: latestSnapshot,
      expectedRobotState: { qpos: target },
    });
    if (!progressValidity.valid) {
      return {
        completed: false,
        reason: firstReason(progressValidity, 'RETREAT_INVALID'),
        retreat,
        snapshot: latestSnapshot,
      };
    }
    await emit(onEvent, {
      type: 'RECOVERY_TARGET_COMPLETED',
      planId: plan.planId,
      retreatId: retreat.retreatId,
      targetIndex: index + 1,
      targetCount: targets.length,
      snapshotId: latestSnapshot.snapshotId,
    });
  }
  return {
    completed: true,
    retreat,
    snapshot: latestSnapshot,
  };
};

const recoverForReplan = async ({
  plan,
  bridge,
  executedTargets,
  recoverySnapshot,
  snapshotFromObservation,
  onEvent,
  tracking,
  reason,
}) => {
  await emit(onEvent, {
    type: 'RECOVERY_STARTED',
    planId: plan.planId,
    snapshotId: recoverySnapshot.snapshotId,
    reason,
  });
  const recovery = await executeRetreat({
    plan,
    bridge,
    executedTargets,
    recoverySnapshot,
    snapshotFromObservation,
    onEvent,
    tracking,
  });
  if (recovery.aborted) {
    return {
      status: 'aborted',
      reason: 'AUTONOMY_ABORTED',
      executedTargets,
      recovery,
      snapshot: recovery.snapshot,
    };
  }
  if (!recovery.completed) {
    bridge.stopMotion?.();
    await emit(onEvent, {
      type: 'RECOVERY_FAILED',
      planId: plan.planId,
      reason: recovery.reason,
    });
    return {
      status: 'failed',
      reason: recovery.reason,
      triggerReason: reason,
      executedTargets,
      recovery,
      snapshot: recovery.snapshot,
    };
  }
  await emit(onEvent, {
    type: 'RECOVERY_COMPLETED',
    planId: plan.planId,
    retreatId: recovery.retreat.retreatId,
    snapshotId: recovery.snapshot.snapshotId,
    reason,
  });
  return {
    status: 'replan',
    reason,
    executedTargets,
    recovery,
    snapshot: recovery.snapshot,
  };
};

export async function executeStagePlan({
  plan,
  bridge,
  snapshotFromObservation,
  onEvent = null,
  actionHandler = null,
  tracking = {},
}) {
  if (!plan?.planId) throw new Error('Stage plan is required');
  if (
    typeof bridge?.observe !== 'function'
    || typeof bridge?.executeChunk !== 'function'
  ) {
    throw new Error('Bridge observe and executeChunk functions are required');
  }
  if (typeof snapshotFromObservation !== 'function') {
    throw new Error('snapshotFromObservation is required');
  }
  const executedTargets = [];
  const startSnapshot = await observationSnapshot({
    bridge,
    snapshotFromObservation,
  });
  const startValidity = validatePlanStart(plan, startSnapshot);
  if (!startValidity.valid) {
    const reason = firstReason(startValidity, 'PLAN_START_INVALID');
    await emit(onEvent, {
      type: 'PLAN_INVALIDATED',
      planId: plan.planId,
      snapshotId: startSnapshot.snapshotId,
      reason,
      phase: 'start',
    });
    return {
      status: 'replan',
      reason,
      executedTargets,
      snapshot: startSnapshot,
    };
  }

  const steps = plan.chunks.flatMap((chunk, chunkIndex) =>
    chunk.jointTargets.map((target, targetIndex) => ({
      target,
      chunk,
      chunkIndex,
      targetIndex,
      isChunkEnd: targetIndex === chunk.jointTargets.length - 1,
    })));
  let latestSnapshot = startSnapshot;
  for (let index = 0; index < steps.length; index += 1) {
    const step = steps[index];
    const target = step.target;
    await emit(onEvent, {
      type: 'CHUNK_STARTED',
      planId: plan.planId,
      chunkIndex: index + 1,
      chunkCount: steps.length,
      semanticChunkId: step.chunk.id,
    });
    let result;
    try {
      result = await bridge.executeChunk({
        planId: plan.planId,
        snapshotId: plan.sourceSnapshotId,
        chunkId: `target-${index + 1}`,
        jointTargets: [target],
        mode: 'forward',
        ...tracking.bridgeOptions,
      });
    } catch (error) {
      if (isAbort(error)) {
        return {
          status: 'aborted',
          reason: 'AUTONOMY_ABORTED',
          executedTargets,
          snapshot: latestSnapshot,
        };
      }
      const recoverySnapshot = await observationSnapshot({
        bridge,
        snapshotFromObservation,
      });
      return recoverForReplan({
        plan,
        bridge,
        executedTargets,
        recoverySnapshot,
        snapshotFromObservation,
        onEvent,
        tracking,
        reason: 'EXECUTION_ERROR',
      });
    }

    latestSnapshot = await observationSnapshot({
      bridge,
      observation: result?.observation,
      snapshotFromObservation,
    });
    if (result?.completed) executedTargets.push([...target]);
    if (!result?.completed) {
      return recoverForReplan({
        plan,
        bridge,
        executedTargets,
        recoverySnapshot: latestSnapshot,
        snapshotFromObservation,
        onEvent,
        tracking,
        reason: result?.stopReason ?? 'TARGET_FAILED',
      });
    }

    const progressValidity = validateExecutionProgress({
      plan,
      liveSnapshot: latestSnapshot,
      expectedRobotState: { qpos: target },
      executionScope: step.chunk.executionScope ?? null,
    });
    if (!progressValidity.valid) {
      const reason = firstReason(progressValidity, 'EXECUTION_INVALID');
      await emit(onEvent, {
        type: 'PLAN_INVALIDATED',
        planId: plan.planId,
        snapshotId: latestSnapshot.snapshotId,
        reason,
        phase: 'execution',
      });
      return recoverForReplan({
        plan,
        bridge,
        executedTargets,
        recoverySnapshot: latestSnapshot,
        snapshotFromObservation,
        onEvent,
        tracking,
        reason,
      });
    }
    await emit(onEvent, {
      type: 'CHUNK_COMPLETED',
      planId: plan.planId,
      snapshotId: latestSnapshot.snapshotId,
      chunkIndex: index + 1,
      chunkCount: steps.length,
      semanticChunkId: step.chunk.id,
    });
    if (step.isChunkEnd && step.chunk.actionAfter) {
      const action = String(step.chunk.actionAfter);
      if (typeof actionHandler !== 'function') {
        return recoverForReplan({
          plan,
          bridge,
          executedTargets,
          recoverySnapshot: latestSnapshot,
          snapshotFromObservation,
          onEvent,
          tracking,
          reason: 'ACTION_HANDLER_MISSING',
        });
      }
      await emit(onEvent, {
        type: 'ACTION_STARTED',
        planId: plan.planId,
        action,
        semanticChunkId: step.chunk.id,
      });
      let actionResult;
      try {
        actionResult = await actionHandler({
          action,
          chunk: step.chunk,
          plan,
          snapshot: latestSnapshot,
        });
      } catch (error) {
        if (isAbort(error)) {
          return {
            status: 'aborted',
            reason: 'AUTONOMY_ABORTED',
            executedTargets,
            snapshot: latestSnapshot,
          };
        }
        const recoverySnapshot = await observationSnapshot({
          bridge,
          snapshotFromObservation,
        });
        return recoverForReplan({
          plan,
          bridge,
          executedTargets,
          recoverySnapshot,
          snapshotFromObservation,
          onEvent,
          tracking,
          reason: 'ACTION_FAILED',
        });
      }
      if (actionResult?.observation) {
        latestSnapshot = await observationSnapshot({
          bridge,
          observation: actionResult.observation,
          snapshotFromObservation,
        });
      }
      if (actionResult?.completed === false) {
        return recoverForReplan({
          plan,
          bridge,
          executedTargets,
          recoverySnapshot: latestSnapshot,
          snapshotFromObservation,
          onEvent,
          tracking,
          reason: actionResult.reason ?? 'ACTION_FAILED',
        });
      }
      await emit(onEvent, {
        type: 'ACTION_COMPLETED',
        planId: plan.planId,
        action,
        semanticChunkId: step.chunk.id,
        snapshotId: latestSnapshot.snapshotId,
      });
    }
  }
  return {
    status: 'completed',
    executedTargets,
    snapshot: latestSnapshot,
  };
}
