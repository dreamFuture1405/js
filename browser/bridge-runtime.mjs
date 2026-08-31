export function installAxisAutonomyBridge(config = {}, helpers = {}) {
  window.__axisAutonomyV3Bridge?.destroy?.();
  const demo = window.__axisAutomationDemo;
  const session = window.__axisAutomationSession;
  if (!demo?.model || !demo?.data || !demo?.mujoco || !demo?.ikController) {
    throw new Error('Axis simulator runtime is not ready');
  }

  const model = demo.model;
  const data = demo.data;
  const controller = demo.ikController;
  const evaluateJointTargetProgress = helpers.evaluateJointTargetProgress;
  if (typeof evaluateJointTargetProgress !== 'function') {
    throw new Error('Joint target tracking helper is required');
  }
  const sleep = (milliseconds) => new Promise((resolve) => setTimeout(resolve, milliseconds));
  const bodyNameToId = new Map();
  const jointNameToId = new Map();
  for (let id = 0; id < Number(model.nbody); id += 1) {
    const name = demo.mujoco.mj_id2name(model, 1, id);
    if (name) bodyNameToId.set(name, id);
  }
  for (let id = 0; id < Number(model.njnt); id += 1) {
    const name = demo.mujoco.mj_id2name(model, 3, id);
    if (name) jointNameToId.set(name, id);
  }
  const robotJointNames = Array.from(
    config.robotJointNames ?? demo.robotControlConfig?.jointNames ?? [],
    String,
  );
  const robotJoints = robotJointNames
    .map((name) => {
      const id = jointNameToId.get(name);
      if (!Number.isInteger(id)) return null;
      return {
        name,
        id,
        qposAddress: Number(model.jnt_qposadr[id]),
        dofAddress: Number(model.jnt_dofadr[id]),
      };
    })
    .filter(Boolean);
  const trackedBodyNames = Array.from(new Set(
    Array.from(config.trackedBodyNames ?? [], String).filter((name) => bodyNameToId.has(name)),
  ));
  const trackedJointNames = Array.from(new Set(
    Array.from(config.trackedJointNames ?? [], String).filter((name) => jointNameToId.has(name)),
  ));
  const endEffectorBodyName = String(
    config.endEffectorBodyName
      ?? demo.robotControlConfig?.endEffectorBodyName
      ?? 'franka/panda_hand',
  );
  const endEffectorBodyId = bodyNameToId.get(endEffectorBodyName) ?? -1;
  const workflow = Array.from(config.workflow ?? [], String);
  const taskId = config.taskId ?? session?.task?.id ?? 'unknown-task';
  const modelId = [
    taskId,
    model.nq,
    model.nv,
    model.nu,
    model.nbody,
    model.ngeom,
  ].join(':');
  let sequence = 0;
  let destroyed = false;
  let abortRequested = false;
  let activeCommand = null;

  const bodyPose = (id) => {
    const positionOffset = id * 3;
    const quaternionOffset = id * 4;
    return {
      position: [
        Number(data.xpos[positionOffset]),
        Number(data.xpos[positionOffset + 1]),
        Number(data.xpos[positionOffset + 2]),
      ],
      quaternion: [
        Number(data.xquat[quaternionOffset + 1]),
        Number(data.xquat[quaternionOffset + 2]),
        Number(data.xquat[quaternionOffset + 3]),
        Number(data.xquat[quaternionOffset]),
      ],
    };
  };
  const checker = () => {
    try {
      return demo.checkerManager?.getStatus?.() ?? null;
    } catch {
      return null;
    }
  };
  const checkerStage = (status) => {
    if (status?.overall) return 'complete';
    const index = Number(status?.currentIndex);
    return Number.isInteger(index) && workflow[index]
      ? workflow[index]
      : workflow[0] ?? 'unknown';
  };
  const stopMotion = () => {
    demo.cancelAutoMoveToObject?.('manual');
    demo.finishEndEffectorDrag?.();
    demo.cancelEndEffectorDrag?.('cancel');
    demo.keyboardStateManager?.onMotionStop?.();
    demo.keyboardStateManager?.clearActiveKeys?.();
    demo.keyboardStateManager?.clearPressedKeys?.();
    controller.settleAtCurrentPose?.({ zeroVelocity: true });
    controller.syncCtrlFromQpos?.();
  };
  const observe = () => {
    if (destroyed) throw new Error('Axis autonomy bridge has been destroyed');
    demo.scene?.updateMatrixWorld?.(true);
    const checkerStatus = checker();
    const objects = Object.fromEntries(
      trackedBodyNames.map((name) => {
        const id = bodyNameToId.get(name);
        return [name, {
          ...bodyPose(id),
          velocity: [],
          bounds: null,
          supportContacts: [],
        }];
      }),
    );
    const articulatedJoints = Object.fromEntries(
      trackedJointNames.map((name) => {
        const id = jointNameToId.get(name);
        const address = Number(model.jnt_qposadr[id]);
        return [name, Number(data.qpos[address])];
      }),
    );
    sequence += 1;
    return {
      sequence,
      capturedAt: Date.now(),
      modelId,
      simulationTime: Number(data.time ?? 0),
      checkerStage: checkerStage(checkerStatus),
      checker: checkerStatus,
      robot: {
        qpos: robotJoints.map((joint) => Number(data.qpos[joint.qposAddress])),
        qvel: robotJoints.map((joint) => Number(data.qvel[joint.dofAddress])),
        endEffectorPose: endEffectorBodyId >= 0
          ? bodyPose(endEffectorBodyId)
          : {
            position: [0, 0, 0],
            quaternion: [0, 0, 0, 1],
          },
        gripperState: demo.gripperClosed ? 'closed' : 'open',
      },
      simulation: {
        qpos: Array.from(data.qpos, Number),
        qvel: Array.from(data.qvel, Number),
        ctrl: Array.from(data.ctrl, Number),
      },
      objects,
      articulatedJoints,
      contacts: {
        count: Math.max(0, Number(data.ncon) || 0),
        pairs: [],
        accessor: 'count_only',
      },
    };
  };
  const executeChunk = async ({
    planId,
    snapshotId,
    chunkId,
    jointTargets,
    pollMilliseconds = 32,
    targetToleranceRadians = 0.028,
    minimumProgressRadians = 0.0015,
    noProgressLimit = 12,
    maximumTargetMilliseconds = 1800,
    mode = 'forward',
  }) => {
    if (destroyed) throw new Error('Axis autonomy bridge has been destroyed');
    if (activeCommand) throw new Error(`Bridge is already executing ${activeCommand.chunkId}`);
    if (!Array.isArray(jointTargets) || jointTargets.length === 0) {
      throw new Error('Chunk requires joint targets');
    }
    if (jointTargets.some((target) => (
      !Array.isArray(target)
      || target.length !== robotJoints.length
      || target.some((value) => !Number.isFinite(Number(value)))
    ))) {
      throw new Error('Chunk joint target dimensions do not match the robot');
    }
    if (abortRequested) throw new Error('AUTONOMY_ABORTED');
    activeCommand = {
      planId: String(planId),
      snapshotId: String(snapshotId),
      chunkId: String(chunkId),
      mode: String(mode),
      targetCount: jointTargets.length,
      targetIndex: 0,
      trackingErrorRadians: null,
      noProgressCount: 0,
      startedAt: performance.now(),
    };
    const executedTargets = [];
    try {
      for (let index = 0; index < jointTargets.length; index += 1) {
        const target = jointTargets[index].map(Number);
        const targetStartedAt = performance.now();
        let bestError = Number.POSITIVE_INFINITY;
        let noProgressCount = 0;
        activeCommand.targetIndex = index + 1;
        while (true) {
          if (abortRequested) throw new Error('AUTONOMY_ABORTED');
          demo.markPolicyHumanInput?.();
          controller.writeJointTargetsToCtrl(new Float64Array(target));
          await sleep(Math.max(16, Math.min(100, Number(pollMilliseconds) || 32)));
          const measured = robotJoints.map(
            (joint) => Number(data.qpos[joint.qposAddress]),
          );
          const tracking = evaluateJointTargetProgress({
            target,
            measured,
            bestError,
            noProgressCount,
            elapsedMilliseconds: performance.now() - targetStartedAt,
            targetToleranceRadians,
            minimumProgressRadians,
            noProgressLimit,
            maximumTargetMilliseconds,
          });
          bestError = tracking.bestError;
          noProgressCount = tracking.noProgressCount;
          activeCommand.trackingErrorRadians = tracking.errorRadians;
          activeCommand.noProgressCount = tracking.noProgressCount;
          if (tracking.reached) {
            executedTargets.push(target);
            break;
          }
          if (tracking.stopReason) {
            return {
              completed: false,
              stopReason: tracking.stopReason,
              planId: activeCommand.planId,
              snapshotId: activeCommand.snapshotId,
              chunkId: activeCommand.chunkId,
              failedTargetIndex: index,
              trackingErrorRadians: tracking.errorRadians,
              elapsedMilliseconds: Math.round(
                performance.now() - activeCommand.startedAt,
              ),
              executedTargets,
              observation: observe(),
            };
          }
        }
      }
      return {
        completed: true,
        planId: activeCommand.planId,
        snapshotId: activeCommand.snapshotId,
        chunkId: activeCommand.chunkId,
        elapsedMilliseconds: Math.round(performance.now() - activeCommand.startedAt),
        executedTargets,
        observation: observe(),
      };
    } finally {
      stopMotion();
      activeCommand = null;
    }
  };
  const setGripper = async ({
    closed,
    timeoutMilliseconds = 1800,
    settleMilliseconds = 120,
  }) => {
    if (destroyed) throw new Error('Axis autonomy bridge has been destroyed');
    if (activeCommand) throw new Error(`Bridge is already executing ${activeCommand.chunkId}`);
    if (abortRequested) throw new Error('AUTONOMY_ABORTED');
    const targetClosed = Boolean(closed);
    demo.setGripperState(targetClosed);
    const startedAt = performance.now();
    while (
      demo.gripperAnimating
      && performance.now() - startedAt < Math.max(100, Number(timeoutMilliseconds) || 1800)
    ) {
      if (abortRequested) throw new Error('AUTONOMY_ABORTED');
      await sleep(32);
    }
    if (demo.gripperAnimating || Boolean(demo.gripperClosed) !== targetClosed) {
      stopMotion();
      return {
        completed: false,
        stopReason: 'GRIPPER_TIMEOUT',
        closed: Boolean(demo.gripperClosed),
        elapsedMilliseconds: Math.round(performance.now() - startedAt),
        observation: observe(),
      };
    }
    await sleep(Math.max(0, Math.min(500, Number(settleMilliseconds) || 120)));
    return {
      completed: true,
      closed: Boolean(demo.gripperClosed),
      elapsedMilliseconds: Math.round(performance.now() - startedAt),
      observation: observe(),
    };
  };

  const api = {
    observe,
    executeChunk,
    setGripper,
    stopMotion,
    beginRun() {
      if (activeCommand) {
        throw new Error(`Bridge is already executing ${activeCommand.chunkId}`);
      }
      abortRequested = false;
      return { ready: true };
    },
    abort() {
      abortRequested = true;
      stopMotion();
      return {
        aborted: true,
        activeCommand: activeCommand ? { ...activeCommand } : null,
      };
    },
    getStatus() {
      return {
        installed: !destroyed,
        modelId,
        sequence,
        trackedBodyNames,
        trackedJointNames,
        robotJointNames,
        activeCommand: activeCommand ? { ...activeCommand } : null,
        abortRequested,
      };
    },
    destroy() {
      abortRequested = true;
      stopMotion();
      destroyed = true;
      delete window.__axisAutonomyV3Bridge;
      return { installed: false };
    },
  };
  window.__axisAutonomyV3Bridge = api;
  return api.getStatus();
}
