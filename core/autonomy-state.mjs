const PHASE_MESSAGES = Object.freeze({
  IDLE: 'Sẵn sàng bắt đầu',
  OBSERVING: 'Đang đọc trạng thái thật của scene',
  PLANNING: 'Đang lập đường an toàn cho bước hiện tại',
  EXECUTING: 'Đang tự động di chuyển',
  VERIFYING: 'Đang xác minh kết quả vật lý',
  RECOVERING: 'Đang tự lùi và lập lại đường',
  COMPLETE: 'Task đã hoàn thành',
  FAILED: 'Tool đã dừng an toàn',
  ABORTED: 'Đã dừng theo yêu cầu',
});

const timelineEvent = (event) => Object.freeze({
  type: String(event.type),
  at: Number(event.at ?? Date.now()),
  stage: event.stage ?? null,
  message: event.message ?? null,
  reason: event.reason ?? null,
});

const withTimeline = (state, event) => ({
  ...state,
  timeline: [...state.timeline, timelineEvent(event)].slice(-40),
});

const stageList = (workflow, activeIndex = 0) =>
  workflow.map((id, index) => ({
    id,
    order: index + 1,
    status: index < activeIndex
      ? 'completed'
      : index === activeIndex
        ? 'active'
        : 'upcoming',
  }));

export function createAutonomyState({
  workflow,
  taskGoal = '',
  runId = null,
}) {
  const normalizedWorkflow = Array.from(workflow ?? [], String);
  return {
    version: 1,
    runId,
    taskGoal: String(taskGoal),
    workflow: normalizedWorkflow,
    activeStage: normalizedWorkflow[0] ?? 'complete',
    activeIndex: normalizedWorkflow.length > 0 ? 0 : -1,
    stages: stageList(normalizedWorkflow),
    phase: normalizedWorkflow.length > 0 ? 'IDLE' : 'COMPLETE',
    message: normalizedWorkflow.length > 0
      ? PHASE_MESSAGES.IDLE
      : PHASE_MESSAGES.COMPLETE,
    snapshotId: null,
    planId: null,
    candidateId: null,
    attempt: 0,
    replanCount: 0,
    progress: {
      chunkIndex: 0,
      chunkCount: 0,
    },
    verifier: null,
    recovery: null,
    terminalReason: null,
    timeline: [],
  };
}

export function reduceAutonomyState(previous, event) {
  let state = withTimeline(previous, event);
  switch (event.type) {
    case 'SNAPSHOT_CAPTURED':
      return {
        ...state,
        snapshotId: event.snapshotId ?? state.snapshotId,
        phase: 'OBSERVING',
        message: event.message ?? PHASE_MESSAGES.OBSERVING,
      };
    case 'PLANNING_STARTED':
      return {
        ...state,
        activeStage: event.stage ?? state.activeStage,
        phase: 'PLANNING',
        message: event.message ?? PHASE_MESSAGES.PLANNING,
        recovery: null,
      };
    case 'PLAN_READY':
      return {
        ...state,
        planId: event.planId ?? null,
        candidateId: event.candidateId ?? null,
        attempt: Number(event.attempt ?? Math.max(1, state.attempt || 1)),
        phase: 'EXECUTING',
        message: event.message ?? PHASE_MESSAGES.EXECUTING,
        progress: {
          chunkIndex: 0,
          chunkCount: Math.max(0, Number(event.chunkCount) || 0),
        },
      };
    case 'CHUNK_STARTED':
    case 'CHUNK_COMPLETED':
      return {
        ...state,
        phase: 'EXECUTING',
        message: event.message ?? PHASE_MESSAGES.EXECUTING,
        progress: {
          chunkIndex: Math.max(0, Number(event.chunkIndex) || 0),
          chunkCount: Math.max(
            state.progress.chunkCount,
            Number(event.chunkCount) || 0,
          ),
        },
      };
    case 'VERIFY_STARTED':
      return {
        ...state,
        phase: 'VERIFYING',
        verifier: event.verifier ?? null,
        message: event.message ?? PHASE_MESSAGES.VERIFYING,
      };
    case 'RECOVERY_STARTED':
      return {
        ...state,
        phase: 'RECOVERING',
        message: event.message ?? PHASE_MESSAGES.RECOVERING,
        replanCount: state.replanCount + 1,
        recovery: {
          reason: String(event.reason ?? 'UNKNOWN'),
          message: event.message ?? PHASE_MESSAGES.RECOVERING,
        },
      };
    case 'STAGE_COMPLETED': {
      const completedIndex = state.workflow.indexOf(event.stage ?? state.activeStage);
      const nextIndex = completedIndex + 1;
      if (nextIndex >= state.workflow.length) {
        return {
          ...state,
          activeStage: 'complete',
          activeIndex: state.workflow.length,
          stages: stageList(state.workflow, state.workflow.length),
          phase: 'COMPLETE',
          message: event.message ?? PHASE_MESSAGES.COMPLETE,
          planId: null,
          candidateId: null,
          progress: { chunkIndex: 0, chunkCount: 0 },
          verifier: null,
          recovery: null,
        };
      }
      return {
        ...state,
        activeStage: state.workflow[nextIndex],
        activeIndex: nextIndex,
        stages: stageList(state.workflow, nextIndex),
        phase: 'OBSERVING',
        message: event.message ?? PHASE_MESSAGES.OBSERVING,
        planId: null,
        candidateId: null,
        attempt: 0,
        progress: { chunkIndex: 0, chunkCount: 0 },
        verifier: null,
        recovery: null,
      };
    }
    case 'RUN_FAILED':
      return {
        ...state,
        phase: 'FAILED',
        message: event.message ?? PHASE_MESSAGES.FAILED,
        terminalReason: event.reason ?? 'UNKNOWN',
      };
    case 'RUN_ABORTED':
      return {
        ...state,
        phase: 'ABORTED',
        message: event.message ?? PHASE_MESSAGES.ABORTED,
        terminalReason: 'ABORTED',
      };
    default:
      return state;
  }
}

export { PHASE_MESSAGES };
