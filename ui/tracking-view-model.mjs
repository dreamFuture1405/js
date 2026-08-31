export function buildTrackingViewModel(state = {}) {
  const STAGE_LABELS = {
    open: 'MỞ CỬA',
    pick: 'GẮP VẬT',
    place: 'ĐẶT VẬT',
    release: 'THẢ VẬT',
    complete: 'HOÀN THÀNH',
  };
  const PHASES = {
    IDLE: { label: 'SẴN SÀNG', color: '#94a3b8' },
    OBSERVING: { label: 'ĐANG QUAN SÁT', color: '#38bdf8' },
    PLANNING: { label: 'ĐANG LẬP ĐƯỜNG', color: '#22d3ee' },
    EXECUTING: { label: 'ĐANG DI CHUYỂN', color: '#a3e635' },
    VERIFYING: { label: 'ĐANG XÁC MINH', color: '#facc15' },
    RECOVERING: { label: 'ĐANG TỰ PHỤC HỒI', color: '#fb923c' },
    COMPLETE: { label: 'TASK HOÀN THÀNH', color: '#4ade80' },
    FAILED: { label: 'ĐÃ DỪNG AN TOÀN', color: '#f87171' },
    ABORTED: { label: 'ĐÃ ABORT', color: '#f87171' },
  };
  const compactId = (value) => {
    const text = String(value ?? '—');
    return text.length <= 22 ? text : `${text.slice(0, 10)}…${text.slice(-9)}`;
  };
  const eventText = (event) => {
    if (event.message) return String(event.message);
    const labels = {
      SNAPSHOT_CAPTURED: 'Đã chụp live snapshot',
      PLANNING_STARTED: 'Bắt đầu lập đường',
      PLAN_READY: 'Đã chọn trajectory',
      CHUNK_STARTED: 'Đang chạy chunk',
      CHUNK_COMPLETED: 'Đã hoàn thành chunk',
      VERIFY_STARTED: 'Bắt đầu xác minh vật lý',
      RECOVERY_STARTED: 'Bắt đầu tự phục hồi',
      STAGE_COMPLETED: 'Bước đã hoàn thành',
      RUN_FAILED: 'Run đã dừng an toàn',
      RUN_ABORTED: 'Người dùng yêu cầu abort',
    };
    return labels[event.type] ?? String(event.type ?? 'EVENT');
  };
  const stages = Array.from(state.stages ?? []);
  const activeIndex = Number.isInteger(state.activeIndex)
    ? state.activeIndex
    : Math.max(0, stages.findIndex((stage) => stage.status === 'active'));
  const phase = PHASES[state.phase] ?? PHASES.IDLE;
  const chunkIndex = Math.max(0, Number(state.progress?.chunkIndex) || 0);
  const chunkCount = Math.max(0, Number(state.progress?.chunkCount) || 0);
  const progressPercent = chunkCount > 0
    ? Math.max(0, Math.min(100, Math.round(chunkIndex / chunkCount * 100)))
    : state.phase === 'COMPLETE'
      ? 100
      : 0;
  const activeStageLabel = STAGE_LABELS[state.activeStage]
    ?? String(state.activeStage ?? 'CHƯA XÁC ĐỊNH').toUpperCase();
  const alertReason = state.phase === 'FAILED' || state.phase === 'ABORTED'
    ? state.terminalReason
    : state.recovery?.reason;
  const alertMessage = state.phase === 'FAILED' || state.phase === 'ABORTED'
    ? state.message
    : state.recovery?.message;
  return {
    modeLabel: 'TỰ ĐỘNG HOÀN TOÀN',
    header: state.activeStage === 'complete'
      ? 'TASK ĐÃ HOÀN THÀNH'
      : `BƯỚC ${Math.max(1, activeIndex + 1)}/${Math.max(1, stages.length)} · ${activeStageLabel}`,
    goal: String(state.taskGoal ?? ''),
    phaseLabel: phase.label,
    phaseColor: phase.color,
    message: String(state.message ?? ''),
    progressPercent,
    progressText: chunkCount > 0
      ? `CHUNK ${chunkIndex}/${chunkCount}`
      : state.phase === 'PLANNING'
        ? 'ĐANG TÌM TRAJECTORY'
        : 'CHƯA CÓ CHUNK',
    snapshotId: compactId(state.snapshotId),
    planId: compactId(state.planId),
    candidateId: compactId(state.candidateId),
    attempt: Math.max(0, Number(state.attempt) || 0),
    replanCount: Math.max(0, Number(state.replanCount) || 0),
    verifier: state.verifier ? String(state.verifier) : '—',
    stages: stages.map((stage, index) => ({
      id: stage.id,
      label: STAGE_LABELS[stage.id] ?? String(stage.id).toUpperCase(),
      status: stage.status,
      icon: stage.status === 'completed' ? '✓' : String(index + 1),
    })),
    alert: alertReason
      ? {
        reason: String(alertReason),
        message: String(alertMessage ?? ''),
      }
      : null,
    timeline: Array.from(state.timeline ?? [])
      .slice(-7)
      .reverse()
      .map((event) => ({
        type: String(event.type ?? 'EVENT'),
        text: eventText(event),
        at: Number(event.at ?? 0),
      })),
  };
}
