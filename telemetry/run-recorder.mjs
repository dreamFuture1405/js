const percentile = (values, ratio) => {
  if (values.length === 0) return null;
  const sorted = [...values].sort((left, right) => left - right);
  const index = Math.max(0, Math.ceil(sorted.length * ratio) - 1);
  return sorted[index];
};

export function summarizeRunEvents(events) {
  const planningEvents = events.filter((event) => event.type === 'PLAN_FINISHED');
  const successfulPlanningEvents = planningEvents.filter((event) => event.success !== false);
  const planningDurations = successfulPlanningEvents
    .map((event) => Number(event.durationMilliseconds))
    .filter(Number.isFinite);
  const planningFailureReasons = {};
  for (const event of planningEvents) {
    if (event.success !== false) continue;
    const reason = String(event.failureReason ?? 'UNKNOWN');
    planningFailureReasons[reason] = (planningFailureReasons[reason] ?? 0) + 1;
  }
  const stopReasons = {};
  for (const event of events) {
    if (event.type !== 'MOTION_STOPPED' || !event.reason) continue;
    const reason = String(event.reason);
    stopReasons[reason] = (stopReasons[reason] ?? 0) + 1;
  }
  return {
    eventCount: events.length,
    planning: {
      attempted: planningEvents.length,
      count: planningDurations.length,
      failed: planningEvents.length - successfulPlanningEvents.length,
      p50Milliseconds: percentile(planningDurations, 0.5),
      p95Milliseconds: percentile(planningDurations, 0.95),
      maximumMilliseconds: planningDurations.length
        ? Math.max(...planningDurations)
        : null,
      failureReasons: Object.fromEntries(
        Object.entries(planningFailureReasons)
          .sort(([left], [right]) => left.localeCompare(right)),
      ),
    },
    stopReasons: Object.fromEntries(
      Object.entries(stopReasons).sort(([left], [right]) => left.localeCompare(right)),
    ),
  };
}

export function createRunRecorder({
  runId,
  now = Date.now,
} = {}) {
  if (!String(runId ?? '')) throw new Error('Run recorder requires a run id');
  const recorded = [];
  let context = {
    snapshotId: null,
    planId: null,
  };
  return {
    setContext(nextContext = {}) {
      context = {
        ...context,
        ...nextContext,
      };
      return { ...context };
    },
    record(entry) {
      if (!entry?.type) throw new Error('Telemetry event type is required');
      const event = Object.freeze({
        runId: String(runId),
        sequence: recorded.length + 1,
        at: Number(now()),
        snapshotId: context.snapshotId,
        planId: context.planId,
        ...entry,
      });
      recorded.push(event);
      return event;
    },
    events() {
      return [...recorded];
    },
    summary() {
      return summarizeRunEvents(recorded);
    },
  };
}
