import test from 'node:test';
import assert from 'node:assert/strict';
import {
  createAutonomyState,
  reduceAutonomyState,
} from '../core/autonomy-state.mjs';

const advance = (state, ...events) =>
  events.reduce((current, event) => reduceAutonomyState(current, event), state);

test('tracks observe, planning, execution, verification, and stage completion', () => {
  const initial = createAutonomyState({
    workflow: ['open', 'pick', 'place'],
    taskGoal: 'Open, pick and place',
  });
  const state = advance(
    initial,
    { type: 'SNAPSHOT_CAPTURED', snapshotId: 'snap-1', at: 100 },
    { type: 'PLANNING_STARTED', stage: 'open', at: 110 },
    {
      type: 'PLAN_READY',
      planId: 'plan-open-1',
      candidateId: 'door-left',
      chunkCount: 12,
      at: 140,
    },
    { type: 'CHUNK_STARTED', chunkIndex: 4, at: 150 },
    { type: 'VERIFY_STARTED', verifier: 'door-joint', at: 190 },
    { type: 'STAGE_COMPLETED', stage: 'open', at: 210 },
  );

  assert.equal(state.phase, 'OBSERVING');
  assert.equal(state.activeStage, 'pick');
  assert.equal(state.snapshotId, 'snap-1');
  assert.equal(state.planId, null);
  assert.equal(state.stages[0].status, 'completed');
  assert.equal(state.stages[1].status, 'active');
  assert.ok(state.timeline.some((event) => event.type === 'VERIFY_STARTED'));
});

test('shows chunk progress, attempt count, and automatic recovery reason', () => {
  const initial = createAutonomyState({
    workflow: ['pick', 'place'],
    taskGoal: 'Pick and place',
  });
  const state = advance(
    initial,
    { type: 'PLANNING_STARTED', stage: 'pick', at: 10 },
    {
      type: 'PLAN_READY',
      planId: 'plan-2',
      candidateId: 'grasp-side',
      chunkCount: 20,
      attempt: 2,
      at: 20,
    },
    { type: 'CHUNK_STARTED', chunkIndex: 6, at: 30 },
    {
      type: 'RECOVERY_STARTED',
      reason: 'UNEXPECTED_COLLISION',
      message: 'Lùi về pregrasp và lập đường mới',
      at: 40,
    },
  );

  assert.equal(state.phase, 'RECOVERING');
  assert.equal(state.progress.chunkIndex, 6);
  assert.equal(state.progress.chunkCount, 20);
  assert.equal(state.attempt, 2);
  assert.equal(state.recovery.reason, 'UNEXPECTED_COLLISION');
  assert.equal(state.replanCount, 1);
});

test('finishes fully automatically without entering a manual phase', () => {
  let state = createAutonomyState({
    workflow: ['pick'],
    taskGoal: 'Pick',
  });
  state = advance(
    state,
    { type: 'PLANNING_STARTED', stage: 'pick', at: 10 },
    {
      type: 'PLAN_READY',
      planId: 'plan-1',
      candidateId: 'candidate-1',
      chunkCount: 1,
      at: 20,
    },
    { type: 'STAGE_COMPLETED', stage: 'pick', at: 30 },
  );

  assert.equal(state.phase, 'COMPLETE');
  assert.equal(state.activeStage, 'complete');
  assert.equal(
    state.timeline.some((event) => String(event.type).includes('MANUAL')),
    false,
  );
});
