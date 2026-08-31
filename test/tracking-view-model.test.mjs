import test from 'node:test';
import assert from 'node:assert/strict';
import { buildTrackingViewModel } from '../ui/tracking-view-model.mjs';

test('builds a full-auto tracking model with stage and chunk progress', () => {
  const model = buildTrackingViewModel({
    taskGoal: 'Open, pick and place',
    activeStage: 'pick',
    activeIndex: 1,
    phase: 'EXECUTING',
    message: 'Đang đi tới pregrasp',
    snapshotId: 'snapshot-long-id',
    planId: 'plan-pick-2',
    candidateId: 'grasp-top',
    attempt: 2,
    replanCount: 1,
    progress: { chunkIndex: 7, chunkCount: 20 },
    recovery: null,
    stages: [
      { id: 'open', order: 1, status: 'completed' },
      { id: 'pick', order: 2, status: 'active' },
      { id: 'place', order: 3, status: 'upcoming' },
    ],
    timeline: [{ type: 'CHUNK_STARTED', at: 100, message: null }],
  });

  assert.equal(model.header, 'BƯỚC 2/3 · GẮP VẬT');
  assert.equal(model.phaseLabel, 'ĐANG DI CHUYỂN');
  assert.equal(model.progressPercent, 35);
  assert.equal(model.progressText, 'CHUNK 7/20');
  assert.equal(model.stages[0].icon, '✓');
  assert.equal(model.stages[1].icon, '2');
  assert.equal(model.modeLabel, 'TỰ ĐỘNG HOÀN TOÀN');
  assert.doesNotMatch(JSON.stringify(model), /người\s*\+\s*tool/i);
});

test('surfaces automatic recovery reason and terminal failure', () => {
  const recovering = buildTrackingViewModel({
    activeStage: 'place',
    activeIndex: 2,
    phase: 'RECOVERING',
    message: 'Đang lùi khỏi cửa lò',
    replanCount: 3,
    attempt: 2,
    progress: { chunkIndex: 4, chunkCount: 12 },
    recovery: {
      reason: 'UNEXPECTED_COLLISION',
      message: 'Đang lùi khỏi cửa lò',
    },
    stages: [
      { id: 'open', order: 1, status: 'completed' },
      { id: 'pick', order: 2, status: 'completed' },
      { id: 'place', order: 3, status: 'active' },
    ],
    timeline: [],
  });
  const failed = buildTrackingViewModel({
    ...recovering,
    phase: 'FAILED',
    terminalReason: 'ATTEMPT_BUDGET_EXHAUSTED',
  });

  assert.equal(recovering.phaseLabel, 'ĐANG TỰ PHỤC HỒI');
  assert.equal(recovering.alert.reason, 'UNEXPECTED_COLLISION');
  assert.equal(failed.phaseLabel, 'ĐÃ DỪNG AN TOÀN');
  assert.equal(failed.alert.reason, 'ATTEMPT_BUDGET_EXHAUSTED');
});

test('can be serialized into the browser without hidden module dependencies', () => {
  const isolated = new Function(`return (${buildTrackingViewModel.toString()})`)();
  const model = isolated({
    activeStage: 'open',
    activeIndex: 0,
    phase: 'PLANNING',
    stages: [{ id: 'open', status: 'active' }],
    progress: { chunkIndex: 0, chunkCount: 0 },
  });

  assert.equal(model.phaseLabel, 'ĐANG LẬP ĐƯỜNG');
});
