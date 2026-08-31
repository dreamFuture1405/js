import test from 'node:test';
import assert from 'node:assert/strict';
import {
  createRunRecorder,
  summarizeRunEvents,
} from '../telemetry/run-recorder.mjs';

test('summarizes planning p50/p95 and failure reasons', () => {
  const events = [
    { type: 'PLAN_FINISHED', durationMilliseconds: 100 },
    { type: 'PLAN_FINISHED', durationMilliseconds: 200 },
    { type: 'PLAN_FINISHED', durationMilliseconds: 300 },
    { type: 'PLAN_FINISHED', durationMilliseconds: 400 },
    { type: 'PLAN_FINISHED', durationMilliseconds: 500 },
    { type: 'MOTION_STOPPED', reason: 'UNEXPECTED_COLLISION' },
    { type: 'MOTION_STOPPED', reason: 'UNEXPECTED_COLLISION' },
    { type: 'MOTION_STOPPED', reason: 'TRACKING_LAG' },
  ];

  const summary = summarizeRunEvents(events);

  assert.equal(summary.planning.count, 5);
  assert.equal(summary.planning.attempted, 5);
  assert.equal(summary.planning.failed, 0);
  assert.equal(summary.planning.p50Milliseconds, 300);
  assert.equal(summary.planning.p95Milliseconds, 500);
  assert.deepEqual(summary.stopReasons, {
    TRACKING_LAG: 1,
    UNEXPECTED_COLLISION: 2,
  });
});

test('records immutable events with run, snapshot, and plan correlation ids', () => {
  const recorder = createRunRecorder({
    runId: 'run-1',
    now: () => 1234,
  });

  recorder.setContext({ snapshotId: 'snap-1', planId: 'plan-1' });
  const event = recorder.record({ type: 'PLAN_FINISHED', durationMilliseconds: 50 });

  assert.deepEqual(event, {
    runId: 'run-1',
    sequence: 1,
    at: 1234,
    snapshotId: 'snap-1',
    planId: 'plan-1',
    type: 'PLAN_FINISHED',
    durationMilliseconds: 50,
  });
  assert.equal(Object.isFrozen(event), true);
  assert.equal(recorder.events().length, 1);
});

test('does not report time-to-crash as successful planning latency', () => {
  const summary = summarizeRunEvents([
    {
      type: 'PLAN_FINISHED',
      durationMilliseconds: 4,
      success: false,
      failureReason: 'WASM_ABORT',
    },
    {
      type: 'PLAN_FINISHED',
      durationMilliseconds: 6,
      success: false,
      failureReason: 'WASM_ABORT',
    },
  ]);

  assert.equal(summary.planning.attempted, 2);
  assert.equal(summary.planning.count, 0);
  assert.equal(summary.planning.failed, 2);
  assert.equal(summary.planning.p50Milliseconds, null);
  assert.deepEqual(summary.planning.failureReasons, {
    WASM_ABORT: 2,
  });
});
