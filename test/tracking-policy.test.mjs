import test from 'node:test';
import assert from 'node:assert/strict';
import { evaluateJointTargetProgress } from '../execution/tracking-policy.mjs';

test('marks a joint target reached only inside the configured error envelope', () => {
  const result = evaluateJointTargetProgress({
    target: [0.2, 0.3],
    measured: [0.181, 0.31],
    bestError: 0.05,
    noProgressCount: 3,
    elapsedMilliseconds: 200,
    targetToleranceRadians: 0.02,
  });

  assert.equal(result.reached, true);
  assert.equal(result.stopReason, null);
  assert.equal(result.noProgressCount, 0);
});

test('resets the stall counter when physical error makes meaningful progress', () => {
  const result = evaluateJointTargetProgress({
    target: [0.4, 0.4],
    measured: [0.31, 0.32],
    bestError: 0.12,
    noProgressCount: 7,
    elapsedMilliseconds: 300,
    minimumProgressRadians: 0.005,
  });

  assert.equal(result.improved, true);
  assert.equal(result.noProgressCount, 0);
  assert.equal(result.stopReason, null);
});

test('stops a target that repeatedly makes no physical progress', () => {
  const result = evaluateJointTargetProgress({
    target: [0.5, 0.5],
    measured: [0.2, 0.2],
    bestError: 0.3,
    noProgressCount: 11,
    elapsedMilliseconds: 500,
    noProgressLimit: 12,
  });

  assert.equal(result.reached, false);
  assert.equal(result.stopReason, 'TARGET_STALLED');
});

test('stops a slowly changing target at its hard deadline', () => {
  const result = evaluateJointTargetProgress({
    target: [0.5, 0.5],
    measured: [0.3, 0.3],
    bestError: 0.21,
    noProgressCount: 1,
    elapsedMilliseconds: 1800,
    maximumTargetMilliseconds: 1800,
  });

  assert.equal(result.reached, false);
  assert.equal(result.stopReason, 'TARGET_TIMEOUT');
});

test('rejects target and measured vectors with different dimensions', () => {
  assert.throws(
    () => evaluateJointTargetProgress({
      target: [0.5],
      measured: [0.5, 0.5],
    }),
    /dimensions/i,
  );
});
