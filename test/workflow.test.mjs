import test from 'node:test';
import assert from 'node:assert/strict';
import {
  trackedEntitiesFromPlan,
  workflowFromPlan,
} from '../task/workflow.mjs';

test('compiles the semantic workflow without caching future trajectories', () => {
  const plan = {
    entities: {
      objectBodyName: 'item',
      containerBodyName: 'microwave',
      doorBodyName: 'microwave_door',
      doorJointName: 'microwave_hinge',
    },
    stages: [
      { kind: 'open_door' },
      { kind: 'grip_pick' },
      { kind: 'carry_inside' },
    ],
  };

  assert.deepEqual(workflowFromPlan(plan), ['open', 'pick', 'place']);
  assert.deepEqual(trackedEntitiesFromPlan(plan), {
    bodyNames: ['item', 'microwave', 'microwave_door'],
    jointNames: ['microwave_hinge'],
  });
});
