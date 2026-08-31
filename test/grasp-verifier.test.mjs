import test from 'node:test';
import assert from 'node:assert/strict';
import { verifyTemporalGrasp } from '../verification/grasp-verifier.mjs';

const contact = (body1Name, body2Name) => ({
  body1_name: body1Name,
  body2_name: body2Name,
});

const sample = ({
  at,
  handZ,
  objectZ,
  objectX = 0.5,
  support = false,
  left = true,
  right = true,
}) => ({
  at,
  bodies: {
    hand: {
      position: [0.5, 0, handZ],
      quaternion: [0, 0, 0, 1],
    },
    item: {
      position: [objectX, 0, objectZ],
      quaternion: [0, 0, 0, 1],
    },
  },
  contacts: [
    ...(support ? [contact('item', 'table')] : []),
    ...(left ? [contact('item', 'left')] : []),
    ...(right ? [contact('item', 'right')] : []),
  ],
});

test('accepts a lifted object with stable hand-object transform over time', () => {
  const samples = [
    sample({ at: 0, handZ: 0.60, objectZ: 0.50 }),
    sample({ at: 140, handZ: 0.61, objectZ: 0.51 }),
    sample({ at: 280, handZ: 0.62, objectZ: 0.52 }),
    sample({ at: 430, handZ: 0.63, objectZ: 0.53 }),
  ];

  const result = verifyTemporalGrasp({
    samples,
    initialObjectPosition: [0.5, 0, 0.50],
    objectBodyName: 'item',
    handBodyName: 'hand',
    leftFingerBodyName: 'left',
    rightFingerBodyName: 'right',
    supportBodyNames: ['table'],
  });

  assert.equal(result.ok, true);
  assert.deepEqual(result.reasons, []);
  assert.ok(result.liftMeters >= 0.029);
  assert.ok(result.maximumRelativeTranslationDriftMeters < 1e-6);
});

test('rejects a sliding object even when finger contacts still exist', () => {
  const samples = [
    sample({ at: 0, handZ: 0.60, objectZ: 0.50 }),
    sample({ at: 150, handZ: 0.61, objectZ: 0.51, objectX: 0.507 }),
    sample({ at: 300, handZ: 0.62, objectZ: 0.52, objectX: 0.516 }),
    sample({ at: 450, handZ: 0.63, objectZ: 0.53, objectX: 0.524 }),
  ];

  const result = verifyTemporalGrasp({
    samples,
    initialObjectPosition: [0.5, 0, 0.50],
    objectBodyName: 'item',
    handBodyName: 'hand',
    leftFingerBodyName: 'left',
    rightFingerBodyName: 'right',
    supportBodyNames: ['table'],
  });

  assert.equal(result.ok, false);
  assert.ok(result.reasons.includes('RELATIVE_TRANSFORM_DRIFT'));
});

test('rejects contact with the support or an observation window that is too short', () => {
  const samples = [
    sample({ at: 0, handZ: 0.60, objectZ: 0.50, support: true }),
    sample({ at: 100, handZ: 0.63, objectZ: 0.53, support: true }),
  ];

  const result = verifyTemporalGrasp({
    samples,
    initialObjectPosition: [0.5, 0, 0.50],
    objectBodyName: 'item',
    handBodyName: 'hand',
    leftFingerBodyName: 'left',
    rightFingerBodyName: 'right',
    supportBodyNames: ['table'],
  });

  assert.equal(result.ok, false);
  assert.ok(result.reasons.includes('SUPPORT_CONTACT_REMAINS'));
  assert.ok(result.reasons.includes('VERIFICATION_WINDOW_TOO_SHORT'));
});

test('rejects one-sided finger engagement', () => {
  const samples = [
    sample({ at: 0, handZ: 0.60, objectZ: 0.50, right: false }),
    sample({ at: 150, handZ: 0.61, objectZ: 0.51, right: false }),
    sample({ at: 300, handZ: 0.62, objectZ: 0.52, right: false }),
    sample({ at: 450, handZ: 0.63, objectZ: 0.53, right: false }),
  ];

  const result = verifyTemporalGrasp({
    samples,
    initialObjectPosition: [0.5, 0, 0.50],
    objectBodyName: 'item',
    handBodyName: 'hand',
    leftFingerBodyName: 'left',
    rightFingerBodyName: 'right',
    supportBodyNames: ['table'],
  });

  assert.equal(result.ok, false);
  assert.ok(result.reasons.includes('DUAL_FINGER_ENGAGEMENT_MISSING'));
});
