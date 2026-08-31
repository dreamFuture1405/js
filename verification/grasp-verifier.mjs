const subtract = (left, right) => left.map((value, index) => value - right[index]);
const distance = (left, right) => Math.hypot(...subtract(left, right));

const normalizeQuaternion = (quaternion) => {
  const length = Math.hypot(...quaternion);
  if (length <= 1e-9) throw new Error('Quaternion has zero length');
  return quaternion.map((value) => value / length);
};

const inverseQuaternion = ([x, y, z, w]) => [-x, -y, -z, w];

const multiplyQuaternion = (left, right) => {
  const [ax, ay, az, aw] = left;
  const [bx, by, bz, bw] = right;
  return [
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz,
  ];
};

const rotateVector = (quaternion, vector) => {
  const rotated = multiplyQuaternion(
    multiplyQuaternion(quaternion, [...vector, 0]),
    inverseQuaternion(quaternion),
  );
  return rotated.slice(0, 3);
};

const relativeTransform = (hand, object) => {
  const handQuaternion = normalizeQuaternion(hand.quaternion);
  const inverseHand = inverseQuaternion(handQuaternion);
  return {
    position: rotateVector(
      inverseHand,
      subtract(object.position, hand.position),
    ),
    quaternion: normalizeQuaternion(multiplyQuaternion(
      inverseHand,
      normalizeQuaternion(object.quaternion),
    )),
  };
};

const quaternionAngle = (left, right) => {
  const dot = Math.abs(left.reduce(
    (sum, value, index) => sum + value * right[index],
    0,
  ));
  return 2 * Math.acos(Math.max(-1, Math.min(1, dot)));
};

const pairIncludes = (contact, first, second) => {
  const names = new Set([contact.body1_name, contact.body2_name]);
  return names.has(first) && names.has(second);
};

export function verifyTemporalGrasp({
  samples,
  initialObjectPosition,
  objectBodyName,
  handBodyName,
  leftFingerBodyName,
  rightFingerBodyName,
  supportBodyNames = [],
  minimumWindowMilliseconds = 400,
  minimumLiftMeters = 0.015,
  maximumRelativeTranslationDriftMeters = 0.008,
  maximumRelativeRotationDriftRadians = 0.12,
  minimumDualContactRatio = 0.5,
}) {
  if (!Array.isArray(samples) || samples.length < 2) {
    throw new Error('Temporal grasp verification requires at least two samples');
  }
  const normalizedSamples = samples.map((sample, index) => {
    const hand = sample?.bodies?.[handBodyName];
    const object = sample?.bodies?.[objectBodyName];
    if (!hand || !object) {
      throw new Error(`Verification sample ${index} is missing hand or object pose`);
    }
    return {
      ...sample,
      hand,
      object,
      relative: relativeTransform(hand, object),
    };
  });
  const first = normalizedSamples[0];
  const last = normalizedSamples.at(-1);
  const windowMilliseconds = Number(last.at) - Number(first.at);
  const liftMeters = Number(last.object.position[2])
    - Number(initialObjectPosition[2]);
  const maximumTranslationDrift = Math.max(
    ...normalizedSamples.map((sample) =>
      distance(first.relative.position, sample.relative.position)),
  );
  const maximumRotationDrift = Math.max(
    ...normalizedSamples.map((sample) =>
      quaternionAngle(first.relative.quaternion, sample.relative.quaternion)),
  );
  const supportSet = new Set(supportBodyNames);
  const supportContactRemains = normalizedSamples.slice(-2).some((sample) =>
    sample.contacts.some((contact) => {
      const names = new Set([contact.body1_name, contact.body2_name]);
      return names.has(objectBodyName)
        && [...supportSet].some((support) => names.has(support));
    }));
  const dualContactCount = normalizedSamples.filter((sample) => {
    const left = sample.contacts.some((contact) =>
      pairIncludes(contact, objectBodyName, leftFingerBodyName));
    const right = sample.contacts.some((contact) =>
      pairIncludes(contact, objectBodyName, rightFingerBodyName));
    return left && right;
  }).length;
  const dualContactRatio = dualContactCount / normalizedSamples.length;
  const reasons = [];
  if (windowMilliseconds < minimumWindowMilliseconds) {
    reasons.push('VERIFICATION_WINDOW_TOO_SHORT');
  }
  if (liftMeters < minimumLiftMeters) {
    reasons.push('OBJECT_NOT_LIFTED');
  }
  if (supportContactRemains) {
    reasons.push('SUPPORT_CONTACT_REMAINS');
  }
  if (
    maximumTranslationDrift > maximumRelativeTranslationDriftMeters
    || maximumRotationDrift > maximumRelativeRotationDriftRadians
  ) {
    reasons.push('RELATIVE_TRANSFORM_DRIFT');
  }
  if (dualContactRatio < minimumDualContactRatio) {
    reasons.push('DUAL_FINGER_ENGAGEMENT_MISSING');
  }
  return {
    ok: reasons.length === 0,
    reasons,
    windowMilliseconds,
    liftMeters,
    maximumRelativeTranslationDriftMeters: maximumTranslationDrift,
    maximumRelativeRotationDriftRadians: maximumRotationDrift,
    dualContactRatio,
    supportContactRemains,
    measuredHandObjectTransform: last.relative,
  };
}
