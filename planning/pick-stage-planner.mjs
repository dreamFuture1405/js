const add = (left, right) => left.map((value, index) => value + right[index]);
const subtract = (left, right) => left.map((value, index) => value - right[index]);
const scale = (vector, factor) => vector.map((value) => value * factor);
const dot = (left, right) =>
  left.reduce((sum, value, index) => sum + value * right[index], 0);
const cross = (left, right) => [
  left[1] * right[2] - left[2] * right[1],
  left[2] * right[0] - left[0] * right[2],
  left[0] * right[1] - left[1] * right[0],
];
const magnitude = (vector) => Math.hypot(...vector);
const normalize = (vector, label) => {
  const length = magnitude(vector);
  if (length <= 1e-9) throw new Error(`${label} has zero length`);
  return scale(vector, 1 / length);
};

const transpose = (matrix) => [
  matrix[0], matrix[3], matrix[6],
  matrix[1], matrix[4], matrix[7],
  matrix[2], matrix[5], matrix[8],
];

const multiplyMatrix = (left, right) => Array.from({ length: 9 }, (_, index) => {
  const row = Math.floor(index / 3);
  const column = index % 3;
  return left[row * 3] * right[column]
    + left[row * 3 + 1] * right[3 + column]
    + left[row * 3 + 2] * right[6 + column];
});

const multiplyVector = (matrix, vector) => [
  matrix[0] * vector[0] + matrix[1] * vector[1] + matrix[2] * vector[2],
  matrix[3] * vector[0] + matrix[4] * vector[1] + matrix[5] * vector[2],
  matrix[6] * vector[0] + matrix[7] * vector[1] + matrix[8] * vector[2],
];

const quaternionMatrix = ([x, y, z, w]) => {
  const length = Math.hypot(x, y, z, w);
  if (length <= 1e-9) throw new Error('Hand quaternion has zero length');
  x /= length;
  y /= length;
  z /= length;
  w /= length;
  return [
    1 - 2 * (y * y + z * z),
    2 * (x * y - z * w),
    2 * (x * z + y * w),
    2 * (x * y + z * w),
    1 - 2 * (x * x + z * z),
    2 * (y * z - x * w),
    2 * (x * z - y * w),
    2 * (y * z + x * w),
    1 - 2 * (x * x + y * y),
  ];
};

const matrixQuaternion = (matrix) => {
  const trace = matrix[0] + matrix[4] + matrix[8];
  let x;
  let y;
  let z;
  let w;
  if (trace > 0) {
    const scaleValue = Math.sqrt(trace + 1) * 2;
    w = 0.25 * scaleValue;
    x = (matrix[7] - matrix[5]) / scaleValue;
    y = (matrix[2] - matrix[6]) / scaleValue;
    z = (matrix[3] - matrix[1]) / scaleValue;
  } else if (matrix[0] > matrix[4] && matrix[0] > matrix[8]) {
    const scaleValue = Math.sqrt(1 + matrix[0] - matrix[4] - matrix[8]) * 2;
    w = (matrix[7] - matrix[5]) / scaleValue;
    x = 0.25 * scaleValue;
    y = (matrix[1] + matrix[3]) / scaleValue;
    z = (matrix[2] + matrix[6]) / scaleValue;
  } else if (matrix[4] > matrix[8]) {
    const scaleValue = Math.sqrt(1 + matrix[4] - matrix[0] - matrix[8]) * 2;
    w = (matrix[2] - matrix[6]) / scaleValue;
    x = (matrix[1] + matrix[3]) / scaleValue;
    y = 0.25 * scaleValue;
    z = (matrix[5] + matrix[7]) / scaleValue;
  } else {
    const scaleValue = Math.sqrt(1 + matrix[8] - matrix[0] - matrix[4]) * 2;
    w = (matrix[3] - matrix[1]) / scaleValue;
    x = (matrix[2] + matrix[6]) / scaleValue;
    y = (matrix[5] + matrix[7]) / scaleValue;
    z = 0.25 * scaleValue;
  }
  const length = Math.hypot(x, y, z, w);
  return [x / length, y / length, z / length, w / length];
};

const basisMatrix = (closingAxis, approachAxis, label) => {
  const approach = normalize(approachAxis, `${label} approach`);
  const projectedClosing = subtract(
    closingAxis,
    scale(approach, dot(closingAxis, approach)),
  );
  const closing = normalize(projectedClosing, `${label} closing`);
  const third = normalize(cross(closing, approach), `${label} third axis`);
  return [
    closing[0], approach[0], third[0],
    closing[1], approach[1], third[1],
    closing[2], approach[2], third[2],
  ];
};

const midpoint = (left, right) => scale(add(left, right), 0.5);

const selectObjectGeom = (body) => body?.geoms
  ?.filter((geom) => Number(geom.contype) !== 0 && Number(geom.rbound) > 0)
  .sort((left, right) => Number(right.rbound) - Number(left.rbound))[0] ?? null;

const selectFingerPad = (body) => {
  const candidates = body?.geoms
    ?.filter((geom) => Number(geom.contype) !== 0 && Number(geom.type) === 6)
    .map((geom) => ({
      geom,
      volume: Number(geom.size?.[0])
        * Number(geom.size?.[1])
        * Number(geom.size?.[2]),
    }))
    .sort((left, right) => right.volume - left.volume);
  return candidates?.[0]?.geom ?? null;
};

const withoutStartDuplicate = (path, start, goal) => {
  const same = (left, right) =>
    left.length === right.length
    && left.every((value, index) => Math.abs(value - right[index]) <= 1e-12);
  const targets = Array.from(path ?? [], (joints) => [...joints]);
  if (targets.length > 0 && same(targets[0], start)) targets.shift();
  return targets.length > 0 ? targets : [[...goal]];
};

const failure = (reason, poseId, detail = {}) => ({
  ready: false,
  failure: {
    reason,
    poseId,
    ...detail,
  },
});

export async function planPickAndLiftStage({
  planner,
  geometry,
  objectBodyName,
  handBodyName,
  leftFingerBodyName,
  rightFingerBodyName,
  robotJointNames,
  startJoints,
  worldUp = [0, 0, 1],
  pregraspHeightMeters = 0.11,
  graspVerticalOffsetMeters = 0.004,
  microLiftMeters = 0.09,
  randomSeed = 1,
}) {
  const objectBody = geometry?.bodies?.[objectBodyName];
  const handBody = geometry?.bodies?.[handBodyName];
  const leftPad = selectFingerPad(geometry?.bodies?.[leftFingerBodyName]);
  const rightPad = selectFingerPad(geometry?.bodies?.[rightFingerBodyName]);
  const objectGeom = selectObjectGeom(objectBody);
  if (!objectBody || !handBody || !leftPad || !rightPad || !objectGeom) {
    return failure('GEOMETRY_MISSING', 'object_pregrasp');
  }

  const currentPadCenter = midpoint(leftPad.position, rightPad.position);
  const currentClosingAxis = normalize(
    subtract(rightPad.position, leftPad.position),
    'current closing axis',
  );
  const currentApproachAxis = normalize(
    subtract(currentPadCenter, handBody.position),
    'current approach axis',
  );
  const handRotation = quaternionMatrix(handBody.quaternion);
  const inverseHandRotation = transpose(handRotation);
  const localClosingAxis = multiplyVector(inverseHandRotation, currentClosingAxis);
  const localApproachAxis = multiplyVector(inverseHandRotation, currentApproachAxis);
  const localToolToPad = multiplyVector(
    inverseHandRotation,
    subtract(currentPadCenter, handBody.position),
  );

  const longestAxisIndex = objectGeom.size.indexOf(Math.max(...objectGeom.size));
  const longAxis = normalize(
    objectGeom.axes[longestAxisIndex],
    'object long axis',
  );
  const normalizedUp = normalize(worldUp, 'world up');
  const desiredApproachAxis = scale(normalizedUp, -1);
  let desiredClosingAxis = cross(normalizedUp, longAxis);
  if (magnitude(desiredClosingAxis) <= 1e-6) {
    desiredClosingAxis = cross(
      subtract(currentPadCenter, objectGeom.position),
      longAxis,
    );
  }
  if (magnitude(desiredClosingAxis) <= 1e-6) {
    desiredClosingAxis = currentClosingAxis;
  }
  if (dot(desiredClosingAxis, currentClosingAxis) < 0) {
    desiredClosingAxis = scale(desiredClosingAxis, -1);
  }

  const localBasis = basisMatrix(
    localClosingAxis,
    localApproachAxis,
    'local gripper',
  );
  const desiredBasis = basisMatrix(
    desiredClosingAxis,
    desiredApproachAxis,
    'desired gripper',
  );
  const targetRotation = multiplyMatrix(desiredBasis, transpose(localBasis));
  const targetQuaternion = matrixQuaternion(targetRotation);
  const graspPadPosition = add(
    objectGeom.position,
    scale(normalizedUp, graspVerticalOffsetMeters),
  );
  const pregraspPadPosition = add(
    graspPadPosition,
    scale(normalizedUp, pregraspHeightMeters),
  );
  const liftPadPosition = add(
    graspPadPosition,
    scale(normalizedUp, microLiftMeters),
  );
  const toolPositionForPad = (padPosition) => subtract(
    padPosition,
    multiplyVector(targetRotation, localToolToPad),
  );
  const poseTargets = [
    {
      id: 'object_pregrasp',
      padPosition: pregraspPadPosition,
      toolPosition: toolPositionForPad(pregraspPadPosition),
      allowedBodyNames: [],
      actionAfter: null,
    },
    {
      id: 'object_grasp',
      padPosition: graspPadPosition,
      toolPosition: toolPositionForPad(graspPadPosition),
      allowedBodyNames: [objectBodyName],
      actionAfter: 'close_gripper',
    },
    {
      id: 'object_micro_lift',
      padPosition: liftPadPosition,
      toolPosition: toolPositionForPad(liftPadPosition),
      allowedBodyNames: [objectBodyName],
      actionAfter: 'verify_temporal_grasp',
      executionScope: {
        allowGripperStateChange: true,
        objects: [],
      },
    },
  ];

  const chunks = [];
  const solvedPoses = [];
  let seed = [...startJoints];
  for (let index = 0; index < poseTargets.length; index += 1) {
    const pose = poseTargets[index];
    const ik = await planner.solveIk({
      bodyName: handBodyName,
      jointNames: robotJointNames,
      targetPosition: pose.toolPosition,
      targetQuaternion,
      seed,
      maximumIterations: 320,
      positionTolerance: 0.004,
      orientationTolerance: 0.035,
    });
    if (!ik.success) {
      return failure('IK_FAILED', pose.id, {
        positionError: ik.position_error,
        orientationError: ik.orientation_error,
        iterations: ik.iterations,
      });
    }
    const path = await planner.planPath({
      jointNames: robotJointNames,
      start: seed,
      goal: ik.joints,
      maximumJointStep: 0.035,
      allowedBodyNames: pose.allowedBodyNames,
      rrtStep: 0.16,
      maximumIterations: index === 0 ? 1400 : 500,
      goalBias: 0.24,
      randomSeed: Number(randomSeed) + index,
    });
    if (!path.success) {
      const [startCheck, goalCheck] = typeof planner.checkConfiguration === 'function'
        ? await Promise.all([
          planner.checkConfiguration({
            jointNames: robotJointNames,
            joints: seed,
            allowedBodyNames: pose.allowedBodyNames,
          }),
          planner.checkConfiguration({
            jointNames: robotJointNames,
            joints: ik.joints,
            allowedBodyNames: pose.allowedBodyNames,
          }),
        ])
        : [{ collisions: [] }, { collisions: [] }];
      return failure('PATH_FAILED', pose.id, {
        method: path.method,
        plannerReason: path.reason,
        iterations: path.iterations,
        targetPadPosition: pose.padPosition,
        targetToolPosition: pose.toolPosition,
        targetQuaternion,
        goalJoints: ik.joints,
        startCollisions: startCheck.collisions,
        goalCollisions: goalCheck.collisions,
      });
    }
    chunks.push({
      id: pose.id,
      jointTargets: withoutStartDuplicate(path.path, seed, ik.joints),
      actionAfter: pose.actionAfter,
      executionScope: pose.executionScope ?? null,
      planningMethod: path.method,
    });
    solvedPoses.push({
      ...pose,
      quaternion: targetQuaternion,
      joints: [...ik.joints],
      ik,
    });
    seed = [...ik.joints];
  }

  return {
    ready: true,
    candidateId: `pick-top:${objectGeom.id}:${randomSeed}`,
    chunks,
    poses: solvedPoses,
    targets: {
      objectGeomId: objectGeom.id,
      graspPadPosition,
      pregraspPadPosition,
      liftPadPosition,
      targetQuaternion,
      desiredClosingAxis: normalize(desiredClosingAxis, 'desired closing axis'),
      desiredApproachAxis,
    },
  };
}
