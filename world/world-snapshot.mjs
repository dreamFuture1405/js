const finiteNumber = (value, fallback = 0) => {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
};

const finiteArray = (values, expectedLength = null) => {
  const normalized = Array.from(values ?? [], (value) => finiteNumber(value));
  if (expectedLength != null && normalized.length !== expectedLength) {
    throw new Error(`Expected ${expectedLength} numeric values, received ${normalized.length}`);
  }
  return normalized;
};

const normalizePose = (source = {}) => ({
  position: finiteArray(source.position, 3),
  quaternion: finiteArray(source.quaternion ?? [0, 0, 0, 1], 4),
});

const deepFreeze = (value) => {
  if (!value || typeof value !== 'object' || Object.isFrozen(value)) return value;
  for (const child of Object.values(value)) deepFreeze(child);
  return Object.freeze(value);
};

const stableStringify = (value) => {
  if (Array.isArray(value)) return `[${value.map(stableStringify).join(',')}]`;
  if (value && typeof value === 'object') {
    return `{${Object.keys(value)
      .sort()
      .map((key) => `${JSON.stringify(key)}:${stableStringify(value[key])}`)
      .join(',')}}`;
  }
  return JSON.stringify(value);
};

const fnv1a = (text) => {
  let hash = 0x811c9dc5;
  for (let index = 0; index < text.length; index += 1) {
    hash ^= text.charCodeAt(index);
    hash = Math.imul(hash, 0x01000193);
  }
  return (hash >>> 0).toString(16).padStart(8, '0');
};

export function createWorldSnapshot(observation, {
  sequence,
  capturedAt = Date.now(),
} = {}) {
  if (!observation || typeof observation !== 'object') {
    throw new TypeError('World observation is required');
  }
  if (!Number.isInteger(sequence) || sequence < 0) {
    throw new TypeError('Snapshot sequence must be a non-negative integer');
  }
  const modelId = String(observation.modelId ?? 'unknown-model');
  const simulationTime = finiteNumber(observation.simulationTime);
  const objects = Object.fromEntries(
    Object.entries(observation.objects ?? {})
      .sort(([left], [right]) => left.localeCompare(right))
      .map(([name, object]) => [
        name,
        {
          ...normalizePose(object),
          velocity: finiteArray(object?.velocity ?? []),
          bounds: object?.bounds
            ? {
              min: finiteArray(object.bounds.min, 3),
              max: finiteArray(object.bounds.max, 3),
            }
            : null,
          supportContacts: Array.from(object?.supportContacts ?? [], String).sort(),
        },
      ]),
  );
  const articulatedJoints = Object.fromEntries(
    Object.entries(observation.articulatedJoints ?? {})
      .sort(([left], [right]) => left.localeCompare(right))
      .map(([name, value]) => [name, finiteNumber(value)]),
  );
  const snapshot = {
    version: 1,
    snapshotId: [
      modelId,
      Math.round(finiteNumber(capturedAt)),
      sequence,
      Math.round(simulationTime * 1_000_000),
    ].join(':'),
    sequence,
    capturedAt: finiteNumber(capturedAt),
    simulationTime,
    modelId,
    checkerStage: String(observation.checkerStage ?? 'unknown'),
    checker: observation.checker ?? null,
    robot: {
      qpos: finiteArray(observation.robot?.qpos ?? []),
      qvel: finiteArray(observation.robot?.qvel ?? []),
      endEffectorPose: normalizePose(observation.robot?.endEffectorPose),
      gripperState: String(observation.robot?.gripperState ?? 'unknown'),
    },
    simulation: {
      qpos: finiteArray(observation.simulation?.qpos ?? []),
      qvel: finiteArray(observation.simulation?.qvel ?? []),
      ctrl: finiteArray(observation.simulation?.ctrl ?? []),
    },
    objects,
    articulatedJoints,
    contacts: {
      count: Math.max(0, Math.trunc(finiteNumber(observation.contacts?.count))),
      pairs: Array.from(observation.contacts?.pairs ?? [], (pair) => ({
        first: String(pair.first ?? ''),
        second: String(pair.second ?? ''),
        distance: finiteNumber(pair.distance),
      })),
      accessor: String(observation.contacts?.accessor ?? 'unknown'),
    },
  };
  return deepFreeze(snapshot);
}

export function snapshotStateSignature(snapshot) {
  const state = {
    modelId: snapshot.modelId,
    checkerStage: snapshot.checkerStage,
    robot: snapshot.robot,
    simulation: snapshot.simulation,
    objects: snapshot.objects,
    articulatedJoints: snapshot.articulatedJoints,
    contacts: snapshot.contacts,
  };
  return fnv1a(stableStringify(state));
}

export { deepFreeze };
