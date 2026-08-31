import test from 'node:test';
import assert from 'node:assert/strict';
import {
  createWorldSnapshot,
  snapshotStateSignature,
} from '../world/world-snapshot.mjs';

const observation = () => ({
  modelId: 'model-4865',
  simulationTime: 12.5,
  checkerStage: 'pick',
  robot: {
    qpos: [0.1, -0.2, 0.3],
    qvel: [0, 0, 0],
    endEffectorPose: {
      position: [0.4, 0.6, -0.2],
      quaternion: [0, 0, 0, 1],
    },
    gripperState: 'open',
  },
  objects: {
    glue_tube_0: {
      position: [0.5, 0.3, -0.1],
      quaternion: [0, 0, 0, 1],
      velocity: [0, 0, 0, 0, 0, 0],
    },
  },
  articulatedJoints: {
    microwave_door: 1.07,
  },
  contacts: {
    count: 2,
    pairs: [],
    accessor: 'count_only',
  },
});

test('creates an immutable, serializable world snapshot with a unique revision id', () => {
  const first = createWorldSnapshot(observation(), {
    sequence: 4,
    capturedAt: 1000,
  });
  const second = createWorldSnapshot(observation(), {
    sequence: 5,
    capturedAt: 1010,
  });

  assert.equal(first.snapshotId, 'model-4865:1000:4:12500000');
  assert.notEqual(first.snapshotId, second.snapshotId);
  assert.equal(Object.isFrozen(first), true);
  assert.equal(Object.isFrozen(first.robot), true);
  assert.equal(Object.isFrozen(first.objects.glue_tube_0), true);
  assert.throws(() => {
    first.robot.qpos[0] = 99;
  });
  assert.doesNotThrow(() => JSON.stringify(first));
});

test('state signature changes when robot, object, articulation, or checker state changes', () => {
  const base = createWorldSnapshot(observation(), { sequence: 1, capturedAt: 1000 });
  const robotMoved = observation();
  robotMoved.robot.qpos[1] += 0.1;
  const objectMoved = observation();
  objectMoved.objects.glue_tube_0.position[0] += 0.02;
  const doorMoved = observation();
  doorMoved.articulatedJoints.microwave_door += 0.05;
  const checkerMoved = observation();
  checkerMoved.checkerStage = 'place';

  const signatures = [
    base,
    createWorldSnapshot(robotMoved, { sequence: 2, capturedAt: 1001 }),
    createWorldSnapshot(objectMoved, { sequence: 3, capturedAt: 1002 }),
    createWorldSnapshot(doorMoved, { sequence: 4, capturedAt: 1003 }),
    createWorldSnapshot(checkerMoved, { sequence: 5, capturedAt: 1004 }),
  ].map(snapshotStateSignature);

  assert.equal(new Set(signatures).size, signatures.length);
});
