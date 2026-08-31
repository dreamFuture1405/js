import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import {
  NativePlannerClient,
  defaultPlannerProcess,
} from '../planner/native-planner-client.mjs';

test('loads and mirrors a model through the native planner process', async (context) => {
  const processConfig = defaultPlannerProcess({
    projectRoot: path.resolve('.'),
  });
  await fs.access(processConfig.command);
  const temporaryRoot = await fs.mkdtemp(path.join(os.tmpdir(), 'axis-planner-'));
  context.after(async () => {
    await fs.rm(temporaryRoot, { recursive: true, force: true });
  });
  const modelPath = path.join(temporaryRoot, 'scene.xml');
  await fs.writeFile(modelPath, `
<mujoco model="client_test">
  <worldbody>
    <body name="item">
      <freejoint/>
      <geom type="sphere" size=".1"/>
    </body>
  </worldbody>
</mujoco>`);
  const client = new NativePlannerClient(processConfig);
  context.after(() => client.close());

  await client.start();
  const summary = await client.loadModel(modelPath);
  const mirrored = await client.mirrorSnapshot({
    simulation: {
      qpos: [0, 0, 0, 1, 0, 0, 0],
      qvel: [0, 0, 0, 0, 0, 0],
      ctrl: [],
    },
  }, ['item']);

  assert.equal(summary.nq, 7);
  assert.deepEqual(mirrored.bodies.item.position, [0, 0, 0]);
  assert.equal(mirrored.summary.model_name.startsWith('client_test'), true);
});

test('rejects requests after the planner process exits', async () => {
  const client = new NativePlannerClient({
    command: process.execPath,
    args: ['-e', 'process.exit(0)'],
    cwd: path.resolve('.'),
    requestTimeoutMilliseconds: 1000,
  });

  await assert.rejects(client.start(), /exit|closed|timeout/i);
  await assert.rejects(client.request('health'), /running/i);
});
