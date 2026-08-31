import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import path from 'node:path';
import {
  exportModelAssets,
  safeWorkingRelativePath,
} from '../model/model-exporter.mjs';

test('maps MuJoCo working paths into safe relative cache paths', () => {
  assert.equal(
    safeWorkingRelativePath('/working/scenes/task.xml'),
    path.join('scenes', 'task.xml'),
  );
  assert.throws(
    () => safeWorkingRelativePath('/working/../secret.txt'),
    /unsafe/i,
  );
  assert.throws(
    () => safeWorkingRelativePath('/etc/passwd'),
    /working/i,
  );
});

test('exports model files in bounded chunks and preserves directory layout', async (context) => {
  const outputRoot = path.resolve(
    'artifacts',
    `model-export-test-${process.pid}-${Date.now()}`,
  );
  context.after(async () => {
    await fs.rm(outputRoot, { recursive: true, force: true });
  });
  const files = new Map([
    ['/working/scenes/task.xml', Buffer.from('<mujoco/>')],
    ['/working/assets/mesh.obj', Buffer.from('123456789')],
  ]);
  const result = await exportModelAssets({
    outputRoot,
    modelId: 'model:test',
    taskXmlPath: '/working/scenes/task.xml',
    manifest: [...files].map(([filePath, content]) => ({
      path: filePath,
      size: content.length,
    })),
    chunkSize: 4,
    readChunk: async (filePath, offset, length) =>
      files.get(filePath).subarray(offset, offset + length),
  });

  assert.equal(
    await fs.readFile(path.join(outputRoot, 'working', 'assets', 'mesh.obj'), 'utf8'),
    '123456789',
  );
  assert.equal(result.fileCount, 2);
  assert.equal(result.byteCount, 18);
  assert.equal(
    result.taskXmlPath,
    path.join(outputRoot, 'working', 'scenes', 'task.xml'),
  );
});
