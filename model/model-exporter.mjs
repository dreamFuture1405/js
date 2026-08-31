import fs from 'node:fs/promises';
import path from 'node:path';

export function safeWorkingRelativePath(filePath) {
  const normalized = String(filePath ?? '').replaceAll('\\', '/');
  if (!normalized.startsWith('/working/')) {
    throw new Error(`Model asset is outside /working: ${normalized}`);
  }
  const relative = normalized.slice('/working/'.length);
  const segments = relative.split('/');
  if (
    !relative
    || segments.some((segment) => !segment || segment === '.' || segment === '..')
  ) {
    throw new Error(`Unsafe model asset path: ${normalized}`);
  }
  return path.join(...segments);
}

const ensureInside = (root, target) => {
  const relative = path.relative(root, target);
  if (relative.startsWith('..') || path.isAbsolute(relative)) {
    throw new Error(`Export target escaped model cache: ${target}`);
  }
};

export async function exportModelAssets({
  outputRoot,
  modelId,
  taskXmlPath,
  manifest,
  readChunk,
  chunkSize = 512 * 1024,
  onProgress = () => {},
}) {
  const absoluteRoot = path.resolve(outputRoot);
  const workingRoot = path.join(absoluteRoot, 'working');
  await fs.mkdir(workingRoot, { recursive: true });
  let byteCount = 0;
  let completedFiles = 0;
  for (const entry of manifest) {
    const relativePath = safeWorkingRelativePath(entry.path);
    const outputPath = path.resolve(workingRoot, relativePath);
    ensureInside(workingRoot, outputPath);
    await fs.mkdir(path.dirname(outputPath), { recursive: true });
    const handle = await fs.open(outputPath, 'w');
    try {
      for (let offset = 0; offset < Number(entry.size); offset += chunkSize) {
        const expected = Math.min(chunkSize, Number(entry.size) - offset);
        const chunk = Buffer.from(await readChunk(entry.path, offset, expected));
        if (chunk.length !== expected) {
          throw new Error(
            `Short model chunk for ${entry.path} at ${offset}: `
            + `expected ${expected}, received ${chunk.length}`,
          );
        }
        await handle.write(chunk, 0, chunk.length, offset);
        byteCount += chunk.length;
      }
    } finally {
      await handle.close();
    }
    completedFiles += 1;
    onProgress({
      completedFiles,
      fileCount: manifest.length,
      byteCount,
      currentPath: entry.path,
    });
  }
  const localTaskXmlPath = path.resolve(
    workingRoot,
    safeWorkingRelativePath(taskXmlPath),
  );
  ensureInside(workingRoot, localTaskXmlPath);
  const metadata = {
    version: 1,
    modelId: String(modelId),
    taskXmlPath: localTaskXmlPath,
    fileCount: completedFiles,
    byteCount,
    exportedAt: new Date().toISOString(),
  };
  await fs.writeFile(
    path.join(absoluteRoot, 'model-cache.json'),
    `${JSON.stringify(metadata, null, 2)}\n`,
  );
  return metadata;
}
