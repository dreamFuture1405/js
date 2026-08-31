export function installAxisModelExportBridge() {
  window.__axisModelExportBridge?.destroy?.();
  const demo = window.__axisAutomationDemo;
  const fs = demo?.mujoco?.FS;
  if (!fs || typeof fs.read !== 'function') {
    throw new Error('MuJoCo virtual filesystem is unavailable');
  }
  let destroyed = false;
  const assertWorkingPath = (filePath) => {
    const normalized = String(filePath ?? '').replaceAll('\\', '/');
    if (
      !normalized.startsWith('/working/')
      || normalized.includes('/../')
      || normalized.endsWith('/..')
    ) {
      throw new Error(`Unsafe MuJoCo working path: ${normalized}`);
    }
    return normalized;
  };
  const manifest = () => {
    const files = [];
    const walk = (directory) => {
      for (const name of fs.readdir(directory)) {
        if (name === '.' || name === '..') continue;
        const filePath = `${directory}/${name}`;
        const stat = fs.stat(filePath);
        if (fs.isDir(stat.mode)) {
          walk(filePath);
        } else {
          files.push({
            path: filePath,
            size: Number(stat.size),
          });
        }
      }
    };
    walk('/working');
    return files.sort((left, right) => left.path.localeCompare(right.path));
  };
  const bytesToBase64 = (bytes) => {
    let binary = '';
    for (let offset = 0; offset < bytes.length; offset += 0x8000) {
      binary += String.fromCharCode(...bytes.subarray(offset, offset + 0x8000));
    }
    return btoa(binary);
  };
  const readChunk = (filePath, offset, length) => {
    if (destroyed) throw new Error('Model export bridge has been destroyed');
    const safePath = assertWorkingPath(filePath);
    const stat = fs.stat(safePath);
    const start = Math.max(0, Math.min(Number(stat.size), Number(offset) || 0));
    const count = Math.max(
      0,
      Math.min(Number(stat.size) - start, Number(length) || 0),
    );
    const bytes = new Uint8Array(count);
    const stream = fs.open(safePath, 'r');
    try {
      const read = fs.read(stream, bytes, 0, count, start);
      return {
        path: safePath,
        offset: start,
        byteLength: Number(read),
        base64: bytesToBase64(bytes.subarray(0, Number(read))),
      };
    } finally {
      fs.close(stream);
    }
  };
  const api = {
    getInfo() {
      const taskId = window.__axisAutomationSession?.task?.id ?? 'unknown';
      const taskXmlPath = `/working/scenes/task_${taskId}.xml`;
      return {
        installed: !destroyed,
        taskId,
        taskXmlPath,
        modelXmlLength: String(demo.modelXml ?? '').length,
        modelId: [
          taskId,
          demo.model.nq,
          demo.model.nv,
          demo.model.nu,
          demo.model.nbody,
          demo.model.ngeom,
        ].join(':'),
      };
    },
    manifest,
    readChunk,
    destroy() {
      destroyed = true;
      delete window.__axisModelExportBridge;
      return { installed: false };
    },
  };
  window.__axisModelExportBridge = api;
  return api.getInfo();
}
