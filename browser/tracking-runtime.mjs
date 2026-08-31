export function installAxisAutonomyTracking(initialState, helpers) {
  window.__axisAutonomyV3Tracking?.destroy?.();
  window.__axisGuidanceV2?.destroy?.();
  window.__axisHumanAssist?.destroy?.();
  const demo = window.__axisAutomationDemo;
  if (!demo?.renderer?.domElement) throw new Error('Axis renderer is not ready');
  const buildViewModel = helpers?.buildTrackingViewModel;
  if (typeof buildViewModel !== 'function') {
    throw new Error('Tracking view-model helper is required');
  }

  let state = structuredClone(initialState);
  let destroyed = false;
  let abortRequested = false;
  const root = document.createElement('section');
  root.id = 'axis-autonomy-v3-tracking';
  root.setAttribute('aria-label', 'Axis autonomy progress');
  Object.assign(root.style, {
    position: 'fixed',
    zIndex: '2147483647',
    width: '390px',
    maxHeight: 'calc(100vh - 28px)',
    overflow: 'auto',
    padding: '15px',
    borderRadius: '14px',
    color: '#e2e8f0',
    background: 'linear-gradient(180deg, rgba(2,6,23,.97), rgba(3,7,18,.94))',
    border: '1px solid rgba(56,189,248,.35)',
    boxShadow: '0 20px 60px rgba(0,0,0,.48)',
    font: '12px/1.45 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace',
    pointerEvents: 'auto',
    backdropFilter: 'blur(12px)',
  });
  document.body.append(root);

  const escapeHtml = (value) => String(value ?? '')
    .replaceAll('&', '&amp;')
    .replaceAll('<', '&lt;')
    .replaceAll('>', '&gt;')
    .replaceAll('"', '&quot;')
    .replaceAll("'", '&#039;');
  const syncPosition = () => {
    const rect = demo.renderer.domElement.getBoundingClientRect();
    Object.assign(root.style, {
      top: `${Math.max(14, rect.top + 14)}px`,
      right: `${Math.max(14, window.innerWidth - rect.right + 14)}px`,
    });
  };
  const render = () => {
    if (destroyed) return;
    syncPosition();
    const view = buildViewModel(state);
    const stages = view.stages.map((stage) => {
      const active = stage.status === 'active';
      const complete = stage.status === 'completed';
      const color = complete ? '#4ade80' : active ? '#38bdf8' : '#64748b';
      const border = active ? 'rgba(56,189,248,.65)' : 'rgba(71,85,105,.65)';
      const status = complete ? 'XONG' : active ? view.phaseLabel : 'CHỜ';
      return `
        <div style="margin-top:7px;padding:9px 10px;border:1px solid ${border};border-radius:9px;background:${active ? 'rgba(8,47,73,.32)' : 'rgba(15,23,42,.54)'}">
          <div style="display:flex;align-items:center;gap:8px;color:${color};font-weight:900">
            <span style="display:grid;place-items:center;width:22px;height:22px;border:1px solid currentColor;border-radius:50%">${escapeHtml(stage.icon)}</span>
            <span>${escapeHtml(stage.label)}</span>
            <span style="margin-left:auto;font-size:9px;letter-spacing:.08em">${escapeHtml(status)}</span>
          </div>
        </div>`;
    }).join('');
    const timeline = view.timeline.length > 0
      ? view.timeline.map((event) => `
        <div style="display:grid;grid-template-columns:7px 1fr;gap:8px;margin-top:6px;color:#94a3b8">
          <span style="width:7px;height:7px;margin-top:5px;border-radius:50%;background:#38bdf8"></span>
          <span>${escapeHtml(event.text)}</span>
        </div>`).join('')
      : '<div style="margin-top:6px;color:#64748b">Chưa có event.</div>';
    const alert = view.alert
      ? `
        <div style="margin-top:10px;padding:10px;border-radius:9px;border:1px solid rgba(251,146,60,.6);background:rgba(124,45,18,.28)">
          <div style="color:#fdba74;font-weight:900">${escapeHtml(view.alert.reason)}</div>
          <div style="margin-top:3px;color:#fed7aa">${escapeHtml(view.alert.message)}</div>
        </div>`
      : '';
    root.innerHTML = `
      <div style="display:flex;align-items:flex-start;gap:10px">
        <div style="min-width:0;flex:1">
          <div style="font-size:10px;letter-spacing:.15em;color:#38bdf8;font-weight:900">AXIS AUTONOMY V3 · ${escapeHtml(view.modeLabel)}</div>
          <div style="margin-top:5px;font-size:19px;color:#f8fafc;font-weight:950">${escapeHtml(view.header)}</div>
          <div style="margin-top:3px;color:#64748b;white-space:nowrap;overflow:hidden;text-overflow:ellipsis">${escapeHtml(view.goal)}</div>
        </div>
        <button data-abort type="button" style="border:1px solid rgba(248,113,113,.75);border-radius:8px;padding:7px 9px;color:#fecaca;background:rgba(127,29,29,.45);font:900 10px ui-monospace,monospace;cursor:pointer">ABORT</button>
      </div>

      <div style="margin-top:11px;padding:10px;border-radius:9px;border:1px solid ${view.phaseColor};background:rgba(15,23,42,.76)">
        <div style="display:flex;gap:10px;align-items:center">
          <span style="color:${view.phaseColor};font-weight:950">${escapeHtml(view.phaseLabel)}</span>
          <span style="margin-left:auto;color:#cbd5e1">${escapeHtml(view.progressText)}</span>
        </div>
        <div style="height:5px;margin-top:8px;border-radius:99px;background:#172033;overflow:hidden">
          <div style="width:${view.progressPercent}%;height:100%;background:${view.phaseColor};transition:width .18s ease"></div>
        </div>
        <div style="margin-top:7px;color:#f8fafc;font-weight:800">${escapeHtml(view.message)}</div>
      </div>

      <div style="margin-top:10px;font-size:10px;letter-spacing:.12em;color:#64748b">TIẾN ĐỘ TOÀN TASK</div>
      ${stages}
      ${alert}

      <div style="display:grid;grid-template-columns:1fr 1fr;gap:7px;margin-top:10px">
        <div style="padding:8px;border-radius:8px;background:rgba(15,23,42,.7)"><div style="color:#64748b">SNAPSHOT</div><div style="margin-top:2px;color:#bae6fd">${escapeHtml(view.snapshotId)}</div></div>
        <div style="padding:8px;border-radius:8px;background:rgba(15,23,42,.7)"><div style="color:#64748b">PLAN</div><div style="margin-top:2px;color:#bae6fd">${escapeHtml(view.planId)}</div></div>
        <div style="padding:8px;border-radius:8px;background:rgba(15,23,42,.7)"><div style="color:#64748b">CANDIDATE</div><div style="margin-top:2px;color:#d9f99d">${escapeHtml(view.candidateId)}</div></div>
        <div style="padding:8px;border-radius:8px;background:rgba(15,23,42,.7)"><div style="color:#64748b">ATTEMPT / REPLAN</div><div style="margin-top:2px;color:#fdba74">${view.attempt} / ${view.replanCount}</div></div>
      </div>

      <div style="margin-top:11px;font-size:10px;letter-spacing:.12em;color:#64748b">LIVE EVENT TIMELINE</div>
      <div style="margin-top:5px;padding:8px 9px;border-radius:9px;background:rgba(15,23,42,.54)">${timeline}</div>
      <div style="margin-top:9px;color:#475569;font-size:10px">Tool tự observe → plan → execute → verify → recover. Không có bước căn tay thủ công.</div>`;
    root.querySelector('[data-abort]')?.addEventListener('click', () => {
      abortRequested = true;
      window.__axisAutonomyV3Bridge?.abort?.();
      state = {
        ...state,
        phase: 'ABORTED',
        message: 'Đã dừng chuyển động theo yêu cầu',
        terminalReason: 'ABORTED',
      };
      render();
    }, { once: true });
  };
  const resize = () => syncPosition();
  window.addEventListener('resize', resize);

  const api = {
    update(nextState) {
      if (destroyed) return { installed: false };
      state = structuredClone(nextState);
      render();
      return api.getStatus();
    },
    getStatus() {
      return {
        installed: !destroyed,
        abortRequested,
        state: structuredClone(state),
        view: buildViewModel(state),
      };
    },
    destroy() {
      if (destroyed) return { installed: false };
      destroyed = true;
      window.removeEventListener('resize', resize);
      root.remove();
      delete window.__axisAutonomyV3Tracking;
      return { installed: false };
    },
  };
  window.__axisAutonomyV3Tracking = api;
  render();
  return api.getStatus();
}
