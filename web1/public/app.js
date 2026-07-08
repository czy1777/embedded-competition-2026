/* eslint-disable no-undef */
const socket = io();

/* ==========================================================================
 *  Helpers
 * ========================================================================== */
function setAll(className, v) {
  const val = (v === undefined || v === null) ? '--' : v;
  document.querySelectorAll('.' + className).forEach(el => (el.textContent = val));
}
function setById(id, v) {
  const el = document.getElementById(id);
  if (el) el.textContent = (v === undefined || v === null) ? '--' : v;
}
/**
 * 同时更新一组 id 的 textContent 和 className。
 * 用于 fall-text / slope-badge / gps-fix / pm25-tag / dev-watch 这种"同数据、多页显示"的元素。
 */
function setMulti(ids, text, className) {
  ids.forEach(id => {
    const el = document.getElementById(id);
    if (!el) return;
    if (text !== undefined) el.textContent = text;
    if (className !== undefined) el.className = className;
  });
}
function formatNum(v, digits) {
  if (v === undefined || v === null || Number.isNaN(v)) return '--';
  return Number(v).toFixed(digits);
}
function formatUptime(s) {
  if (!s) return '--';
  const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), sec = s % 60;
  if (h > 0) return `${h}h ${m}m`;
  if (m > 0) return `${m}m ${sec}s`;
  return `${sec}s`;
}
function pushSeries(arr, v) { arr.shift(); arr.push(v); }
function escapeHtml(s) {
  return String(s).replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}

/* ==========================================================================
 *  连接状态 + 时钟
 *  连接判定分两层：socket.io ↔ 后端  +  SPARK 设备 heartbeat 心跳
 *  设备 heartbeat 大约 1.25s 一次(sensor_threads 5 槽轮转)，超时 8s 判为离线
 * ========================================================================== */
const DEVICE_HEARTBEAT_TIMEOUT_MS = 8000;
let wsConnected = false;
let deviceOnline = false;
let offlineTimer = null;

function setConn(state, label) {
  const dot = document.getElementById('conn-dot');
  const text = document.getElementById('conn-text');
  const map = {
    on:      'h-2 w-2 rounded-full bg-emerald-400 pulse-dot',
    waiting: 'h-2 w-2 rounded-full bg-amber-400 pulse-dot',
    off:     'h-2 w-2 rounded-full bg-rose-400 pulse-dot',
  };
  dot.className = map[state] || map.waiting;
  text.textContent = label;
}
function refreshConn() {
  if (!wsConnected) return setConn('off', '服务断开');
  setConn(deviceOnline ? 'on' : 'waiting', deviceOnline ? '设备在线' : '等待设备…');
}
function markDeviceAlive() {
  deviceOnline = true;
  refreshConn();
  clearTimeout(offlineTimer);
  offlineTimer = setTimeout(() => { deviceOnline = false; refreshConn(); }, DEVICE_HEARTBEAT_TIMEOUT_MS);
}
socket.on('connect',    () => { wsConnected = true;  refreshConn(); });
socket.on('disconnect', () => { wsConnected = false; refreshConn(); });
refreshConn();

function tick() {
  const d = new Date();
  document.getElementById('now-clock').textContent =
    `${String(d.getHours()).padStart(2, '0')}:${String(d.getMinutes()).padStart(2, '0')}:${String(d.getSeconds()).padStart(2, '0')}`;
}
tick();
setInterval(tick, 1000);

/* ==========================================================================
 *  Chart.js defaults + 各图表初始化
 * ========================================================================== */
Chart.defaults.color = 'rgba(255, 255, 255, 0.6)';
Chart.defaults.borderColor = 'rgba(255, 255, 255, 0.08)';
Chart.defaults.font.family = "Inter, system-ui, -apple-system, sans-serif";

/* 跌倒事件时间线:X 轴时间,Y 轴常量 1,每个事件一个点 */
const fallChart = new Chart(document.getElementById('chart-fall'), {
  type: 'scatter',
  data: { datasets: [{
    label: '跌倒事件',
    data: [],
    backgroundColor: 'rgba(251, 113, 133, 0.85)',
    borderColor: '#fb7185',
    pointRadius: 6, pointHoverRadius: 9,
  }] },
  options: {
    responsive: true, maintainAspectRatio: false, animation: false,
    plugins: {
      legend: { display: false },
      tooltip: {
        callbacks: {
          label: (ctx) => new Date(ctx.parsed.x).toLocaleString('zh-CN', { hour12: false }),
        },
      },
    },
    scales: {
      x: {
        type: 'linear',
        ticks: {
          callback: (v) => {
            const d = new Date(v);
            return `${d.getMonth() + 1}/${d.getDate()} ${String(d.getHours()).padStart(2, '0')}:${String(d.getMinutes()).padStart(2, '0')}`;
          },
          maxTicksLimit: 6, font: { size: 10 },
        },
      },
      y: { min: 0, max: 2, display: false },
    },
  },
});

const KP_COLORS = ['#f87171', '#60a5fa', '#fbbf24', '#34d399', '#f472b6', '#22d3ee'];
const KP_LABELS = ['左髋', '右髋', '左膝', '右膝', '左踝', '右踝'];
const kpScatter = new Chart(document.getElementById('chart-kp-scatter'), {
  type: 'scatter',
  data: { datasets: KP_LABELS.map((label, i) => ({
    label, data: [], backgroundColor: KP_COLORS[i], borderColor: KP_COLORS[i],
    pointRadius: 6, pointHoverRadius: 8,
  })) },
  options: {
    responsive: true, maintainAspectRatio: false, animation: false,
    plugins: { legend: { position: 'bottom', labels: { boxWidth: 10, padding: 8, font: { size: 11 } } } },
    scales: {
      x: { min: 0, max: 320, ticks: { stepSize: 80 } },
      y: { min: 0, max: 320, reverse: true, ticks: { stepSize: 80 } },
    },
  },
});

const M_WINDOW = 72;
const mSpm = Array(M_WINDOW).fill(null);
const mSym = Array(M_WINDOW).fill(null);
const mKneeL = Array(M_WINDOW).fill(null);
const mKneeR = Array(M_WINDOW).fill(null);
const mStab = Array(M_WINDOW).fill(null);
function sparklineConfig(datasets, yOpts) {
  return {
    type: 'line',
    data: { labels: Array(M_WINDOW).fill(''), datasets },
    options: {
      responsive: true, maintainAspectRatio: false, animation: false,
      plugins: { legend: { display: false }, tooltip: { enabled: false } },
      scales: { x: { display: false }, y: { display: false, ...(yOpts || {}) } },
      elements: { point: { radius: 0 } },
    },
  };
}
const chartSpm = new Chart(document.getElementById('chart-m-spm'),
  sparklineConfig([{ data: mSpm, borderColor: '#38bdf8', backgroundColor: 'rgba(56, 189, 248, 0.15)', borderWidth: 2, tension: 0.35, fill: true }],
    { suggestedMin: 0, suggestedMax: 150 }));
const chartSym = new Chart(document.getElementById('chart-m-symmetry'),
  sparklineConfig([{ data: mSym, borderColor: '#34d399', backgroundColor: 'rgba(52, 211, 153, 0.15)', borderWidth: 2, tension: 0.35, fill: true }],
    { min: 0, max: 100 }));
const chartKnee = new Chart(document.getElementById('chart-m-knee'),
  sparklineConfig([
    { data: mKneeL, borderColor: '#f472b6', borderWidth: 1.8, tension: 0.35 },
    { data: mKneeR, borderColor: '#22d3ee', borderWidth: 1.8, tension: 0.35 },
  ], { suggestedMin: 0, suggestedMax: 90 }));
const chartStab = new Chart(document.getElementById('chart-m-stability'),
  sparklineConfig([{ data: mStab, borderColor: '#a78bfa', backgroundColor: 'rgba(167, 139, 250, 0.15)', borderWidth: 2, tension: 0.35, fill: true }],
    { min: 0, max: 100 }));

const ANK_WINDOW = 300;
const ankL = Array(ANK_WINDOW).fill(null);
const ankR = Array(ANK_WINDOW).fill(null);
const ankChart = new Chart(document.getElementById('chart-kp-ankle'), {
  type: 'line',
  data: { labels: Array(ANK_WINDOW).fill(''), datasets: [
    { label: '左踝 x', data: ankL, borderColor: '#f472b6', borderWidth: 1.6, tension: 0.25, pointRadius: 0 },
    { label: '右踝 x', data: ankR, borderColor: '#22d3ee', borderWidth: 1.6, tension: 0.25, pointRadius: 0 },
  ] },
  options: {
    responsive: true, maintainAspectRatio: false, animation: false,
    plugins: { legend: { position: 'bottom', labels: { boxWidth: 10, padding: 6, font: { size: 11 } } } },
    scales: { x: { display: false }, y: { min: 60, max: 260, ticks: { stepSize: 50 } } },
  },
});

const histAnkleChart = new Chart(document.getElementById('chart-hist-ankle'), {
  type: 'line',
  data: { datasets: [
    { label: '左踝 x', data: [], borderColor: '#f472b6', borderWidth: 1.2, tension: 0.2, pointRadius: 0, parsing: false },
    { label: '右踝 x', data: [], borderColor: '#22d3ee', borderWidth: 1.2, tension: 0.2, pointRadius: 0, parsing: false },
  ] },
  options: {
    responsive: true, maintainAspectRatio: false, animation: false,
    plugins: { legend: { position: 'bottom', labels: { boxWidth: 10, padding: 6, font: { size: 11 } } } },
    scales: {
      x: { type: 'linear', ticks: { callback: (v) => new Date(v).toLocaleTimeString('zh-CN', { hour12: false }), maxTicksLimit: 8, font: { size: 10 } } },
      y: { min: 0, max: 320, ticks: { stepSize: 80 } },
    },
  },
});
const histMetricsChart = new Chart(document.getElementById('chart-hist-metrics'), {
  type: 'line',
  data: { datasets: [
    { label: '步频 spm', data: [], borderColor: '#38bdf8', borderWidth: 1.6, tension: 0.3, pointRadius: 0, parsing: false, yAxisID: 'y1' },
    { label: '对称性 %', data: [], borderColor: '#34d399', borderWidth: 1.6, tension: 0.3, pointRadius: 0, parsing: false, yAxisID: 'y2' },
    { label: '稳定性',   data: [], borderColor: '#a78bfa', borderWidth: 1.6, tension: 0.3, pointRadius: 0, parsing: false, yAxisID: 'y2' },
  ] },
  options: {
    responsive: true, maintainAspectRatio: false, animation: false,
    plugins: { legend: { position: 'bottom', labels: { boxWidth: 10, padding: 6, font: { size: 11 } } } },
    scales: {
      x: { type: 'linear', ticks: { callback: (v) => new Date(v).toLocaleTimeString('zh-CN', { hour12: false }), maxTicksLimit: 8, font: { size: 10 } } },
      y1: { position: 'left',  min: 0, max: 160, title: { display: true, text: 'spm', font: { size: 10 } } },
      y2: { position: 'right', min: 0, max: 100, grid: { display: false }, title: { display: true, text: '% / 分', font: { size: 10 } } },
    },
  },
});

const ALL_CHARTS = [fallChart, kpScatter, ankChart, chartSpm, chartSym, chartKnee, chartStab, histAnkleChart, histMetricsChart];

/* ==========================================================================
 *  Socket handlers — 实时数据
 * ========================================================================== */
socket.on('health', (m) => {
  setAll('js-hr', m.hr);
  setAll('js-spo2', m.spo2);
  setAll('js-body-temp', formatNum(m.body_temp, 1));

  if (m.fall) {
    setMulti(['fall-text', 'fall-text-2'], '已检测', 'text-rose-300');
    const box = document.getElementById('fall-icon-box');
    if (box) box.className = 'h-10 w-10 bg-rose-500/30 rounded-lg flex items-center justify-center animate-pulse';
  } else {
    setMulti(['fall-text', 'fall-text-2'], '正常', 'text-emerald-300');
    const box = document.getElementById('fall-icon-box');
    if (box) box.className = 'h-10 w-10 bg-emerald-500/20 rounded-lg flex items-center justify-center';
  }
});

socket.on('env', (m) => {
  setAll('js-pm25', formatNum(m.pm25, 1));
  setAll('js-amb-temp', formatNum(m.temp, 1));
  setAll('js-humidity', formatNum(m.humidity, 1));

  let tagText, tagCls;
  if (m.pm25 < 35) { tagText = '空气良好'; tagCls = 'js-pm25-tag text-xs mt-1 text-emerald-300'; }
  else if (m.pm25 < 75) { tagText = '轻度污染'; tagCls = 'js-pm25-tag text-xs mt-1 text-amber-300'; }
  else { tagText = '中度+ 污染'; tagCls = 'js-pm25-tag text-xs mt-1 text-rose-300'; }
  setMulti(['pm25-tag', 'pm25-tag-2'], tagText, tagCls);
});

socket.on('attitude', (m) => {
  setAll('js-pitch', formatNum(m.pitch, 2));
  const slope = m.slope || 'flat';
  let text, cls;
  const base = 'px-3 py-1 rounded-full border text-sm';
  if (slope === 'uphill') { text = '上坡'; cls = `${base} bg-amber-500/20 border-amber-400/30 text-amber-200`; }
  else if (slope === 'downhill') { text = '下坡'; cls = `${base} bg-sky-500/20 border-sky-400/30 text-sky-200`; }
  else { text = '平地'; cls = `${base} bg-emerald-500/20 border-emerald-400/30 text-emerald-200`; }
  setMulti(['slope-badge', 'slope-badge-2'], text, cls);
});

socket.on('gps', (m) => {
  setAll('js-gps-lat', formatNum(m.lat, 6));
  setAll('js-gps-lon', formatNum(m.lon, 6));
  setAll('js-gps-utc', m.utc || '--');
  const base = 'text-xs px-2 py-0.5 rounded-full border';
  if (m.fix) {
    setMulti(['gps-fix', 'gps-fix-2'], '已定位', `${base} bg-emerald-500/20 border-emerald-400/30 text-emerald-200`);
    updateMapLocation(m.lon, m.lat);
  } else {
    setMulti(['gps-fix', 'gps-fix-2'], '未定位', `${base} bg-slate-700/60 border-white/10 text-white/60`);
  }
});

/* ==========================================================================
 *  高德地图:GPS 定位显示
 *  - GPS 原始坐标为 WGS-84,高德底图为 GCJ-02,用 AMap.convertFrom 转换
 *  - 首次定位 setCenter,之后仅移动 marker,不打扰用户手动缩放/拖动
 * ========================================================================== */
let amap = null;
let amapMarker = null;
let amapCentered = false;
let lastGpsLngLat = null;   // 缓存最近 GPS 坐标,env 页首次显示时补一次定位

function initAmap() {
  if (amap || !window.AMap) return;
  amap = new AMap.Map('map', {
    zoom: 16,
    viewMode: '2D',
    mapStyle: 'amap://styles/normal',
  });
  if (lastGpsLngLat) updateMapLocation(lastGpsLngLat[0], lastGpsLngLat[1]);
}

function updateMapLocation(lon, lat) {
  lastGpsLngLat = [lon, lat];
  const statusEl = document.getElementById('map-status');
  if (!amap || !window.AMap) return;
  AMap.convertFrom([lon, lat], 'gps', (status, result) => {
    if (status !== 'complete' || !result.locations || !result.locations.length) {
      if (statusEl) statusEl.textContent = '坐标转换失败';
      return;
    }
    const p = result.locations[0];
    if (!amapMarker) {
      amapMarker = new AMap.Marker({ position: p, map: amap, title: 'SPARK 助行器' });
    } else {
      amapMarker.setPosition(p);
    }
    if (!amapCentered) { amap.setCenter(p); amapCentered = true; }
    if (statusEl) statusEl.textContent = `${lat.toFixed(6)}, ${lon.toFixed(6)}`;
  });
}

socket.on('heartbeat', (m) => {
  markDeviceAlive();
  setAll('js-uptime', formatUptime(m.uptime));

  const badge = document.getElementById('watch-badge');
  const dot = document.getElementById('watch-dot');
  const text = document.getElementById('watch-text');
  if (badge && dot && text) {
    const badgeBase = 'inline-flex items-center gap-1.5 px-2.5 py-1 rounded-full text-xs border';
    if (m.watch_connected) {
      badge.className = `${badgeBase} bg-emerald-500/20 border-emerald-400/30 text-emerald-200`;
      dot.className = 'h-1.5 w-1.5 rounded-full bg-emerald-400 pulse-dot';
      text.textContent = '手表在线';
    } else {
      badge.className = `${badgeBase} bg-rose-500/20 border-rose-400/30 text-rose-200`;
      dot.className = 'h-1.5 w-1.5 rounded-full bg-rose-400';
      text.textContent = '手表离线';
    }
  }
});

/* ==========================================================================
 *  跌倒事件时间线
 * ========================================================================== */
const fallEvents = [];   // ts 数组,升序

function formatAgo(ms) {
  const s = Math.round(ms / 1000);
  if (s < 60) return `${s} 秒前`;
  const m = Math.round(s / 60);
  if (m < 60) return `${m} 分钟前`;
  const h = Math.round(m / 60);
  if (h < 24) return `${h} 小时前`;
  return `${Math.round(h / 24)} 天前`;
}

function renderFallTimeline() {
  const now = Date.now();
  fallChart.data.datasets[0].data = fallEvents.map(ts => ({ x: ts, y: 1 }));
  fallChart.options.scales.x.min = now - 7 * 24 * 3600 * 1000;
  fallChart.options.scales.x.max = now;
  fallChart.update('none');

  const todayStart = new Date(); todayStart.setHours(0, 0, 0, 0);
  const weekStart = now - 7 * 24 * 3600 * 1000;
  setById('fall-today-count', fallEvents.filter(t => t >= todayStart.getTime()).length);
  setById('fall-week-count',  fallEvents.filter(t => t >= weekStart).length);

  const list = document.getElementById('fall-list');
  list.innerHTML = '';
  const recent = fallEvents.slice(-5).reverse();
  if (recent.length === 0) {
    list.innerHTML = '<li class="text-white/40">暂无跌倒事件</li>';
    return;
  }
  recent.forEach(ts => {
    const li = document.createElement('li');
    li.className = 'flex items-center justify-between';
    li.innerHTML = `<span>· ${new Date(ts).toLocaleString('zh-CN', { hour12: false })}</span><span class="text-white/40">${formatAgo(now - ts)}</span>`;
    list.appendChild(li);
  });
}

(async function loadFallHistory() {
  try {
    const r = await fetch('/api/falls');
    if (!r.ok) return;
    const data = await r.json();
    fallEvents.push(...(data.events || []));
    renderFallTimeline();
  } catch {}
})();

socket.on('fall_event', (evt) => {
  if (!evt || typeof evt.ts !== 'number') return;
  fallEvents.push(evt.ts);
  renderFallTimeline();
});

/* ==========================================================================
 *  步态指标 (每 5 秒更新一次)
 * ========================================================================== */
socket.on('gait_metrics', (m) => {
  setAll('js-spm', m.spm);
  setAll('js-sym', m.symmetry);
  setAll('js-knee-l', m.kneeAmpL);
  setAll('js-knee-r', m.kneeAmpR);
  setAll('js-stab', m.stability);

  pushSeries(mSpm, m.spm);       chartSpm.update('none');
  pushSeries(mSym, m.symmetry);  chartSym.update('none');
  pushSeries(mKneeL, m.kneeAmpL); pushSeries(mKneeR, m.kneeAmpR); chartKnee.update('none');
  pushSeries(mStab, m.stability); chartStab.update('none');
});

const HINT_LEVEL_CSS = {
  good: 'bg-emerald-500/20 border-emerald-400/30 text-emerald-200',
  ok:   'bg-amber-500/20 border-amber-400/30 text-amber-200',
  warn: 'bg-rose-500/20 border-rose-400/30 text-rose-200',
};
const HINT_ICON = { rhythm: 'activity', symmetry: 'scale', stiffness: 'move', balance: 'gauge' };

socket.on('gait_hints', (hints) => {
  const bar = document.getElementById('hint-bar');
  bar.innerHTML = '';
  if (!Array.isArray(hints) || hints.length === 0) {
    bar.innerHTML = '<span class="px-3 py-1 rounded-full bg-slate-700/60 border border-white/10 text-white/50 text-xs">等待步态分析…</span>';
    return;
  }
  for (const h of hints) {
    const css = HINT_LEVEL_CSS[h.level] || HINT_LEVEL_CSS.ok;
    const iconName = HINT_ICON[h.key] || 'circle';
    const badge = document.createElement('span');
    badge.className = `inline-flex items-center gap-1.5 px-3 py-1 rounded-full border ${css} text-xs`;
    badge.innerHTML = `<i data-lucide="${iconName}" class="h-3 w-3"></i>${h.text}`;
    bar.appendChild(badge);
  }
  if (window.lucide) window.lucide.createIcons();
});

/* ==========================================================================
 *  K230 关键点实时
 * ========================================================================== */
let lastKpTs = null;
const kpRateBuf = [];
socket.on('kp', (m) => {
  if (!Array.isArray(m.points) || m.points.length !== 6) return;
  setAll('js-kp-seq', m.seq);

  kpScatter.data.datasets.forEach((ds, i) => {
    const [x, y, c] = m.points[i];
    ds.data = [{ x, y, c }];
    ds.backgroundColor = c >= 70 ? KP_COLORS[i] : KP_COLORS[i] + '60';
  });
  kpScatter.update('none');

  pushSeries(ankL, m.points[4][0]);
  pushSeries(ankR, m.points[5][0]);
  ankChart.update('none');

  const now = Date.now();
  if (lastKpTs) {
    kpRateBuf.push(now - lastKpTs);
    if (kpRateBuf.length > 20) kpRateBuf.shift();
    const avg = kpRateBuf.reduce((a, b) => a + b, 0) / kpRateBuf.length;
    setById('kp-rate', (1000 / avg).toFixed(1));
  }
  lastKpTs = now;
});

/* ==========================================================================
 *  历史回放
 * ========================================================================== */
const KP_L_ANKLE_IDX = 4, KP_R_ANKLE_IDX = 5;

function fillDatetimeInput(id, ms) {
  const d = new Date(ms);
  const pad = (n) => String(n).padStart(2, '0');
  document.getElementById(id).value =
    `${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())}T${pad(d.getHours())}:${pad(d.getMinutes())}`;
}
fillDatetimeInput('hist-from', Date.now() - 5 * 60 * 1000);
fillDatetimeInput('hist-to', Date.now());

async function loadHistory(fromMs, toMs) {
  const status = document.getElementById('hist-status');
  status.textContent = '加载中…';
  try {
    const r = await fetch(`/api/history?from=${fromMs}&to=${toMs}`);
    if (!r.ok) {
      const e = await r.json().catch(() => ({}));
      throw new Error(e.error || `HTTP ${r.status}`);
    }
    const data = await r.json();
    renderHistory(data);
    const dsInfo = data.kp.downsampled ? `(降采样 ${data.kp.total}→${data.kp.returned})` : `(${data.kp.returned} 帧)`;
    status.textContent = `${data.kp.total} 帧 kp · ${data.gait.length} 条指标 ${dsInfo}`;
  } catch (e) {
    status.textContent = `加载失败:${e.message}`;
  }
}

function renderHistory(data) {
  histAnkleChart.data.datasets[0].data = data.kp.frames.map(f => ({ x: f.ts, y: f.points[KP_L_ANKLE_IDX][0] }));
  histAnkleChart.data.datasets[1].data = data.kp.frames.map(f => ({ x: f.ts, y: f.points[KP_R_ANKLE_IDX][0] }));
  histAnkleChart.update('none');

  histMetricsChart.data.datasets[0].data = data.gait.map(m => ({ x: m.ts, y: m.spm }));
  histMetricsChart.data.datasets[1].data = data.gait.map(m => ({ x: m.ts, y: m.symmetry }));
  histMetricsChart.data.datasets[2].data = data.gait.map(m => ({ x: m.ts, y: m.stability }));
  histMetricsChart.update('none');
}

document.querySelectorAll('.hist-quick').forEach(btn => {
  btn.addEventListener('click', () => {
    const mins = Number(btn.dataset.quick);
    const to = Date.now();
    const from = to - mins * 60 * 1000;
    fillDatetimeInput('hist-from', from);
    fillDatetimeInput('hist-to', to);
    loadHistory(from, to);
  });
});
document.getElementById('hist-load').addEventListener('click', () => {
  const fromStr = document.getElementById('hist-from').value;
  const toStr = document.getElementById('hist-to').value;
  if (!fromStr || !toStr) {
    document.getElementById('hist-status').textContent = '请先选择时间范围';
    return;
  }
  loadHistory(new Date(fromStr).getTime(), new Date(toStr).getTime());
});
document.getElementById('hist-clear').addEventListener('click', () => {
  histAnkleChart.data.datasets.forEach(d => (d.data = []));
  histAnkleChart.update('none');
  histMetricsChart.data.datasets.forEach(d => (d.data = []));
  histMetricsChart.update('none');
  document.getElementById('hist-status').textContent = '已清空';
});

/* ==========================================================================
 *  AI 综合分析
 * ========================================================================== */
const RISK_LEVEL_CSS = {
  low:    'bg-emerald-500/20 border-emerald-400/30 text-emerald-200',
  medium: 'bg-amber-500/20 border-amber-400/30 text-amber-200',
  high:   'bg-rose-500/20 border-rose-400/30 text-rose-200',
};
const RISK_LEVEL_LABEL = { low: '低风险', medium: '中风险', high: '高风险' };

(async function loadAiProvider() {
  try {
    const r = await fetch('/api/status'); await r.json();
    document.getElementById('ai-provider-tag').textContent = 'provider · 已就绪';
  } catch {
    document.getElementById('ai-provider-tag').textContent = 'provider · 加载失败';
  }
})();

const aiBtn = document.getElementById('ai-analyze-btn');
aiBtn.addEventListener('click', async () => {
  const windowMinutes = Number(document.getElementById('ai-window').value);
  aiBtn.disabled = true;
  aiBtn.innerHTML = '<i data-lucide="loader" class="h-3.5 w-3.5 animate-spin"></i> 分析中…';
  if (window.lucide) window.lucide.createIcons();
  try {
    const r = await fetch('/api/gait/analyze', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ windowMinutes }),
    });
    const data = await r.json();
    if (!r.ok) throw new Error(data.error || `HTTP ${r.status}`);
    renderAiReport(data);
  } catch (e) {
    document.getElementById('ai-report-empty').classList.remove('hidden');
    document.getElementById('ai-report-empty').innerHTML = `<span class="text-rose-300">分析失败:${escapeHtml(e.message)}</span>`;
    document.getElementById('ai-report').classList.add('hidden');
  } finally {
    aiBtn.disabled = false;
    aiBtn.innerHTML = '<i data-lucide="wand-sparkles" class="h-3.5 w-3.5"></i> 一键分析';
    if (window.lucide) window.lucide.createIcons();
  }
});

function renderAiReport(data) {
  document.getElementById('ai-report-empty').classList.add('hidden');
  document.getElementById('ai-report').classList.remove('hidden');

  document.getElementById('ai-summary').textContent = data.summary || '—';
  document.getElementById('ai-disclaimer').textContent = data.disclaimer || '';

  const risks = document.getElementById('ai-risks');
  risks.innerHTML = '';
  (data.risks || []).forEach(r => {
    const css = RISK_LEVEL_CSS[r.level] || RISK_LEVEL_CSS.medium;
    const label = RISK_LEVEL_LABEL[r.level] || r.level;
    const el = document.createElement('div');
    el.className = `px-3 py-2 rounded-lg border ${css} text-xs max-w-xs`;
    el.innerHTML =
      `<div class="flex items-center justify-between gap-2 mb-0.5"><span class="font-semibold">${escapeHtml(r.name || '')}</span><span class="text-[10px] opacity-80">${escapeHtml(label)}</span></div>` +
      `<div class="text-[11px] opacity-90">${escapeHtml(r.evidence || '')}</div>`;
    risks.appendChild(el);
  });

  const sug = document.getElementById('ai-suggestions');
  sug.innerHTML = '';
  (data.suggestions || []).forEach(s => {
    const li = document.createElement('li');
    li.textContent = s;
    sug.appendChild(li);
  });

  if (window.lucide) window.lucide.createIcons();
}

/* ==========================================================================
 *  分页导航
 * ========================================================================== */
const PAGE_META = {
  health:   { crumb: '健康监测',   title: '心率 / 血氧 / 体温 / 跌倒' },
  env:      { crumb: '环境定位',   title: 'PM2.5 / 温湿度 / GPS / 姿态' },
  gait:     { crumb: '步态分析',   title: '实时监测步态关键点' },
  history:  { crumb: '历史回放',   title: '步态数据时间窗查询' },
};

function showPage(name) {
  if (!PAGE_META[name]) name = 'health';
  document.querySelectorAll('.page').forEach(p => {
    p.classList.toggle('active', p.id === `page-${name}`);
  });
  document.querySelectorAll('.nav-link').forEach(a => {
    a.classList.toggle('nav-active', a.dataset.page === name);
  });
  document.getElementById('page-crumb').textContent = PAGE_META[name].crumb;
  document.getElementById('page-title').textContent = PAGE_META[name].title;
  // 关闭移动端 sidebar
  document.getElementById('sidebar').classList.remove('open');
  document.getElementById('sidebar-mask').classList.remove('open');
  // 图表在隐藏容器里初始尺寸为 0,切换后需要 resize
  requestAnimationFrame(() => ALL_CHARTS.forEach(c => c && c.resize && c.resize()));
  // 地图懒加载:首次进 env 页时初始化
  if (name === 'env') initAmap();
}

document.querySelectorAll('.nav-link').forEach(a => {
  a.addEventListener('click', (e) => {
    e.preventDefault();
    location.hash = a.dataset.page;
  });
});
window.addEventListener('hashchange', () => {
  showPage(location.hash.slice(1) || 'health');
});

/* 汉堡菜单 (移动端) */
document.getElementById('menu-toggle').addEventListener('click', () => {
  document.getElementById('sidebar').classList.toggle('open');
  document.getElementById('sidebar-mask').classList.toggle('open');
});
document.getElementById('sidebar-mask').addEventListener('click', () => {
  document.getElementById('sidebar').classList.remove('open');
  document.getElementById('sidebar-mask').classList.remove('open');
});

/* ==========================================================================
 *  启动
 * ========================================================================== */
if (window.lucide) window.lucide.createIcons();
showPage(location.hash.slice(1) || 'health');
