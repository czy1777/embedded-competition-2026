/**
 * gait_analyzer.js
 *
 * 步态分析核心:接收 K230 6 关键点帧,按 30 秒滑动窗口,每 5 秒计算一次:
 *   - spm       步频 (每分钟步数)
 *   - symmetry  左右步幅对称性指数 (0-100)
 *   - kneeAngleL/R + kneeAmpL/R  膝关节角度和窗口内幅值
 *   - stability 平衡稳定性 (0-100)
 * 并输出 4 类"轻提示"(rhythm / symmetry / stiffness / balance),
 * 通过 Socket.IO 广播:
 *   'gait_metrics'  数值面板 + 曲线数据点
 *   'gait_hints'    徽章条
 *
 * 关键点索引 (COCO 11-16):
 *   0=L-Hip  1=R-Hip  2=L-Knee  3=R-Knee  4=L-Ankle  5=R-Ankle
 * 每点: [x, y, conf]  x/y ∈ [0, 320]  conf ∈ [0, 99]
 *
 * 供实施 7 AI 分析使用:导出 summarizeWindow() 返回一段时间的完整指标序列
 */

const KP_L_HIP = 0, KP_R_HIP = 1;
const KP_L_KNEE = 2, KP_R_KNEE = 3;
const KP_L_ANKLE = 4, KP_R_ANKLE = 5;

const WINDOW_SEC = 30;
const COMPUTE_INTERVAL_MS = 5000;
const MIN_CONF = 30;         // 低于此置信度的点不参与角度计算
const MIN_FRAMES = 20;       // 帧数不足时不出结果

class GaitAnalyzer {
  constructor(io, store) {
    this.io = io;
    this.store = store;       // KpStore,用于 gait_metric 持久化
    this.buffer = [];         // { ts, seq, points }
    this.lastDeviceId = null;
    this.latest = null;
    this.timer = setInterval(() => this.compute(), COMPUTE_INTERVAL_MS);
  }

  stop() {
    clearInterval(this.timer);
  }

  pushFrame(envelope) {
    if (!envelope || !Array.isArray(envelope.points) || envelope.points.length !== 6) return;
    if (envelope.deviceId) this.lastDeviceId = envelope.deviceId;
    const ts = Date.now();
    this.buffer.push({ ts, seq: envelope.seq, points: envelope.points });
    const cutoff = ts - WINDOW_SEC * 1000;
    while (this.buffer.length && this.buffer[0].ts < cutoff) this.buffer.shift();
  }

  compute() {
    if (this.buffer.length < MIN_FRAMES) return;

    const frames = this.buffer;
    const windowSec = (frames[frames.length - 1].ts - frames[0].ts) / 1000;
    if (windowSec < 3) return;

    // --- 1. 步频 (spm) ---
    // L_ankle_x - R_ankle_x 差分的过零次数,每个过零算半步
    const dxs = frames.map(f => f.points[KP_L_ANKLE][0] - f.points[KP_R_ANKLE][0]);
    let crossings = 0;
    for (let i = 1; i < dxs.length; i++) {
      if ((dxs[i - 1] < 0 && dxs[i] >= 0) || (dxs[i - 1] > 0 && dxs[i] <= 0)) crossings++;
    }
    const spm = Math.round(crossings * 60 / windowSec);  // 每分钟步数

    // --- 2. 对称性 ---
    const lxs = frames.map(f => f.points[KP_L_ANKLE][0]);
    const rxs = frames.map(f => f.points[KP_R_ANKLE][0]);
    const lAmp = Math.max(...lxs) - Math.min(...lxs);
    const rAmp = Math.max(...rxs) - Math.min(...rxs);
    const maxAmp = Math.max(lAmp, rAmp);
    const symmetry = maxAmp > 0
      ? Math.round(100 * (1 - Math.abs(lAmp - rAmp) / maxAmp))
      : 0;

    // --- 3. 膝关节角度 ---
    const latest = frames[frames.length - 1].points;
    const kneeAngleL = calcAngle(latest[KP_L_HIP], latest[KP_L_KNEE], latest[KP_L_ANKLE]);
    const kneeAngleR = calcAngle(latest[KP_R_HIP], latest[KP_R_KNEE], latest[KP_R_ANKLE]);

    const kneeLs = frames
      .map(f => calcAngle(f.points[KP_L_HIP], f.points[KP_L_KNEE], f.points[KP_L_ANKLE]))
      .filter(x => x !== null);
    const kneeRs = frames
      .map(f => calcAngle(f.points[KP_R_HIP], f.points[KP_R_KNEE], f.points[KP_R_ANKLE]))
      .filter(x => x !== null);
    const kneeAmpL = kneeLs.length ? Math.round(Math.max(...kneeLs) - Math.min(...kneeLs)) : 0;
    const kneeAmpR = kneeRs.length ? Math.round(Math.max(...kneeRs) - Math.min(...kneeRs)) : 0;

    // --- 4. 稳定性 ---
    // L 踝 y 的相邻帧差分标准差,越小越稳。std ≈ 30 时对应剧烈跳动,归一到 100 分制。
    const lys = frames.map(f => f.points[KP_L_ANKLE][1]);
    const diffs = [];
    for (let i = 1; i < lys.length; i++) diffs.push(lys[i] - lys[i - 1]);
    const meanD = diffs.reduce((a, b) => a + b, 0) / diffs.length;
    const std = Math.sqrt(diffs.reduce((a, b) => a + (b - meanD) ** 2, 0) / diffs.length);
    const stability = Math.max(0, Math.min(100, Math.round(100 - std * 3)));

    const metrics = {
      ts: Date.now(),
      windowSec: Math.round(windowSec),
      samples: frames.length,
      spm,
      symmetry,
      kneeAngleL: kneeAngleL !== null ? Math.round(kneeAngleL) : null,
      kneeAngleR: kneeAngleR !== null ? Math.round(kneeAngleR) : null,
      kneeAmpL,
      kneeAmpR,
      stability,
    };
    const hints = deriveHints(metrics);

    this.latest = metrics;
    if (this.store) this.store.insertMetric(this.lastDeviceId, metrics);

    this.io.emit('gait_metrics', metrics);
    this.io.emit('gait_hints', hints);

    console.log(`[gait] spm=${metrics.spm} sym=${metrics.symmetry} kneeAmp=${metrics.kneeAmpL}/${metrics.kneeAmpR} stab=${metrics.stability} samples=${metrics.samples}`);
  }

  /* 供 AI 分析 / 历史回放:从 SQLite 拉一段时间窗内的完整指标序列 */
  summarizeWindow(fromMs, toMs, deviceId) {
    if (!this.store) return [];
    return this.store.queryMetricRange(deviceId || this.lastDeviceId || 'unknown', fromMs, toMs);
  }

  getLatest() {
    return this.latest;
  }
}

/* ---------- 辅助 ---------- */
function calcAngle(a, b, c) {
  if (!a || !b || !c) return null;
  if (a[2] < MIN_CONF || b[2] < MIN_CONF || c[2] < MIN_CONF) return null;
  const v1x = a[0] - b[0], v1y = a[1] - b[1];
  const v2x = c[0] - b[0], v2y = c[1] - b[1];
  const m1 = Math.hypot(v1x, v1y);
  const m2 = Math.hypot(v2x, v2y);
  if (m1 === 0 || m2 === 0) return null;
  let cos = (v1x * v2x + v1y * v2y) / (m1 * m2);
  cos = Math.max(-1, Math.min(1, cos));
  return Math.acos(cos) * 180 / Math.PI;
}

function deriveHints(m) {
  const hints = [];

  if (m.spm < 60) {
    hints.push({ key: 'rhythm', level: 'warn', text: `步频偏慢 · ${m.spm} spm` });
  } else if (m.spm > 130) {
    hints.push({ key: 'rhythm', level: 'warn', text: `步频偏快 · ${m.spm} spm` });
  } else if (m.spm >= 80 && m.spm <= 120) {
    hints.push({ key: 'rhythm', level: 'good', text: `步频正常 · ${m.spm} spm` });
  } else {
    hints.push({ key: 'rhythm', level: 'ok', text: `步频 ${m.spm} spm` });
  }

  if (m.symmetry < 60) {
    hints.push({ key: 'symmetry', level: 'warn', text: `左右步态偏侧 · ${m.symmetry}%` });
  } else if (m.symmetry < 80) {
    hints.push({ key: 'symmetry', level: 'ok', text: `轻微偏侧 · ${m.symmetry}%` });
  } else {
    hints.push({ key: 'symmetry', level: 'good', text: `左右对称良好 · ${m.symmetry}%` });
  }

  const kneeMin = Math.min(m.kneeAmpL, m.kneeAmpR);
  if (kneeMin < 15) {
    hints.push({ key: 'stiffness', level: 'warn', text: `膝屈伸幅度偏低 · ${kneeMin}°` });
  } else if (kneeMin < 30) {
    hints.push({ key: 'stiffness', level: 'ok', text: `膝屈伸幅度一般 · ${kneeMin}°` });
  } else {
    hints.push({ key: 'stiffness', level: 'good', text: `膝屈伸正常 · ${kneeMin}°` });
  }

  if (m.stability < 50) {
    hints.push({ key: 'balance', level: 'warn', text: `平衡稳定性下降 · ${m.stability}` });
  } else if (m.stability < 75) {
    hints.push({ key: 'balance', level: 'ok', text: `平衡尚可 · ${m.stability}` });
  } else {
    hints.push({ key: 'balance', level: 'good', text: `平衡良好 · ${m.stability}` });
  }

  return hints;
}

module.exports = { GaitAnalyzer };
