/* --- 极简 .env 加载 (KEY=VALUE 行, 支持注释) --- */
(function loadDotenv() {
  const fs = require('fs');
  const p = require('path').join(__dirname, '.env');
  if (!fs.existsSync(p)) return;
  for (const line of fs.readFileSync(p, 'utf-8').split(/\r?\n/)) {
    const s = line.trim();
    if (!s || s.startsWith('#')) continue;
    const eq = s.indexOf('=');
    if (eq < 0) continue;
    const k = s.slice(0, eq).trim();
    let v = s.slice(eq + 1).trim();
    if ((v.startsWith('"') && v.endsWith('"')) || (v.startsWith("'") && v.endsWith("'"))) {
      v = v.slice(1, -1);
    }
    if (!(k in process.env)) process.env[k] = v;
  }
})();

const path = require('path');
const http = require('http');
const express = require('express');
const { Server } = require('socket.io');
const config = require('./config');
const { startMqttBridge } = require('./lib/mqtt_bridge');
const { GaitAnalyzer } = require('./lib/gait_analyzer');
const { KpStore } = require('./lib/kp_store');
const aiProvider = require('./lib/ai_provider');

const app = express();
const server = http.createServer(app);
const io = new Server(server, { cors: { origin: '*' } });

app.use(express.static(path.join(__dirname, 'public')));
app.use(express.json({ limit: '256kb' }));

const store = new KpStore(path.join(__dirname, 'db', 'gait.sqlite'));
const gait = new GaitAnalyzer(io, store);

/* AI 分析限流:同一 device 10 秒最多一次 */
const AI_MIN_INTERVAL_MS = 10 * 1000;
const aiLastCallAt = new Map();

/**
 * 跌倒事件查询
 *   GET /api/falls?from=<ms>&to=<ms>&device=<id>
 *   默认最近 7 天
 */
app.get('/api/falls', (req, res) => {
  const now = Date.now();
  const to = Number(req.query.to) || now;
  const from = Number(req.query.from) || (to - 7 * 24 * 3600 * 1000);
  const device = req.query.device || config.deviceId;
  if (from >= to) return res.status(400).json({ error: 'from must be < to' });
  const events = store.queryFallRange(device, from, to);
  res.json({ from, to, device, events });
});

app.get('/api/status', (req, res) => {
  res.json({
    ok: true,
    mqtt: config.mqtt,
    topicPrefix: config.topicPrefix,
    channels: config.channels,
    gait: gait.getLatest(),
    kpRowsTotal: store.totalRows(),
    uptime: process.uptime(),
  });
});

/**
 * 步态历史查询
 *   GET /api/history?from=<ms>&to=<ms>&device=<id>
 *   返回:
 *     { kp: {frames, total, returned, downsampled}, gait: [metric...] }
 */
app.get('/api/history', (req, res) => {
  const now = Date.now();
  const to = Number(req.query.to) || now;
  const from = Number(req.query.from) || (to - 5 * 60 * 1000);   // 默认最近 5 分钟
  const device = req.query.device || config.deviceId;
  if (from >= to) return res.status(400).json({ error: 'from must be < to' });
  if (to - from > 24 * 3600 * 1000) return res.status(400).json({ error: 'range too large (>24h)' });

  const kp = store.queryRange(device, from, to);
  const metrics = gait.summarizeWindow(from, to, device);
  res.json({ from, to, device, kp, gait: metrics });
});

/**
 * AI 综合分析
 *   POST /api/gait/analyze
 *   body: { from?, to?, device?, windowMinutes? }
 *     - 提供 from/to  → 使用该时间窗
 *     - 提供 windowMinutes → 使用最近 N 分钟(常用)
 *     - 都不提供 → 默认最近 5 分钟
 *   response: {
 *     summary, full, risks, suggestions, disclaimer,
 *     metricsSnapshot, provider, model, latencyMs
 *   }
 */
app.post('/api/gait/analyze', async (req, res) => {
  const body = req.body || {};
  const now = Date.now();
  const device = body.device || config.deviceId;

  // 限流
  const last = aiLastCallAt.get(device) || 0;
  if (now - last < AI_MIN_INTERVAL_MS) {
    return res.status(429).json({
      error: `请等待 ${Math.ceil((AI_MIN_INTERVAL_MS - (now - last)) / 1000)} 秒后再试`,
    });
  }

  // 时间窗
  let from, to, windowMinutes;
  if (body.windowMinutes) {
    windowMinutes = Number(body.windowMinutes);
    to = now;
    from = to - windowMinutes * 60 * 1000;
  } else if (body.from && body.to) {
    from = Number(body.from);
    to = Number(body.to);
    windowMinutes = Math.max(1, Math.round((to - from) / 60000));
  } else {
    windowMinutes = 5;
    to = now;
    from = to - windowMinutes * 60 * 1000;
  }
  if (from >= to) return res.status(400).json({ error: 'from must be < to' });

  const metricsSeries = gait.summarizeWindow(from, to, device);
  if (metricsSeries.length < 2) {
    return res.status(400).json({
      error: `分析窗口内步态指标不足(仅 ${metricsSeries.length} 条)。请先让被观察对象走一小段路(每 5 秒采样一次)。`,
    });
  }

  aiLastCallAt.set(device, now);
  const t0 = Date.now();
  try {
    const report = await aiProvider.analyzeGait({ metricsSeries, windowMinutes });
    res.json({
      ...report,
      metricsSnapshot: {
        from, to, windowMinutes,
        samples: metricsSeries.length,
        first: metricsSeries[0],
        last: metricsSeries[metricsSeries.length - 1],
        series: metricsSeries,
      },
      latencyMs: Date.now() - t0,
    });
  } catch (e) {
    console.error('[ai] analyze failed:', e.message);
    res.status(502).json({
      error: `AI 分析失败:${e.message}`,
      provider: aiProvider.currentProviderName(),
      latencyMs: Date.now() - t0,
    });
  }
});

io.on('connection', (sock) => {
  console.log(`[ws] client connected: ${sock.id}`);
  const latest = gait.getLatest();
  if (latest) sock.emit('gait_metrics', latest);
  sock.on('disconnect', () => console.log(`[ws] client gone: ${sock.id}`));
});

/* 跌倒事件边缘检测:仅在 fall 由 0→1 时记录一次,避免持续期间重复入库 */
const lastFallByDevice = new Map();

startMqttBridge({
  host: config.mqtt.host,
  port: config.mqtt.port,
  topicPattern: `${config.topicPrefix}/+/+`,
  channels: config.channels,
  io,
  onKpFrame: (envelope) => {
    gait.pushFrame(envelope);
    store.insertFrame(envelope);
  },
  onHealth: (envelope) => {
    const dev = envelope.deviceId || 'unknown';
    const prev = lastFallByDevice.get(dev) || 0;
    const cur = envelope.fall ? 1 : 0;
    if (!prev && cur) {
      const ts = Date.now();
      store.insertFall(dev, ts);
      io.emit('fall_event', { deviceId: dev, ts });
      console.log(`[fall] event recorded device=${dev} ts=${ts}`);
    }
    lastFallByDevice.set(dev, cur);
  },
});

server.listen(config.http.port, () => {
  console.log(`[http] listening on http://localhost:${config.http.port}`);
  console.log(`[ai] provider=${aiProvider.currentProviderName()} model=${process.env.AI_MODEL || 'deepseek-chat'}`);
});
