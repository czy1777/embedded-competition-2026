const mqtt = require('mqtt');
const config = require('../config');

const url = `mqtt://${config.mqtt.host}:${config.mqtt.port}`;
const prefix = `${config.topicPrefix}/${config.deviceId}`;
const client = mqtt.connect(url, {
  clientId: `mock-pub-${Math.random().toString(16).slice(2, 10)}`,
  reconnectPeriod: 3000,
});

const startedAt = Date.now();
let kpSeq = 0;

const ts = () => Date.now();
const uptime = () => Math.floor((Date.now() - startedAt) / 1000);
const rand = (lo, hi) => lo + Math.random() * (hi - lo);
const pub = (channel, body) => client.publish(`${prefix}/${channel}`, JSON.stringify(body));

client.on('connect', () => {
  console.log(`[mock] connected to ${url}`);
  console.log(`[mock] publishing under ${prefix}/{health,env,attitude,gps,kp,heartbeat}`);

  setInterval(() => pub('health', {
    hr: Math.round(rand(62, 92)),
    spo2: Math.round(rand(95, 100)),
    body_temp: +rand(36.2, 37.1).toFixed(2),
    fall: Math.random() < 0.01 ? 1 : 0,
    ts: ts(),
  }), 500);

  setInterval(() => pub('env', {
    pm25: +rand(5, 80).toFixed(1),
    temp: +rand(22, 28).toFixed(1),
    humidity: +rand(40, 65).toFixed(1),
    ts: ts(),
  }), 2000);

  setInterval(() => {
    const pitch = +rand(-7, 7).toFixed(2);
    const slope = pitch > 5 ? 'uphill' : pitch < -5 ? 'downhill' : 'flat';
    pub('attitude', { pitch, slope, ts: ts() });
  }, 250);

  setInterval(() => pub('gps', {
    lat: +(30.27 + rand(-0.001, 0.001)).toFixed(6),
    lon: +(120.15 + rand(-0.001, 0.001)).toFixed(6),
    fix: 1,
    utc: new Date().toISOString().substr(11, 12),
    ts: ts(),
  }), 1000);

  // 10Hz K230 步态:正弦摆动的双脚,COCO 顺序 [L-Hip, R-Hip, L-Knee, R-Knee, L-Ankle, R-Ankle]
  setInterval(() => {
    kpSeq = (kpSeq + 1) & 0xff;
    const t = Date.now() / 1000;
    const phase = t * 2 * Math.PI * 1.0; // 1Hz 步频 ≈ 60 spm
    const cx = 160, cy = 160;
    const pts = [
      [cx - 40,                       cy - 60, 95],
      [cx + 40,                       cy - 60, 95],
      [cx - 40 + Math.sin(phase) * 15, cy - 20, 92],
      [cx + 40 - Math.sin(phase) * 15, cy - 20, 92],
      [cx - 40 + Math.sin(phase) * 35, cy + 30, 88],
      [cx + 40 - Math.sin(phase) * 35, cy + 30, 88],
    ].map(([x, y, c]) => [Math.round(x), Math.round(y), c]);
    pub('kp', { seq: kpSeq, points: pts, ts: ts() });
  }, 100);

  setInterval(() => pub('heartbeat', {
    uptime: uptime(),
    watch_connected: 1,
    wifi_rssi: Math.round(rand(-75, -40)),
    ts: ts(),
  }), 1000);
});

client.on('error', (e) => console.error('[mock] error', e.message));
client.on('reconnect', () => console.log('[mock] reconnecting...'));
