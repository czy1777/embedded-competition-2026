const mqtt = require('mqtt');

function startMqttBridge({ host, port, topicPattern, channels, io, onKpFrame, onHealth }) {
  const url = `mqtt://${host}:${port}`;
  const allowed = new Set(channels);
  const client = mqtt.connect(url, {
    clientId: `spark-web-bridge-${Math.random().toString(16).slice(2, 10)}`,
    reconnectPeriod: 3000,
    connectTimeout: 8000,
  });

  client.on('connect', () => {
    console.log(`[mqtt] connected to ${url}`);
    client.subscribe(topicPattern, (err, granted) => {
      if (err) {
        console.error('[mqtt] subscribe failed', err.message);
      } else {
        console.log(`[mqtt] subscribed: ${granted.map((g) => g.topic).join(', ')}`);
      }
    });
  });

  client.on('message', (topic, payload) => {
    const parts = topic.split('/');
    if (parts.length < 3) return;
    const channel = parts[parts.length - 1];
    const deviceId = parts[1];
    if (!allowed.has(channel)) return;

    let body;
    try {
      body = JSON.parse(payload.toString('utf-8'));
    } catch (e) {
      console.warn(`[mqtt] bad json on ${topic}: ${e.message}`);
      return;
    }

    const envelope = { deviceId, ...body };
    io.emit(channel, envelope);

    if (channel === 'kp' && typeof onKpFrame === 'function') {
      onKpFrame(envelope);
    }
    if (channel === 'health' && typeof onHealth === 'function') {
      onHealth(envelope);
    }
  });

  client.on('reconnect', () => console.log('[mqtt] reconnecting...'));
  client.on('error', (e) => console.error('[mqtt] error', e.message));
  client.on('offline', () => console.warn('[mqtt] offline'));

  return client;
}

module.exports = { startMqttBridge };
