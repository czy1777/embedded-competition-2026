module.exports = {
  mqtt: {
    host: process.env.MQTT_HOST || 'broker.emqx.io',
    port: parseInt(process.env.MQTT_PORT || '1883', 10),
  },
  topicPrefix: process.env.TOPIC_PREFIX || 'walker',
  deviceId: process.env.DEVICE_ID || 'spark-walker-001',
  http: {
    port: parseInt(process.env.PORT || '3000', 10),
  },
  channels: ['health', 'env', 'attitude', 'gps', 'kp', 'heartbeat'],
};
