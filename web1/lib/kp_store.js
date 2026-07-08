/**
 * kp_store.js — K230 步态帧的 SQLite 持久化
 *
 *   每帧一行:device_id / seq / ts (毫秒) / points (JSON [[x,y,c]×6])
 *   K230 ~10Hz,一小时 ≈ 36k 行,每行 ~100 字节 → 单日约 350MB。
 *   实际按 seq 去重 + K230 无人时不发,数据量会小很多。
 *
 *   如需清理:调 pruneOlderThan(days)。首版不自动清理,方便调试。
 */

const path = require('path');
const fs = require('fs');
const Database = require('better-sqlite3');

class KpStore {
  constructor(dbPath) {
    const dir = path.dirname(dbPath);
    if (!fs.existsSync(dir)) fs.mkdirSync(dir, { recursive: true });

    this.db = new Database(dbPath);
    this.db.pragma('journal_mode = WAL');
    this.db.pragma('synchronous = NORMAL');

    this.db.exec(`
      CREATE TABLE IF NOT EXISTS kp_frame (
        id         INTEGER PRIMARY KEY AUTOINCREMENT,
        device_id  TEXT    NOT NULL,
        seq        INTEGER NOT NULL,
        ts         INTEGER NOT NULL,
        points     TEXT    NOT NULL
      );
      CREATE INDEX IF NOT EXISTS idx_kp_ts        ON kp_frame(ts);
      CREATE INDEX IF NOT EXISTS idx_kp_device_ts ON kp_frame(device_id, ts);

      CREATE TABLE IF NOT EXISTS gait_metric (
        id            INTEGER PRIMARY KEY AUTOINCREMENT,
        device_id     TEXT    NOT NULL,
        ts            INTEGER NOT NULL,
        spm           INTEGER,
        symmetry      INTEGER,
        knee_angle_l  INTEGER,
        knee_angle_r  INTEGER,
        knee_amp_l    INTEGER,
        knee_amp_r    INTEGER,
        stability     INTEGER,
        samples       INTEGER
      );
      CREATE INDEX IF NOT EXISTS idx_metric_ts        ON gait_metric(ts);
      CREATE INDEX IF NOT EXISTS idx_metric_device_ts ON gait_metric(device_id, ts);

      CREATE TABLE IF NOT EXISTS fall_event (
        id         INTEGER PRIMARY KEY AUTOINCREMENT,
        device_id  TEXT    NOT NULL,
        ts         INTEGER NOT NULL
      );
      CREATE INDEX IF NOT EXISTS idx_fall_ts        ON fall_event(ts);
      CREATE INDEX IF NOT EXISTS idx_fall_device_ts ON fall_event(device_id, ts);
    `);

    this.insertStmt = this.db.prepare(
      'INSERT INTO kp_frame (device_id, seq, ts, points) VALUES (?, ?, ?, ?)'
    );
    this.rangeStmt = this.db.prepare(
      'SELECT seq, ts, points FROM kp_frame WHERE device_id = ? AND ts BETWEEN ? AND ? ORDER BY ts ASC'
    );
    this.countStmt = this.db.prepare(
      'SELECT COUNT(*) as n FROM kp_frame WHERE device_id = ? AND ts BETWEEN ? AND ?'
    );
    this.pruneStmt = this.db.prepare('DELETE FROM kp_frame WHERE ts < ?');
    this.totalStmt = this.db.prepare('SELECT COUNT(*) as n FROM kp_frame');

    this.insertMetricStmt = this.db.prepare(`
      INSERT INTO gait_metric
        (device_id, ts, spm, symmetry, knee_angle_l, knee_angle_r, knee_amp_l, knee_amp_r, stability, samples)
      VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
    `);
    this.metricRangeStmt = this.db.prepare(`
      SELECT ts, spm, symmetry, knee_angle_l, knee_angle_r, knee_amp_l, knee_amp_r, stability, samples
      FROM gait_metric
      WHERE device_id = ? AND ts BETWEEN ? AND ?
      ORDER BY ts ASC
    `);

    this.insertFallStmt = this.db.prepare(
      'INSERT INTO fall_event (device_id, ts) VALUES (?, ?)'
    );
    this.fallRangeStmt = this.db.prepare(
      'SELECT ts FROM fall_event WHERE device_id = ? AND ts BETWEEN ? AND ? ORDER BY ts ASC'
    );
  }

  insertFall(deviceId, ts) {
    try {
      this.insertFallStmt.run(deviceId || 'unknown', ts);
    } catch (e) {
      console.error('[kp_store] insert fall failed:', e.message);
    }
  }

  queryFallRange(deviceId, fromMs, toMs) {
    return this.fallRangeStmt.all(deviceId, fromMs, toMs).map(r => r.ts);
  }

  insertMetric(deviceId, m) {
    if (!m) return;
    try {
      this.insertMetricStmt.run(
        deviceId || 'unknown',
        m.ts,
        m.spm ?? null,
        m.symmetry ?? null,
        m.kneeAngleL ?? null,
        m.kneeAngleR ?? null,
        m.kneeAmpL ?? null,
        m.kneeAmpR ?? null,
        m.stability ?? null,
        m.samples ?? null,
      );
    } catch (e) {
      console.error('[kp_store] insert metric failed:', e.message);
    }
  }

  queryMetricRange(deviceId, fromMs, toMs) {
    return this.metricRangeStmt.all(deviceId, fromMs, toMs).map(r => ({
      ts: r.ts,
      spm: r.spm,
      symmetry: r.symmetry,
      kneeAngleL: r.knee_angle_l,
      kneeAngleR: r.knee_angle_r,
      kneeAmpL: r.knee_amp_l,
      kneeAmpR: r.knee_amp_r,
      stability: r.stability,
      samples: r.samples,
    }));
  }

  insertFrame(envelope) {
    if (!envelope || !Array.isArray(envelope.points) || envelope.points.length !== 6) return;
    const deviceId = envelope.deviceId || 'unknown';
    const seq = Number(envelope.seq) || 0;
    const ts = Date.now();
    const points = JSON.stringify(envelope.points);
    try {
      this.insertStmt.run(deviceId, seq, ts, points);
    } catch (e) {
      console.error('[kp_store] insert failed:', e.message);
    }
  }

  /**
   * 查询时间窗内的 kp 帧;超过 maxRows 时等距降采样。
   * @returns { frames: [{seq, ts, points}], total: number, downsampled: bool }
   */
  queryRange(deviceId, fromMs, toMs, maxRows = 5000) {
    const total = this.countStmt.get(deviceId, fromMs, toMs).n;
    const rows = this.rangeStmt.all(deviceId, fromMs, toMs).map(r => ({
      seq: r.seq,
      ts: r.ts,
      points: JSON.parse(r.points),
    }));
    let frames = rows;
    let downsampled = false;
    if (rows.length > maxRows) {
      const step = rows.length / maxRows;
      frames = [];
      for (let i = 0; i < maxRows; i++) frames.push(rows[Math.floor(i * step)]);
      downsampled = true;
    }
    return { frames, total, returned: frames.length, downsampled };
  }

  pruneOlderThan(days) {
    const cutoff = Date.now() - days * 86400 * 1000;
    const r = this.pruneStmt.run(cutoff);
    return r.changes;
  }

  totalRows() {
    return this.totalStmt.get().n;
  }
}

module.exports = { KpStore };
