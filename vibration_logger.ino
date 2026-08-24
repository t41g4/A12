/* =====================================================================
 *  シェアサイクル 要メンテ車両検知システム  振動データロガー
 *  PROJECT_SPEC.md 第3版 準拠版
 *
 *  対象  : ESP32 + MPU6050 + microSD
 *  役割  : 第55章 Priority 1「取得系の是正」の実装
 *          第11.2章の受入基準を満たす生データを取得する
 *
 *  ---------------------------------------------------------------
 *  旧コードからの主要変更点
 *  ---------------------------------------------------------------
 *   1. loop()ポーリング          → ハードウェアタイマ + FIFOバーストリード   (第10.4章)
 *   2. 毎サンプル open/close     → RAMリングバッファ + 一括書き込み          (第10.4章)
 *   3. ±2 g                      → ±4 g (AFS_SEL = 1)                        (第10.2章)
 *   4. CLKSEL 未設定             → CLKSEL = 1 を明示設定                     (第10.5章)
 *   5. SMPLRT_DIV 未設定         → 7 (= 125 Hz)。128 Hz は分周で作れないため  (第58.1章)
 *   6. FILE_WRITE で毎回上書き   → run_id ごとの別ファイル + FILE_APPEND     (第43章)
 *   7. millis() ms               → esp_timer_get_time() µs (int64)           (第44.1章)
 *   8. 小数2桁                   → 小数3桁                                   (第10.6章)
 *   9. ジャイロ欠落              → gx,gy,gz を保存                           (第44.1章)
 *  10. 品質指標なし              → overflow/dropped/fs_effective/clip_rate   (第11.2章)
 *  11. 重力ベクトル未取得        → START時に静止1秒で計測                    (第12.2章)
 *  12. I2C 100 kHz               → 400 kHz (FIFOバースト読み出しに必須)
 *  13. Adafruit_MPU6050          → レジスタ直接制御（既定値への依存を排除）
 *
 *  ---------------------------------------------------------------
 *  BLEコマンド（Write）
 *  ---------------------------------------------------------------
 *      START:S_FLAT_001     指定 run_id で測定開始
 *      START                run_id 自動採番 (RUN_00001)
 *      STOP                 測定終了・メタデータ確定
 *
 *  ---------------------------------------------------------------
 *  出力ファイル
 *  ---------------------------------------------------------------
 *      /data/<run_id>.csv        生データ            (第44.1章)
 *      /fifo/<run_id>_fifo.csv   FIFO読み出しログ    (第44.1章)
 *      /meta/<run_id>_meta.csv   走行メタデータ      (第44.2章)
 * ===================================================================== */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include "esp_timer.h"

/* =====================================================================
 *  設定値（すべて仕様書の章番号と対応）
 * ===================================================================== */

// --- ピン ---
#define LED_PIN        2
#define SD_CS          5
#define I2C_SDA        21
#define I2C_SCL        22

// --- センサ設定（第10章・第58.1章） -----------------------------------
// fs = 1000 Hz / (1 + SMPLRT_DIV)。DLPF有効時のゲート出力レートが 1 kHz。
//   SMPLRT_DIV = 7  ->  125.000 Hz  N=125, Δf=1.000 Hz  ← 採用
//   128 Hz は分周では作れない（1000/128 = 7.8125）
#define SMPLRT_DIV_VAL   7
#define FS_NOMINAL_HZ    125.0f

#define AFS_SEL          1        // ±4 g          (第10.2章)
#define DLPF_CFG         3        // 44 Hz         (第10.3章)
#define CLKSEL           1        // Gyro X PLL    (第10.5章)
#define FS_SEL           1        // ±500 dps（ジャイロ。姿勢推定用）

// --- 換算係数 ---
#define ACCEL_LSB_PER_G  8192.0f  // ±4 g
#define GRAVITY          9.80665f
#define GYRO_LSB_PER_DPS 65.5f    // ±500 dps
#define DEG2RAD          0.0174532925f

// --- 飽和判定（第11.2章 clip_rate） -----------------------------------
// int16 のレンジ端。余裕を見て 32700 以上を「レンジ端到達」とみなす。
#define CLIP_THRESHOLD   32700

// --- FIFO 読み出し周期（第11.2章 read_interval_max_ms） ----------------
// FIFO 1024 byte / 12 byte = 85 サンプル = 680 ms 相当（125 Hz時）
// 受入基準は「容量の50%（= 340 ms）以内」。100 ms は十分な余裕。
#define FIFO_READ_PERIOD_US   100000UL   // 100 ms
#define FIFO_CAPACITY_SAMPLES 85
#define READ_INTERVAL_LIMIT_MS 340       // 容量50%相当（受入基準）

// --- 静止キャリブレーション（第12.2章） -------------------------------
#define CALIB_DURATION_MS  1000          // 静止1秒間の3軸平均

// --- バッファ ---------------------------------------------------------
#define SAMPLE_QUEUE_LEN   2048          // 125 Hz で約16秒分の余裕
#define WRITE_BUF_SIZE     4096          // SDへの一括書き込み単位
#define WRITE_FLUSH_BYTES  3000          // これを超えたら書き出す

// --- MPU6050 レジスタ -------------------------------------------------
#define MPU_ADDR            0x68
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_FIFO_EN         0x23
#define REG_INT_STATUS      0x3A
#define REG_SIGNAL_PATH_RST 0x68
#define REG_USER_CTRL       0x6A
#define REG_PWR_MGMT_1      0x6B
#define REG_FIFO_COUNT_H    0x72
#define REG_FIFO_R_W        0x74
#define REG_WHO_AM_I        0x75

// --- BLE UUID ---------------------------------------------------------
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHAR_CMD_UUID       "abcdefab-1234-5678-1234-abcdefabcdef"
#define CHAR_STATUS_UUID    "abcdefab-1234-5678-1234-abcdefabcd00"

/* =====================================================================
 *  型定義・グローバル
 * ===================================================================== */

// 生LSB値のまま渡す（float変換は書き込みタスク側で行う）
struct RawSample {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

enum LoggerState : uint8_t {
  ST_IDLE = 0,      // 待機
  ST_CALIB,         // 静止キャリブレーション中
  ST_LOGGING,       // 記録中
  ST_FINALIZE       // 終了処理中
};

volatile LoggerState  gState = ST_IDLE;

// タスク間通信
static QueueHandle_t     qSamples   = nullptr;   // サンプル本体
static QueueHandle_t     qCommands  = nullptr;   // BLEコマンド
static SemaphoreHandle_t semTimer   = nullptr;   // タイマ割り込み通知

// ファイル
static File fData, fFifo;
static char gRunId[32] = "";

// 走行統計（第11.2章の受入指標）
struct RunStats {
  uint64_t t_start_us;             // FIFOリセット時刻 = サンプル0の時刻
  uint64_t t_end_us;               // 最終FIFO読み出し時刻
  uint32_t n_samples;              // 記録したサンプル総数
  uint32_t fifo_overflow_count;    // FIFOオーバーフロー回数
  uint32_t n_dropped_queue;        // キュー溢れによる欠損（書き込みが追いつかない）
  uint32_t n_clip;                 // レンジ端に到達したサンプル数
  uint32_t read_count;             // FIFO読み出し回数
  uint32_t read_interval_max_ms;   // FIFO読み出し間隔の最大値
  uint32_t max_fifo_samples;       // 1回の読み出しで得た最大サンプル数
};
static RunStats gStats;

// 静止時重力ベクトル（第12.2章）
struct Gravity {
  float gx, gy, gz;                // m/s²
  float norm;                      // |g|
  float tilt_deg;                  // Z軸と鉛直方向のなす角
  uint32_t n;                      // 平均に使ったサンプル数
  bool valid;
};
static Gravity gGravity;
static int64_t gCalibSum[3];
static uint32_t gCalibN;
static uint64_t gCalibStartUs;

static BLECharacteristic *pStatusChar = nullptr;

/* =====================================================================
 *  I2C 低レベル
 * ===================================================================== */

static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool mpuRead(uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t got = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)len);
  if (got != len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static uint8_t mpuRead8(uint8_t reg) {
  uint8_t v = 0;
  mpuRead(reg, &v, 1);
  return v;
}

/* =====================================================================
 *  MPU6050 初期化（第10章）
 *  すべてのレジスタを明示的に設定し、書き込み後に読み戻して検証する。
 *  ライブラリ既定値に依存しないことが本関数の目的。
 * ===================================================================== */

static bool mpuInit() {
  uint8_t who = mpuRead8(REG_WHO_AM_I);
  Serial.printf("[MPU] WHO_AM_I = 0x%02X (expect 0x68)\n", who);
  if (who != 0x68) return false;

  // デバイスリセット
  mpuWrite(REG_PWR_MGMT_1, 0x80);
  delay(100);
  mpuWrite(REG_SIGNAL_PATH_RST, 0x07);
  delay(100);

  // クロック源 CLKSEL = 1（Gyro X PLL）+ スリープ解除   ... 第10.5章
  mpuWrite(REG_PWR_MGMT_1, CLKSEL & 0x07);
  delay(10);

  // DLPF_CFG = 3（44 Hz）                                ... 第10.3章
  mpuWrite(REG_CONFIG, DLPF_CFG & 0x07);

  // SMPLRT_DIV = 7 → fs = 125 Hz                         ... 第58.1章
  mpuWrite(REG_SMPLRT_DIV, SMPLRT_DIV_VAL);

  // ジャイロ ±500 dps
  mpuWrite(REG_GYRO_CONFIG, (FS_SEL & 0x03) << 3);

  // 加速度 AFS_SEL = 1（±4 g）                           ... 第10.2章
  mpuWrite(REG_ACCEL_CONFIG, (AFS_SEL & 0x03) << 3);

  delay(50);

  // --- 読み戻し検証 ---
  uint8_t r_pwr  = mpuRead8(REG_PWR_MGMT_1);
  uint8_t r_cfg  = mpuRead8(REG_CONFIG);
  uint8_t r_div  = mpuRead8(REG_SMPLRT_DIV);
  uint8_t r_acc  = mpuRead8(REG_ACCEL_CONFIG);
  uint8_t r_gyr  = mpuRead8(REG_GYRO_CONFIG);

  Serial.printf("[MPU] PWR_MGMT_1=0x%02X CONFIG=0x%02X SMPLRT_DIV=%u "
                "ACCEL_CONFIG=0x%02X GYRO_CONFIG=0x%02X\n",
                r_pwr, r_cfg, r_div, r_acc, r_gyr);

  bool ok = ((r_pwr & 0x07) == CLKSEL)
         && ((r_cfg & 0x07) == DLPF_CFG)
         && (r_div == SMPLRT_DIV_VAL)
         && (((r_acc >> 3) & 0x03) == AFS_SEL)
         && (((r_gyr >> 3) & 0x03) == FS_SEL);

  if (!ok) Serial.println("[MPU] 設定の読み戻しが一致しません");
  return ok;
}

// FIFO を停止・クリアしてから再開する。戻り値はリセット完了時刻(µs)。
// この時刻がサンプル通し番号 0 の基準時刻になる（第11.3章）。
static uint64_t mpuFifoReset() {
  mpuWrite(REG_USER_CTRL, 0x00);        // FIFO 停止
  mpuWrite(REG_FIFO_EN,   0x00);        // 何も溜めない
  mpuWrite(REG_USER_CTRL, 0x04);        // FIFO_RESET
  delay(2);
  mpuWrite(REG_FIFO_EN,   0x78);        // XG,YG,ZG,ACCEL を FIFO へ
  mpuWrite(REG_USER_CTRL, 0x40);        // FIFO 有効
  mpuRead8(REG_INT_STATUS);             // オーバーフローフラグをクリア
  return (uint64_t)esp_timer_get_time();
}

/* =====================================================================
 *  タイマ割り込み
 *  ISR では I2C を触れないため、セマフォを渡すだけにする。
 * ===================================================================== */

static hw_timer_t *gTimer = nullptr;

void IRAM_ATTR onFifoTimer() {
  BaseType_t hpw = pdFALSE;
  xSemaphoreGiveFromISR(semTimer, &hpw);
  if (hpw) portYIELD_FROM_ISR();
}

static void timerSetup() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  gTimer = timerBegin(1000000);                     // 1 MHz
  timerAttachInterrupt(gTimer, &onFifoTimer);
  timerAlarm(gTimer, FIFO_READ_PERIOD_US, true, 0);
#else
  gTimer = timerBegin(0, 80, true);                 // 80 MHz / 80 = 1 MHz
  timerAttachInterrupt(gTimer, &onFifoTimer, true);
  timerAlarmWrite(gTimer, FIFO_READ_PERIOD_US, true);
  timerAlarmEnable(gTimer);
#endif
}

/* =====================================================================
 *  取得タスク（Core 1）
 *  タイマ通知ごとに FIFO をまとめて読み出し、キューへ流す。
 * ===================================================================== */

static uint64_t gLastReadUs = 0;

static void drainFifo() {
  uint64_t now = (uint64_t)esp_timer_get_time();

  // --- 読み出し間隔の記録（第11.2章 read_interval_max_ms） ---
  if (gLastReadUs != 0) {
    uint32_t itv_ms = (uint32_t)((now - gLastReadUs) / 1000ULL);
    if (itv_ms > gStats.read_interval_max_ms) gStats.read_interval_max_ms = itv_ms;
  }
  gLastReadUs = now;

  // --- オーバーフロー検査（第11.2章） ---
  // 発生した時点でサンプルの連続性が失われ、時刻再構成が成立しない。
  uint8_t intStatus = mpuRead8(REG_INT_STATUS);
  if (intStatus & 0x10) {
    gStats.fifo_overflow_count++;
    Serial.println("[FIFO] OVERFLOW 発生 — この走行は受入基準を満たしません");
    mpuFifoReset();
    return;
  }

  // --- FIFO 残量 ---
  uint8_t cbuf[2];
  if (!mpuRead(REG_FIFO_COUNT_H, cbuf, 2)) return;
  uint16_t fifoBytes = ((uint16_t)cbuf[0] << 8) | cbuf[1];
  uint16_t nSamples  = fifoBytes / 12;
  if (nSamples == 0) return;
  if (nSamples > gStats.max_fifo_samples) gStats.max_fifo_samples = nSamples;
  gStats.read_count++;

  // --- バーストリード ---
  // ESP32 の Wire バッファは 128 byte。12の倍数で切って 120 byte ずつ読む。
  uint8_t buf[120];
  uint16_t remaining = nSamples;
  uint16_t readTotal = 0;

  while (remaining > 0) {
    uint16_t chunk = (remaining > 10) ? 10 : remaining;   // 10サンプル = 120 byte
    if (!mpuRead(REG_FIFO_R_W, buf, chunk * 12)) break;

    for (uint16_t s = 0; s < chunk; s++) {
      const uint8_t *p = &buf[s * 12];
      RawSample smp;
      smp.ax = (int16_t)((p[0] << 8) | p[1]);
      smp.ay = (int16_t)((p[2] << 8) | p[3]);
      smp.az = (int16_t)((p[4] << 8) | p[5]);
      smp.gx = (int16_t)((p[6] << 8) | p[7]);
      smp.gy = (int16_t)((p[8] << 8) | p[9]);
      smp.gz = (int16_t)((p[10] << 8) | p[11]);

      if (gState == ST_CALIB) {
        // 静止1秒の3軸平均（第12.2章）
        gCalibSum[0] += smp.ax;
        gCalibSum[1] += smp.ay;
        gCalibSum[2] += smp.az;
        gCalibN++;
      } else if (gState == ST_LOGGING) {
        // 飽和カウント（第11.2章 clip_rate）
        if (abs(smp.ax) >= CLIP_THRESHOLD ||
            abs(smp.ay) >= CLIP_THRESHOLD ||
            abs(smp.az) >= CLIP_THRESHOLD) {
          gStats.n_clip++;
        }
        if (xQueueSend(qSamples, &smp, 0) == pdTRUE) {
          gStats.n_samples++;
        } else {
          gStats.n_dropped_queue++;   // 書き込みが追いつかなかった
        }
      }
    }
    readTotal += chunk;
    remaining -= chunk;
  }

  if (gState == ST_LOGGING) {
    gStats.t_end_us = now;
    // FIFO読み出しログ（第44.1章）
    if (fFifo) {
      char line[96];
      int n = snprintf(line, sizeof(line), "%lu,%llu,%u,%u\n",
                       (unsigned long)gStats.read_count,
                       (unsigned long long)now,
                       (unsigned)readTotal,
                       (unsigned)fifoBytes);
      fFifo.write((const uint8_t *)line, n);
    }
  }
}

static void taskSampler(void *arg) {
  for (;;) {
    if (xSemaphoreTake(semTimer, portMAX_DELAY) == pdTRUE) {
      if (gState == ST_CALIB || gState == ST_LOGGING) {
        drainFifo();
      }
    }
  }
}

/* =====================================================================
 *  書き込みタスク（Core 0）
 *  キューから取り出し、テキスト化してまとめて SD へ書く。
 *  1サンプルごとの open/close は行わない（第10.4章）。
 * ===================================================================== */

static char     gWriteBuf[WRITE_BUF_SIZE];
static size_t   gWriteLen = 0;
static uint32_t gSampleIndex = 0;
static uint64_t gLastFlushUs = 0;

static void flushWriteBuf(bool sync) {
  if (gWriteLen > 0 && fData) {
    fData.write((const uint8_t *)gWriteBuf, gWriteLen);
    gWriteLen = 0;
  }
  if (sync && fData) fData.flush();
  if (sync && fFifo) fFifo.flush();
}

static void taskWriter(void *arg) {
  RawSample smp;
  for (;;) {
    if (xQueueReceive(qSamples, &smp, pdMS_TO_TICKS(200)) == pdTRUE) {

      // 物理量へ換算（第10.6章：単位を混在させない）
      float ax = smp.ax / ACCEL_LSB_PER_G * GRAVITY;
      float ay = smp.ay / ACCEL_LSB_PER_G * GRAVITY;
      float az = smp.az / ACCEL_LSB_PER_G * GRAVITY;
      float gx = smp.gx / GYRO_LSB_PER_DPS * DEG2RAD;
      float gy = smp.gy / GYRO_LSB_PER_DPS * DEG2RAD;
      float gz = smp.gz / GYRO_LSB_PER_DPS * DEG2RAD;

      // t_us は公称 fs による暫定値。
      // 最終的な時刻は PC 側で fs_effective により再構成する（第11.3章）。
      uint64_t t_us = (uint64_t)(gSampleIndex * (1000000.0 / FS_NOMINAL_HZ));

      int n = snprintf(&gWriteBuf[gWriteLen], WRITE_BUF_SIZE - gWriteLen,
                       "%lu,%llu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                       (unsigned long)gSampleIndex,
                       (unsigned long long)t_us,
                       ax, ay, az, gx, gy, gz);
      if (n > 0) gWriteLen += n;
      gSampleIndex++;

      if (gWriteLen >= WRITE_FLUSH_BYTES) flushWriteBuf(false);
    }

    // 2秒に1回は確実に書き出す（電源断時の損失を限定する）
    uint64_t now = (uint64_t)esp_timer_get_time();
    if (now - gLastFlushUs > 2000000ULL) {
      gLastFlushUs = now;
      flushWriteBuf(true);
    }
  }
}

/* =====================================================================
 *  ファイル管理
 * ===================================================================== */

static void ensureDir(const char *path) {
  if (!SD.exists(path)) SD.mkdir(path);
}

// 既存 run_id を上書きしない（旧コードの致命的な欠陥への対処）
static void resolveRunId(char *runId, size_t cap) {
  char path[64];
  snprintf(path, sizeof(path), "/data/%s.csv", runId);
  if (!SD.exists(path)) return;

  Serial.printf("[SD] %s は既に存在します。連番を付与します\n", path);
  char base[32];
  strncpy(base, runId, sizeof(base) - 1);
  base[sizeof(base) - 1] = '\0';
  for (int i = 2; i < 100; i++) {
    snprintf(runId, cap, "%s_dup%d", base, i);
    snprintf(path, sizeof(path), "/data/%s.csv", runId);
    if (!SD.exists(path)) return;
  }
}

static bool openRunFiles() {
  ensureDir("/data");
  ensureDir("/fifo");
  ensureDir("/meta");
  resolveRunId(gRunId, sizeof(gRunId));

  char path[64];
  snprintf(path, sizeof(path), "/data/%s.csv", gRunId);
  fData = SD.open(path, FILE_APPEND);
  if (!fData) { Serial.printf("[SD] %s を開けません\n", path); return false; }

  // ヘッダのコメント行（第44.1章）
  char hdr[512];
  int n = snprintf(hdr, sizeof(hdr),
    "# unit: m/s2, rad/s\n"
    "# time_unit: us\n"
    "# time_note: t_us is nominal; reconstruct with fs_effective (spec 11.3)\n"
    "# fs_nominal_hz: %.3f\n"
    "# smplrt_div: %d\n"
    "# afs_sel: %d (+-4g)\n"
    "# dlpf_cfg: %d (44Hz)\n"
    "# fs_sel: %d (+-500dps)\n"
    "# clksel: %d (PLL Gyro X)\n"
    "# acquisition: fifo_burst\n"
    "# fifo_read_period_ms: %lu\n"
    "# run_id: %s\n"
    "sample_index,t_us,ax,ay,az,gx,gy,gz\n",
    FS_NOMINAL_HZ, SMPLRT_DIV_VAL, AFS_SEL, DLPF_CFG, FS_SEL, CLKSEL,
    (unsigned long)(FIFO_READ_PERIOD_US / 1000), gRunId);
  fData.write((const uint8_t *)hdr, n);

  snprintf(path, sizeof(path), "/fifo/%s_fifo.csv", gRunId);
  fFifo = SD.open(path, FILE_APPEND);
  if (fFifo) fFifo.println("read_id,read_timestamp_us,n_samples_read,fifo_count_before_read");

  return true;
}

/* =====================================================================
 *  メタデータ書き出し（第11.2章・第44.2章）
 * ===================================================================== */

static void writeMetadata() {
  double elapsed_s = (gStats.t_end_us > gStats.t_start_us)
                   ? (gStats.t_end_us - gStats.t_start_us) / 1e6 : 0.0;
  double fs_eff    = (elapsed_s > 0) ? gStats.n_samples / elapsed_s : 0.0;
  double fs_err    = (fs_eff > 0) ? (fs_eff - FS_NOMINAL_HZ) / FS_NOMINAL_HZ * 100.0 : 0.0;
  double clip_rate = (gStats.n_samples > 0)
                   ? 100.0 * gStats.n_clip / gStats.n_samples : 0.0;

  // 受入基準の自動判定（第11.2章）
  bool ok_ovf   = (gStats.fifo_overflow_count == 0);
  bool ok_drop  = (gStats.n_dropped_queue == 0);
  bool ok_fs    = (fabs(fs_err) <= 2.0);
  bool ok_read  = (gStats.read_interval_max_ms <= READ_INTERVAL_LIMIT_MS);
  bool ok_clip  = (clip_rate <= 0.01);
  bool ok_tilt  = (gGravity.valid && fabs(gGravity.tilt_deg) <= 15.0);
  bool accepted = ok_ovf && ok_drop && ok_fs && ok_read && ok_clip;

  char path[64];
  snprintf(path, sizeof(path), "/meta/%s_meta.csv", gRunId);
  File fm = SD.open(path, FILE_WRITE);   // メタは1走行1ファイルなので上書きで可
  if (!fm) { Serial.println("[SD] メタデータを書けません"); return; }

  fm.println("key,value");
  fm.printf("run_id,%s\n", gRunId);
  fm.printf("acquisition,fifo_burst\n");
  fm.printf("fs_nominal_hz,%.3f\n", FS_NOMINAL_HZ);
  fm.printf("fs_effective_hz,%.4f\n", fs_eff);
  fm.printf("fs_error_percent,%.3f\n", fs_err);
  fm.printf("smplrt_div,%d\n", SMPLRT_DIV_VAL);
  fm.printf("afs_sel,%d\n", AFS_SEL);
  fm.printf("dlpf_cfg,%d\n", DLPF_CFG);
  fm.printf("clksel,%d\n", CLKSEL);
  fm.printf("t_start_us,%llu\n", (unsigned long long)gStats.t_start_us);
  fm.printf("t_end_us,%llu\n",   (unsigned long long)gStats.t_end_us);
  fm.printf("duration_s,%.3f\n", elapsed_s);
  fm.printf("n_samples,%lu\n",   (unsigned long)gStats.n_samples);
  fm.printf("fifo_overflow_count,%lu\n", (unsigned long)gStats.fifo_overflow_count);
  fm.printf("n_dropped,%lu\n",   (unsigned long)gStats.n_dropped_queue);
  fm.printf("read_count,%lu\n",  (unsigned long)gStats.read_count);
  fm.printf("read_interval_max_ms,%lu\n", (unsigned long)gStats.read_interval_max_ms);
  fm.printf("max_fifo_samples,%lu\n", (unsigned long)gStats.max_fifo_samples);
  fm.printf("fifo_capacity_samples,%d\n", FIFO_CAPACITY_SAMPLES);
  fm.printf("n_clip,%lu\n",      (unsigned long)gStats.n_clip);
  fm.printf("clip_rate_percent,%.4f\n", clip_rate);
  fm.printf("gravity_x,%.4f\n",  gGravity.gx);
  fm.printf("gravity_y,%.4f\n",  gGravity.gy);
  fm.printf("gravity_z,%.4f\n",  gGravity.gz);
  fm.printf("gravity_norm,%.4f\n", gGravity.norm);
  fm.printf("tilt_angle_deg,%.2f\n", gGravity.tilt_deg);
  fm.printf("calib_n_samples,%lu\n", (unsigned long)gGravity.n);

  // 受入判定
  fm.printf("accept_overflow,%s\n", ok_ovf  ? "PASS" : "FAIL");
  fm.printf("accept_dropped,%s\n",  ok_drop ? "PASS" : "FAIL");
  fm.printf("accept_fs,%s\n",       ok_fs   ? "PASS" : "FAIL");
  fm.printf("accept_read_interval,%s\n", ok_read ? "PASS" : "FAIL");
  fm.printf("accept_clip,%s\n",     ok_clip ? "PASS" : "FAIL");
  fm.printf("accept_tilt,%s\n",     ok_tilt ? "PASS" : "FAIL");
  fm.printf("excluded,%s\n",        accepted ? "false" : "true");
  fm.printf("excluded_reason,%s\n",
            accepted ? "" :
            (!ok_ovf  ? "fifo_overflow" :
             !ok_drop ? "sample_dropped" :
             !ok_fs   ? "fs_out_of_range" :
             !ok_read ? "read_interval_exceeded" : "clip_rate_exceeded"));

  // 以下は実験者が手入力する項目（第44.2章）
  fm.println("# --- 以下は手入力 ---");
  fm.println("run_order,");
  fm.println("session_id,");
  fm.println("vehicle_id,");
  fm.println("rider_id,");
  fm.println("sensor_id,");
  fm.println("tire_id,");
  fm.println("tire_run_count,");
  fm.println("tire_condition,");
  fm.println("tire_state,");
  fm.println("pressure_kpa,");
  fm.println("surface_type,");
  fm.println("sensor_position,");
  fm.println("mount_reference_id,");
  fm.println("distance_m,200");
  fm.println("rider_weight_kg,");
  fm.println("temperature,");
  fm.println("weather,");
  fm.println("notes,");
  fm.close();

  // シリアル・BLE への要約
  char sum[256];
  snprintf(sum, sizeof(sum),
    "%s n=%lu dur=%.1fs fs_eff=%.2fHz(%+.2f%%) ovf=%lu drop=%lu "
    "clip=%.4f%% read_max=%lums tilt=%.1fdeg -> %s",
    gRunId, (unsigned long)gStats.n_samples, elapsed_s, fs_eff, fs_err,
    (unsigned long)gStats.fifo_overflow_count,
    (unsigned long)gStats.n_dropped_queue, clip_rate,
    (unsigned long)gStats.read_interval_max_ms, gGravity.tilt_deg,
    accepted ? "ACCEPT" : "REJECT");
  Serial.println(sum);
  if (pStatusChar) { pStatusChar->setValue(sum); pStatusChar->notify(); }
}

/* =====================================================================
 *  測定の開始・終了
 * ===================================================================== */

static void startCalibration(const char *runId) {
  if (gState != ST_IDLE) { Serial.println("[CMD] 測定中です"); return; }

  strncpy(gRunId, runId, sizeof(gRunId) - 1);
  gRunId[sizeof(gRunId) - 1] = '\0';

  memset(&gStats, 0, sizeof(gStats));
  memset(&gGravity, 0, sizeof(gGravity));
  gCalibSum[0] = gCalibSum[1] = gCalibSum[2] = 0;
  gCalibN = 0;
  gSampleIndex = 0;
  gWriteLen = 0;
  gLastReadUs = 0;
  xQueueReset(qSamples);

  if (!openRunFiles()) return;

  mpuFifoReset();
  gCalibStartUs = (uint64_t)esp_timer_get_time();
  gState = ST_CALIB;
  Serial.printf("[CMD] %s : 静止キャリブレーション開始（%d ms 動かさないでください）\n",
                gRunId, CALIB_DURATION_MS);
}

static void finishCalibration() {
  if (gCalibN < 32) {
    Serial.println("[CALIB] サンプル不足。取得系を確認してください");
    gGravity.valid = false;
  } else {
    float mx = (float)gCalibSum[0] / gCalibN / ACCEL_LSB_PER_G * GRAVITY;
    float my = (float)gCalibSum[1] / gCalibN / ACCEL_LSB_PER_G * GRAVITY;
    float mz = (float)gCalibSum[2] / gCalibN / ACCEL_LSB_PER_G * GRAVITY;
    float nn = sqrtf(mx * mx + my * my + mz * mz);
    gGravity.gx = mx; gGravity.gy = my; gGravity.gz = mz;
    gGravity.norm = nn;
    gGravity.tilt_deg = (nn > 0.1f)
                      ? acosf(fabsf(mz) / nn) * 180.0f / PI : 0.0f;
    gGravity.n = gCalibN;
    gGravity.valid = true;

    Serial.printf("[CALIB] g=(%.3f, %.3f, %.3f) |g|=%.3f tilt=%.1f deg  %s\n",
                  mx, my, mz, nn, gGravity.tilt_deg,
                  (gGravity.tilt_deg <= 15.0f) ? "OK" : "*** 15度超過（第12章違反）***");
  }

  // 記録本体の開始。ここでの FIFO リセット時刻がサンプル0の基準（第11.3章）
  gStats.t_start_us = mpuFifoReset();
  gStats.t_end_us   = gStats.t_start_us;
  gLastReadUs = 0;
  gState = ST_LOGGING;
  Serial.println("[CMD] 測定開始");
}

static void stopLogging() {
  if (gState != ST_LOGGING) return;
  gState = ST_FINALIZE;

  // キューに残ったサンプルを書き切る
  delay(300);
  uint32_t guard = 0;
  while (uxQueueMessagesWaiting(qSamples) > 0 && guard++ < 200) delay(20);

  flushWriteBuf(true);
  if (fData) fData.close();
  if (fFifo) fFifo.close();

  writeMetadata();
  gState = ST_IDLE;
}

/* =====================================================================
 *  BLE
 *  コールバックではキューに積むだけ。SD/I2C はここで触らない。
 * ===================================================================== */

struct Command { char text[40]; };

class CmdCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    auto raw = c->getValue();          // core 2.x: std::string / 3.x: String
    Command cmd;
    strncpy(cmd.text, raw.c_str(), sizeof(cmd.text) - 1);
    cmd.text[sizeof(cmd.text) - 1] = '\0';
    xQueueSend(qCommands, &cmd, 0);
  }
};

static void handleCommand(const char *text) {
  Serial.printf("[BLE] %s\n", text);

  if (strncmp(text, "START", 5) == 0) {
    char runId[32];
    const char *colon = strchr(text, ':');
    if (colon && strlen(colon + 1) > 0) {
      strncpy(runId, colon + 1, sizeof(runId) - 1);
      runId[sizeof(runId) - 1] = '\0';
    } else {
      static uint32_t autoNo = 0;
      snprintf(runId, sizeof(runId), "RUN_%05lu", (unsigned long)++autoNo);
    }
    startCalibration(runId);

  } else if (strncmp(text, "STOP", 4) == 0) {
    stopLogging();
  }
}

/* =====================================================================
 *  setup / loop
 * ===================================================================== */

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Vibration Logger (SPEC v3) ===");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // I2C 400 kHz（FIFOバースト読み出しに必須）
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!mpuInit()) {
    Serial.println("[FATAL] MPU6050 初期化失敗");
    while (1) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }
  }

  if (!SD.begin(SD_CS, SPI, 20000000)) {
    Serial.println("[FATAL] SDカード初期化失敗");
    while (1) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(500); }
  }
  Serial.println("[SD] OK");

  qSamples  = xQueueCreate(SAMPLE_QUEUE_LEN, sizeof(RawSample));
  qCommands = xQueueCreate(8, sizeof(Command));
  semTimer  = xSemaphoreCreateBinary();
  if (!qSamples || !qCommands || !semTimer) {
    Serial.println("[FATAL] メモリ確保失敗");
    while (1) delay(1000);
  }

  // 取得は Core 1、書き込みは Core 0（BLE/Wi-Fiと分離する）
  xTaskCreatePinnedToCore(taskSampler, "sampler", 4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(taskWriter,  "writer",  8192, nullptr, 3, nullptr, 0);

  timerSetup();

  // BLE
  BLEDevice::init("ESP32_Vibration");
  BLEServer *server = BLEDevice::createServer();
  BLEService *service = server->createService(SERVICE_UUID);

  BLECharacteristic *cmdChar = service->createCharacteristic(
      CHAR_CMD_UUID, BLECharacteristic::PROPERTY_WRITE);
  cmdChar->setCallbacks(new CmdCallback());

  pStatusChar = service->createCharacteristic(
      CHAR_STATUS_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pStatusChar->setValue("IDLE");

  service->start();
  BLEDevice::getAdvertising()->start();
  Serial.println("[BLE] Ready  ->  START:S_FLAT_001 / STOP");
}

void loop() {
  // コマンド処理（BLEコールバックからキュー経由で受ける）
  Command cmd;
  if (xQueueReceive(qCommands, &cmd, 0) == pdTRUE) {
    handleCommand(cmd.text);
  }

  // キャリブレーション終了判定
  if (gState == ST_CALIB &&
      (uint64_t)esp_timer_get_time() - gCalibStartUs >= CALIB_DURATION_MS * 1000ULL) {
    finishCalibration();
  }

  // LED : 記録中は点滅、キャリブ中は点灯、待機は消灯
  static uint32_t ledMs = 0;
  if (gState == ST_LOGGING) {
    if (millis() - ledMs > 300) { ledMs = millis(); digitalWrite(LED_PIN, !digitalRead(LED_PIN)); }
  } else if (gState == ST_CALIB) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  // 進捗表示は1秒に1回だけ（旧コードは毎サンプル出力してこれが律速していた）
  static uint32_t logMs = 0;
  if (gState == ST_LOGGING && millis() - logMs > 1000) {
    logMs = millis();
    double el = (gStats.t_end_us - gStats.t_start_us) / 1e6;
    Serial.printf("  t=%5.1fs n=%6lu fs=%6.2fHz q=%3u ovf=%lu clip=%lu\n",
                  el, (unsigned long)gStats.n_samples,
                  el > 0 ? gStats.n_samples / el : 0.0,
                  (unsigned)uxQueueMessagesWaiting(qSamples),
                  (unsigned long)gStats.fifo_overflow_count,
                  (unsigned long)gStats.n_clip);
  }

  delay(5);
}
