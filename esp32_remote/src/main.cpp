#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

/* ---------------- 可调参数 ---------------- */
static const char *AP_SSID = "omni-car";
static const char *AP_PASS = "12345678";

#define UART_TX_PIN   4
#define UART_RX_PIN   5
#define UART_BAUD     115200

#define WS_PORT       81
#define HTTP_PORT     80

#define TX_PERIOD_MS  20      /* 50 Hz */
#define LOST_MS       300     /* 收不到网页数据多久后归零 */

/* 调试开关：1 = 忽略手机，固定发送测试指令
 * 用来验证 ESP32 -> STM32 的链路是否正常。验完改回 0
 */
#define FORCE_TEST_CMD  0

/* ---------------- 状态 ---------------- */
static WebServer        http(HTTP_PORT);
static WebSocketsServer ws(WS_PORT);

static volatile int16_t  cmd_vx   = 0;   /* mm/s   */
static volatile int16_t  cmd_vy   = 0;   /* mm/s   */
static volatile int16_t  cmd_wz   = 0;   /* mrad/s */
static volatile uint8_t  cmd_flag = 0;   /* bit0 = 使能 */
static volatile uint32_t last_ws_ms = 0;

/* 调试：统计从手机收到的消息 */
static volatile uint32_t ws_msg_count = 0;
static char              ws_last_msg[40] = "";

/* ---------------- 网页 ---------------- */
static const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Omni Car</title>
<style>
  html,body{margin:0;height:100%;background:#111;color:#eee;
            font-family:-apple-system,"Segoe UI",sans-serif;overflow:hidden;
            display:flex;flex-direction:column;align-items:center;justify-content:center;
            -webkit-user-select:none;user-select:none;-webkit-tap-highlight-color:transparent}
  .row{display:flex;gap:30px}
  .box{display:flex;flex-direction:column;align-items:center}
  .stick{width:150px;height:150px;border-radius:50%;background:#1c1c1c;
         border:2px solid #333;position:relative;touch-action:none}
  .knob{width:58px;height:58px;border-radius:50%;background:#2e7d5b;
        position:absolute;left:46px;top:46px;pointer-events:none}
  .lbl{margin-top:10px;font-size:13px;color:#777}
  #arm{margin-top:24px;padding:12px 40px;border-radius:8px;border:2px solid #444;
       background:#222;color:#888;font-size:16px;font-weight:600;letter-spacing:2px}
  #arm.on{background:#7a1f1f;border-color:#c0392b;color:#fff}
  #st{margin-top:14px;font-size:13px}
  #st.ok{color:#4caf50} #st.no{color:#e57373}
  #num{margin-top:6px;font-size:12px;color:#555;font-variant-numeric:tabular-nums}
</style></head><body>

<div class="row">
  <div class="box"><div class="stick" id="s1"><div class="knob" id="k1"></div></div>
       <div class="lbl">平移</div></div>
  <div class="box"><div class="stick" id="s2"><div class="knob" id="k2"></div></div>
       <div class="lbl">转向</div></div>
</div>

<button id="arm">使能</button>
<div id="st" class="no">未连接</div>
<div id="num">vx 0   vy 0   wz 0</div>

<script>
const VMAX = 500;    /* mm/s   —— 改这里调最大速度 */
const WMAX = 3800;   /* mrad/s —— 改这里调最大转速 */
const DZ   = 0.06;   /* 摇杆死区 */

let vx = 0, vy = 0, wz = 0;
let armed = false;
let ws = null, wsOk = false;

const stEl  = document.getElementById('st');
const numEl = document.getElementById('num');
const armEl = document.getElementById('arm');

function dz(v){ return Math.abs(v) < DZ ? 0 : v; }

function bindStick(stickId, knobId, onMove){
  const s = document.getElementById(stickId);
  const k = document.getElementById(knobId);
  const R = 75, KR = 29, MAX = R - KR;
  let active = false;

  function place(nx, ny){
    k.style.left = (R - KR + nx * MAX) + 'px';
    k.style.top  = (R - KR + ny * MAX) + 'px';
    onMove(dz(nx), dz(ny));
  }

  function handle(e){
    const r = s.getBoundingClientRect();
    let dx = e.clientX - r.left - R;
    let dy = e.clientY - r.top  - R;
    const d = Math.hypot(dx, dy);
    if (d > MAX){ dx = dx / d * MAX; dy = dy / d * MAX; }
    place(dx / MAX, dy / MAX);
  }

  s.addEventListener('pointerdown', e => {
    active = true; s.setPointerCapture(e.pointerId); handle(e);
  });
  s.addEventListener('pointermove', e => { if (active) handle(e); });
  s.addEventListener('pointerup',   () => { active = false; place(0, 0); });
  s.addEventListener('pointercancel',() => { active = false; place(0, 0); });
}

bindStick('s1', 'k1', (nx, ny) => {
  vx = Math.round(-ny * VMAX);
  vy = Math.round(-nx * VMAX);
});

bindStick('s2', 'k2', (nx) => {
  wz = Math.round(-nx * WMAX);
});

armEl.addEventListener('click', () => {
  armed = !armed;
  armEl.textContent = armed ? '已使能' : '使能';
  armEl.className = armed ? 'on' : '';
  if (!armed) { vx = 0; vy = 0; wz = 0; }
});

function connect(){
  ws = new WebSocket('ws://' + location.hostname + ':81/');
  ws.onopen  = () => { wsOk = true;  stEl.textContent = '已连接'; stEl.className = 'ok'; };
  ws.onclose = () => { wsOk = false; armed = false;
                       armEl.textContent = '使能'; armEl.className = '';
                       stEl.textContent = '未连接'; stEl.className = 'no';
                       setTimeout(connect, 1000); };
  ws.onerror = () => { ws.close(); };
}
connect();

let txCount = 0, httpBusy = false;

setInterval(() => {
  numEl.textContent = 'vx ' + vx + '   vy ' + vy + '   wz ' + wz + '   tx ' + txCount;

  const e = armed ? 1 : 0;

  if (wsOk && ws && ws.readyState === 1){
    ws.send(vx + ',' + vy + ',' + wz + ',' + e);
    txCount++;
  } else if (!httpBusy){
    httpBusy = true;
    fetch('/cmd?vx=' + vx + '&vy=' + vy + '&wz=' + wz + '&e=' + e)
      .then(r => { if (r.ok) txCount++; })
      .catch(() => {})
      .finally(() => { httpBusy = false; });
  }
}, 20);
</script>
</body></html>
)HTML";

/* ---------------- 帧格式 ----------------
 * [0] 0xAA  [1] 0x55
 * [2..3] vx  int16 LE  mm/s
 * [4..5] vy  int16 LE  mm/s
 * [6..7] wz  int16 LE  mrad/s
 * [8]    flags        bit0 = 使能
 * [9]    校验 = 前 9 字节累加取低 8 位
 */
#define FRAME_LEN 10

static void send_frame(void)
{
  uint8_t f[FRAME_LEN];
  int16_t vx = cmd_vx, vy = cmd_vy, wz = cmd_wz;

  f[0] = 0xAA;
  f[1] = 0x55;
  memcpy(&f[2], &vx, 2);
  memcpy(&f[4], &vy, 2);
  memcpy(&f[6], &wz, 2);
  f[8] = cmd_flag;

  uint8_t sum = 0;
  for (int i = 0; i < FRAME_LEN - 1; i++) sum += f[i];
  f[9] = sum;

  Serial1.write(f, FRAME_LEN);
}

/* ---------------- WebSocket ---------------- */
static void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t len)
{
  if (type != WStype_TEXT) return;
  if (len == 0 || len >= 48) return;

  char msg[48];
  memcpy(msg, payload, len);
  msg[len] = '\0';

  ws_msg_count++;
  memcpy(ws_last_msg, msg, len);
  ws_last_msg[len] = '\0';

  int a = 0, b = 0, c = 0, d = 0;
  if (sscanf(msg, "%d,%d,%d,%d", &a, &b, &c, &d) != 4) return;

  if (a >  1500) a =  1500;
  if (a < -1500) a = -1500;
  if (b >  1500) b =  1500;
  if (b < -1500) b = -1500;
  if (c >  4000) c =  4000;
  if (c < -4000) c = -4000;

  cmd_vx   = (int16_t)a;
  cmd_vy   = (int16_t)b;
  cmd_wz   = (int16_t)c;
  cmd_flag = (uint8_t)(d & 1);
  last_ws_ms = millis();
}

/* ---------------- 初始化 ---------------- */
void setup()
{
  Serial.begin(115200);
  delay(2000);

  Serial.printf("\n\n=== BOOT ===\n");
  Serial.printf("chip   : %s rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("reset  : %d  (1=上电 4=崩溃 9=欠压)\n", (int)esp_reset_reason());
  Serial.printf("heap   : %u\n", (unsigned)ESP.getFreeHeap());
  Serial.printf("flash  : %u\n", (unsigned)ESP.getFlashChipSize());

  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.printf("uart1  : tx=%d rx=%d @%d\n", UART_TX_PIN, UART_RX_PIN, UART_BAUD);

  /* 射频自检：先扫一遍周围的 AP，验证接收链路 */
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(200);
  int n = WiFi.scanNetworks();
  Serial.printf("scan   : %d 个网络\n", n);
  for (int i = 0; i < n && i < 8; i++)
  {
    Serial.printf("   %-22s %4d dBm  ch%d\n",
                  WiFi.SSID(i).c_str(), (int)WiFi.RSSI(i), (int)WiFi.channel(i));
  }
  WiFi.scanDelete();

  /* 启动 AP
   * softAP(ssid, pass, channel, hidden, max_connection)
   *   channel 6      : 避开 1/11 拥挤，按现场实测改
   *   max_connection : 只允许 1 个客户端，减少协议栈负担
   */
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);                  /* 关闭省电，降低延迟 */
  WiFi.setTxPower(WIFI_POWER_11dBm);    /* 天线失配，降功率换稳定 */
  uint8_t ok = WiFi.softAP(AP_SSID, AP_PASS, 6, 0, 1);
  Serial.printf("softAP : %d  ssid=%s  ip=%s\n",
                ok, AP_SSID, WiFi.softAPIP().toString().c_str());
  Serial.printf("ap mac : %s  ch=%d\n",
                WiFi.softAPmacAddress().c_str(), (int)WiFi.channel());

  http.on("/", []() { http.send_P(200, "text/html", PAGE); });

  /* HTTP 兜底接口：WebSocket 不可用时用这个
   * /cmd?vx=..&vy=..&wz=..&e=..
   */
  http.on("/cmd", []() {
    int a = http.arg("vx").toInt();
    int b = http.arg("vy").toInt();
    int c = http.arg("wz").toInt();
    int d = http.arg("e").toInt();

    if (a >  1500) a =  1500;
    if (a < -1500) a = -1500;
    if (b >  1500) b =  1500;
    if (b < -1500) b = -1500;
    if (c >  4000) c =  4000;
    if (c < -4000) c = -4000;

    cmd_vx   = (int16_t)a;
    cmd_vy   = (int16_t)b;
    cmd_wz   = (int16_t)c;
    cmd_flag = (uint8_t)(d & 1);
    last_ws_ms = millis();

    http.send(200, "text/plain", "ok");
  });

  http.onNotFound([]() { http.send(404, "text/plain", "not found"); });
  http.begin();

  ws.begin();
  ws.onEvent(onWsEvent);

  Serial.printf("http   : %d  ws: %d\n", HTTP_PORT, WS_PORT);
  Serial.printf("=== READY ===\n");
}

void loop()
{
  http.handleClient();
  ws.loop();

#if FORCE_TEST_CMD
  /* 固定测试指令：不依赖手机，直接验证串口链路 */
  cmd_vx   = 300;
  cmd_vy   = 0;
  cmd_wz   = 0;
  cmd_flag = 1;
#else
  /* 网页断线：归零并撤销使能 */
  if (millis() - last_ws_ms > LOST_MS)
  {
    cmd_vx   = 0;
    cmd_vy   = 0;
    cmd_wz   = 0;
    cmd_flag = 0;
  }
#endif

  static uint32_t t_last = 0;
  if (millis() - t_last >= TX_PERIOD_MS)
  {
    t_last = millis();
    send_frame();
  }

  /* 每 3 秒报告一次状态：能看出手机数据有没有到 ESP32 */
  static uint32_t t_stat = 0;
  if (millis() - t_stat >= 3000)
  {
    t_stat = millis();
    Serial.printf("[ap] cli=%d msgs=%u last=%s | vx=%d vy=%d wz=%d en=%d\n",
                  (int)WiFi.softAPgetStationNum(),
                  (unsigned)ws_msg_count,
                  ws_last_msg,
                  cmd_vx, cmd_vy, cmd_wz, cmd_flag);
  }
}
