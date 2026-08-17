#!/usr/bin/env python3
"""
小车实时画面调试工具 (HSV滑块版)
================================
浏览器打开 http://172.20.10.4:8000
拖动滑块实时调节HSV阈值，调好后终端输出最终参数
"""
import cv2
import numpy as np
import threading
import time
import os
import json
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

DEBUG_DIR = "/tmp/car_debug"
os.makedirs(DEBUG_DIR, exist_ok=True)

# ==================== 全局可调参数（浏览器滑块修改）====================
HSV = {"h_min": 0, "h_max": 95, "s_min": 0, "s_max": 23, "v_min": 237, "v_max": 255}
CUT_RATIO = 0.60
MIN_AREA = 40
PROCESS_EVERY_N = 5    # 每5帧处理1帧(30fps→6fps)，降低带宽
JPEG_QUALITY = 30      # 低画质JPEG，减小文件
SCALE = 0.5            # 画面缩放到50%，大幅降低传输量

hsv_lock = threading.Lock()
frame_count = 0
error_count = 0
latest_info_text = "初始化中..."
running = True

LOWER_WHITE = np.array([0, 0, 140])
UPPER_WHITE = np.array([40, 80, 255])

def update_hsv_bounds():
    global LOWER_WHITE, UPPER_WHITE
    with hsv_lock:
        LOWER_WHITE = np.array([HSV["h_min"], HSV["s_min"], HSV["v_min"]])
        UPPER_WHITE = np.array([HSV["h_max"], HSV["s_max"], HSV["v_max"]])

def process_frame(frame):
    global frame_count, error_count, latest_info_text
    try:
        update_hsv_bounds()
        h, w = frame.shape[:2]
        y0 = int(h * CUT_RATIO)
        roi = frame[y0:h, :]

        blur = cv2.GaussianBlur(roi, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_WHITE, UPPER_WHITE)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)   # 闭运算：填补白线内部缝隙
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)    # 开运算：消除小噪点

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            mask, connectivity=8)

        # 缩放以减小传输量
        roi_small = cv2.resize(roi, None, fx=SCALE, fy=SCALE)
        mask_small = cv2.resize(mask, None, fx=SCALE, fy=SCALE)
        result = roi_small.copy()

        cx_screen = roi_small.shape[1] // 2
        cv2.line(result, (cx_screen, 0), (cx_screen, result.shape[0]), (0, 255, 0), 1)

        cents = []
        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] > MIN_AREA:
                cx, cy = centroids[i]
                # 质心也要缩放
                cents.append((int(cx * SCALE), int(cy * SCALE),
                              int(stats[i, cv2.CC_STAT_AREA] * SCALE * SCALE)))

        cents.sort(key=lambda p: p[0])

        with hsv_lock:
            hsv_info = f"HSV:[{HSV['h_min']},{HSV['s_min']},{HSV['v_min']}]~[{HSV['h_max']},{HSV['s_max']},{HSV['v_max']}]"
        lines = [hsv_info, f"帧:{frame_count} 连通块:{len(cents)}"]

        if len(cents) >= 2:
            L, R = cents[0], cents[-1]
            cv2.circle(result, (L[0], L[1]), 6, (255, 0, 0), -1)
            cv2.circle(result, (R[0], R[1]), 6, (0, 0, 255), -1)
            cv2.putText(result, f"L{L[0]}", (L[0]+8, L[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
            cv2.putText(result, f"R{R[0]}", (R[0]+8, R[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

            if L[0] < cx_screen and R[0] > cx_screen:
                mx = int((L[0] + R[0]) / 2)
                cv2.circle(result, (mx, result.shape[0]//2), 6, (0, 255, 255), -1)
                cv2.line(result, (mx, 0), (mx, result.shape[0]), (0, 255, 255), 2)
                err = mx - cx_screen
                lines.append(f"✅ L={L[0]} R={R[0]} 误差={err}")
            else:
                lines.append(f"⚠ 位置异常 L={L[0]} R={R[0]}")
        elif len(cents) == 1:
            cx, cy, a = cents[0]
            side = "L" if cx < cx_screen else "R"
            color = (255, 0, 0) if cx < cx_screen else (0, 0, 255)
            cv2.circle(result, (cx, cy), 6, color, -1)
            lines.append(f"⚠ 仅{side}线 x={cx}")
        else:
            lines.append("❌ 未检测到白线")

        latest_info_text = "\n".join(lines)

        for i, line in enumerate(lines):
            cv2.putText(result, line, (5, 14 + i * 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)

        for name, img in [("original", roi_small), ("mask", mask_small), ("result", result)]:
            p = os.path.join(DEBUG_DIR, f"{name}.jpg")
            t = os.path.join(DEBUG_DIR, f"{name}.tmp")
            # ARM64 OpenCV可能缺JPEG编码器，用imencode兜底
            ok, buf = cv2.imencode('.jpg', img)
            if ok:
                with open(t, 'wb') as f:
                    f.write(buf.tobytes())
                os.rename(t, p)

    except Exception as e:
        error_count += 1
        if error_count <= 3:
            import traceback
            print(f"❌ 处理出错(#{error_count}): {e}")
            traceback.print_exc()


def camera_loop():
    global frame_count
    raw_count = 0
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("❌ 摄像头打不开!")
        return

    print("✅ 摄像头 OK")
    print("   浏览器打开 → http://10.234.15.42:8000")
    print("   (如果IP变了，用 hostname -I 查看)\n")

    last_report = time.time()
    while running:
        ret, frame = cap.read()
        if not ret:
            print("⚠ 断线重连...")
            cap.release()
            time.sleep(1)
            cap = cv2.VideoCapture(0)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            continue

        raw_count += 1
        if raw_count % PROCESS_EVERY_N != 0:
            continue

        frame_count += 1
        process_frame(frame)

        now = time.time()
        if now - last_report > 15:
            print(f"📊 {frame_count}帧 | {latest_info_text.split(chr(10))[0]}")
            last_report = now

    cap.release()


# ==================== HTTP 服务器 ====================
PLACEHOLDER = (
    b'\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01\x00\x00\x00\x01'
    b'\x08\x02\x00\x00\x00\x90wS\xde\x00\x00\x00\x0cIDATx\x9cc\xf8\x0f'
    b'\x00\x00\x01\x01\x00\x05\x18\xd8N\x00\x00\x00\x00IEND\xaeB`\x82')

class H(BaseHTTPRequestHandler):
    def do_GET(self):
        try:
            path = self.path.split('?')[0]
            if path == '/':
                self._html()
            elif path == '/set_hsv':
                self._handle_hsv()
            elif 'original' in path:
                self._file('original.jpg')
            elif 'mask' in path:
                self._file('mask.jpg')
            elif 'result' in path:
                self._file('result.jpg')
            elif 'info' in path:
                self.send_response(200)
                self.send_header('Content-type', 'text/plain; charset=utf-8')
                self.end_headers()
                self.wfile.write(latest_info_text.encode())
            elif 'get_hsv' in path:
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                with hsv_lock:
                    self.wfile.write(json.dumps(HSV).encode())
            else:
                self.send_response(404)
                self.end_headers()
        except:
            pass

    def _handle_hsv(self):
        """接收浏览器滑块发来的HSV更新"""
        try:
            qs = parse_qs(urlparse(self.path).query)
            changed = False
            for key in HSV:
                if key in qs:
                    val = int(qs[key][0])
                    if HSV[key] != val:
                        HSV[key] = val
                        changed = True
            if changed:
                # 只在参数真的变化时打印（滑块松开时）
                pass
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'ok')
        except:
            self.send_response(400)
            self.end_headers()

    def _file(self, name):
        p = os.path.join(DEBUG_DIR, name)
        if os.path.exists(p):
            with open(p, 'rb') as f:
                d = f.read()
            self.send_response(200)
            self.send_header('Content-type', 'image/jpeg')
            self.send_header('Content-Length', str(len(d)))
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(d)
        else:
            self.send_response(200)
            self.send_header('Content-type', 'image/png')
            self.send_header('Content-Length', str(len(PLACEHOLDER)))
            self.end_headers()
            self.wfile.write(PLACEHOLDER)

    def _html(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html; charset=utf-8')
        self.end_headers()
        self.wfile.write(HTML.encode())

    def log_message(self, format, *args):
        pass


HTML = r"""<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>小车HSV调试</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:monospace;background:#111;color:#eee;padding:6px;font-size:12px}
h1{font-size:14px;color:#0f0;margin:2px 0}
.controls{background:#1a1a1a;padding:8px;margin:4px 0;border-radius:4px}
.slider-row{display:flex;align-items:center;gap:6px;margin:3px 0}
.slider-row label{width:45px;text-align:right;color:#aaa;font-size:11px}
.slider-row input[type=range]{flex:1;max-width:200px}
.slider-row .val{width:35px;font-size:11px;color:#0f0;text-align:center}
.col{display:inline-block;width:32%;vertical-align:top;padding:0 4px}
.col h3{font-size:11px;color:#888;margin:2px 0}
.btn{background:#0a0;color:#fff;border:none;padding:4px 12px;border-radius:3px;
 cursor:pointer;font-size:12px;margin:2px}
.btn2{background:#555}
.grid{display:flex;gap:4px;flex-wrap:wrap}
.panel{flex:1;min-width:200px;background:#1a1a1a;padding:3px;border-radius:3px}
.panel h2{font-size:11px;color:#999;margin-bottom:2px}
.panel img{width:100%;border:1px solid #333}
#info{background:#1a2a1a;padding:5px;margin:3px 0;white-space:pre-wrap;
font-size:10px;border-left:2px solid #0f0;min-height:25px}
</style></head><body>
<h1>&#x1f697; 小车HSV实时标定</h1>
<div class="controls">
<div class="col"><h3>H (色相)</h3>
<div class="slider-row"><label>Min</label><input type="range" id="h_min" min="0" max="179" value="0" onchange="hsvChanged()"><span class="val" id="h_min_v">0</span></div>
<div class="slider-row"><label>Max</label><input type="range" id="h_max" min="0" max="179" value="40" onchange="hsvChanged()"><span class="val" id="h_max_v">40</span></div>
</div>
<div class="col"><h3>S (饱和度)</h3>
<div class="slider-row"><label>Min</label><input type="range" id="s_min" min="0" max="255" value="0" onchange="hsvChanged()"><span class="val" id="s_min_v">0</span></div>
<div class="slider-row"><label>Max</label><input type="range" id="s_max" min="0" max="255" value="80" onchange="hsvChanged()"><span class="val" id="s_max_v">80</span></div>
</div>
<div class="col"><h3>V (亮度) &#x2b50;关键</h3>
<div class="slider-row"><label>Min</label><input type="range" id="v_min" min="0" max="255" value="140" onchange="hsvChanged()"><span class="val" id="v_min_v">140</span></div>
<div class="slider-row"><label>Max</label><input type="range" id="v_max" min="0" max="255" value="255" onchange="hsvChanged()"><span class="val" id="v_max_v">255</span></div>
</div>
<button class="btn" onclick="printParams()">&#x1f4cb; 打印最终参数</button>
<button class="btn btn2" onclick="resetSliders()">重置</button>
</div>
<pre id="info">等待数据...</pre>
<div class="grid">
<div class="panel"><h2>&#x1f4f7; ROI原图</h2><img id="orig" src="/original.jpg"></div>
<div class="panel"><h2>&#x2b1c; Mask二值图</h2><img id="mask" src="/mask.jpg"></div>
<div class="panel"><h2>&#x1f3af; 检测结果</h2><img id="res" src="/result.jpg"></div>
</div>
<script>
var keys=['h_min','h_max','s_min','s_max','v_min','v_max'];
var timer=null;

function hsvChanged(){
  keys.forEach(function(k){
    document.getElementById(k+'_v').textContent=document.getElementById(k).value;
  });
  // 防抖：停止拖动200ms后才发送
  if(timer) clearTimeout(timer);
  timer=setTimeout(sendHSV,200);
}

function sendHSV(){
  var params=keys.map(function(k){return k+'='+document.getElementById(k).value}).join('&');
  fetch('/set_hsv?'+params).catch(function(){});
}

function printParams(){
  var h_min=document.getElementById('h_min').value;
  var h_max=document.getElementById('h_max').value;
  var s_min=document.getElementById('s_min').value;
  var s_max=document.getElementById('s_max').value;
  var v_min=document.getElementById('v_min').value;
  var v_max=document.getElementById('v_max').value;
  var txt='LOWER_WHITE = np.array(['+h_min+', '+s_min+', '+v_min+'])\n'+
          'UPPER_WHITE = np.array(['+h_max+', '+s_max+', '+v_max+'])';
  alert('复制下面两行到巡线代码中:\n\n'+txt);
  // 同时发到服务器，让终端也打印
  fetch('/set_hsv?'+params+'&print=1').catch(function(){});
  document.getElementById('info').textContent='参数已输出! '+txt.replace(/\n/g,' | ');
}

function resetSliders(){
  document.getElementById('h_min').value=0;  document.getElementById('h_max').value=40;
  document.getElementById('s_min').value=0;  document.getElementById('s_max').value=80;
  document.getElementById('v_min').value=140; document.getElementById('v_max').value=255;
  keys.forEach(function(k){
    document.getElementById(k+'_v').textContent=document.getElementById(k).value;
  });
  sendHSV();
}

// 画面刷新 - 800ms间隔降低带宽
setInterval(function(){
  var t=Date.now();
  document.getElementById('orig').src='/original.jpg?'+t;
  document.getElementById('mask').src='/mask.jpg?'+t;
  document.getElementById('res').src='/result.jpg?'+t;
  fetch('/info.txt?'+t).then(function(r){return r.text()}).then(function(t){
    document.getElementById('info').textContent=t;
  });
},800);
</script></body></html>"""


def start_http():
    server = HTTPServer(('0.0.0.0', 8000), H)
    server.serve_forever()


if __name__ == '__main__':
    t = threading.Thread(target=start_http, daemon=True)
    t.start()
    try:
        camera_loop()
    except KeyboardInterrupt:
        print("\n⏹ 退出")
        running = False
