#!/usr/bin/env python3
"""讯飞在线语音合成 WebSocket API"""
import hashlib, hmac, base64, time, json, sys, os, struct
import urllib.request, urllib.parse

APPID = "2697a716"
API_KEY = "424d575268594135073cbe862124f0b2"
API_SECRET = "NmU5Yjg0YzJmOWFhODE0OTQyYjQ4YmUy"

# 尝试用 websocket-client 库 (pip install websocket-client)
# 如果没装，降级用 espeak
try:
    import websocket
    HAVE_WS = True
except ImportError:
    HAVE_WS = False

def tts_with_ws(text, output="/tmp/tts_out.pcm"):
    """WebSocket TTS"""
    import ssl

    # HMAC-SHA256 签名
    date_str = time.strftime("%a, %d %b %Y %H:%M:%S GMT", time.gmtime())
    signature_origin = f"host: ws-api.xfyun.cn\ndate: {date_str}\nGET /v2/tts HTTP/1.1"
    signature_sha = hmac.new(API_SECRET.encode(), signature_origin.encode(), hashlib.sha256).digest()
    signature = base64.b64encode(signature_sha).decode()
    authorization_origin = f'api_key="{API_KEY}", algorithm="hmac-sha256", headers="host date request-line", signature="{signature}"'
    authorization = base64.b64encode(authorization_origin.encode()).decode()

    url = f"wss://ws-api.xfyun.cn/v2/tts?authorization={urllib.parse.quote(authorization)}&date={urllib.parse.quote(date_str)}&host=ws-api.xfyun.cn"

    # 构建请求
    req_data = {
        "common": {"app_id": APPID},
        "business": {
            "aue": "raw",
            "auf": "audio/L16;rate=16000",
            "vcn": "xiaoyan",
            "speed": 50,
            "volume": 100,
            "pitch": 50,
            "tte": "utf8"
        },
        "data": {
            "status": 2,
            "text": base64.b64encode(text.encode("utf-8")).decode()
        }
    }

    ws = None
    try:
        ws = websocket.create_connection(url, timeout=20, sslopt={"cert_reqs": ssl.CERT_NONE})
        ws.send(json.dumps(req_data))

        audio_data = b""
        while True:
            msg = ws.recv()
            resp = json.loads(msg)
            if resp.get("code") != 0:
                print(f"TTS ERR: code={resp.get('code')} msg={resp.get('message','')}", file=sys.stderr)
                break
            audio_b64 = resp.get("data", {}).get("audio", "")
            if audio_b64:
                audio_data += base64.b64decode(audio_b64)
            if resp.get("data", {}).get("status") == 2:
                break

        if audio_data:
            with open(output, "wb") as f:
                f.write(audio_data)
            os.system(f"aplay -q -f S16_LE -r 16000 -c 1 {output} 2>/dev/null")
            return 0
        return 1
    except Exception as e:
        print(f"TTS WS ERR: {e}", file=sys.stderr)
        return 1
    finally:
        if ws:
            ws.close()

def tts(text, output="/tmp/tts_out.pcm"):
    if HAVE_WS:
        return tts_with_ws(text, output)
    else:
        # 降级方案
        print("TTS: websocket-client未安装, 用espeak", file=sys.stderr)
        os.system(f"espeak -v zh \"{text}\" --stdout 2>/dev/null | aplay -q 2>/dev/null")
        return 0

if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(1)
    sys.exit(tts(sys.argv[1]))

