#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""讯飞离线语音合成（libmsc.so QTTS），接口对齐 tts_http.py 的在线版。

用法（与在线版 tts_http.py 完全一致）:
    python tts_offline.py "要合成的文本"

与在线版 tts_http.py 的区别:
    - 不联网、不走 WebSocket，纯本地 libmsc.so 合成
    - 离线 SDK 只用 APPID，不需要 API_KEY / API_SECRET（那两个仅在线 WebSocket 用）
    - 合成结果为 16k / 16bit / 单声道 PCM，用 aplay 播放

依赖（均在本包 speech_command 内）:
    lib/{arch}/libmsc.so          讯飞离线合成库
    config/AIUI/tts/xiaoyan.jet   发音人资源
    config/AIUI/tts/common.jet    公共资源
    bin/                          MSPLogin 的 work_dir
"""

import ctypes
import os
import sys
import time

APPID = "2697a716"  # src2 的 APPID
# 离线 SDK 只用 APPID，下面两个仅作对照（在线 WebSocket 才需要）
API_KEY = "424d575268594135073cbe862124f0b2"
API_SECRET = "NmU5Yjg0YzJmOWFhODE0OTQyYjQ4YmUy"

VOICE_NAME = "xiaoyan"
SAMPLE_RATE = 16000
SPEED = 50
VOLUME = 50
PITCH = 50
RDN = 2

MSP_SUCCESS = 0
MSP_TTS_FLAG_STILL_HAVE_DATA = 1
MSP_TTS_FLAG_DATA_END = 2

_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # speech_command
_LIB_DIR = os.path.join(_PKG_DIR, "lib")
_WORK_DIR = os.path.join(_PKG_DIR, "bin")
_TTS_RES_PATH = (
    "fo|" + os.path.join(_PKG_DIR, "config", "AIUI", "tts", VOICE_NAME + ".jet")
    + ";fo|" + os.path.join(_PKG_DIR, "config", "AIUI", "tts", "common.jet")
)


def find_msc_library():
    """按 CPU 架构自动探测 libmsc.so 路径。"""
    machine = os.uname().machine.lower() if hasattr(os, "uname") else ""
    if machine in ("x86_64", "amd64"):
        archs = ["x64", "x86", "arm64", "arm32"]
    elif machine in ("aarch64", "arm64"):
        archs = ["arm64", "arm32", "x64", "x86"]
    else:
        archs = ["arm64", "x64", "x86", "arm32"]

    for arch in archs:
        lib = os.path.join(_LIB_DIR, arch, "libmsc.so")
        if os.path.exists(lib):
            return lib
    raise RuntimeError("libmsc.so not found in " + _LIB_DIR)


class OfflineTts:
    """ctypes 封装 libmsc.so 的 QTTS 离线合成接口。"""

    def __init__(self):
        self.lib = ctypes.CDLL(find_msc_library(), mode=ctypes.RTLD_GLOBAL)
        self.logged_in = False
        self._bind_functions()

    def _bind_functions(self):
        self.lib.MSPLogin.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p]
        self.lib.MSPLogin.restype = ctypes.c_int
        self.lib.MSPLogout.argtypes = []
        self.lib.MSPLogout.restype = ctypes.c_int
        self.lib.QTTSSessionBegin.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_int)]
        self.lib.QTTSSessionBegin.restype = ctypes.c_char_p
        self.lib.QTTSTextPut.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint, ctypes.c_char_p]
        self.lib.QTTSTextPut.restype = ctypes.c_int
        self.lib.QTTSAudioGet.argtypes = [
            ctypes.c_char_p, ctypes.POINTER(ctypes.c_uint),
            ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
        ]
        self.lib.QTTSAudioGet.restype = ctypes.c_void_p
        self.lib.QTTSSessionEnd.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
        self.lib.QTTSSessionEnd.restype = ctypes.c_int

    def login(self):
        if self.logged_in:
            return
        login_params = "appid = {}, work_dir = {}".format(APPID, _WORK_DIR)
        ret = self.lib.MSPLogin(None, None, login_params.encode("utf-8"))
        if ret != MSP_SUCCESS:
            raise RuntimeError("MSPLogin failed, error code: {}".format(ret))
        self.logged_in = True

    def logout(self):
        if self.logged_in:
            self.lib.MSPLogout()
            self.logged_in = False

    def _session_params(self):
        return (
            "engine_type = local"
            ", voice_name = {}".format(VOICE_NAME)
            + ", text_encoding = UTF8"
            + ", tts_res_path = {}".format(_TTS_RES_PATH)
            + ", sample_rate = {}".format(SAMPLE_RATE)
            + ", speed = {}".format(SPEED)
            + ", volume = {}".format(VOLUME)
            + ", pitch = {}".format(PITCH)
            + ", rdn = {}".format(RDN)
        )

    def synthesize(self, text):
        """合成文本，返回 16k / 16bit / 单声道 PCM 字节。"""
        self.login()
        err_code = ctypes.c_int(MSP_SUCCESS)
        session_id = self.lib.QTTSSessionBegin(
            self._session_params().encode("utf-8"), ctypes.byref(err_code))
        if err_code.value != MSP_SUCCESS or not session_id:
            raise RuntimeError("QTTSSessionBegin failed, error code: {}".format(err_code.value))

        audio = bytearray()
        try:
            text_bytes = text.encode("utf-8")
            ret = self.lib.QTTSTextPut(session_id, text_bytes, len(text_bytes), None)
            if ret != MSP_SUCCESS:
                raise RuntimeError("QTTSTextPut failed, error code: {}".format(ret))

            while True:
                audio_len = ctypes.c_uint(0)
                synth_status = ctypes.c_int(MSP_TTS_FLAG_STILL_HAVE_DATA)
                data_ptr = self.lib.QTTSAudioGet(
                    session_id,
                    ctypes.byref(audio_len),
                    ctypes.byref(synth_status),
                    ctypes.byref(err_code),
                )
                if err_code.value != MSP_SUCCESS:
                    raise RuntimeError("QTTSAudioGet failed, error code: {}".format(err_code.value))
                if data_ptr and audio_len.value > 0:
                    audio += ctypes.string_at(data_ptr, audio_len.value)
                if synth_status.value == MSP_TTS_FLAG_DATA_END:
                    break
                time.sleep(0.01)
        finally:
            self.lib.QTTSSessionEnd(session_id, b"Normal")
        return bytes(audio)


_tts = None


def _get_tts():
    global _tts
    if _tts is None:
        _tts = OfflineTts()
    return _tts


def tts(text, output="/tmp/tts_out.pcm"):
    """离线合成并播放，接口对齐 tts_http.py 的 tts()。成功返回 0，失败返回 1。"""
    try:
        audio = _get_tts().synthesize(text)
        if not audio:
            print("TTS OFFLINE ERR: empty audio", file=sys.stderr)
            return 1
        with open(output, "wb") as f:
            f.write(audio)
        os.system("aplay -q -f S16_LE -r {} -c 1 {} 2>/dev/null".format(SAMPLE_RATE, output))
        return 0
    except Exception as e:
        print("TTS OFFLINE ERR: {}".format(e), file=sys.stderr)
        return 1


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: python tts_offline.py <text>", file=sys.stderr)
        sys.exit(1)
    sys.exit(tts(sys.argv[1]))
