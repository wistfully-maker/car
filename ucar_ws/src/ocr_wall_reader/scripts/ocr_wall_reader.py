#!/usr/bin/python3
# -*- coding: UTF-8 -*-
"""
OCR wall-text recognition node.

Subscribes to a camera topic, calls OCR on demand, publishes the
full recognised text as a plain string to /ocr_wall_reader/result.

Trigger: /ocr_wall_reader/capture (any message → one OCR shot)
"""

import base64
import threading
import time

import cv2
import numpy as np
import requests


# ---------------------------------------------------------------------------
# API helpers
# ---------------------------------------------------------------------------

class OcrClient:
    """Thin wrapper around access-token and OCR endpoints."""

    TOKEN_URL = "https://aip.baidubce.com/oauth/2.0/token"
    OCR_URL = "https://aip.baidubce.com/rest/2.0/ocr/v1/accurate_basic"

    def __init__(self, api_key, secret_key, timeout=5.0):
        self._api_key = api_key
        self._secret_key = secret_key
        self._timeout = float(timeout)
        self._token = None
        self._token_expires = 0.0

    # -- token ---------------------------------------------------------------

    def ensure_token(self):
        if self._token is not None and time.time() < self._token_expires - 60:
            return
        self._refresh_token()

    def _refresh_token(self):
        resp = requests.get(
            self.TOKEN_URL,
            params={
                "grant_type": "client_credentials",
                "client_id": self._api_key,
                "client_secret": self._secret_key,
            },
            timeout=self._timeout,
        )
        resp.raise_for_status()
        body = resp.json()
        if "error" in body:
            raise RuntimeError(
                "Token error: {} — {}".format(
                    body.get("error"), body.get("error_description", "")
                )
            )
        self._token = body["access_token"]
        self._token_expires = time.time() + body.get("expires_in", 2592000)

    # -- OCR -----------------------------------------------------------------

    def recognize(self, image: np.ndarray) -> list:
        """Encode *image* (BGR numpy array) and call OCR.

        Returns a list of recognised text strings.
        """
        self.ensure_token()
        _, buffer = cv2.imencode(".jpg", image)
        b64 = base64.b64encode(buffer).decode("ascii")

        resp = requests.post(
            self.OCR_URL + "?access_token=" + self._token,
            data={"image": b64},
            headers={"Content-Type": "application/x-www-form-urlencoded"},
            timeout=self._timeout,
        )
        resp.raise_for_status()
        body = resp.json()
        if "error_code" in body:
            raise RuntimeError(
                "OCR error {}: {}".format(
                    body.get("error_code"), body.get("error_msg", "")
                )
            )
        return [item["words"] for item in body.get("words_result", [])]


# ---------------------------------------------------------------------------
# Pre-processing helpers
# ---------------------------------------------------------------------------

def preprocess_for_wall_text(image: np.ndarray) -> np.ndarray:
    """Light enhancement to improve wall-text OCR accuracy.

    - Converts to grayscale.
    - Applies CLAHE (contrast-limited adaptive histogram equalization).
    - Sharpens slightly with unsharp masking.
    - Returns a 3-channel BGR image.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    equalized = clahe.apply(gray)

    blur = cv2.GaussianBlur(equalized, (0, 0), 3.0)
    sharpened = cv2.addWeighted(equalized, 1.5, blur, -0.5, 0)

    return cv2.cvtColor(sharpened, cv2.COLOR_GRAY2BGR)


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

def main():
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    rospy.init_node("ocr_wall_reader")

    # ---- parameters --------------------------------------------------------
    api_key = rospy.get_param("~api_key", "")
    secret_key = rospy.get_param("~secret_key", "")
    image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
    preprocess = rospy.get_param("~preprocess", True)
    request_timeout = rospy.get_param("~request_timeout", 5.0)
    flip_code = rospy.get_param("~flip_code", 0)   # 0=none, 1=horizontal, -1=both

    if not api_key or not secret_key:
        rospy.logfatal(
            "OCR node requires ~api_key and ~secret_key parameters."
        )
        return

    # ---- state -------------------------------------------------------------
    bridge = CvBridge()
    lock = threading.Lock()
    latest_frame = None

    # ---- client ------------------------------------------------------------
    try:
        client = OcrClient(api_key, secret_key, timeout=request_timeout)
        client.ensure_token()
        rospy.loginfo("OCR token OK")
    except Exception as exc:
        rospy.logerr("Token failed: %s", exc)
        return

    # ---- publisher ---------------------------------------------------------
    result_pub = rospy.Publisher("/ocr_wall_reader/result", String, queue_size=10)

    # ---- image subscriber --------------------------------------------------

    def image_callback(msg: Image):
        nonlocal latest_frame
        try:
            latest_frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    rospy.Subscriber(image_topic, Image, image_callback, queue_size=1)

    # ---- capture trigger ---------------------------------------------------

    def do_ocr():
        with lock:
            if latest_frame is None:
                rospy.logwarn("No camera frame yet.")
                return
            img = latest_frame.copy()

        try:
            if flip_code:
                img = cv2.flip(img, flip_code)

            if preprocess:
                img = preprocess_for_wall_text(img)

            words = client.recognize(img)
            text = "".join(words)

            rospy.loginfo("OCR: %s", text)
            result_pub.publish(String(data=text))

        except Exception as exc:
            rospy.logerr("OCR failed: %s", exc)

    def capture_callback(_msg):
        threading.Thread(target=do_ocr, daemon=True).start()

    rospy.Subscriber("/ocr_wall_reader/capture", String, capture_callback, queue_size=10)

    rospy.loginfo(
        "OCR ready.  Send to /ocr_wall_reader/capture to trigger.\n"
        "  Camera : %s\n"
        "  Result : /ocr_wall_reader/result",
        image_topic,
    )
    rospy.spin()


if __name__ == "__main__":
    main()

