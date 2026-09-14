import cv2


def center_crop_4_3(frame):
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("frame must be HxWx3")
    height, width = frame.shape[:2]
    target_width = int(round(height * 4.0 / 3.0))
    if width < target_width:
        raise ValueError("frame is narrower than 4:3")
    left = (width - target_width) // 2
    return frame[:, left:left + target_width].copy()


def transform_line_frame(frame, width, height, crop_mode):
    if crop_mode != "center_4_3":
        raise ValueError("unsupported crop_mode: %s" % crop_mode)
    if type(width) is not int or type(height) is not int or width <= 0 or height <= 0:
        raise ValueError("output dimensions must be positive integers")
    return cv2.resize(center_crop_4_3(frame), (width, height), interpolation=cv2.INTER_AREA)
