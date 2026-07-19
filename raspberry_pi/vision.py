"""vision.py — shared camera + orange-cone detection for the Pi programs.

Used by cone_follower.py, cone_visitor.py and hsv_tuner.py so the camera
setup and HSV pipeline live in exactly one place. Tune HSV_LOW/HSV_HIGH with
hsv_tuner.py and paste the printed values here.
"""

import cv2
import numpy as np

# Camera device: a /dev/videoN index isn't stable (the Pi 4's onboard
# bcm2835 codec/ISP nodes and camera nodes can renumber across boots/
# replugs). Use the udev by-id symlink instead — find yours with
# `ls -l /dev/v4l/by-id/` (the "...-video-index0" entry is the capture
# node; "...-video-index1" is UVC metadata-only, not usable for capture).
CAMERA_DEVICE = "/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-720P_SN0001-video-index0"

# HSV bounds for orange, tuned with hsv_tuner.py. Wide H range (up to 67)
# risks catching yellow/green clutter if it shows up in frame later --
# retest with hsv_tuner.py if false positives appear outside the cone.
HSV_LOW = (0, 46, 239)
HSV_HIGH = (67, 171, 255)

FRAME_W, FRAME_H = 640, 480
MIN_CONE_AREA = 300        # px^2, rejects speckle

KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


def open_camera():
    cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)
    # MJPG is essential: raw YUYV at 720p won't fit through USB 2.0 at speed
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # always process the freshest frame
    if not cap.isOpened():
        raise RuntimeError(f"Camera not found at {CAMERA_DEVICE}")
    return cap


def cone_mask(frame, low=None, high=None):
    """HSV threshold + morphological cleanup. Returns the binary mask."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(low or HSV_LOW), np.array(high or HSV_HIGH))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)
    return mask


def detect_cones(frame):
    """Return list of dicts {bearing, height, area, box}, sorted largest-first.

    bearing: -1 (left edge) .. +1 (right edge); box: (x, y, w, h) in px.
    """
    mask = cone_mask(frame)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cones = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_CONE_AREA:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if h < w * 0.8:          # cones are taller than wide; reject flat blobs
            continue
        cx = x + w / 2
        cones.append({
            "bearing": (cx / FRAME_W) * 2 - 1,
            "height": h,
            "area": area,
            "box": (x, y, w, h),
        })
    cones.sort(key=lambda c: c["area"], reverse=True)
    return cones


def annotate(frame, cones, target=None, lines=()):
    """Draw detection boxes (target in red, others green) and status text.

    Modifies and returns `frame`.
    """
    for cone in cones:
        x, y, w, h = cone["box"]
        is_target = target is not None and cone["box"] == target["box"]
        color = (0, 0, 255) if is_target else (0, 200, 0)
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2 if is_target else 1)
        cv2.putText(frame, f"h={cone['height']}", (x, max(12, y - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
    for i, text in enumerate(lines):
        cv2.putText(frame, text, (8, 20 + 20 * i),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    return frame
