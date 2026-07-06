"""Headless hand tracker for the XY stepping robot.

Captures a video stream, detects a single human hand, reduces it to one
point (the barycentric center of its landmarks), maps that point from image space
into the physical working area, and streams the position to the Arduino over
serial as ``MOVE <x_mm> <y_mm> <speed>`` lines.

Image -> plane mapping (no depth; camera is fixed above the plane):

    The full camera frame maps directly onto the working plane: the stream
    resolution (e.g. 1920x1080) is detected at startup, and wherever the hand's
    center lands within that resolution is scaled proportionally onto the
    configured plane size (``plane_*_cm``). A center that lands off-frame is
    clamped to the nearest plane edge. Output is millimetres with the origin at a
    corner, to match the firmware (see MachineConfig.h / stepsPerMM).
"""

import argparse
import math
import os
import time
from dataclasses import dataclass

import cv2
import mediapipe as mp
import numpy as np
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2

try:
    import serial
except ImportError:
    serial = None

BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
HandLandmarkerResult = mp.tasks.vision.HandLandmarkerResult
VisionRunningMode = mp.tasks.vision.RunningMode

MODEL_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "models/hand_landmarker.task"
)


@dataclass
class Config:
    camera_index: int = 0

    # Physical size of the reachable XY plane, in cm. The full camera frame is
    # mapped onto this plane, so the whole frame corresponds to the whole plane.
    plane_width_cm: float = 40.0
    plane_height_cm: float = 40.0

    # Flip an axis if the camera mounting orientation is mirrored relative to
    # the machine's own axes.
    invert_x: bool = False
    invert_y: bool = False

    serial_port: str = "/dev/ttyACM0"
    serial_baud: int = 115200
    move_speed: float = 200.0  # the "speed" field of the MOVE command (mm/s)

    min_send_interval_s: float = 0.05  # cap the serial rate (<= 20 Hz here)
    deadband_mm: float = 2.0  # skip sends smaller than this

    min_detection_confidence: float = 0.5
    min_tracking_confidence: float = 0.5


def image_to_plane_mm(
    pixel_x: float,
    pixel_y: float,
    resolution: tuple[int, int],
    cfg: Config,
) -> tuple[float, float]:
    """Map a pixel position in the camera frame to plane coordinates in mm.

    The full frame maps onto the full plane: the pixel's fractional position
    within ``resolution`` is scaled onto ``plane_*_cm``. A position that lands
    off-frame is clamped to the nearest plane edge.
    """
    res_w, res_h = resolution

    # Fractional position within the frame, scaled onto the plane (cm).
    px = (pixel_x / res_w) * cfg.plane_width_cm
    py = (pixel_y / res_h) * cfg.plane_height_cm

    # A partly off-screen hand can land just outside the frame; clamp it.
    px = min(max(px, 0.0), cfg.plane_width_cm)
    py = min(max(py, 0.0), cfg.plane_height_cm)

    if cfg.invert_x:
        px = cfg.plane_width_cm - px
    if cfg.invert_y:
        py = cfg.plane_height_cm - py

    return px * 10.0, py * 10.0  # cm -> mm, corner origin


def centroid_point(hand_landmarks) -> tuple[float, float]:
    """Barycentric center (x, y) of a hand's landmarks in normalized coords.

    Averaging all landmarks gives the hand's centroid, a stable palm-centered
    point. With a top-down camera and no depth this is the hand's projection
    onto the plane.
    """
    xs = [lm.x for lm in hand_landmarks]
    ys = [lm.y for lm in hand_landmarks]
    return float(np.mean(xs)), float(np.mean(ys))


class MotionSender:
    """Streams MOVE commands to the Arduino, or prints them in mock mode."""

    def __init__(self, cfg: Config, mock: bool = False):
        self.cfg = cfg
        self.mock = mock
        self._port = None
        if mock:
            print("[sender] mock mode: commands printed to stdout")
            return
        if serial is None:
            raise RuntimeError(
                "pyserial is not installed; install it or run with --mock"
            )
        self._port = serial.Serial(
            cfg.serial_port, cfg.serial_baud, timeout=0.1
        )
        print(f"[sender] serial open on {cfg.serial_port} @ {cfg.serial_baud}")

    def send(self, x_mm: float, y_mm: float) -> None:
        line = f"MOVE {x_mm:.1f} {y_mm:.1f} {self.cfg.move_speed:.1f}"
        if self.mock:
            print(line)
            return
        self._port.write((line + "\n").encode("ascii"))

    def close(self) -> None:
        if self._port is not None:
            self._port.close()


def draw_landmarks(frame, result: HandLandmarkerResult) -> None:
    """Overlay landmarks for the optional debug window (--show)."""
    for hand_landmarks in result.hand_landmarks:
        proto = landmark_pb2.NormalizedLandmarkList()
        proto.landmark.extend(
            landmark_pb2.NormalizedLandmark(x=lm.x, y=lm.y, z=lm.z)
            for lm in hand_landmarks
        )
        solutions.drawing_utils.draw_landmarks(
            image=frame,
            landmark_list=proto,
            connections=solutions.hands.HAND_CONNECTIONS,
            landmark_drawing_spec=solutions.drawing_styles.get_default_hand_landmarks_style(),
            connection_drawing_spec=solutions.drawing_styles.get_default_hand_connections_style(),
        )


def draw_center(
    frame,
    pixel: tuple[float, float],
    mm: tuple[float, float],
) -> None:
    """Mark the computed barycentric center on the debug window (--show)."""
    cx, cy = int(round(pixel[0])), int(round(pixel[1]))
    color = (0, 0, 255)  # BGR: red
    cv2.drawMarker(frame, (cx, cy), color, cv2.MARKER_CROSS, 20, 2)
    cv2.circle(frame, (cx, cy), 8, color, 2)
    cv2.putText(
        frame,
        f"({mm[0]:.0f}, {mm[1]:.0f}) mm",
        (cx + 12, cy - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        color,
        1,
        cv2.LINE_AA,
    )


def run(cfg: Config, sender: MotionSender, show: bool = False) -> None:
    options = HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=MODEL_PATH),
        running_mode=VisionRunningMode.VIDEO,
        num_hands=1,  # only one hand is relevant
        min_hand_detection_confidence=cfg.min_detection_confidence,
        min_tracking_confidence=cfg.min_tracking_confidence,
    )

    cap = cv2.VideoCapture(index=cfg.camera_index)

    res_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    res_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if res_w <= 0 or res_h <= 0:
        raise RuntimeError(
            f"could not read camera resolution (camera index {cfg.camera_index} "
            "may be unavailable)"
        )
    resolution = (res_w, res_h)
    print(
        f"[calib] camera resolution {res_w}x{res_h} mapped onto working plane "
        f"{cfg.plane_width_cm:.0f} x {cfg.plane_height_cm:.0f} cm"
    )

    last_send_time = 0.0
    last_x_mm: float | None = None
    last_y_mm: float | None = None

    try:
        with HandLandmarker.create_from_options(options) as landmarker:
            while cap.isOpened():
                success, frame = cap.read()
                if not success:
                    print("Ignoring empty camera frame")
                    continue

                # The model wants RGB; OpenCV gives BGR.
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(
                    image_format=mp.ImageFormat.SRGB, data=frame_rgb
                )

                # detect_for_video is synchronous (result matches this frame);
                # the timestamp only needs to increase monotonically.
                timestamp_ms = int(time.monotonic() * 1000)
                result = landmarker.detect_for_video(mp_image, timestamp_ms)

                center_px: tuple[float, float] | None = None
                center_mm: tuple[float, float] | None = None
                if result.hand_landmarks:
                    norm_x, norm_y = centroid_point(result.hand_landmarks[0])
                    center_px = (norm_x * res_w, norm_y * res_h)
                    x_mm, y_mm = image_to_plane_mm(
                        center_px[0], center_px[1], resolution, cfg
                    )
                    center_mm = (x_mm, y_mm)

                    now = time.monotonic()
                    rate_ok = now - last_send_time >= cfg.min_send_interval_s
                    moved = (
                        last_x_mm is None
                        or math.hypot(x_mm - last_x_mm, y_mm - last_y_mm)
                        >= cfg.deadband_mm
                    )
                    if rate_ok and moved:
                        sender.send(x_mm, y_mm)
                        last_send_time = now
                        last_x_mm, last_y_mm = x_mm, y_mm

                if show:
                    draw_landmarks(frame, result)
                    if center_px is not None:
                        draw_center(frame, center_px, center_mm)
                    cv2.imshow("Hand Tracking", cv2.flip(frame, 1))
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
    except KeyboardInterrupt:
        print("\n[main] interrupted, shutting down")
    finally:
        cap.release()
        if show:
            cv2.destroyAllWindows()
        sender.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mock",
        action="store_true",
        help="print MOVE commands instead of opening a serial port",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="open a debug window with landmark overlay (press 'q' to quit)",
    )
    parser.add_argument("--port", help="serial device (overrides config)")
    parser.add_argument(
        "--camera-index", type=int, help="camera index (overrides config)"
    )
    parser.add_argument(
        "--plane-width",
        type=float,
        metavar="CM",
        help="working plane width in cm (overrides config)",
    )
    parser.add_argument(
        "--plane-height",
        type=float,
        metavar="CM",
        help="working plane height in cm (overrides config)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cfg = Config()
    if args.port:
        cfg.serial_port = args.port
    if args.camera_index is not None:
        cfg.camera_index = args.camera_index
    if args.plane_width is not None:
        cfg.plane_width_cm = args.plane_width
    if args.plane_height is not None:
        cfg.plane_height_cm = args.plane_height

    sender = MotionSender(cfg, mock=args.mock)
    run(cfg, sender, show=args.show)


if __name__ == "__main__":
    main()
