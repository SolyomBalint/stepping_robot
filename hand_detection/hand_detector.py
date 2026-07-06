"""Headless hand tracker for the XY stepping robot.

Captures a top-down video stream, detects a single human hand, reduces it to one
point (the median of its landmarks), maps that point from image space into the
physical working area, and streams the position to the Arduino over serial as
``MOVE <x_mm> <y_mm> <speed>`` lines.

Image -> plane mapping (no depth; camera is fixed above the plane):

    The frame's real-world footprint at the plane is derived from the camera
    field of view and its distance to the plane:

        footprint = 2 * distance * tan(fov / 2)

    The rectangular working area (``plane_*_cm``) is assumed centered inside that
    footprint. Only points that land inside the working area are sent; anything
    outside the box is dropped. Output is millimetres with the origin at a corner,
    to match the firmware (see MachineConfig.h / stepsPerMM).
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
    # --- Camera geometry -------------------------------------------------
    camera_index: int = 0
    camera_distance_cm: float = 120.0  # camera lens -> plane surface
    camera_hfov_deg: float = 66.0  # horizontal field of view (camera-specific)
    camera_vfov_deg: float = 52.0  # vertical field of view

    # --- Working area ----------------------------------------------------
    # Physical size of the reachable XY box, in cm. Assumed centered in the
    # camera footprint. Points outside this box are not sent.
    plane_width_cm: float = 40.0
    plane_height_cm: float = 40.0

    # Flip an axis if the camera mounting orientation is mirrored relative to
    # the machine's own axes.
    invert_x: bool = False
    invert_y: bool = False

    # --- Serial link -----------------------------------------------------
    serial_port: str = "/dev/ttyACM0"
    serial_baud: int = 115200
    move_speed: float = 200.0  # the "speed" field of the MOVE command (mm/s)

    # --- Output smoothing ------------------------------------------------
    min_send_interval_s: float = 0.05  # cap the serial rate (<= 20 Hz here)
    deadband_mm: float = 2.0  # skip sends smaller than this

    # --- Detection -------------------------------------------------------
    min_detection_confidence: float = 0.5
    min_tracking_confidence: float = 0.5


def footprint_cm(cfg: Config) -> tuple[float, float]:
    """Real-world size (cm) the full frame covers at the plane distance."""
    width = (
        2.0
        * cfg.camera_distance_cm
        * math.tan(math.radians(cfg.camera_hfov_deg) / 2.0)
    )
    height = (
        2.0
        * cfg.camera_distance_cm
        * math.tan(math.radians(cfg.camera_vfov_deg) / 2.0)
    )
    return width, height


def image_to_plane_mm(
    norm_x: float,
    norm_y: float,
    cfg: Config,
    footprint: tuple[float, float],
) -> tuple[float, float] | None:
    """Map a normalized image point (0..1) to plane coordinates in mm.

    Returns ``None`` if the point falls outside the configured working area.
    """
    fw, fh = footprint

    # Position within the camera footprint, cm, origin at the frame's top-left.
    fx = norm_x * fw
    fy = norm_y * fh

    # The working box is centered in the footprint; shift into box-local cm.
    px = fx - (fw - cfg.plane_width_cm) / 2.0
    py = fy - (fh - cfg.plane_height_cm) / 2.0

    if not (
        0.0 <= px <= cfg.plane_width_cm and 0.0 <= py <= cfg.plane_height_cm
    ):
        return None

    if cfg.invert_x:
        px = cfg.plane_width_cm - px
    if cfg.invert_y:
        py = cfg.plane_height_cm - py

    return px * 10.0, py * 10.0  # cm -> mm, corner origin


def median_point(hand_landmarks) -> tuple[float, float]:
    """Median (x, y) of a hand's landmarks in normalized image coordinates.

    The median is robust to individual finger outliers, giving a stable
    palm-centered point. With a top-down camera and no depth this is the hand's
    projection onto the plane.
    """
    xs = [lm.x for lm in hand_landmarks]
    ys = [lm.y for lm in hand_landmarks]
    return float(np.median(xs)), float(np.median(ys))


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


def run(cfg: Config, sender: MotionSender, show: bool = False) -> None:
    options = HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=MODEL_PATH),
        running_mode=VisionRunningMode.VIDEO,
        num_hands=1,  # only one hand is relevant
        min_hand_detection_confidence=cfg.min_detection_confidence,
        min_tracking_confidence=cfg.min_tracking_confidence,
    )

    footprint = footprint_cm(cfg)
    print(
        f"[calib] footprint at {cfg.camera_distance_cm:.0f} cm: "
        f"{footprint[0]:.1f} x {footprint[1]:.1f} cm; "
        f"working area {cfg.plane_width_cm:.0f} x {cfg.plane_height_cm:.0f} cm (centered)"
    )
    if cfg.plane_width_cm > footprint[0] or cfg.plane_height_cm > footprint[1]:
        print(
            "[calib] WARNING: working area is larger than the camera footprint"
        )

    cap = cv2.VideoCapture(index=cfg.camera_index)

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

                if result.hand_landmarks:
                    norm_x, norm_y = median_point(result.hand_landmarks[0])
                    point_mm = image_to_plane_mm(norm_x, norm_y, cfg, footprint)

                    if point_mm is not None:
                        x_mm, y_mm = point_mm
                        now = time.monotonic()
                        rate_ok = (
                            now - last_send_time >= cfg.min_send_interval_s
                        )
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
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cfg = Config()
    if args.port:
        cfg.serial_port = args.port
    if args.camera_index is not None:
        cfg.camera_index = args.camera_index

    sender = MotionSender(cfg, mock=args.mock)
    run(cfg, sender, show=args.show)


if __name__ == "__main__":
    main()
