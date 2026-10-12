#!/usr/bin/env python3
"""
===============================================================================
Project: Vision-Based Gesture-Controlled Person-Following Robot
File: vision/cc_ll.py
Description: Production-hardened Low-Latency Vision Controller.
             Features:
             - CLI parameterization (--camera 0 or --camera http://IP:8080/video)
             - Automatic fallback from network MJPEG to local USB camera
             - Cold-start capability: Starts IDLE even if gesture_model.pkl is missing
             - Pair target using purely geometric closed-fist heuristic
             - Automatic on-the-fly model training via sys.executable
             - Persistent Unix domain socket connections with auto-reconnect
             - Bounding-box to wrist association for target exclusivity
===============================================================================
"""

import argparse
import csv
import json
import math
import os
import socket
import subprocess
import sys
import threading
import time
import urllib.request

import cv2
import joblib
import mediapipe as mp
import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

from mediapipe.tasks import python
from mediapipe.tasks.python import vision

# Local imports
from gesture_features import extract_features

# =============================================================================
# CONSTANTS & CONFIGURATION
# =============================================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
GESTURE_DATA_DIR = os.path.join(SCRIPT_DIR, "gesture_data")
HAND_MODEL_PATH = os.path.join(SCRIPT_DIR, "hand_landmarker.task")
MODEL_PATH = os.path.join(SCRIPT_DIR, "gesture_model.pkl")
PERSON_MODEL_PATH = os.path.join(SCRIPT_DIR, "yolo11n.pt")

GESTURES = ["FOLLOW", "STOP", "DOCK"]
PAIRING_HOLD_TIME = 3.0
GESTURE_CONFIRMATION_TIME = 1.0
REJECTION_GRACE_TIME = 0.3
RECALIBRATION_COUNTDOWN = 3.0
GESTURE_RECORD_DURATION = 8.0
SAMPLE_RATE = 20
SAMPLE_INTERVAL = 1.0 / SAMPLE_RATE

GESTURE_SOCKET_PATH = "/tmp/vbgc_gesture.sock"
TARGET_SOCKET_PATH = "/tmp/vbgc_target.sock"
TARGET_PUBLISH_INTERVAL = 0.08  # ~12.5 Hz

ASSUMED_PERSON_HEIGHT_METERS = 1.70
CAMERA_FOCAL_LENGTH_PIXELS = 600.0
MIN_TARGET_DISTANCE = 0.50
MAX_TARGET_DISTANCE = 5.00

os.makedirs(GESTURE_DATA_DIR, exist_ok=True)


# =============================================================================
# VIDEO STREAMING PRIMITIVES
# =============================================================================
class ResilientCameraStream:
    """Combines a threaded MJPEG reader with fallback to standard V4L2 USB camera."""

    def __init__(self, source):
        self.source = source
        self.is_url = str(source).startswith("http://") or str(source).startswith("https://")
        self.frame = None
        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        self.cap_local = None

    def start(self):
        if self.is_url:
            print(f"[INFO] Connecting to MJPEG network stream at {self.source}...")
            self.running = True
            self.thread = threading.Thread(target=self._mjpeg_worker, daemon=True)
            self.thread.start()
            # Wait up to 3 seconds for first frame
            start_wait = time.time()
            while time.time() - start_wait < 3.0:
                with self.lock:
                    if self.frame is not None:
                        print("[OK] Connected to network MJPEG camera stream.")
                        return True
                time.sleep(0.1)

            print("[WARN] MJPEG stream failed to respond. Falling back to USB Camera (/dev/video0)...")
            self.running = False

        # Fallback to local OpenCV camera
        cam_idx = 0
        if not self.is_url:
            try:
                cam_idx = int(self.source)
            except ValueError:
                cam_idx = 0

        self.cap_local = cv2.VideoCapture(cam_idx)
        if not self.cap_local.isOpened():
            print(f"[ERROR] Could not open local camera index {cam_idx}")
            return False

        print(f"[OK] Opened local camera index {cam_idx}")
        return True

    def _mjpeg_worker(self):
        try:
            req = urllib.request.Request(self.source, headers={"User-Agent": "Mozilla/5.0"})
            stream = urllib.request.urlopen(req, timeout=4)
            buffer = b""

            while self.running:
                chunk = stream.read(4096)
                if not chunk:
                    break
                buffer += chunk

                while self.running:
                    start = buffer.find(b"\xff\xd8")
                    if start == -1:
                        if len(buffer) > 1:
                            buffer = buffer[-1:]
                        break

                    end = buffer.find(b"\xff\xd9", start + 2)
                    if end == -1:
                        if start > 0:
                            buffer = buffer[start:]
                        break

                    jpg = buffer[start : end + 2]
                    buffer = buffer[end + 2 :]

                    decoded = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if decoded is not None:
                        with self.lock:
                            self.frame = decoded
        except Exception as exc:
            print(f"[WARN] MJPEG stream disconnected: {exc}")
        finally:
            self.running = False

    def read(self):
        if self.cap_local is not None:
            ret, frame = self.cap_local.read()
            return ret, frame

        with self.lock:
            if self.frame is None:
                return False, None
            return True, self.frame.copy()

    def release(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.cap_local:
            self.cap_local.release()


# =============================================================================
# PERSISTENT UNIX SOCKET CLIENT
# =============================================================================
class PersistentSocketClient:
    """Maintains a persistent socket connection to prevent connection churn."""

    def __init__(self, path):
        self.path = path
        self.sock = None

    def connect(self):
        if not os.path.exists(self.path):
            return False
        try:
            self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.sock.settimeout(0.1)
            self.sock.connect(self.path)
            return True
        except (socket.error, OSError):
            self.sock = None
            return False

    def send(self, payload_str):
        if self.sock is None:
            if not self.connect():
                return False
        try:
            self.sock.sendall(payload_str.encode("utf-8"))
            return True
        except (socket.error, BrokenPipeError, ConnectionResetError):
            self.close()
            return False

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None


# =============================================================================
# GESTURE & TRACKING UTILITIES
# =============================================================================
def is_closed_fist(hand):
    """Purely geometric heuristic to detect a closed fist. Does NOT require ML model."""
    finger_pairs = [(8, 6), (12, 10), (16, 14), (20, 18)]
    folded_fingers = sum(1 for tip, pip in finger_pairs if hand[tip].y > hand[pip].y)

    index_mcp = hand[5]
    pinky_mcp = hand[17]
    thumb_tip = hand[4]

    palm_width = math.hypot(index_mcp.x - pinky_mcp.x, index_mcp.y - pinky_mcp.y)
    if palm_width < 1e-8:
        return False

    thumb_to_index = math.hypot(thumb_tip.x - index_mcp.x, thumb_tip.y - index_mcp.y)
    thumb_to_pinky = math.hypot(thumb_tip.x - pinky_mcp.x, thumb_tip.y - pinky_mcp.y)

    thumb_folded = (thumb_to_index < palm_width * 0.95) and (thumb_to_pinky < palm_width * 0.95)
    return (folded_fingers == 4) and thumb_folded


def hand_person_id(hand, people, frame_shape):
    """Associates detected hand wrist to the bounding box of a tracked person."""
    height, width = frame_shape[:2]
    wrist_x = hand[0].x * width
    wrist_y = hand[0].y * height

    for person in people:
        x1, y1, x2, y2 = person["box"]
        # Allow 5% margin around person bounding box
        pad_x = (x2 - x1) * 0.05
        pad_y = (y2 - y1) * 0.05
        if (x1 - pad_x) <= wrist_x <= (x2 + pad_x) and (y1 - pad_y) <= wrist_y <= (y2 + pad_y):
            return person["id"]
    return None


def predict_gesture(hand, model_data):
    """Classifies hand pose with open-set k-NN distance rejection."""
    if model_data is None:
        return "MODEL_NOT_TRAINED", 0.0, 0.0, 0.0

    model = model_data["model"]
    training_features = model_data["training_features"]
    training_labels = model_data["training_labels"]
    distance_thresholds = model_data["distance_thresholds"]
    k_val = model_data.get("distance_k", 5)

    features = np.array(extract_features(hand), dtype=np.float32)
    probabilities = model.predict_proba([features])[0]
    pred_idx = probabilities.argmax()
    candidate = model.classes_[pred_idx]
    confidence = float(probabilities[pred_idx])

    class_samples = training_features[training_labels == candidate]
    distances = np.linalg.norm(class_samples - features, axis=1)
    k = min(k_val, len(distances))
    distance = float(np.mean(np.sort(distances)[:k]))
    threshold = float(distance_thresholds.get(candidate, 0.5))

    prediction = candidate if distance <= threshold else "REJECTED"
    return prediction, confidence, distance, threshold


def run_calibration(cap, landmarker, yolo_tracker):
    """Interactive calibration: records samples for FOLLOW, STOP, DOCK and trains model."""
    print("\n" + "=" * 60 + "\n  INTERACTIVE GESTURE CALIBRATION ACTIVATED\n" + "=" * 60)

    # 3-second ready countdown
    start_t = time.time()
    while time.time() - start_t < RECALIBRATION_COUNTDOWN:
        ret, frame = cap.read()
        if not ret:
            return False
        frame = cv2.flip(frame, 1)
        rem = RECALIBRATION_COUNTDOWN - (time.time() - start_t)
        cv2.putText(
            frame,
            "GET READY TO CALIBRATE",
            (20, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
        )
        cv2.putText(
            frame,
            f"Starting in {rem:.1f}s",
            (20, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.imshow("Gesture Controller", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            return False

    for gesture in GESTURES:
        print(f"[INFO] Recording gesture: {gesture}")
        samples = []
        start_rec = time.time()
        last_sample = 0.0

        while time.time() - start_rec < GESTURE_RECORD_DURATION:
            ret, frame = cap.read()
            if not ret:
                return False
            frame = cv2.flip(frame, 1)
            now = time.time()
            rem = GESTURE_RECORD_DURATION - (now - start_rec)

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            res = landmarker.detect(mp_img)

            if res.hand_landmarks:
                hand = res.hand_landmarks[0]
                if now - last_sample >= SAMPLE_INTERVAL:
                    samples.append(extract_features(hand))
                    last_sample = now

            cv2.putText(
                frame,
                f"RECORDING: {gesture}",
                (20, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                frame,
                f"Time Left: {rem:.1f}s | Samples: {len(samples)}",
                (20, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            cv2.imshow("Gesture Controller", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                return False

        csv_path = os.path.join(GESTURE_DATA_DIR, f"{gesture}.csv")
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([f"f{i}" for i in range(63)])
            writer.writerows(samples)
        print(f"[OK] Saved {len(samples)} samples to {csv_path}")

    # Retrain model using active python interpreter
    train_script = os.path.join(SCRIPT_DIR, "train_gesture_model.py")
    res = subprocess.run([sys.executable, train_script], cwd=SCRIPT_DIR, text=True)
    return res.returncode == 0


# =============================================================================
# MAIN CONTROLLER LOOP
# =============================================================================
def main():
    parser = argparse.ArgumentParser(description="VBGC Robot Vision Controller")
    parser.add_argument(
        "--camera",
        default="http://172.16.61.125:8080/video",
        help="Camera URL or webcam device index (e.g., '0')",
    )
    args = parser.parse_args()

    # Load MediaPipe Hand Landmarker
    if not os.path.exists(HAND_MODEL_PATH):
        print(f"[FATAL] Hand landmarker asset missing: {HAND_MODEL_PATH}")
        sys.exit(1)

    base_opt = python.BaseOptions(model_asset_path=HAND_MODEL_PATH)
    options = vision.HandLandmarkerOptions(
        base_options=base_opt,
        num_hands=1,
        min_hand_detection_confidence=0.5,
        min_hand_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    landmarker = vision.HandLandmarker.create_from_options(options)

    # Initialize YOLO Person Tracker
    person_tracker = None
    if YOLO is not None and os.path.exists(PERSON_MODEL_PATH):
        person_tracker = YOLO(PERSON_MODEL_PATH)
        print("[OK] YOLO11 Person Tracker loaded.")
    else:
        print("[WARN] Running without YOLO person tracker (YOLO or weights missing).")

    # Start Video Stream
    cap = ResilientCameraStream(args.camera)
    if not cap.start():
        sys.exit(1)

    # Initialize Persistent Sockets
    gesture_client = PersistentSocketClient(GESTURE_SOCKET_PATH)
    target_client = PersistentSocketClient(TARGET_SOCKET_PATH)

    # Attempt to load model (Cold boot allowed!)
    model_data = None
    if os.path.exists(MODEL_PATH):
        try:
            model_data = joblib.load(MODEL_PATH)
            print("[OK] Loaded pre-trained gesture model.")
        except Exception as e:
            print(f"[WARN] Failed to load {MODEL_PATH}: {e}")
    else:
        print("[NOTICE] No gesture_model.pkl found. System started in UNCALIBRATED mode.")
        print("[NOTICE] Pair with a 3-second CLOSED FIST to begin automatic calibration.")

    state = "IDLE"  # IDLE | ACTIVE
    target_person_id = None
    pairing_fist_start_time = None
    candidate_gesture = None
    gesture_start_time = None
    confirmed_gesture = None
    rejection_start_time = None
    last_target_publish_time = 0.0

    print("\n" + "=" * 50)
    print("  VBGC VISION CONTROLLER RUNNING")
    print("  Show CLOSED FIST for 3s to Pair/Calibrate")
    print("  Press 'C' when IDLE to force recalibration")
    print("  Press 'Q' or ESC to Quit")
    print("=" * 50 + "\n")

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            time.sleep(0.01)
            continue

        frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]

        # 1. Track People with YOLO
        people = []
        if person_tracker is not None:
            res_yolo = person_tracker.track(
                frame, persist=True, classes=[0], tracker="bytetrack.yaml", verbose=False
            )
            if res_yolo and res_yolo[0].boxes and res_yolo[0].boxes.id is not None:
                boxes = res_yolo[0].boxes.xyxy.cpu().numpy()
                p_ids = res_yolo[0].boxes.id.int().cpu().tolist()
                for b, pid in zip(boxes, p_ids):
                    people.append({"id": pid, "box": b.tolist()})

        # Draw Person Bounding Boxes
        for p in people:
            bx1, by1, bx2, by2 = [int(v) for v in p["box"]]
            is_tgt = (p["id"] == target_person_id)
            color = (0, 255, 0) if is_tgt else (255, 100, 0)
            thick = 3 if is_tgt else 1
            cv2.rectangle(frame, (bx1, by1), (bx2, by2), color, thick)
            label = f"TARGET ID: {p['id']}" if is_tgt else f"Person {p['id']}"
            cv2.putText(
                frame, label, (bx1, max(20, by1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2
            )

        # 2. Detect Hands with MediaPipe
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        mp_res = landmarker.detect(mp_img)

        hand_detected = bool(mp_res.hand_landmarks)
        hand = mp_res.hand_landmarks[0] if hand_detected else None
        fist_detected = is_closed_fist(hand) if hand else False
        current_pid = hand_person_id(hand, people, frame.shape) if hand else None

        now = time.time()

        # =====================================================================
        # STATE MACHINE: IDLE (Pairing Mode)
        # =====================================================================
        if state == "IDLE":
            cv2.putText(
                frame,
                "STATE: IDLE (Show Fist 3s to Pair)",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )

            if fist_detected:
                if pairing_fist_start_time is None:
                    pairing_fist_start_time = now

                held_time = now - pairing_fist_start_time
                progress = min(held_time / PAIRING_HOLD_TIME, 1.0)

                # Visual Progress Bar
                cv2.rectangle(frame, (20, 50), (320, 70), (255, 255, 255), 2)
                cv2.rectangle(frame, (20, 50), (20 + int(300 * progress), 70), (0, 255, 0), -1)
                cv2.putText(
                    frame,
                    f"Pairing: {held_time:.1f}s",
                    (330, 67),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

                if held_time >= PAIRING_HOLD_TIME:
                    target_person_id = current_pid
                    state = "ACTIVE"
                    pairing_fist_start_time = None
                    print(f"\n[OK] TARGET PAIRED! Person ID: {target_person_id}")

                    # If model not trained yet, trigger automatic calibration
                    if model_data is None:
                        success = run_calibration(cap, landmarker, person_tracker)
                        if success and os.path.exists(MODEL_PATH):
                            model_data = joblib.load(MODEL_PATH)
                            print("[OK] Calibration complete. Ready for control.")
            else:
                pairing_fist_start_time = None

        # =====================================================================
        # STATE MACHINE: ACTIVE (Tracking & Following Mode)
        # =====================================================================
        elif state == "ACTIVE":
            cv2.putText(
                frame,
                f"STATE: ACTIVE (Target ID: {target_person_id})",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

            # Locate target bounding box and publish coordinates
            target_box = next((p["box"] for p in people if p["id"] == target_person_id), None)
            if target_box is not None:
                bx1, by1, bx2, by2 = target_box
                box_h = max(1.0, by2 - by1)
                box_cx = (bx1 + bx2) / 2.0

                # Monocular distance estimate
                dist = (ASSUMED_PERSON_HEIGHT_METERS * CAMERA_FOCAL_LENGTH_PIXELS) / box_h
                dist = max(MIN_TARGET_DISTANCE, min(MAX_TARGET_DISTANCE, dist))
                offset_x = box_cx - (w / 2.0)
                angle = math.atan2(-offset_x, CAMERA_FOCAL_LENGTH_PIXELS)

                target_x = dist * math.cos(angle)
                target_y = dist * math.sin(angle)

                if now - last_target_publish_time >= TARGET_PUBLISH_INTERVAL:
                    target_msg = json.dumps({"x": round(target_x, 3), "y": round(target_y, 3)}) + "\n"
                    target_client.send(target_msg)
                    last_target_publish_time = now

                cv2.putText(
                    frame,
                    f"Dist: {dist:.2f}m | Angle: {math.degrees(angle):.1f} deg",
                    (20, 65),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                )

            # Classify gestures ONLY if the hand belongs to the paired target
            is_target_hand = (current_pid is not None and current_pid == target_person_id)

            if hand_detected and is_target_hand and model_data is not None:
                pred, conf, dist_metric, thresh = predict_gesture(hand, model_data)

                cv2.putText(
                    frame,
                    f"Gesture: {pred} ({conf*100:.0f}%)",
                    (20, 95),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0) if pred in GESTURES else (0, 0, 255),
                    2,
                )

                if pred in GESTURES:
                    rejection_start_time = None
                    if pred != candidate_gesture:
                        candidate_gesture = pred
                        gesture_start_time = now
                    else:
                        elapsed = now - (gesture_start_time or now)
                        if elapsed >= GESTURE_CONFIRMATION_TIME:
                            if confirmed_gesture != pred:
                                confirmed_gesture = pred
                                print(f"[COMMAND CONFIRMED] -> {pred}")
                                gesture_client.send(pred + "\n")

                                if pred == "DOCK":
                                    print("[INFO] DOCK received. Releasing target.")
                                    state = "IDLE"
                                    target_person_id = None
                                    confirmed_gesture = None
                else:
                    if candidate_gesture is not None:
                        if rejection_start_time is None:
                            rejection_start_time = now
                        if now - rejection_start_time > REJECTION_GRACE_TIME:
                            candidate_gesture = None
                            gesture_start_time = None
            elif hand_detected and not is_target_hand:
                cv2.putText(
                    frame,
                    "Hand Ignored: Not Paired Target",
                    (20, 95),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2,
                )

        cv2.imshow("Gesture Controller", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break
        elif key == ord("c") and state == "IDLE":
            run_calibration(cap, landmarker, person_tracker)
            if os.path.exists(MODEL_PATH):
                model_data = joblib.load(MODEL_PATH)

    cap.release()
    cv2.destroyAllWindows()
    gesture_client.close()
    target_client.close()
    landmarker.close()
    print("[INFO] Vision Controller exited cleanly.")


if __name__ == "__main__":
    main()
