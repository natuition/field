#!/usr/bin/env python3
import cv2
import threading
import os
import signal
import sys
import numpy as np
from flask import Flask, Response, jsonify
from flask_cors import CORS
from config import config
import detection
from adapters import CameraAdapterManager


class ServerCamLive:
    """
    Camera streaming server based on Flask + CameraAdapterManager.
    - If `app` is provided: attaches /video and /info routes to it.
    - If `app` is None: creates its own Flask app and runs it.
    """

    def __init__(self, app=None, port=8080, use_detector=True, display_zones=False):
        """
        :param app: Optional Flask app instance.
        :param port: Port used if running standalone.
        :param use_detector: Enable YOLO TRT detection on frames.
        """
        self.external_app = app is not None
        self.app = app or Flask(__name__)
        CORS(self.app)

        self.port = port
        self.use_detector = use_detector
        self.display_zones = display_zones

        self.cam = None
        self.detector = None
        self.thread_alive = False
        self.video_frame = None
        self.thread_lock = threading.Lock()
        self.capture_thread = None

        # Register routes (always added)
        self._register_routes()

    # ─────────── DRAWING ZONES ───────────
    def draw_zone_circle(self, image, cx, cy, radius):
        """Draws a circle showing the undistorted zone."""
        return cv2.circle(image, (cx, cy), radius, (0, 0, 255), thickness=3)

    def draw_zone_poly(self, image, np_poly_points):
        """Draws polygon of the working zone."""
        return cv2.polylines(image, [np_poly_points], isClosed=True, color=(0, 255, 255), thickness=5)

    # ─────────── ROUTES ───────────
    def _register_routes(self):
        """Attach /video and /info routes to the Flask app."""
        self.app.add_url_rule("/video", view_func=self.stream_frames)
        self.app.add_url_rule("/info", view_func=self.camera_info)
        
    def _unregister_routes(self):
        """Remove previously added routes from the Flask app."""
        removed = []
        for rule in list(self.app.url_map.iter_rules()):
            if rule.rule in self._registered_routes:
                removed.append(rule.rule)
                self.app.url_map._rules.remove(rule)
                self.app.view_functions.pop(rule.endpoint, None)
        if removed:
            print(f"[{self.__class__.__name__}] 🧹 Removed routes: {', '.join(removed)}")

    def camera_info(self):
        """Return camera backend and adapter info as JSON."""
        if self.cam:
            return jsonify({
                "backend": self.cam.backend(),
                "class": self.cam.whoami()
            })
        return jsonify({"status": "camera not initialized"})

    # ─────────── CAMERA ───────────
    def init_camera(self):
        """Initialize camera with CameraAdapterManager."""
        try:
            self.cam = CameraAdapterManager(
                crop_w_from=config.CROP_W_FROM,
                crop_w_to=config.CROP_W_TO,
                crop_h_from=config.CROP_H_FROM,
                crop_h_to=config.CROP_H_TO,
                cv_rotate_code=None,
                ispdigitalgainrange_from=config.ISP_DIGITAL_GAIN_RANGE_FROM,
                ispdigitalgainrange_to=config.ISP_DIGITAL_GAIN_RANGE_TO,
                gainrange_from=config.GAIN_RANGE_FROM,
                gainrange_to=config.GAIN_RANGE_TO,
                exposuretimerange_from=config.EXPOSURE_TIME_RANGE_FROM,
                exposuretimerange_to=config.EXPOSURE_TIME_RANGE_TO,
                aelock=config.AE_LOCK,
                capture_width=config.CAMERA_W,
                capture_height=config.CAMERA_H,
                display_width=config.CAMERA_W,
                display_height=config.CAMERA_H,
                framerate=config.CAMERA_FRAMERATE,
                nvidia_flip_method=config.CAMERA_FLIP_METHOD
            )
            print(f"[{self.__class__.__name__}] ✅ Camera initialized using backend: {self.cam.backend()} ({self.cam.whoami()})")
        except Exception as e:
            print(f"[{self.__class__.__name__}] ❌ Failed to initialize camera: {e}")
            sys.exit(1)

    def capture_loop(self):
        """Continuously capture frames from the camera."""
        try:
            while self.thread_alive:
                frame = self.cam.get_image()
                with self.thread_lock:
                    self.video_frame = frame
        except Exception as e:
            print(f"[{self.__class__.__name__}] ⚠️ Capture error: {e}")
        finally:
            if self.cam:
                self.cam.release()
            print(f"[{self.__class__.__name__}] 🛑 Camera released.")

    # ─────────── STREAMING ───────────
    def rescale_frame(self, frame, percent=50):
        """Resize frame to reduce stream bandwidth."""
        width = int(frame.shape[1] * percent / 100)
        height = int(frame.shape[0] * percent / 100)
        return cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)

    def encode_frame(self):
        """Generate MJPEG byte stream."""
        while True:
            with self.thread_lock:
                frame = self.video_frame

            if frame is None:
                continue

            # Optional detector
            if self.detector:
                boxes = self.detector.detect(frame, True)
                frame = detection.draw_boxes(frame, boxes)
                
            # Optional zone display
            if self.display_zones:
                try:
                    poly_zone_points_cv = np.array(config.WORKING_ZONE_POLY_POINTS, np.int32).reshape((-1, 1, 2))
                    frame = self.draw_zone_circle(frame, config.SCENE_CENTER_X, config.SCENE_CENTER_Y, config.UNDISTORTED_ZONE_RADIUS)
                    frame = self.draw_zone_circle(frame, config.SCENE_CENTER_X, config.SCENE_CENTER_Y, 2)
                    frame = self.draw_zone_poly(frame, poly_zone_points_cv)
                except Exception as e:
                    print(f"[{self.__class__.__name__}] ⚠️ Zone drawing failed: {e}")

            frame = self.rescale_frame(frame, percent=50)
            ok, encoded = cv2.imencode(".jpg", frame)
            if not ok:
                continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" +
                bytearray(encoded) +
                b"\r\n"
            )

    def stream_frames(self):
        """Flask endpoint returning the MJPEG stream."""
        return Response(self.encode_frame(), mimetype="multipart/x-mixed-replace; boundary=frame")

    # ─────────── CONTROL ───────────
    def start(self):
        """Initialize camera, detector, and capture thread.
        If no external app, also runs Flask server."""
        print(f"[{self.__class__.__name__}] 🔄 Restarting nvargus-daemon (if available)...")
        os.system("sudo systemctl restart nvargus-daemon")

        self.init_camera()

        if self.use_detector:
            print(f"[{self.__class__.__name__}] 🧠 Initializing detector...")
            self.detector = detection.YoloTRTDetector(
                config.PERIPHERY_MODEL_PATH,
                config.PERIPHERY_CLASSES_FILE,
                config.PERIPHERY_CONFIDENCE_THRESHOLD,
                config.PERIPHERY_NMS_THRESHOLD,
                config.PERIPHERY_INPUT_SIZE,
            )

        self.thread_alive = True
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()

        if not self.external_app:
            print(f"[{self.__class__.__name__}] 🌐 Starting internal Flask server on http://0.0.0.0:{self.port}/video")
            self.app.run("0.0.0.0", self.port, debug=False, use_reloader=False)
        else:
            print(f"[{self.__class__.__name__}] 🔗 ServerCamLive attached to external Flask app.")

    def stop(self):
        """Stop the capture thread and remove routes."""
        print(f"[{self.__class__.__name__}] 🛑 Stopping camera stream...")
        
        self.thread_alive = False
        if self.cam:
            self.cam.release()
            
        # Remove routes if attached to an external app
        if self.app and self.external_app:
            self._unregister_routes()
            
        print(f"[{self.__class__.__name__}] ✅ Streaming stopped.")


# ─────────── Main Entrypoint ───────────
if __name__ == "__main__":
    display_zones = "--zones" in sys.argv
    use_detector = not ("--no-detector" in sys.argv)

    server = ServerCamLive(port=8080, use_detector=use_detector, display_zones=display_zones)
    
    def handle_exit(sig, frame):
        print("\n[ServerCamLive] 🛑 Ctrl+C detected — stopping server and releasing camera...")
        server.stop()
        sys.exit(0)

    # Capture Ctrl+C (SIGINT) et kill (SIGTERM)
    signal.signal(signal.SIGINT, handle_exit)
    signal.signal(signal.SIGTERM, handle_exit)
    
    server.start()