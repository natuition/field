#!/usr/bin/env python3
import os, sys, cv2, time, threading, signal, json
from flask import Flask, request, jsonify, Response
from config import config
from adapters import CameraAdapterManager

# ── GStreamer (fourni par JetPack) ─────────────────────────────────────────────
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
gi.require_version('GstSdp', '1.0')
from gi.repository import Gst, GstWebRTC, GstSdp, GObject, GLib

Gst.init(None)
GObject.threads_init()


class ServerCamLiveWebRTC:
    """
    WebRTC streaming server using:
      - CameraAdapterManager for frame capture (Aravis / IMX219)
      - GStreamer pipeline: appsrc -> (NVENC or x264enc) -> rtph264pay -> webrtcbin
      - Flask for signaling (no extra libs needed on JetPack)
    """

    def __init__(self, app: Flask | None = None, port: int = 8080,
                 width: int = 960, height: int = 540, fps: int = 30):
        self.external_app = app is not None
        self.app = app or Flask(__name__)
        self.port = port
        self.width, self.height, self.fps = width, height, fps

        # Camera
        self.cam = None
        self.capture_thread = None
        self._run_capture = False
        self._last_frame = None
        self._frame_lock = threading.Lock()

        # GStreamer / WebRTC
        self.pipeline = None
        self.appsrc = None
        self.webrtcbin = None
        self.loop = GLib.MainLoop()
        self._gst_thread = None

        # Signaling sync
        self._answer_ready = threading.Event()
        self._gathering_done = threading.Event()
        self._local_sdp_answer = None

        # Register routes
        self._register_routes()

    # ─────────────────────────── Routes Flask ───────────────────────────
    def _register_routes(self):
        @self.app.route("/")
        def index():
            return Response(self._client_html(), mimetype="text/html")

        @self.app.route("/offer", methods=["POST"])
        def offer():
            """Receive SDP offer (from browser), set remote desc, create answer, return SDP answer."""
            data = request.get_json(force=True)
            sdp = data["sdp"]
            # Set remote description
            ok, sdpmsg = GstSdp.SDPMessage.new()
            GstSdp.sdp_message_parse_buffer(bytes(sdp.encode("utf-8")), sdpmsg)
            remote_desc = GstWebRTC.WebRTCSessionDescription.new(GstWebRTC.WebRTCSDPType.OFFER, sdpmsg)
            self.webrtcbin.emit("set-remote-description", remote_desc)

            # Create answer (async)
            self.webrtcbin.emit("create-answer", None, self._on_create_answer, None)

            # Wait until local SDP answer is generated and ICE gathered
            if not self._answer_ready.wait(timeout=5.0):
                return jsonify({"error": "timeout creating answer"}), 500
            # Optionally wait ICE complete to bundle candidates in SDP (simpler client)
            self._gathering_done.wait(timeout=5.0)

            return jsonify({"sdp": self._local_sdp_answer, "type": "answer"})

        @self.app.route("/info")
        def info():
            backend = self.cam.backend() if self.cam else None
            whoami = self.cam.whoami() if self.cam else None
            return jsonify({
                "backend": backend,
                "class": whoami,
                "webrtc": bool(self.webrtcbin is not None),
                "width": self.width,
                "height": self.height,
                "fps": self.fps,
            })

    # ───────────────────── Client HTML minimal (WebRTC) ─────────────────────
    def _client_html(self) -> str:
        return f"""
<!doctype html>
<html><head><meta charset="utf-8"><title>WebRTC Jetson</title></head>
<body style="margin:0;background:#111;color:#eee;font-family:system-ui">
  <div style="padding:8px">WebRTC stream (H.264) — {self.width}x{self.height}@{self.fps}fps</div>
  <video id="v" autoplay playsinline style="width:100vw;height:calc(100vh - 40px);object-fit:contain;background:#000"></video>
<script>
(async () => {{
  const pc = new RTCPeerConnection();
  pc.ontrack = e => (document.getElementById('v').srcObject = e.streams[0]);
  pc.addTransceiver('video', {{direction: 'recvonly'}});
  const offer = await pc.createOffer();
  await pc.setLocalDescription(offer);

  const res = await fetch('/offer', {{
    method: 'POST',
    headers: {{'Content-Type':'application/json'}},
    body: JSON.stringify({{sdp: pc.localDescription.sdp, type: 'offer'}})
  }});
  const data = await res.json();
  if (data.sdp) {{
    await pc.setRemoteDescription({{type: 'answer', sdp: data.sdp}});
  }} else {{
    console.error('Offer failed', data);
  }}
}})();
</script>
</body></html>
"""

    # ───────────────────────── GStreamer / WebRTC ─────────────────────────
    def _build_pipeline(self):
        """Create pipeline: appsrc -> (encoder) -> rtph264pay -> webrtcbin"""
        # Encoder: prefer NVENC if available, fallback to x264
        enc_factory = Gst.ElementFactory.find("nvv4l2h264enc") or Gst.ElementFactory.find("omxh264enc")
        use_nvenc = enc_factory is not None

        self.pipeline = Gst.Pipeline.new("p")

        # appsrc
        self.appsrc = Gst.ElementFactory.make("appsrc", "src")
        self.appsrc.set_property("is-live", True)
        self.appsrc.set_property("block", True)
        self.appsrc.set_property("format", Gst.Format.TIME)
        # caps: I420 @ width x height @ fps
        caps = Gst.Caps.from_string(
            f"video/x-raw,format=I420,width={self.width},height={self.height},framerate={self.fps}/1"
        )
        self.appsrc.set_property("caps", caps)

        convert = Gst.ElementFactory.make("videoconvert", None)

        if use_nvenc:
            encoder = Gst.ElementFactory.make("nvv4l2h264enc", "enc")
            # low-latency settings
            encoder.set_property("insert-sps-pps", True)
            encoder.set_property("iframeinterval", self.fps)  # keyframe every 1s
            encoder.set_property("bitrate", 4000)             # ~4Mbps
        else:
            encoder = Gst.ElementFactory.make("x264enc", "enc")
            encoder.set_property("tune", "zerolatency")
            encoder.set_property("speed-preset", "ultrafast")
            encoder.set_property("bitrate", 2000)
            encoder.set_property("key-int-max", self.fps)

        h264parse = Gst.ElementFactory.make("h264parse", None)
        h264parse.set_property("config-interval", 1)

        pay = Gst.ElementFactory.make("rtph264pay", "pay")
        pay.set_property("pt", 96)
        pay.set_property("config-interval", 1)

        self.webrtcbin = Gst.ElementFactory.make("webrtcbin", "webrtc")
        # STUN public (peut être omis en LAN)
        self.webrtcbin.set_property("stun-server", "stun://stun.l.google.com:19302")

        # Signals for SDP/ICE
        self.webrtcbin.connect("on-ice-candidate", self._on_ice_candidate)
        self.webrtcbin.connect("ice-gathering-state-change", self._on_ice_gather_state)

        for el in [self.appsrc, convert, encoder, h264parse, pay, self.webrtcbin]:
            self.pipeline.add(el)

        if not Gst.Element.link_many(self.appsrc, convert, encoder, h264parse, pay):
            raise RuntimeError("Failed to link elements before webrtcbin")
        # Link pay → webrtcbin (request pad)
        pay_src_pad = pay.get_static_pad("src")
        webrtc_sink_pad = self.webrtcbin.get_request_pad("sink_%u")
        pay_src_pad.link(webrtc_sink_pad)

    def _on_create_answer(self, webrtc, promise, _):
        """GStreamer async callback when answer is created."""
        reply = promise.get_reply()
        answer = reply.get_value("answer")
        promise = Gst.Promise.new()
        self.webrtcbin.emit("set-local-description", answer, promise)
        promise.interrupt()  # we don't need to wait here

        # Serialize SDP
        sdp_text = answer.sdp.as_text()
        self._local_sdp_answer = sdp_text
        self._answer_ready.set()

    def _on_ice_candidate(self, webrtc, mlineindex, candidate):
        # Trickle ICE path (browser typically handles these incrementally).
        # Here we do nothing explicit; candidates are included as gathering completes.
        pass

    def _on_ice_gather_state(self, webrtc, state):
        # GST_WEBRTC_ICE_GATHERING_STATE_COMPLETE == 2 (older) or 3 (newer); be permissive
        try:
            val = int(state.value_nick)  # not reliable
        except Exception:
            val = None
        # Just mark done when state nick says "complete"
        if str(state).lower().endswith("complete"):
            self._gathering_done.set()

    def _gst_mainloop(self):
        try:
            self.loop.run()
        except Exception:
            pass

    # ───────────────────────── Capture & push ─────────────────────────
    def _capture_loop(self):
        """Grab frames from CameraAdapterManager, push to appsrc as I420 buffers with timestamps."""
        nanos_per_frame = int(1e9 / self.fps)
        pts = 0
        while self._run_capture:
            try:
                frame_bgr = self.cam.get_image()  # numpy BGR
            except Exception:
                time.sleep(0.01)
                continue

            # Resize to target, convert BGR->I420
            if frame_bgr.shape[1] != self.width or frame_bgr.shape[0] != self.height:
                frame_bgr = cv2.resize(frame_bgr, (self.width, self.height), interpolation=cv2.INTER_AREA)
            frame_yuv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2YUV_I420)

            # Create Gst.Buffer
            buf = Gst.Buffer.new_allocate(None, frame_yuv.nbytes, None)
            buf.fill(0, frame_yuv.tobytes())
            buf.pts = pts
            buf.dts = pts
            buf.duration = nanos_per_frame
            pts += nanos_per_frame

            # Push to appsrc
            ret = self.appsrc.emit("push-buffer", buf)
            if ret != Gst.FlowReturn.OK:
                # If downstream not ready yet, wait a bit
                time.sleep(0.005)

        # EOS when stopping
        try:
            self.appsrc.emit("end-of-stream")
        except Exception:
            pass

    # ───────────────────────── Lifecycle ─────────────────────────
    def start(self):
        # Camera via manager
        self.cam = CameraAdapterManager(
            crop_w_from=config.CROP_W_FROM, crop_w_to=config.CROP_W_TO,
            crop_h_from=config.CROP_H_FROM, crop_h_to=config.CROP_H_TO,
            cv_rotate_code=None,
            ispdigitalgainrange_from=config.ISP_DIGITAL_GAIN_RANGE_FROM,
            ispdigitalgainrange_to=config.ISP_DIGITAL_GAIN_RANGE_TO,
            gainrange_from=config.GAIN_RANGE_FROM, gainrange_to=config.GAIN_RANGE_TO,
            exposuretimerange_from=config.EXPOSURE_TIME_RANGE_FROM,
            exposuretimerange_to=config.EXPOSURE_TIME_RANGE_TO,
            aelock=config.AE_LOCK,
            capture_width=self.width, capture_height=self.height,
            display_width=self.width, display_height=self.height,
            framerate=self.fps,
            nvidia_flip_method=config.CAMERA_FLIP_METHOD,
            backend="auto",
        )
        print(f"[ServerCamLiveWebRTC] 🎥 Camera: {self.cam.backend()} ({self.cam.whoami()})")

        # Build pipeline and start GLib mainloop in thread
        self._build_pipeline()
        self.pipeline.set_state(Gst.State.PLAYING)
        self._gst_thread = threading.Thread(target=self._gst_mainloop, daemon=True)
        self._gst_thread.start()

        # Start capture → appsrc
        self._run_capture = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        if not self.external_app:
            print(f"[ServerCamLiveWebRTC] 🌐 Serving on http://0.0.0.0:{self.port}")
            self.app.run("0.0.0.0", self.port, debug=False, use_reloader=False)

    def stop(self):
        print("[ServerCamLiveWebRTC] 🛑 Stopping ...")
        self._run_capture = False
        try:
            if self.capture_thread:
                self.capture_thread.join(timeout=1.0)
        except Exception:
            pass

        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        if self.cam:
            self.cam.release()
        try:
            self.loop.quit()
        except Exception:
            pass
        print("[ServerCamLiveWebRTC] ✅ Stopped.")


# ─────────────────────────── Entrypoint ───────────────────────────
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    server = ServerCamLiveWebRTC(port=args.port, width=args.width, height=args.height, fps=args.fps)

    def handle_exit(sig, frame):
        print("\n[ServerCamLiveWebRTC] CTRL+C")
        server.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_exit)
    signal.signal(signal.SIGTERM, handle_exit)

    server.start()
