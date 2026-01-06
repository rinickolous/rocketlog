from pathlib import Path
from typing import Optional

import gi
from PySide6 import QtCore, QtGui

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")


from gi.repository import Gst, GstApp  # type: ignore # noqa: E402


class GstVideo(QtCore.QObject):
    """
    Builds a GStreamer pipeline and delivers frames to Qt via signals.
    Uses appsink for preview so we can display in a QLabel.

    Two modes:
    - preview pipeline: v4l2src -> videoconvert -> RGB -> appsink
    - record pipeline: v4l2src -> videoconvert -> tee -> (RGB -> appsink) + (x264 -> mp4mux -> filesink)
    """

    frame_ready = QtCore.Signal(QtGui.QImage)
    error = QtCore.Signal(str)
    info = QtCore.Signal(str)

    # ---------------------------------------- #

    def __init__(self, camera_device: Optional[str] = None, parent=None):
        super().__init__(parent)
        self._device_id = camera_device

        self.pipeline: Optional[Gst.Pipeline] = None
        self.appsink: Optional[GstApp.AppSink] = None

        self._bus_timer = QtCore.QTimer(self)
        self._bus_timer.setInterval(50)
        self._bus_timer.timeout.connect(self._poll_bus)

        self._is_record_pipeline = False
        self._current_pipeline_str = ""

    # ---------------------------------------- #

    @property
    def current_pipeline_str(self) -> str:
        return self._current_pipeline_str

    # ---------------------------------------- #

    def set_source(self, device_id: str) -> None:
        """
        Switch preview source. For v4l2 this is usually '/dev/video0'.
        Restarts the preview pipeline.
        """
        self._device_id = device_id
        self.stop_preview()
        self.start_preview()

    # ---------------------------------------- #

    def start_preview(self) -> None:
        self.stop_recording()

        device = getattr(self, "_device_id", "/dev/video0")
        # v4l2src: selected device
        # videoconvert to RGB
        # appsink emit-signals so we can receive frames
        # pipeline_str = (
        #     f"v4l2src {device_part}do-timestamp=true ! "
        #     f"videoconvert ! video/x-raw,format=RGB ! "
        #     f"appsink name=appsink emit-signals=true sync=false max-buffers=1 drop=true"
        # )
        pipeline_str = (
            f"v4l2src device={device} do-timestamp=true ! "
            "videoconvert ! videoscale ! "
            "video/x-raw,format=RGB,width=1280,height=720,framerate=30/1 ! "
            "appsink name=appsink emit-signals=true max-buffers=1 drop=true sync=false"
        )

        self._build_and_play(pipeline_str, is_record=False)

    # ---------------------------------------- #

    def stop_preview(self) -> None:
        if self.pipeline is not None:
            try:
                # Stop playback and release the device
                self.pipeline.set_state(Gst.State.NULL)
            finally:
                self.pipeline = None
                self.appsink = None

    # ---------------------------------------- #

    def start_recording(self, mp4_path: Path) -> None:
        self.stop_recording()

        device = getattr(self, "_device_id", "/dev/video0")
        # v4l2src: selected device
        # tee to split stream
        # branch 1: RGB -> appsink for preview
        # branch 2: x264enc -> mp4mux -> filesink for recording
        pipeline_str = (
            f"v4l2src device={device} do-timestamp=true ! "
            f"videoconvert ! tee name=t "
            f"t. ! queue ! video/x-raw,format=RGB ! "
            f"appsink name=appsink emit-signals=true sync=false max-buffers=1 drop=true "
            f"t. ! queue ! videoconvert ! "
            f"x264enc speed-preset=ultrafast tune=zerolatency bitrate=4000 key-int-max=30 ! "
            f'mp4mux faststart=true ! filesink location="{str(mp4_path)}"'
        )

        self._build_and_play(pipeline_str, is_record=True)

    def stop_recording(self) -> None:
        self._bus_timer.stop()
        if self.pipeline is not None:
            try:
                self.pipeline.set_state(Gst.State.NULL)
            except Exception:
                pass
        self.pipeline = None
        self.appsink = None
        self._is_record_pipeline = False
        self._current_pipeline_str = ""

    # ---------------------------------------- #

    def stop_recording_gracefully(self, timeout_s: float = 3.0) -> None:
        """
        Ensure MP4 is finalized:
        - Send EOS
        - wait for EOS message on bus or timeout
        - set NULL
        """
        if self.pipeline is None:
            return

        if not self._is_record_pipeline:
            self.stop_recording()
            return

        try:
            self.pipeline.send_event(Gst.Event.new_eos())
            bus = self.pipeline.get_bus()
            # Wait for EOS or ERROR
            msg = bus.timed_pop_filtered(
                int(timeout_s * Gst.SECOND),
                Gst.MessageType.EOS | Gst.MessageType.ERROR,
            )
            if msg is not None:
                if msg.type == Gst.MessageType.ERROR:
                    err, dbg = msg.parse_error()
                    self.error.emit(
                        f"GStreamer Error during EOS: {err.message}\n{dbg or ''}"
                    )
        except Exception as e:
            self.error.emit(f"Failed to stop recoridng gracefully: {e}")
        finally:
            self.stop_recording()

    # ---------------------------------------- #
    #   Internal                               #
    # ---------------------------------------- #

    def _build_and_play(self, pipeline_str: str, is_record: bool) -> None:
        self._current_pipeline_str = pipeline_str
        self._is_record_pipeline = is_record

        try:
            pipeline = Gst.parse_launch(pipeline_str)
            if not isinstance(pipeline, Gst.Pipeline):
                pipeline = pipeline  # parse_launch returns a Gst.Element; it is still a pipeline-like bin
            self.pipeline = pipeline

            appsink = pipeline.get_by_name("appsink")
            if appsink is None:
                raise RuntimeError("Failed to get appsink from pipeline.")

            # Attach callback
            appsink.connect("new-sample", self._on_new_sample)
            self.appsink = appsink

            ret = pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("Failed to set pipeline to PLAYING state.")

            self._bus_timer.start()
            self.info.emit("Started GStreamer pipeline.")
        except Exception as e:
            self.error.emit(
                f"Failed to start GStreamer pipeline:\n{e}\n\nPipeline:\n{pipeline_str}"
            )
            self.stop_recording()

    # ---------------------------------------- #

    def _poll_bus(self) -> None:
        if self.pipeline is None:
            return

        bus = self.pipeline.get_bus()
        while True:
            msg = bus.pop()
            if msg is None:
                break

            if msg.type == Gst.MessageType.ERROR:
                err, dbg = msg.parse_error()
                self.error.emit(f"GStreamer Error: {err.message}\n{dbg or ''}")
                self.stop_recording()
                break
            elif msg.type == Gst.MessageType.WARNING:
                warn, dbg = msg.parse_warning()
                self.info.emit(f"GStreamer Warning: {warn.message}\n{dbg or ''}")
            elif msg.type == Gst.MessageType.STATE_CHANGED:
                old_state, new_state, pending_state = msg.parse_state_changed()
                if msg.src == self.pipeline:
                    self.info.emit(
                        f"GStreamer Pipeline state changed: {old_state.value_nick} -> {new_state.value_nick}"
                    )

    # ---------------------------------------- #

    def _on_new_sample(self, sink: GstApp.AppSink) -> None:
        """
        Runs on a GStreamer streaming thread.
        Convert buffer -> QImage (RGB) and emit to Qt.
        """
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        caps = sample.get_caps()
        s = caps.get_structure(0)
        width = s.get_value("width")
        height = s.get_value("height")

        # Map Buffer
        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.ERROR

        try:
            # Copy bytes out
            data = bytes(mapinfo.data)

            # Create QImage from RGB packed data
            # bytes_per_line = width * 3
            img = QtGui.QImage(
                data, width, height, width * 3, QtGui.QImage.Format.Format_RGB888
            ).copy()
            self.frame_ready.emit(img)
        finally:
            buf.unmap(mapinfo)

        return Gst.FlowReturn.OK
