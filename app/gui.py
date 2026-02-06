#!/usr/bin/env python3
"""
Hamamatsu C7942 X-ray GUI Application
Dear PyGui interface for camera control, image display, and Faxitron integration.
"""

import sys
import os
import time
import threading
import pathlib
import numpy as np

import dearpygui.dearpygui as dpg

# Allow importing app.py from same directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from app import HamamatsuTeensy

FRAME_W = HamamatsuTeensy.FRAME_WIDTH   # 2400
FRAME_H = HamamatsuTeensy.FRAME_HEIGHT  # 2320
ASPECT  = FRAME_W / FRAME_H             # ~1.034
DARK_DIR = pathlib.Path.home() / ".xray"
DARK_PATH = DARK_DIR / "dark_field.npy"

# Display at reduced resolution to keep the texture manageable
DISP_SCALE = 4
DISP_W = FRAME_W // DISP_SCALE  # 600
DISP_H = FRAME_H // DISP_SCALE  # 580


class XrayGUI:
    def __init__(self):
        # Hardware
        self.teensy = None

        # Acquisition state
        self.acq_mode = "idle"
        self.acq_thread = None
        self.acq_stop = threading.Event()
        self.integration_time = 1.0  # seconds between dual-shot pairs

        # Frame data (protected by lock)
        self.frame_lock = threading.Lock()
        self.raw_frame = None           # float32 (H, W), latest single raw
        self.display_frame = None       # float32 (H, W), after integration + dark
        self.frame_buffer = []          # list of float32 frames for integration
        self.integration_n = 1          # target buffer size
        self.new_frame_ready = threading.Event()

        # Dark field
        self.dark_field = None
        self._load_dark_field()

        # Windowing
        self.win_min = 0.0
        self.win_max = 4095.0
        self.hist_eq = False

        # Stats
        self.frame_count = 0
        self.fps = 0.0
        self._fps_time = time.time()
        self._fps_count = 0
        self._status_msg = ""

        # Progress tracking (written by worker, read by main)
        self._progress = 0.0       # 0.0 - 1.0
        self._progress_text = ""   # e.g. "Capturing 3/8..."

        # DPG ids (assigned in _build_ui)
        self._texture_id = None

    # ── Dark field persistence ──────────────────────────────────────

    def _load_dark_field(self):
        if DARK_PATH.exists():
            try:
                self.dark_field = np.load(DARK_PATH).astype(np.float32)
            except Exception:
                self.dark_field = None

    def _save_dark_field(self):
        DARK_DIR.mkdir(parents=True, exist_ok=True)
        np.save(DARK_PATH, self.dark_field)

    # ── Acquisition helpers (run in worker thread) ──────────────────

    def _trigger_and_read(self):
        """Trigger sensor and block until frame is ready. Returns float32 (H,W)."""
        self.teensy.start_trigger()
        for _ in range(200):  # 20s timeout
            if self.acq_stop.is_set():
                return None
            state = self.teensy.get_state()
            if state['state'] == 3:  # DONE
                break
            time.sleep(0.1)
        else:
            raise TimeoutError("Frame acquisition timed out")
        raw = self.teensy.get_frame()
        pixels = HamamatsuTeensy.unpack_12bit(raw)
        return pixels.reshape((FRAME_H, FRAME_W)).astype(np.float32)

    def _push_frame(self, frame):
        """Apply dark subtraction, buffer, integrate, signal main thread."""
        if self.dark_field is not None:
            frame = frame - self.dark_field

        with self.frame_lock:
            self.raw_frame = frame
            self.frame_buffer.append(frame)
            if len(self.frame_buffer) > self.integration_n:
                self.frame_buffer = self.frame_buffer[-self.integration_n:]
            self.display_frame = np.mean(self.frame_buffer, axis=0)

        self.frame_count += 1
        now = time.time()
        self._fps_count += 1
        dt = now - self._fps_time
        if dt >= 1.0:
            self.fps = self._fps_count / dt
            self._fps_count = 0
            self._fps_time = now

        self.new_frame_ready.set()

    def _do_single_shot(self):
        self._progress_text = "Acquiring..."
        self._progress = 0.5
        frame = self._trigger_and_read()
        if frame is not None:
            self._push_frame(frame)
        self._progress = 1.0

    def _do_dual_shot(self):
        # First shot: clear sensor
        self._progress_text = "Clearing sensor..."
        frame = self._trigger_and_read()
        if frame is None or self.acq_stop.is_set():
            return
        # Wait integration time
        self._progress_text = "Integrating..."
        t_start = time.time()
        while time.time() < t_start + self.integration_time:
            if self.acq_stop.is_set():
                return
            elapsed = time.time() - t_start
            self._progress = min(elapsed / max(self.integration_time, 0.01), 1.0)
            time.sleep(0.05)
        # Second shot: keep
        self._progress_text = "Reading..."
        frame = self._trigger_and_read()
        if frame is not None:
            self._push_frame(frame)

    def _acquisition_worker(self):
        mode = self.acq_mode
        try:
            if mode == "single":
                self._do_single_shot()
            elif mode == "dual":
                self._do_dual_shot()
            elif mode == "continuous":
                i = 0
                while not self.acq_stop.is_set():
                    i += 1
                    self._progress_text = f"Continuous #{i}"
                    self._do_dual_shot()
            elif mode == "capture_n":
                for i in range(self.integration_n):
                    if self.acq_stop.is_set():
                        break
                    self._progress_text = f"Capturing {i+1}/{self.integration_n}"
                    self._progress = i / self.integration_n
                    self._do_dual_shot()
                self._progress = 1.0
            elif mode == "dark":
                self._progress_text = "Capturing dark..."
                self._progress = 0.5
                frame = self._trigger_and_read()
                if frame is not None:
                    with self.frame_lock:
                        self.dark_field = frame
                    self._save_dark_field()
                    self._status_msg = "Dark field captured"
                self._progress = 1.0
        except Exception as e:
            self._status_msg = f"Error: {e}"
        finally:
            self.acq_mode = "idle"
            self._progress = 0.0
            self._progress_text = ""

    def _start_acquisition(self, mode):
        if self.teensy is None:
            self._status_msg = "Not connected"
            return
        if self.acq_mode != "idle":
            return
        self.acq_mode = mode
        self.acq_stop.clear()
        self._progress = 0.0
        self.acq_thread = threading.Thread(target=self._acquisition_worker, daemon=True)
        self.acq_thread.start()

    def _stop_acquisition(self):
        self.acq_stop.set()

    # ── Display pipeline (main thread) ──────────────────────────────

    def _frame_to_texture(self, frame):
        """Apply windowing and convert to RGBA float32 flat array for DPG texture."""
        # Downsample for display
        ds = frame.reshape(DISP_H, DISP_SCALE, DISP_W, DISP_SCALE).mean(axis=(1, 3))

        if self.hist_eq:
            ds = self._histogram_equalize(ds)
        else:
            lo, hi = self.win_min, self.win_max
            if hi <= lo:
                hi = lo + 1
            ds = (ds - lo) / (hi - lo)

        ds = np.clip(ds, 0.0, 1.0).astype(np.float32)

        # Build RGBA (grayscale: R=G=B=val, A=1)
        rgba = np.empty((DISP_H, DISP_W, 4), dtype=np.float32)
        rgba[:, :, 0] = ds
        rgba[:, :, 1] = ds
        rgba[:, :, 2] = ds
        rgba[:, :, 3] = 1.0
        return rgba.flatten().tolist()

    @staticmethod
    def _histogram_equalize(img):
        flat = img.flatten()
        # Use the actual data range, not a fixed 0-4096
        lo, hi = float(flat.min()), float(flat.max())
        if hi <= lo:
            return np.zeros_like(img)
        # Map to 4096 bins within the data range
        nbins = 4096
        hist, bins = np.histogram(flat, bins=nbins, range=(lo, hi))
        cdf = hist.cumsum().astype(np.float64)
        cdf_max = cdf[-1]
        if cdf_max == 0:
            return np.zeros_like(img)
        cdf_norm = cdf / cdf_max
        # Map pixel values to bin indices
        indices = np.clip(((img - lo) / (hi - lo) * (nbins - 1)), 0, nbins - 1).astype(np.int32)
        return cdf_norm[indices].astype(np.float32)

    def _update_display(self):
        """Called from main thread when new_frame_ready is set."""
        with self.frame_lock:
            if self.display_frame is None:
                return
            frame = self.display_frame.copy()

        # Update texture
        texture_data = self._frame_to_texture(frame)
        dpg.set_value(self._texture_id, texture_data)

        # Update histogram (normalized to 0-1 on Y)
        flat = frame.flatten()
        lo, hi = float(flat.min()), float(flat.max())
        hist_vals, hist_edges = np.histogram(flat, bins=256, range=(lo, hi) if hi > lo else (0, 1))
        peak = hist_vals.max()
        if peak > 0:
            hist_norm = (hist_vals / peak).tolist()
        else:
            hist_norm = [0.0] * len(hist_vals)
        hist_centers = (hist_edges[:-1] + hist_edges[1:]) / 2
        zeros = [0] * len(hist_centers)
        dpg.set_value("hist_series", [hist_centers.tolist(), hist_norm, zeros])
        dpg.set_axis_limits_constraints("hist_x", lo, hi)
        dpg.fit_axis_data("hist_x")
        dpg.set_axis_limits("hist_y", 0.0, 1.05)

    def _refresh_texture_from_settings(self):
        """Re-render current display_frame with new windowing settings."""
        with self.frame_lock:
            if self.display_frame is None:
                return
            frame = self.display_frame.copy()
        texture_data = self._frame_to_texture(frame)
        dpg.set_value(self._texture_id, texture_data)

    # ── Callbacks ───────────────────────────────────────────────────

    def _cb_connect(self, sender=None, app_data=None):
        try:
            self.teensy = HamamatsuTeensy()
            self.teensy.ping()
            self._status_msg = "Connected"
            dpg.set_value("conn_status", "Connected")
        except Exception as e:
            self.teensy = None
            self._status_msg = f"Connection failed: {e}"
            dpg.set_value("conn_status", f"Failed: {e}")

    def _cb_start(self, sender=None, app_data=None):
        mode = dpg.get_value("acq_mode_combo")
        mode_map = {
            "Single Shot": "single",
            "Dual Shot": "dual",
            "Continuous": "continuous",
            "Capture N": "capture_n",
        }
        self.integration_time = dpg.get_value("integ_time_slider")
        self.integration_n = int(dpg.get_value("integ_n_slider"))
        self._start_acquisition(mode_map.get(mode, "single"))

    def _cb_stop(self, sender=None, app_data=None):
        self._stop_acquisition()

    def _cb_auto_window(self, sender=None, app_data=None):
        with self.frame_lock:
            if self.display_frame is None:
                return
            frame = self.display_frame.copy()
        lo = float(np.percentile(frame, 1))
        hi = float(np.percentile(frame, 99))
        self.win_min = lo
        self.win_max = hi
        dpg.set_value("win_min_drag", lo)
        dpg.set_value("win_max_drag", hi)
        dpg.set_value("hist_min_line", lo)
        dpg.set_value("hist_max_line", hi)
        self._refresh_texture_from_settings()

    def _cb_hist_eq_toggle(self, sender, value):
        self.hist_eq = value
        self._refresh_texture_from_settings()

    def _cb_win_min_changed(self, sender, value):
        self.win_min = value
        dpg.set_value("hist_min_line", value)
        self._refresh_texture_from_settings()

    def _cb_win_max_changed(self, sender, value):
        self.win_max = value
        dpg.set_value("hist_max_line", value)
        self._refresh_texture_from_settings()

    def _cb_hist_min_dragged(self, sender, app_data):
        val = dpg.get_value(sender)
        if isinstance(val, (list, tuple)):
            val = val[0]
        self.win_min = float(val)
        dpg.set_value("win_min_drag", self.win_min)
        self._refresh_texture_from_settings()

    def _cb_hist_max_dragged(self, sender, app_data):
        val = dpg.get_value(sender)
        if isinstance(val, (list, tuple)):
            val = val[0]
        self.win_max = float(val)
        dpg.set_value("win_max_drag", self.win_max)
        self._refresh_texture_from_settings()

    def _cb_clear_buffer(self, sender=None, app_data=None):
        with self.frame_lock:
            self.frame_buffer.clear()
            self.display_frame = None
        self._status_msg = "Buffer cleared"

    def _cb_capture_n(self, sender=None, app_data=None):
        self.integration_n = int(dpg.get_value("integ_n_slider"))
        self._start_acquisition("capture_n")

    def _cb_capture_dark(self, sender=None, app_data=None):
        self._start_acquisition("dark")

    def _cb_clear_dark(self, sender=None, app_data=None):
        self.dark_field = None
        if DARK_PATH.exists():
            DARK_PATH.unlink()
        self._status_msg = "Dark field cleared"

    def _cb_export_png(self, sender=None, app_data=None):
        dpg.show_item("file_dialog")

    def _cb_file_selected(self, sender, app_data):
        filepath = app_data.get("file_path_name", "")
        if not filepath:
            return
        if not filepath.lower().endswith(".png"):
            filepath += ".png"
        with self.frame_lock:
            if self.display_frame is None:
                self._status_msg = "No frame to export"
                return
            frame = self.display_frame.copy()

        # Apply current windowing
        lo, hi = self.win_min, self.win_max
        if hi <= lo:
            hi = lo + 1
        normed = np.clip((frame - lo) / (hi - lo), 0, 1)
        img8 = (normed * 255).astype(np.uint8)

        try:
            from PIL import Image
            Image.fromarray(img8, mode='L').save(filepath)
            self._status_msg = f"Exported: {filepath}"
        except Exception as e:
            self._status_msg = f"Export failed: {e}"

    def _cb_faxitron_expose(self, sender=None, app_data=None):
        if self.teensy is None:
            self._status_msg = "Not connected"
            return
        def _do():
            try:
                v = int(dpg.get_value("fax_voltage"))
                t = float(dpg.get_value("fax_exposure"))
                mode_str = dpg.get_value("fax_mode_combo")
                mode = (HamamatsuTeensy.FAXITRON_MODE_REMOTE if mode_str == "Remote"
                        else HamamatsuTeensy.FAXITRON_MODE_FRONT_PANEL)
                self.teensy.set_faxitron_mode(mode)
                self.teensy.set_faxitron_voltage(v)
                self.teensy.set_faxitron_exposure_time(t)
                dpg.set_value("fax_status", "Exposing...")
                self.teensy.perform_faxitron_exposure()
                dpg.set_value("fax_status", "Done")
            except Exception as e:
                dpg.set_value("fax_status", f"Error: {e}")
        threading.Thread(target=_do, daemon=True).start()

    def _cb_faxitron_refresh(self, sender=None, app_data=None):
        if self.teensy is None:
            return
        def _do():
            try:
                state = self.teensy.get_faxitron_state() or "unknown"
                dpg.set_value("fax_status", state)
            except Exception as e:
                dpg.set_value("fax_status", f"Error: {e}")
        threading.Thread(target=_do, daemon=True).start()

    # ── Build UI ────────────────────────────────────────────────────

    def _build_ui(self):
        # Texture registry
        blank = [0.0] * (DISP_W * DISP_H * 4)
        with dpg.texture_registry():
            self._texture_id = dpg.add_dynamic_texture(
                width=DISP_W, height=DISP_H, default_value=blank
            )

        # File dialog for PNG export
        with dpg.file_dialog(
            directory_selector=False, show=False, tag="file_dialog",
            callback=self._cb_file_selected, width=600, height=400,
            default_filename="xray_frame.png"
        ):
            dpg.add_file_extension(".png")

        # Main window
        with dpg.window(tag="primary"):
            # Menu bar
            with dpg.menu_bar():
                with dpg.menu(label="File"):
                    dpg.add_menu_item(label="Export PNG...", callback=self._cb_export_png)

            # Two-column layout
            with dpg.group(horizontal=True):
                # Left: image display + status
                with dpg.child_window(width=-270, tag="image_panel"):
                    dpg.add_image(self._texture_id, tag="main_image")
                    # Status area at bottom
                    dpg.add_separator()
                    dpg.add_text("Idle", tag="status_text")
                    dpg.add_progress_bar(default_value=0.0, tag="progress_bar", width=-1)
                    dpg.add_text("Frames: 0 | FPS: 0.0", tag="stats_text")

                # Right: control panel
                with dpg.child_window(width=250, tag="control_panel"):
                    # ── Connection ──
                    with dpg.collapsing_header(label="Connection", default_open=True):
                        dpg.add_button(label="Connect", callback=self._cb_connect, width=-1)
                        dpg.add_text("Disconnected", tag="conn_status")

                    # ── Acquisition ──
                    with dpg.collapsing_header(label="Acquisition", default_open=True):
                        dpg.add_combo(
                            items=["Single Shot", "Dual Shot", "Continuous", "Capture N"],
                            default_value="Dual Shot", tag="acq_mode_combo", width=-1
                        )
                        dpg.add_slider_float(
                            label="Integ. time (s)", default_value=1.0,
                            min_value=0.0, max_value=10.0, tag="integ_time_slider", width=-60
                        )
                        with dpg.group(horizontal=True):
                            dpg.add_button(label="Start", callback=self._cb_start, width=115)
                            dpg.add_button(label="Stop", callback=self._cb_stop, width=115)

                    # ── Integration ──
                    with dpg.collapsing_header(label="Integration", default_open=True):
                        dpg.add_slider_int(
                            label="N frames", default_value=1,
                            min_value=1, max_value=32, tag="integ_n_slider", width=-60
                        )
                        with dpg.group(horizontal=True):
                            dpg.add_button(label="Clear Buffer", callback=self._cb_clear_buffer, width=115)
                            dpg.add_button(label="Capture N", callback=self._cb_capture_n, width=115)

                    # ── Image Controls ──
                    with dpg.collapsing_header(label="Image", default_open=True):
                        with dpg.group(horizontal=True):
                            dpg.add_button(label="Auto Window", callback=self._cb_auto_window, width=115)
                            dpg.add_checkbox(label="Hist EQ", callback=self._cb_hist_eq_toggle, tag="hist_eq_cb")
                        dpg.add_drag_float(
                            label="Min", default_value=0.0, speed=10,
                            min_value=-1000, max_value=8000,
                            callback=self._cb_win_min_changed, tag="win_min_drag", width=-40
                        )
                        dpg.add_drag_float(
                            label="Max", default_value=4095.0, speed=10,
                            min_value=-1000, max_value=8000,
                            callback=self._cb_win_max_changed, tag="win_max_drag", width=-40
                        )

                    # ── Histogram ──
                    with dpg.collapsing_header(label="Histogram", default_open=True):
                        with dpg.plot(
                            height=120, width=-1, tag="hist_plot",
                            no_title=True, no_mouse_pos=True,
                            no_box_select=True,
                        ):
                            dpg.add_plot_axis(
                                dpg.mvXAxis, label="", tag="hist_x",
                                no_tick_labels=True,
                            )
                            with dpg.plot_axis(
                                dpg.mvYAxis, label="", tag="hist_y",
                                no_tick_labels=True, lock_min=True, lock_max=True,
                            ):
                                dpg.add_shade_series(
                                    [0], [0], y2=[0], tag="hist_series"
                                )
                            # Draggable lines for min/max
                            dpg.add_drag_line(
                                label="Min", color=[255, 100, 100, 255],
                                default_value=0.0, tag="hist_min_line",
                                callback=self._cb_hist_min_dragged
                            )
                            dpg.add_drag_line(
                                label="Max", color=[100, 100, 255, 255],
                                default_value=4095.0, tag="hist_max_line",
                                callback=self._cb_hist_max_dragged
                            )

                    # ── Dark Field ──
                    with dpg.collapsing_header(label="Dark Field", default_open=False):
                        with dpg.group(horizontal=True):
                            dpg.add_button(label="Capture Dark", callback=self._cb_capture_dark, width=115)
                            dpg.add_button(label="Clear Dark", callback=self._cb_clear_dark, width=115)
                        dark_status = "Loaded" if self.dark_field is not None else "None"
                        dpg.add_text(f"Dark: {dark_status}", tag="dark_status")

                    # ── Faxitron ──
                    with dpg.collapsing_header(label="Faxitron", default_open=False):
                        dpg.add_slider_int(
                            label="kV", default_value=20,
                            min_value=1, max_value=35, tag="fax_voltage", width=-40
                        )
                        dpg.add_input_float(
                            label="Exp (s)", default_value=5.0, step=0.1,
                            min_value=0.1, max_value=99.9, tag="fax_exposure", width=-60
                        )
                        dpg.add_combo(
                            items=["Remote", "Front Panel"],
                            default_value="Remote", tag="fax_mode_combo", width=-1
                        )
                        with dpg.group(horizontal=True):
                            dpg.add_button(label="Expose", callback=self._cb_faxitron_expose, width=115)
                            dpg.add_button(label="Refresh", callback=self._cb_faxitron_refresh, width=115)
                        dpg.add_text("--", tag="fax_status")

    # ── Main loop ───────────────────────────────────────────────────

    def _resize_image(self):
        """Scale image to fill available space while maintaining aspect ratio."""
        try:
            pw, ph = dpg.get_item_rect_size("image_panel")
        except Exception:
            return
        # Reserve space for status bar area (~70px: separator + text + progress + stats)
        avail_w = max(pw - 16, 10)   # padding
        avail_h = max(ph - 80, 10)

        # Fit to available space maintaining aspect ratio
        if avail_w / avail_h > ASPECT:
            # Height-limited
            img_h = int(avail_h)
            img_w = int(img_h * ASPECT)
        else:
            # Width-limited
            img_w = int(avail_w)
            img_h = int(img_w / ASPECT)

        dpg.configure_item("main_image", width=img_w, height=img_h)

    def _render_tick(self):
        """Called every frame from the render loop."""
        if self.new_frame_ready.is_set():
            self.new_frame_ready.clear()
            self._update_display()

        # Scale image to panel
        self._resize_image()

        # Update progress bar
        dpg.set_value("progress_bar", self._progress)
        overlay = self._progress_text if self._progress_text else ""
        dpg.configure_item("progress_bar", overlay=overlay)

        # Update status text
        if self.acq_mode != "idle":
            mode_names = {
                "single": "Single Shot", "dual": "Dual Shot",
                "continuous": "Continuous", "capture_n": "Capture N",
                "dark": "Dark Capture",
            }
            status = mode_names.get(self.acq_mode, self.acq_mode)
        else:
            status = "Idle"
        if self._status_msg:
            status += f"  --  {self._status_msg}"
        dpg.set_value("status_text", status)

        # Update stats line
        dark_str = " | Dark: active" if self.dark_field is not None else ""
        buf_n = len(self.frame_buffer)
        stats = f"Frames: {self.frame_count} | FPS: {self.fps:.1f} | Buffer: {buf_n}/{self.integration_n}{dark_str}"
        dpg.set_value("stats_text", stats)

        # Update dark status text
        dark_status = "Loaded" if self.dark_field is not None else "None"
        dpg.set_value("dark_status", f"Dark: {dark_status}")

    def run(self):
        dpg.create_context()
        self._build_ui()

        dpg.create_viewport(title="Hamamatsu C7942 X-ray", width=1200, height=800)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("primary", True)

        while dpg.is_dearpygui_running():
            self._render_tick()
            dpg.render_dearpygui_frame()

        # Cleanup
        self._stop_acquisition()
        if self.acq_thread and self.acq_thread.is_alive():
            self.acq_thread.join(timeout=2.0)
        dpg.destroy_context()


if __name__ == "__main__":
    XrayGUI().run()
