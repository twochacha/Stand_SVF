# stand_main.py
import sys
import time
import threading
from collections import deque
import os

# pyqtgraph должен знать, что Qt-биндинги = PySide6
os.environ.setdefault("PYQTGRAPH_QT_LIB", "PySide6")

import numpy as np
import pandas as pd
import pyqtgraph as pg
from PySide6 import QtCore
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QHBoxLayout

import serial

from reader import read_thread
from stand_gui import ControlPanel
from stand_protocol import parse_stand_packet


# ======= Настройки =======
DATA_BAUD = 921600
DATA_READ_CHUNK = 256
WINDOW_SIZE = 20000

VCP_BAUD = 115200
VCP_DIR = -1

display_channels = ["C1", "C2", "C3", "V"]
ZERO_CHANNELS = {"C1", "C2"}

# ID -> имя канала
CH_NAME = {
    0xC1: "C1",
    0xC2: "C2",
    0xC3: "C3",
    0xC4: "C4",
    0xC5: "C5",
    0xC6: "C6",
}

# КАЛИБРОВКА: y = raw*k + b
CAL = {
    "C1": {"k": 6.40539066e-5, "b": 0},
    "C2": {"k": 7.127378931e-5, "b": 0},
    "C3": {"k": -1.56784372e-5, "b": 98.1605203},
    "C4": {"k": 1.0, "b": 0.0},
    "C5": {"k": 1.0, "b": 0.0},
    "C6": {"k": 1.0, "b": 0.0},
}


def apply_cal(name: str, raw: float) -> float:
    c = CAL.get(name, {"k": 1.0, "b": 0.0})
    return raw * float(c["k"]) + float(c["b"])


# Ограничения положения
MIN_MM = 16.0
MAX_MM = 78.0
MARGIN_MM = 1.0
SAFE_MIN = MIN_MM + MARGIN_MM
SAFE_MAX = MAX_MM - MARGIN_MM
MID_MM = (SAFE_MIN + SAFE_MAX) / 2.0


def set_motion_limits(min_mm: float, max_mm: float):
    global MIN_MM, MAX_MM, SAFE_MIN, SAFE_MAX, MID_MM

    min_mm = float(min_mm)
    max_mm = float(max_mm)

    if max_mm <= min_mm:
        raise ValueError("MAX_MM должен быть больше MIN_MM")

    if (max_mm - min_mm) <= (2.0 * MARGIN_MM):
        raise ValueError(
            f"Диапазон должен быть больше {2.0 * MARGIN_MM:.1f} мм с учетом защитного отступа"
        )

    MIN_MM = min_mm
    MAX_MM = max_mm
    SAFE_MIN = MIN_MM + MARGIN_MM
    SAFE_MAX = MAX_MM - MARGIN_MM
    MID_MM = (SAFE_MIN + SAFE_MAX) / 2.0

AUTO_SPEED = 250   # мм/с
JOG_SPEED = 500    # мм/с

ZERO_SAMPLES = 500


class StandVCP:
    """
    Управление стендом по VCP (115200) с ожиданием DONE.
    - Команда плюса БЕЗ '+': A2 F500
    - Команда минуса:       A-2 F500
    - reset_input_buffer() перед каждой командой (чтобы не ловить старый DONE)
    - lock + busy (чтобы не было параллельных/двойных команд)
    """

    def __init__(self):
        self.ser: serial.Serial | None = None
        self.lock = threading.Lock()
        self.busy = False
        self._stop_flag = threading.Event()

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def abort(self):
        self._stop_flag.set()
        s = self.ser
        if not s:
            return
        try:
            s.cancel_read()
        except Exception:
            pass
        try:
            s.cancel_write()
        except Exception:
            pass

    def connect(self, port: str, baud: int = 115200, timeout: float = 0.2):
        self.disconnect()
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(0.2)
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

    def disconnect(self):
        self.abort()
        got = self.lock.acquire(timeout=0.5)
        if got:
            self.lock.release()

        if self.ser:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass

        self.ser = None
        self.busy = False

    def send_wait_done(self, cmd: str, timeout_s: float = 200.0):
        if not self.is_connected():
            return

        with self.lock:
            if self.busy:
                raise RuntimeError("VCP busy")
            self.busy = True
            self._stop_flag.clear()

            try:
                try:
                    self.ser.reset_input_buffer()
                except Exception:
                    pass

                self.ser.write((cmd + "\r\n").encode())
                self.ser.flush()

                t0 = time.time()
                while True:
                    if self._stop_flag.is_set():
                        print("[VCP] command aborted")
                        return

                    if time.time() - t0 > timeout_s:
                        print("[VCP] timeout")
                        return

                    raw = self.ser.readline()
                    if not raw:
                        continue

                    line = raw.decode(errors="ignore").strip()
                    if line.upper() == "DONE":
                        break

            finally:
                self.busy = False

    def move_rel_mm(self, delta_mm: float, speed: int):
        mm_i = int(round(abs(delta_mm)))
        if mm_i == 0:
            return

        if delta_mm >= 0:
            cmd = f"A{mm_i} F{int(speed)}"
        else:
            cmd = f"A-{mm_i} F{int(speed)}"

        self.send_wait_done(cmd)

    def jog_rel(self, delta_mm: float):
        self.move_rel_mm(delta_mm, 500)

    def move_rel_async(self, delta_mm: float, speed: int, on_done=None, on_error=None):
        def _run():
            try:
                self.move_rel_mm(delta_mm, speed)
                if on_done:
                    on_done()
            except Exception as e:
                if on_error:
                    on_error(e)

        threading.Thread(target=_run, daemon=True).start()

    def jog_async(self, delta_mm: float, on_done=None, on_error=None):
        self.move_rel_async(delta_mm, 500, on_done=on_done, on_error=on_error)

class MotionWorker:
    """Очередь команд движения (jog / auto / pressure)."""

    def __init__(self, vcp: StandVCP, get_pos_mm, get_pressure, pos_lock: threading.Lock):
        self.vcp = vcp
        self.get_pos_mm = get_pos_mm
        self.get_pressure = get_pressure
        self.pos_lock = pos_lock

        self._q = deque()
        self._qlock = threading.Lock()

        self._stop_flag = threading.Event()
        self._auto_flag = threading.Event()
        self._mode = None

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ================= PUBLIC API =================

    def enqueue_jog(self, delta_mm: float):
        with self._qlock:
            if self._auto_flag.is_set():
                return
            self._q.clear()
            self._q.append(("jog", float(delta_mm)))

    def start_auto(self, runs: int, speed: int):
        self._auto_flag.clear()
        try:
            self.vcp.abort()
        except Exception:
            pass

        with self._qlock:
            self._q.clear()
            self._q.append(("auto", int(runs), int(speed)))

    def start_pressure(self, params: dict):
        self._auto_flag.clear()
        try:
            self.vcp.abort()
        except Exception:
            pass

        with self._qlock:
            self._q.clear()
            self._q.append(("pressure", dict(params)))

    def stop_all(self):
        self._auto_flag.clear()
        self._mode = None
        with self._qlock:
            self._q.clear()

    def is_auto_running(self) -> bool:
        return self._auto_flag.is_set()

    # ================= HELPERS =================

    def _clamp_delta_to_safe(self, pos_mm: float, delta_mm: float) -> float:
        target = pos_mm + delta_mm
        if target < SAFE_MIN:
            return SAFE_MIN - pos_mm
        if target > SAFE_MAX:
            return SAFE_MAX - pos_mm
        return delta_mm

    def _send_move_physical(self, delta_mm_physical: float, speed: int):
        if abs(delta_mm_physical) < 0.5:
            return
        self.vcp.move_rel_mm(delta_mm_physical * VCP_DIR, int(speed))

    # ================= WORKER =================

    def _run(self):

        while not self._stop_flag.is_set():

            item = None
            with self._qlock:
                if self._q:
                    item = self._q.popleft()

            if item is None:
                time.sleep(0.01)
                continue

            kind = item[0]

            # ============================================================
            # ======================== JOG ===============================
            # ============================================================
            if kind == "jog":
                if not self.vcp.is_connected():
                    continue

                delta_req = float(item[1])

                with self.pos_lock:
                    pos = self.get_pos_mm()

                if pos is None:
                    continue

                # если за границей — разрешаем только внутрь
                if pos >= SAFE_MAX and delta_req > 0:
                    continue
                if pos <= SAFE_MIN and delta_req < 0:
                    continue

                delta = self._clamp_delta_to_safe(pos, delta_req)
                self._send_move_physical(delta, JOG_SPEED)
                continue

            # ============================================================
            # ======================== AUTO ==============================
            # ============================================================
            if kind == "auto":
                if not self.vcp.is_connected():
                    continue

                runs = int(item[1])
                speed = int(item[2])

                self._auto_flag.set()
                self._mode = "auto"

                legs_done = 0

                try:
                    while self._auto_flag.is_set() and legs_done < runs:

                        with self.pos_lock:
                            pos = self.get_pos_mm()

                        if pos is None:
                            break

                        # если вышли за границу — едем только внутрь
                        if pos > SAFE_MAX:
                            leg_dir = -1.0
                        elif pos < SAFE_MIN:
                            leg_dir = +1.0
                        else:
                            # внутри диапазона
                            leg_dir = +1.0 if pos <= MID_MM else -1.0

                        # считаем расстояние до края
                        edge = SAFE_MAX if leg_dir > 0 else SAFE_MIN
                        delta = edge - pos

                        delta = self._clamp_delta_to_safe(pos, delta)

                        # если двигаться некуда — выходим
                        if abs(delta) < 1.0:
                            break

                        # ОДНА команда на всю ногу
                        self._send_move_physical(delta, speed)

                        legs_done += 1

                finally:
                    self._auto_flag.clear()
                    self._mode = None

                continue

            # ============================================================
            # ====================== PRESSURE ============================
            # ============================================================
            if kind == "pressure":
                if not self.vcp.is_connected():
                    continue

                params = dict(item[1])

                step_mm   = float(params.get("step_mm", 1.0))
                p_target  = float(params.get("p_target", 5.0))
                p_limit   = float(params.get("p_limit", 10.0))

                v_min     = int(params.get("v_min", 100))
                v_max     = int(params.get("v_max", 2000))
                kp        = float(params.get("kp", 200.0))
                deadband  = float(params.get("deadband", 0.2))

                runs      = int(params.get("runs", 2))

                # антидребезг края (НО без "отъезда")
                EDGE_TOL = 0.5
                UNLATCH_EPS = 0.8
                edge_latched = False

                self._auto_flag.set()
                self._mode = "pressure"

                legs_done = 0

                # стартовое направление: если уже на/за границей — только внутрь
                with self.pos_lock:
                    pos0 = self.get_pos_mm()
                if pos0 is None:
                    self._auto_flag.clear()
                    self._mode = None
                    continue

                if pos0 >= SAFE_MAX - EDGE_TOL:
                    leg_dir = -1.0
                elif pos0 <= SAFE_MIN + EDGE_TOL:
                    leg_dir = +1.0
                else:
                    leg_dir = +1.0 if pos0 <= MID_MM else -1.0

                try:
                    while self._auto_flag.is_set() and legs_done < runs:

                        with self.pos_lock:
                            pos = self.get_pos_mm()
                        if pos is None:
                            break

                        p = self.get_pressure()
                        if p is None:
                            break

                        # ===== АВАРИЯ: ПРЕДЕЛ ДАВЛЕНИЯ =====
                        if p >= p_limit:
                            # важно: выходим так, чтобы НЕ ждать DONE бесконечно
                            self._auto_flag.clear()
                            try:
                                self.vcp.abort()
                            except Exception:
                                pass
                            break

                        # если ВЫШЛИ за границы — разрешаем ТОЛЬКО внутрь
                        if pos > SAFE_MAX:
                            leg_dir = -1.0
                        elif pos < SAFE_MIN:
                            leg_dir = +1.0

                        # ===== КРАЙ ДОСТИГНУТ? (без доводок/шагов) =====
                        edge = SAFE_MAX if leg_dir > 0 else SAFE_MIN
                        reached = (pos >= edge - EDGE_TOL) if leg_dir > 0 else (pos <= edge + EDGE_TOL)

                        if reached:
                            if not edge_latched:
                                legs_done += 1
                                edge_latched = True

                                # если цикл окончен — ВЫХОДИМ, НЕ РАЗВОРАЧИВАЕМСЯ
                                if legs_done >= runs:
                                    break

                                # иначе разворот для следующей "ноги"
                                leg_dir *= -1.0

                            time.sleep(0.01)
                            continue

                        # снятие latch только когда реально ушли внутрь
                        if edge_latched:
                            if (leg_dir > 0 and pos < SAFE_MAX - UNLATCH_EPS) or (leg_dir < 0 and pos > SAFE_MIN + UNLATCH_EPS):
                                edge_latched = False

                        # ===== РЕГУЛЯТОР СКОРОСТИ (НЕ abs(err)!) =====
                        err = p_target - p

                        # если выше цели/в deadband -> минимум скорости (не ускоряемся!)
                        if err <= deadband:
                            v_cmd = v_min
                        else:
                            v_cmd = int(v_min + kp * err)  # err > 0
                            if v_cmd > v_max:
                                v_cmd = v_max
                            if v_cmd < v_min:
                                v_cmd = v_min

                        # ===== ДВИЖЕНИЕ: строго один шаг step_mm =====
                        delta_cmd = leg_dir * step_mm
                        delta_cmd = self._clamp_delta_to_safe(pos, delta_cmd)

                        # если двигаться нельзя — просто ждём
                        if abs(delta_cmd) < 0.5:
                            time.sleep(0.01)
                            continue

                        self._send_move_physical(delta_cmd, v_cmd)

                finally:
                    # на выходе чистим режим и очередь, чтобы не было "поехал обратно после окончания"
                    self._auto_flag.clear()
                    self._mode = None
                    with self._qlock:
                        self._q.clear()

                continue

def process_stand_thread(
    data_queue,
    queue_lock,
    raw_bytes_buffer,
    raw_lock,
    lock,
    channel_buffers,
    save_buffer,
    display_channels,
    offset,
    last_values,
    last_lock, stop_event, reset_event
):
    """Парсинг потока регистратора + авто-ноль C1/C2 + CAL для всех каналов."""

    stream_buf = bytearray()
    write_index = 0
    speed_win_s = 1.0
    pos_hist = deque()
    zero_acc = {ch: 0.0 for ch in ZERO_CHANNELS}
    zero_cnt = 0
    zero_done = False

    pkt_cnt = 0
    last_print = time.time()

    # SYNC state
    locked = False
    expected_start = None
    stable_t0 = None

    EXPECT_LEN = 0x20
    STABLE_SEC = 2.0
    MAX_BUF = 8192

    def looks_like_frame_at(pos: int) -> bool:
        if pos + EXPECT_LEN > len(stream_buf):
            return False
        if stream_buf[pos + 1] != EXPECT_LEN:
            return False
        payload = stream_buf[pos + 2: pos + EXPECT_LEN]
        if (len(payload) % 5) != 0:
            return False
        for i in range(0, len(payload), 5):
            if payload[i] not in (0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6):
                return False
        return True

    def consume_one_locked() -> bytes | None:
        nonlocal expected_start, locked
        if len(stream_buf) < EXPECT_LEN:
            return None
        if stream_buf[1] != EXPECT_LEN:
            locked = False
            expected_start = None
            return None
        pkt = bytes(stream_buf[:EXPECT_LEN])
        try:
            start, _, _ = parse_stand_packet(pkt)
        except Exception:
            locked = False
            expected_start = None
            return None

        if expected_start is not None and start != expected_start:
            locked = False
            expected_start = None
            return None

        del stream_buf[:EXPECT_LEN]
        expected_start = (start + 1) & 0xFF
        return pkt

    while not stop_event.is_set():
        if reset_event.is_set():
            with lock:
                for ch in display_channels:
                    channel_buffers[ch].fill(np.nan)
                save_buffer.clear()
                write_index = 0

            with raw_lock:
                raw_bytes_buffer.clear()

            with last_lock:
                for ch in display_channels:
                    last_values[ch] = None

            pos_hist.clear()
            reset_event.clear()

        with queue_lock:
            chunk = data_queue.popleft() if data_queue else None

        if not chunk:
            time.sleep(0.001)
            continue

        stream_buf += chunk

        packets: list[bytes] = []

        if locked:
            while True:
                pkt = consume_one_locked()
                if pkt is None:
                    break
                packets.append(pkt)
        else:
            nowp = time.perf_counter()

            found_pos = None
            for pos in range(0, max(0, len(stream_buf) - EXPECT_LEN + 1)):
                if not looks_like_frame_at(pos):
                    continue

                pkt1 = bytes(stream_buf[pos:pos + EXPECT_LEN])
                pkt2_pos = pos + EXPECT_LEN
                if pkt2_pos + EXPECT_LEN > len(stream_buf):
                    continue
                if not looks_like_frame_at(pkt2_pos):
                    continue

                try:
                    s1, _, _ = parse_stand_packet(pkt1)
                    s2, _, _ = parse_stand_packet(bytes(stream_buf[pkt2_pos:pkt2_pos + EXPECT_LEN]))
                except Exception:
                    continue

                if s2 == ((s1 + 1) & 0xFF):
                    found_pos = pos
                    break

            if found_pos is not None:
                if found_pos > 0:
                    del stream_buf[:found_pos]

                if stable_t0 is None:
                    stable_t0 = nowp
                    expected_start = None

                ok_any = False
                while True:
                    if len(stream_buf) < EXPECT_LEN:
                        break
                    pkt = bytes(stream_buf[:EXPECT_LEN])
                    try:
                        start, _, _ = parse_stand_packet(pkt)
                    except Exception:
                        stable_t0 = None
                        expected_start = None
                        break

                    if expected_start is None:
                        expected_start = (start + 1) & 0xFF
                    elif start != expected_start:
                        stable_t0 = None
                        expected_start = None
                        del stream_buf[:1]
                        break

                    del stream_buf[:EXPECT_LEN]
                    expected_start = (start + 1) & 0xFF
                    ok_any = True

                if ok_any and stable_t0 is not None and (nowp - stable_t0) >= STABLE_SEC:
                    locked = True
                    stable_t0 = None
                    print("[SYNC] LOCKED по START-инкременту")

            if not locked and len(stream_buf) > MAX_BUF:
                del stream_buf[:-256]

        if not packets:
            continue

        now = time.perf_counter()

        for pkt in packets:
            try:
                _start, _lenp, pairs = parse_stand_packet(pkt)
            except Exception:
                locked = False
                expected_start = None
                continue

            frame = {ch: np.nan for ch in display_channels}

            for pid, raw in pairs:
                name = CH_NAME.get(pid)
                if name in frame:
                    frame[name] = float(raw)

            # ---- скорость V по позиции C3 (средняя за 1 секунду) ----
            pos = frame.get("C3", np.nan)
            if not np.isnan(pos):
                pos_hist.append((now, float(pos)))

                t_min = now - speed_win_s
                while pos_hist and pos_hist[0][0] < t_min:
                    pos_hist.popleft()

                if len(pos_hist) >= 2:
                    t0, p0 = pos_hist[0]
                    dt = now - t0
                    if dt > 1e-6:
                        frame["V"] = (float(pos) - p0) / dt   # мм/с
                        frame["V"] = frame["V"] / 1000
                    else:
                        frame["V"] = np.nan
                else:
                    frame["V"] = np.nan
            else:
                frame["V"] = np.nan

            # авто-ноль C1/C2
            if not zero_done:
                for ch in ZERO_CHANNELS:
                    v = frame.get(ch, np.nan)
                    if not np.isnan(v):
                        zero_acc[ch] += v
                zero_cnt += 1

                if zero_cnt >= ZERO_SAMPLES:
                    for ch in ZERO_CHANNELS:
                        offset[ch] = zero_acc[ch] / zero_cnt
                    zero_done = True
                    print(f"[ZERO] done for {sorted(ZERO_CHANNELS)}: {ZERO_SAMPLES} frames")
                continue

            # apply tare + CAL
            for ch in display_channels:
                if ch == "V":
                    continue

                raw = frame[ch]
                if np.isnan(raw):
                    continue

                if ch in ZERO_CHANNELS:
                    raw = raw - offset[ch]
                frame[ch] = apply_cal(ch, raw)

            with lock:
                for ch in display_channels:
                    channel_buffers[ch][write_index] = frame[ch]
                save_buffer.append({**frame, "t_perf": now})
                write_index = (write_index + 1) % WINDOW_SIZE

            with last_lock:
                for ch in display_channels:
                    vv = frame[ch]
                    if not np.isnan(vv):
                        last_values[ch] = float(vv)

            pkt_cnt += 1

        if time.time() - last_print >= 1.0:
            print(f"[PKT] {pkt_cnt} packets/s, stream_buf={len(stream_buf)} bytes")
            pkt_cnt = 0
            last_print = time.time()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SVF_Stand_v1")
        self.resize(1400, 850)

        # shared buffers
        self.data_queue = deque()
        self.queue_lock = threading.Lock()
        self.data_stop = threading.Event()
        self.reset_event = threading.Event()
        self.data_threads = []
        self.raw_bytes_buffer = bytearray()
        self.raw_lock = threading.Lock()

        self.lock = threading.Lock()
        self.save_buffer = []
        self.channel_buffers = {name: np.full(WINDOW_SIZE, np.nan, dtype=float) for name in display_channels}

        self.offset = {ch: 0.0 for ch in display_channels}

        self.last_values = {ch: None for ch in display_channels}
        self.last_lock = threading.RLock()

        self.vcp = StandVCP()
        self.motion = MotionWorker(self.vcp, self._get_pos_mm, self._get_pressure, self.last_lock)

        self._data_threads_started = False

        # UI
        root = QWidget()
        layout = QHBoxLayout(root)

        self.panel = ControlPanel()
        self.panel.set_motion_limits(MIN_MM, MAX_MM)
        layout.addWidget(self.panel, 0)

        self.win = pg.GraphicsLayoutWidget(show=False, title="Стенд — каналы")
        layout.addWidget(self.win, 1)

        self.setCentralWidget(root)
        self.panel.start_pressure.connect(self._start_pressure)
        self.panel.motion_limits_changed.connect(self._set_motion_limits)

        # plots
        self.curves = {}
        for i, ch in enumerate(display_channels):
            p = self.win.addPlot(row=i, col=0, title=ch)
            if ch == "C3":
                p.setLabel("left", "мм")
            elif ch == "V":
                p.setLabel("left", "мм/с")
            elif ch in ("C1", "C2"):
                p.setLabel("left", "кг")
            else:
                p.setLabel("left", "ед.")
            p.enableAutoRange(axis="y")
            self.curves[ch] = p.plot()

        # timers
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._update_plots)
        self.timer.start(20)

        self.ui_timer = QtCore.QTimer()
        self.ui_timer.timeout.connect(self._update_ui_state)
        self.ui_timer.start(200)

        # wiring: ports refresh
        self.panel.sel_data.refresh_clicked.connect(self.panel.sel_data.refresh_ports)
        self.panel.sel_vcp.refresh_clicked.connect(self.panel.sel_vcp.refresh_ports)
        self.panel.sel_data.refresh_ports()
        self.panel.sel_vcp.refresh_ports()

        # wiring: connect/disconnect
        self.panel.sel_data.connect_clicked.connect(self._connect_data)
        self.panel.sel_data.disconnect_clicked.connect(self._disconnect_data)
        self.panel.sel_vcp.connect_clicked.connect(self._connect_vcp)
        self.panel.sel_vcp.disconnect_clicked.connect(self._disconnect_vcp)

        # wiring: motion
        self.panel.start_auto.connect(self._start_auto)
        self.panel.stop_auto.connect(self._stop_auto)
        self.panel.jog.connect(self._jog)

    def _set_motion_limits(self, min_mm: float, max_mm: float):
        try:
            set_motion_limits(min_mm, max_mm)
            self.panel.set_motion_limits(MIN_MM, MAX_MM)
            print(
                f"[LIMITS] MIN_MM={MIN_MM:.1f}, MAX_MM={MAX_MM:.1f}, "
                f"SAFE_MIN={SAFE_MIN:.1f}, SAFE_MAX={SAFE_MAX:.1f}"
            )
        except Exception as e:
            print(f"[LIMITS] error: {e}")

    def _get_pos_mm(self):
        with self.last_lock:
            v = self.last_values.get("C3")
        if v is None:
            return None
        try:
            vv = float(v)
        except Exception:
            return None
        if np.isnan(vv):
            return None
        return vv

    def _get_pressure(self):
        with self.last_lock:
            c1 = self.last_values.get("C1")
            c2 = self.last_values.get("C2")

        if c1 is None and c2 is None:
            return None

        try:
            v1 = float(c1) if c1 is not None else float("-inf")
            v2 = float(c2) if c2 is not None else float("-inf")
        except Exception:
            return None

        if np.isnan(v1) and np.isnan(v2):
            return None
        if np.isnan(v1):
            return v2
        if np.isnan(v2):
            return v1
        return max(v1, v2)
    
    def _perform_reset(self):
        self.timer.stop()

        with self.lock:
            for ch in display_channels:
                self.channel_buffers[ch].fill(0.0)
            self.save_buffer.clear()

        with self.raw_lock:
            self.raw_bytes_buffer.clear()

        with self.last_lock:
            for ch in display_channels:
                self.last_values[ch] = None

        self.reset_event.clear()

        for ch in display_channels:
            self.curves[ch].setData(self.channel_buffers[ch])

        self.timer.start(20)
        print("[INFO] buffers cleared")

    def _start_pressure(self, params: dict):
        if not self.vcp.is_connected():
            print("[PCTRL] VCP not connected")
            return
        self.panel.set_auto_running(True)
        self.motion.start_pressure(params)

    def _connect_data(self):
        port = self.panel.sel_data.selected_port()
        self.data_stop = threading.Event()
        if not port:
            print("[DATA] port not selected")
            return

        if self._data_threads_started:
            print("[DATA] already started")
            self.panel.sel_data.set_connected(True)
            return

        self.data_stop.clear()
        t_read = threading.Thread(
            target=read_thread,
            args=(
                port, DATA_BAUD, DATA_READ_CHUNK,
                self.data_queue, self.queue_lock,
                self.raw_bytes_buffer, self.raw_lock,
                self.data_stop
            ),
            daemon=True,
        )

        t_proc = threading.Thread(
            target=process_stand_thread,
            args=(
                self.data_queue,
                self.queue_lock,
                self.raw_bytes_buffer,
                self.raw_lock,
                self.lock,
                self.channel_buffers,
                self.save_buffer,
                display_channels,
                self.offset,
                self.last_values,
                self.last_lock,
                self.data_stop,
                self.reset_event,
            ),
            daemon=True,
        )

        t_read.start()
        t_proc.start()
        self.data_threads = [t_read, t_proc]
        self._data_threads_started = True
        self.panel.sel_data.set_connected(True)

    def _disconnect_data(self):
        if not self._data_threads_started:
            self.panel.sel_data.set_connected(False)
            return

        self.data_stop.set()

        with self.queue_lock:
            self.data_queue.clear()

        for t in getattr(self, "data_threads", []):
            try:
                t.join(timeout=1.5)
            except Exception:
                pass

        self._perform_reset()

        self._data_threads_started = False
        self.panel.sel_data.set_connected(False)
        print("[DATA] disconnected")

    def _connect_vcp(self):
        port = self.panel.sel_vcp.selected_port()
        if not port:
            print("[VCP] port not selected")
            return
        try:
            self.vcp.connect(port)
            self.panel.sel_vcp.set_connected(True)
        except Exception as e:
            print(f"[VCP] connect error: {e}")

    def _disconnect_vcp(self):
        self.vcp.abort()
        self.motion.stop_all()
        self.vcp.disconnect()
        self.panel.sel_vcp.set_connected(False)
        print("[VCP] disconnected")

    def _start_auto(self, runs: int, speed: int):
        if not self.vcp.is_connected():
            print("[AUTO] VCP not connected")
            return
        self.panel.set_auto_running(True)
        self.motion.start_auto(runs, speed)
    
    def _clear_plot_buffers(self):
        self._perform_reset()

    def _stop_auto(self):
        self.motion.stop_all()
        self.panel.set_auto_running(False)
        self.vcp.abort()
        self.motion.stop_all()

    def _update_ui_state(self):
        if not self.motion.is_auto_running():
            self.panel.set_auto_running(False)

    def _jog(self, delta_mm: float):
        if not self.vcp.is_connected():
            print("[JOG] VCP not connected")
            return
        self.motion.enqueue_jog(delta_mm)

    def _update_plots(self):
        with self.lock:
            buffers_snapshot = {
                ch: self.channel_buffers[ch].copy()
                for ch in display_channels
            }

        for ch in display_channels:
            y = buffers_snapshot[ch]
            mask = np.isfinite(y)

            if not np.any(mask):
                self.curves[ch].clear()
                continue

            self.curves[ch].setData(y[mask])
            
        
    def keyPressEvent(self, event):
        key = event.key()

        if key == QtCore.Qt.Key.Key_N:
            try:
                self._perform_reset()
                print("[INFO] clear requested")
            except Exception as e:
                print(f"[N ERROR] {e}")

            event.accept()
            return

        elif key == QtCore.Qt.Key.Key_S:
            try:
                with self.lock:
                    if not self.save_buffer:
                        print("[INFO] nothing to save")
                        event.accept()
                        return
                    rows = list(self.save_buffer)

                df = pd.DataFrame(rows)
                if df.empty or "t_perf" not in df.columns:
                    print("[ERROR] save_buffer invalid")
                    event.accept()
                    return

                t0 = df["t_perf"].iloc[0]
                df["Time (s)"] = df["t_perf"] - t0
                df = df.drop(columns=["t_perf"])

                ts = time.strftime("%Y_%m_%d_%H_%M_%S")
                fname = f"stand_data_{ts}.csv"
                df.to_csv(fname, index=False)
                print(f"[INFO] saved: {fname}")

                with self.raw_lock:
                    raw_name = f"raw_bytes_{ts}.txt"
                    with open(raw_name, "w", encoding="utf-8") as f:
                        f.write(" ".join(f"{b:02X}" for b in self.raw_bytes_buffer))

                self._perform_reset()
                print("[INFO] clear requested after save")

            except Exception as e:
                print(f"[S ERROR] {e}")

            event.accept()
            return

        super().keyPressEvent(event)
            
if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())
