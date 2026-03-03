# stand_main.py
import sys
import time
import threading
from collections import deque
import os
import PyQt5

os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = os.path.join(
    os.path.dirname(PyQt5.__file__), "Qt5", "plugins", "platforms"
)

import numpy as np
import pandas as pd
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QHBoxLayout

import serial

from reader import read_thread
from stand_gui import ControlPanel
from stand_protocol import parse_stand_packet


# ======= Настройки =======
DATA_BAUD = 921600
DATA_READ_CHUNK = 256
WINDOW_SIZE = 9000

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


# Ограничения положения (с запасом по 2 мм)
MIN_MM = 16.0
MAX_MM = 78.0
MARGIN_MM = 2.0
SAFE_MIN = MIN_MM + MARGIN_MM   # 18
SAFE_MAX = MAX_MM - MARGIN_MM   # 76
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
        # Работает на Windows (pyserial)
            s.cancel_read()
        except Exception:
            pass
        try:
            s.cancel_write()
        except Exception:
            pass
    def connect(self, port: str, baud: int = 115200, timeout: float = 0.2):
        # всегда закрываем старое
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

    @staticmethod
    def fmt_move_cmd(delta_mm: float, speed: int) -> str:
        d = int(round(delta_mm))
        if d >= 0:
            return f"A{d} F{speed}"      # ВАЖНО: без '+'
        else:
            return f"A-{abs(d)} F{speed}"

    def send_wait_done(self, cmd: str, timeout_s: float = 30.0):
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

    # ВАЖНО: для плюса НЕ пишем '+'
        if delta_mm >= 0:
            cmd = f"A{mm_i} F{int(speed)}"        # было A+{mm_i}
        else:
            cmd = f"A-{mm_i} F{int(speed)}"

        self.send_wait_done(cmd)

    def jog_rel(self, delta_mm: float):
        self.move_rel_mm(delta_mm, 500)

    # --- async (чтобы UI не фризился) ---
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
        self.get_pressure = get_pressure  # функция -> float|None (max(C1,C2))
        self.pos_lock = pos_lock

        self._q = deque()
        self._qlock = threading.Lock()

        self._stop_flag = threading.Event()
        self._auto_flag = threading.Event()        # общий флаг режима (auto/pressure)
        self._mode = None                          # "auto" | "pressure" | None

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ---------- public API ----------

    def enqueue_jog(self, delta_mm: float):
        """Ручной шаг. Если идёт авто/pressure — игнорируем."""
        with self._qlock:
            if self._auto_flag.is_set():
                return
            # оставляем только последний jog
            self._q = deque([it for it in self._q if it[0] != "jog"])
            # если уже есть задача — не добавляем jog (чтобы не “ехать бесконечно”)
            if self._q:
                return
            self._q.append(("jog", float(delta_mm)))

    def start_auto(self, runs: int, speed: int):
        with self._qlock:
            # чистим хвост старых auto/pressure задач
            self._q = deque([it for it in self._q if it[0] not in ("auto", "pressure")])
            self._q.append(("auto", int(runs), int(speed)))

    def start_pressure(self, params: dict):
        """
        params:
          step_mm: float
          p_target: float
          p_limit: float
          v_min: int
          v_max: int
          kp: float           (скорость ~ kp * (p_target - p))
          deadband: float     (зона удержания)
        """
        with self._qlock:
            self._q = deque([it for it in self._q if it[0] not in ("auto", "pressure")])
            self._q.append(("pressure", dict(params)))

    def stop_all(self):
        self._auto_flag.clear()
        self._mode = None
        with self._qlock:
            self._q.clear()

    def is_auto_running(self) -> bool:
        return self._auto_flag.is_set()

    # ---------- helpers ----------

    def _clamp_delta_to_safe(self, pos_mm: float, delta_mm: float) -> float:
        """Ограничить шаг так, чтобы не выйти за SAFE_MIN..SAFE_MAX."""
        target = pos_mm + delta_mm
        if target < SAFE_MIN:
            return SAFE_MIN - pos_mm
        if target > SAFE_MAX:
            return SAFE_MAX - pos_mm
        return delta_mm

    def _blocked_by_edges(self, pos_mm: float, delta_mm: float) -> bool:
        """
        delta_mm — это "физический" знак по позиции (мм).
        pos>=SAFE_MAX -> запрещаем delta>0 (дальше вправо/вверх по позиции)
        pos<=SAFE_MIN -> запрещаем delta<0
        """
        if pos_mm >= SAFE_MAX and delta_mm > 0:
            return True
        if pos_mm <= SAFE_MIN and delta_mm < 0:
            return True
        return False

    def _send_move_physical(self, delta_mm_physical: float, speed: int):
        """
        delta_mm_physical: +мм = увеличить C3 (позицию), -мм = уменьшить C3.
        VCP_DIR — только тут!
        """
        if abs(delta_mm_physical) < 0.5:
            return
        self.vcp.move_rel_mm(delta_mm_physical * VCP_DIR, int(speed))

    # ---------- worker thread ----------

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

            # ---------------- jog ----------------
            if kind == "jog":
                if not self.vcp.is_connected():
                    continue

                delta_req = float(item[1])

                with self.pos_lock:
                    pos = self.get_pos_mm()
                if pos is None:
                    continue

                # на границах разрешаем только “внутрь”
                if self._blocked_by_edges(pos, delta_req):
                    continue

                delta = self._clamp_delta_to_safe(pos, delta_req)
                self._send_move_physical(delta, JOG_SPEED)
                continue

            # ---------------- auto ----------------
            if kind == "auto":
                if not self.vcp.is_connected():
                    continue

                runs = int(item[1])
                speed = int(item[2])

                self._auto_flag.set()
                self._mode = "auto"

                for _ in range(runs):
                    legs_done = 0

                    # направление как сейчас — по половине хода
                    with self.pos_lock:
                        pos0 = self.get_pos_mm()
                    if pos0 is None:
                        self._auto_flag.clear()
                        self._mode = None
                        continue

                    leg_dir = +1.0 if pos0 <= MID_MM else -1.0

                    while self._auto_flag.is_set():

                        with self.pos_lock:
                            pos = self.get_pos_mm()
                        if pos is None:
                            break

                        edge = SAFE_MAX if leg_dir > 0 else SAFE_MIN

                        delta = self._clamp_delta_to_safe(pos, edge - pos)

                        if abs(delta) < 0.5:
        # дошли до края
                            leg_dir *= -1.0
                            legs_done += 1
                            if legs_done >= runs:
                                break
                            continue

                        self._send_move_physical(delta, speed)

                    if not self._auto_flag.is_set():
                        break

                    with self.pos_lock:
                        pos2 = self.get_pos_mm()
                    if pos2 is None:
                        break

                    d2 = self._clamp_delta_to_safe(pos2, t2 - pos2)
                    self._send_move_physical(d2, speed)

                self._auto_flag.clear()
                self._mode = None
                continue

            # ---------------- pressure ----------------
            if kind == "pressure":
                if not self.vcp.is_connected():
                    continue

                params = dict(item[1])

                step_mm = float(params.get("step_mm", 1.0))
                p_target = float(params.get("p_target", 5.0))
                p_limit = float(params.get("p_limit", 10.0))

                v_min = int(params.get("v_min", 100))
                v_max = int(params.get("v_max", 2000))
                kp = float(params.get("kp", 200.0))               # мм/с на 1 кг ошибки
                deadband = float(params.get("deadband", 0.2))     # кг

                runs = int(params.get("runs", 1))                 # прогонов туда-обратно
                creep_step = float(params.get("creep_step_mm", 1.0))
                if creep_step < 1.0:
                    creep_step = 1.0

                accel_up = float(params.get("accel_up", 800.0))       # мм/с^2
                accel_down = float(params.get("accel_down", 800.0))   # мм/с^2

                edge_tol = 0.5   # мм
                legs_done = 0    # 2 legs = 1 прогон туда-обратно

                self._auto_flag.set()
                self._mode = "pressure"

                # старт скорости: строго v_min
                v_cur = float(v_min)              # float, чтобы не квантовать скорость
                t_prev = time.perf_counter()

                alpha_err = float(params.get("alpha_err", 0.25))  # 0..1, меньше = плавнее
                err_f = 0.0
                err_f_inited = False

                # направление "как автопрогон": по половине хода
                with self.pos_lock:
                    pos0 = self.get_pos_mm()
                if pos0 is None:
                    self._auto_flag.clear()
                    self._mode = None
                    continue

                leg_dir = +1.0 if pos0 <= MID_MM else -1.0  # + к SAFE_MAX, - к SAFE_MIN

                while self._auto_flag.is_set():
                    with self.pos_lock:
                        pos = self.get_pos_mm()
                    if pos is None:
                        break

                    p = self.get_pressure()
                    if p is None:
                        break

                    # dt для плавной скорости
                    t_now = time.perf_counter()
                    dt = t_now - t_prev
                    if dt <= 0.0:
                        dt = 0.02
                    if dt > 0.2:
                        dt = 0.2
                    t_prev = t_now

                    # жёсткий предел — стоп
                    if p >= p_limit:
                        self._auto_flag.clear()
                        break

                    # текущая "нога" к краю, разворот только на краях
                    edge = SAFE_MAX if leg_dir > 0 else SAFE_MIN
                    if (leg_dir > 0 and pos >= edge - edge_tol) or (leg_dir < 0 and pos <= edge + edge_tol):
                        leg_dir *= -1.0
                        legs_done += 1
                        if legs_done >= runs:
                            self._auto_flag.clear()
                            break
                        continue

                    err = p_target - p


                    if not err_f_inited:
                        err_f = err
                        err_f_inited = True
                    else:
                        err_f = (1.0 - alpha_err) * err_f + alpha_err * err

                    # шаг и целевая скорость
# шаг и целевая скорость
                    if err_f > deadband:
                        step_use = abs(step_mm)

                        v_target = float(v_min) + float(kp) * abs(err_f)
                        if v_target > float(v_max):
                            v_target = float(v_max)
                        if v_target < float(v_min):
                            v_target = float(v_min)
                    else:
                        step_use = abs(creep_step)
                        v_target = float(v_min)

# плавно меняем v_cur -> v_target (ограничение изменения за цикл)
                    dv = v_target - v_cur

# максимум изменения скорости за один цикл (мм/с за dt)
# твой "Плавность" теперь трактуем как mm/s^2
                    max_dv_up = float(accel_up) * dt
                    max_dv_dn = float(accel_down) * dt

                    if dv > 0:
                        if dv > max_dv_up:
                            dv = max_dv_up
                    else:
                        if dv < -max_dv_dn:
                            dv = -max_dv_dn

                    v_cur = v_cur + dv

# границы
                    if v_cur < float(v_min):
                        v_cur = float(v_min)
                    if v_cur > float(v_max):
                        v_cur = float(v_max)

# ВАЖНО: округляем только перед отправкой команды
                    v_cmd = int(round(v_cur))
                    if v_cmd < v_min:
                        v_cmd = v_min
                    if v_cmd > v_max:
                        v_cmd = v_max

                    # движение всегда только в сторону текущей ноги (как автопрогон)
                    delta_cmd = leg_dir * step_use

                    # безопасные границы
                    delta_cmd = self._clamp_delta_to_safe(pos, delta_cmd)
                    if abs(delta_cmd) < 0.5:
                        time.sleep(0.02)
                        continue

                    self._send_move_physical(delta_cmd, v_cmd)

                self._auto_flag.clear()
                self._mode = None
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
            # сброс внутренних состояний потока парсинга
            with lock:
                for ch in display_channels:
                    channel_buffers[ch][:] = np.nan
                save_buffer.clear()
                write_index = 0

            with raw_lock:
                raw_bytes_buffer.clear()

            pos_hist.clear()

            # опционально: сброс SYNC/LOCK состояния, чтобы старт был "с нуля"
            stream_buf.clear()
            locked = False
            expected_start = None
            stable_t0 = None

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
            # ---- скорость V по позиции C3 (средняя за 1 секунду) ----


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
        layout.addWidget(self.panel, 0)

        self.win = pg.GraphicsLayoutWidget(show=False, title="Стенд — каналы")
        layout.addWidget(self.win, 1)

        self.setCentralWidget(root)
        self.panel.start_pressure.connect(self._start_pressure)
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
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self._update_plots)
        self.timer.start(20)

        # UI polling для статуса автоцикла
        self.ui_timer = pg.QtCore.QTimer()
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
            args=(port, DATA_BAUD, DATA_READ_CHUNK,
                self.data_queue, self.queue_lock,
                self.raw_bytes_buffer, self.raw_lock,
                self.data_stop),                 # <-- ДОБАВИЛИ
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

    # стопаем оба потока
        self.data_stop.set()

    # очистка очереди/буферов — чтобы при новом старте не было мусора и старого SYNC
        with self.queue_lock:
            self.data_queue.clear()
        with self.lock:
            for ch in display_channels:
                self.channel_buffers[ch][:] = np.nan
            self.save_buffer.clear()

    # ждём завершения
        for t in getattr(self, "data_threads", []):
            try:
                t.join(timeout=1.5)
            except Exception:
                pass

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

    def _stop_auto(self):
        self.motion.stop_all()
        self.panel.set_auto_running(False)
        self.vcp.abort()
        self.motion.stop_all()
    def _update_ui_state(self):
        # когда автоцикл завершился сам — вернём кнопки
        if not self.motion.is_auto_running():
            self.panel.set_auto_running(False)

    def _jog(self, delta_mm: float):
        if not self.vcp.is_connected():
            print("[JOG] VCP not connected")
            return
        self.motion.enqueue_jog(delta_mm)

    def _update_plots(self):
        with self.lock:
            for ch in display_channels:
                self.curves[ch].setData(self.channel_buffers[ch])

    def keyPressEvent(self, event):
        key = event.key()
        if key == ord("N"):
            self.reset_event.set()
            print("[ИНФО] Буферы/графики очищены (reset_event).")
        elif key == ord("S"):
            with self.lock:
                if not self.save_buffer:
                    print("[ИНФО] Нечего сохранять.")
                    return
                df = pd.DataFrame(self.save_buffer)

            t0 = df["t_perf"].iloc[0]
            df["Time (s)"] = df["t_perf"] - t0
            df = df.drop(columns=["t_perf"])

            ts = time.strftime("%Y_%m_%d_%H_%M_%S")
            fname = f"stand_data_{ts}.csv"
            df.to_csv(fname, index=False)
            print(f"[ИНФО] Данные сохранены: {fname}")

            with self.raw_lock:
                raw_name = f"raw_bytes_{ts}.txt"
                with open(raw_name, "w") as f:
                    f.write(" ".join(f"{b:02X}" for b in self.raw_bytes_buffer))
                self.raw_bytes_buffer.clear()
            self.reset_event.set()
            print("[ИНФО] После сохранения выполнен сброс буферов (reset_event).")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    stand = StandVCP()
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
