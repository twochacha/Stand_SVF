
from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QComboBox, QPushButton, QLabel,
    QSpinBox, QGroupBox, QFormLayout, QDoubleSpinBox
)


class PortSelector(QWidget):
    """Строка выбора порта: комбобокс + Обновить + Подключить/Отключить."""
    
    connect_clicked = Signal()
    disconnect_clicked = Signal()
    refresh_clicked = Signal()

    def __init__(self, title: str):
        super().__init__()
        self._connected = False

        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)

        row.addWidget(QLabel(title))

        self.combo = QComboBox()
        self.combo.setMinimumWidth(120)
        row.addWidget(self.combo)

        self.btn_refresh = QPushButton("Обновить")
        self.btn_refresh.clicked.connect(self.refresh_clicked)
        row.addWidget(self.btn_refresh)

        self.btn_connect = QPushButton("Подключить")
        self.btn_connect.clicked.connect(self._on_connect)
        row.addWidget(self.btn_connect)

        self.setLayout(row)
    


    def _on_connect(self):
        if self._connected:
            self.disconnect_clicked.emit()
        else:
            self.connect_clicked.emit()

    def set_connected(self, connected: bool):
        self._connected = connected
        self.btn_connect.setText("Отключить" if connected else "Подключить")
        self.combo.setEnabled(not connected)

    def refresh_ports(self):
        import serial.tools.list_ports

        ports = serial.tools.list_ports.comports()
        current = self.combo.currentText()
        self.combo.clear()
        for p in ports:
            self.combo.addItem(p.device)
        if current:
            idx = self.combo.findText(current)
            if idx >= 0:
                self.combo.setCurrentIndex(idx)

    def selected_port(self) -> str:
        return self.combo.currentText().strip()


class ControlPanel(QWidget):
    """Панель управления: подключения + автопрогон + контроль по давлению + ручной режим."""

    start_auto = Signal(int, int)       # runs, speed
    start_pressure = Signal(dict)       # params dict
    stop_auto = Signal()                # stop everything (auto/pressure)
    jog = Signal(float)                 # delta_mm
    motion_limits_changed = Signal(float, float)  # min_mm, max_mm

    def __init__(self):
        super().__init__()

        layout = QVBoxLayout()

    # ===== Пределы хода =====
        gb_limits = QGroupBox("Пределы хода")
        limits_form = QFormLayout()

        self.sp_min_mm = QDoubleSpinBox()
        self.sp_min_mm.setDecimals(1)
        self.sp_min_mm.setRange(-100000.0, 100000.0)
        self.sp_min_mm.setSingleStep(0.5)
        self.sp_min_mm.setValue(16.0)
        limits_form.addRow("MIN_MM:", self.sp_min_mm)

        self.sp_max_mm = QDoubleSpinBox()
        self.sp_max_mm.setDecimals(1)
        self.sp_max_mm.setRange(-100000.0, 100000.0)
        self.sp_max_mm.setSingleStep(0.5)
        self.sp_max_mm.setValue(78.0)
        limits_form.addRow("MAX_MM:", self.sp_max_mm)

        self.btn_apply_limits = QPushButton("Применить")
        limits_form.addRow(self.btn_apply_limits)

        gb_limits.setLayout(limits_form)
        layout.addWidget(gb_limits)

    # ===== Подключения =====
        gb_ports = QGroupBox("Подключения")
        ports_layout = QVBoxLayout()
        self.sel_data = PortSelector("Телереабос:")
        self.sel_vcp = PortSelector("Стенд:")
        ports_layout.addWidget(self.sel_data)
        ports_layout.addWidget(self.sel_vcp)
        gb_ports.setLayout(ports_layout)
        layout.addWidget(gb_ports)

    # ===== Автопрогон =====
        gb_auto = QGroupBox("Автопрогон")
        auto_row = QHBoxLayout()

        auto_row.addWidget(QLabel("Прогонов (в одну сторону):"))
        self.spin_runs = QSpinBox()
        self.spin_runs.setRange(1, 100000)
        self.spin_runs.setValue(1)
        auto_row.addWidget(self.spin_runs)

        auto_row.addWidget(QLabel("Скорость (мм/с):"))
        self.spin_speed = QSpinBox()
        self.spin_speed.setRange(1, 3500)
        self.spin_speed.setValue(250)
        auto_row.addWidget(self.spin_speed)

        self.btn_start = QPushButton("Старт")
        self.btn_stop = QPushButton("Стоп")
        self.btn_stop.setEnabled(False)
        auto_row.addWidget(self.btn_start)
        auto_row.addWidget(self.btn_stop)

        gb_auto.setLayout(auto_row)
        layout.addWidget(gb_auto)

    # ===== Контроль по давлению =====
        gb_press = QGroupBox("Контроль по давлению")
        press_form = QFormLayout()

        self.sp_alpha_err = QDoubleSpinBox()
        self.sp_alpha_err.setDecimals(2)
        self.sp_alpha_err.setRange(0.01, 1.0)
        self.sp_alpha_err.setValue(0.25)
        press_form.addRow("Сглаживание ошибки (0..1):", self.sp_alpha_err)

        self.sp_step_mm = QDoubleSpinBox()
        self.sp_step_mm.setDecimals(2)
        self.sp_step_mm.setRange(0.1, 10.0)
        self.sp_step_mm.setValue(1.0)
        press_form.addRow("Шаг без нагрузки (мм):", self.sp_step_mm)

        self.sp_creep_step = QDoubleSpinBox()
        self.sp_creep_step.setDecimals(2)
        self.sp_creep_step.setRange(1.0, 50.0)
        self.sp_creep_step.setValue(1.0)
        press_form.addRow("Шаг в нагрузке (мм):", self.sp_creep_step)

        self.sp_kp = QDoubleSpinBox()
        self.sp_kp.setDecimals(1)
        self.sp_kp.setRange(0.0, 100000.0)
        self.sp_kp.setValue(300.0)
        press_form.addRow("Коэффициент прироста скорости:", self.sp_kp)

        self.sp_p_runs = QSpinBox()
        self.sp_p_runs.setRange(1, 100000)
        self.sp_p_runs.setValue(1)
        press_form.addRow("Прогонов (в одну сторону):", self.sp_p_runs)

        self.sp_p_target = QDoubleSpinBox()
        self.sp_p_target.setDecimals(3)
        self.sp_p_target.setRange(-1e9, 1e9)
        self.sp_p_target.setValue(0.0)
        press_form.addRow("Целевое давление (кг):", self.sp_p_target)

        self.sp_p_limit = QDoubleSpinBox()
        self.sp_p_limit.setDecimals(3)
        self.sp_p_limit.setRange(-1e9, 1e9)
        self.sp_p_limit.setValue(0.0)
        press_form.addRow("Предельное давление (кг):", self.sp_p_limit)

        self.sp_v_min = QSpinBox()
        self.sp_v_min.setRange(1, 3500)
        self.sp_v_min.setValue(100)
        press_form.addRow("Мин. скорость (мм/с):", self.sp_v_min)

        self.sp_v_max = QSpinBox()
        self.sp_v_max.setRange(1, 3500)
        self.sp_v_max.setValue(1000)
        press_form.addRow("Макс скорость (мм/с):", self.sp_v_max)

        self.sp_accel = QSpinBox()
        self.sp_accel.setRange(1, 100000)
        self.sp_accel.setValue(800)
        press_form.addRow("Ускорение (мм/с²):", self.sp_accel)

        press_btns = QHBoxLayout()
        self.btn_p_start = QPushButton("Старт")
        self.btn_p_stop = QPushButton("Стоп")
        self.btn_p_stop.setEnabled(False)
        press_btns.addWidget(self.btn_p_start)
        press_btns.addWidget(self.btn_p_stop)
        press_form.addRow(press_btns)

        gb_press.setLayout(press_form)
        layout.addWidget(gb_press)

    # ===== Ручной режим =====
        gb_jog = QGroupBox("Ручной режим")
        jog_row = QHBoxLayout()
        self.btn_left = QPushButton("-2 мм")
        self.btn_right = QPushButton("+2 мм")
        jog_row.addWidget(self.btn_left)
        jog_row.addWidget(self.btn_right)
        gb_jog.setLayout(jog_row)
        layout.addWidget(gb_jog)

        layout.addStretch(1)
        self.setLayout(layout)

    # ===== wiring =====

        self.btn_start.clicked.connect(
        lambda: self.start_auto.emit(int(self.spin_runs.value()), int(self.spin_speed.value()))
    )
        self.btn_stop.clicked.connect(lambda: self.stop_auto.emit())

        def _emit_pressure_start():
            params = {
                "runs": int(self.sp_p_runs.value()),
            "step_mm": float(self.sp_step_mm.value()),
            "p_target": float(self.sp_p_target.value()),
            "p_limit": float(self.sp_p_limit.value()),
            "v_min": int(self.sp_v_min.value()),
            "v_max": int(self.sp_v_max.value()),
            "kp": float(self.sp_kp.value()),
            "deadband": 0.05,
            "creep_step_mm": float(self.sp_creep_step.value()),
            "accel_up": float(self.sp_accel.value()),
            "accel_down": float(self.sp_accel.value()),
            "alpha_err": float(self.sp_alpha_err.value()),
        }
            self.start_pressure.emit(params)

        self.btn_p_start.clicked.connect(_emit_pressure_start)
        self.btn_p_stop.clicked.connect(lambda: self.stop_auto.emit())

        self.btn_left.clicked.connect(lambda: self.jog.emit(-2.0))
        self.btn_right.clicked.connect(lambda: self.jog.emit(+2.0))
        self.btn_apply_limits.clicked.connect(
            lambda: self.motion_limits_changed.emit(
                float(self.sp_min_mm.value()),
                float(self.sp_max_mm.value()),
            )
        )

    def set_motion_limits(self, min_mm: float, max_mm: float):
        self.sp_min_mm.setValue(float(min_mm))
        self.sp_max_mm.setValue(float(max_mm))

    def set_auto_running(self, running: bool):
        # Авто
        self.btn_start.setEnabled(not running)
        self.btn_stop.setEnabled(running)

        # Давление
        self.btn_p_start.setEnabled(not running)
        self.btn_p_stop.setEnabled(running)
