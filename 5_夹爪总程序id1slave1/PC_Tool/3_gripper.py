import struct
import json
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

import serial
from serial.tools import list_ports


UNIT_CFG_P_MDEG_V_DEG = 0x000B

# Safety regs
REG_SAFE_SUM_TH_H = 0x0030
REG_SAFE_SUM_TH_L = 0x0031
REG_SAFE_MAX_TH = 0x0032
REG_SAFE_UNLOCK = 0x0034
SAFE_UNLOCK_VALUE = 0xA55A

# Goal regs
REG_MODE = 0x0011
REG_ENABLE = 0x0012
REG_CLEAR_FAULT = 0x0013
REG_UNIT_CFG = 0x0014
REG_SAVE_ZERO = 0x0016
REG_SAVE_MOTOR_ZERO = 0x0017
REG_PERSIST_VALID = 0x0018
REG_V_DES_H = 0x0020
REG_P_DES_H = 0x0022
REG_KP_1_100 = 0x0024
REG_KD_1_100 = 0x0025
REG_TFF_H = 0x0026

# Estimator regs
REG_EST_TOR_TH = 0x0036
REG_EST_VEL_IDLE_TH = 0x0037
REG_EST_ALPHA = 0x0038
REG_EST_BIAS_LEARN = 0x0039
REG_EST_FRIC_B = 0x003A
REG_EST_COULOMB_POS = 0x003B
REG_EST_COULOMB_NEG = 0x003C
REG_EST_STATIC_POS = 0x003D
REG_EST_STATIC_NEG = 0x003E
REG_EST_VEL_SIGN_TH = 0x003F
REG_EST_CONTACT_DIR = 0x0040
REG_DM_PMAX_MRAD = 0x0041
REG_DM_PMAX_APPLY = 0x0042
REG_EST_BASE_OPEN_H = 0x0043
REG_EST_BASE_CLOSE_H = 0x0045
REG_EST_GRIP_TH = 0x0047

# Feedback regs
REG_FB_POS_H = 0x0100
REG_FB_ERR = 0x0103
REG_FB_MOS_TEMP = 0x0104
REG_FB_ROTOR_TEMP = 0x0105
REG_SAFE_FLAGS = 0x0106
REG_SAFE_SUM_H = 0x0107
REG_SAFE_SUM_L = 0x0108
REG_SAFE_MAX = 0x0109
REG_FB_TOR_RAW_H = 0x010A
REG_FB_TOR_FILT_H = 0x010C
REG_FB_TOR_EXT_H = 0x010E
REG_FB_FORCE_H = 0x0110
REG_FB_CONTACT_FLAG = 0x0112
REG_FB_TOR_FRIC_H = 0x0113
REG_FB_AS_ABS_H = 0x0115
REG_FB_AS_STATUS = 0x0117
REG_FB_MOTOR_POS_H = 0x0118
REG_FB_MOTOR_CMD_H = 0x011A
REG_FB_MOTOR_RAW_H = 0x011C
REG_FB_MOTOR_PMAX = 0x011E
REG_FB_MOTOR_CLAMP = 0x011F
REG_FB_GOAL_STATUS = 0x0120
REG_FB_FRIC_RAW_H = 0x0121
REG_FB_BASELINE_H = 0x0123
REG_FB_CONTACT_RAW_H = 0x0125
REG_FB_CONTACT_NET_H = 0x0127
REG_FB_VEL_EST_H = 0x0129
REG_FB_POS_EST_H = 0x012B
REG_FB_PRESS_BASE = 0x0130

EST_READ_QTY = 18
LIVE_POLL_PERIOD_S = 0.2
PRESSURE_FEATURE_REGS = 5
PRESSURE_REGION_NAMES = ("TOTAL", "BE_SUM", "BE_C", "BE_D", "BE_E", "LE_C")

MOTOR_ERR_CODES = {
    0x0: "正常",
    0x1: "已使能",
    0x8: "过压",
    0x9: "欠压",
    0xA: "过流",
    0xB: "MOS过温",
    0xC: "线圈过温",
    0xD: "通信丢失",
    0xE: "过载",
}
MOTOR_ERR_FAULT_SET = {0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xE}

# Compensation / ratio loop
RATIO_FB_CORR_GAIN = 0.8
RATIO_FB_TOL_DEG = 0.65
RATIO_FB_CORR_MAX = 8
RATIO_FB_STABLE_TIMEOUT_S = 1.0
RATIO_FB_STABLE_INTERVAL_S = 0.15
RATIO_FB_STABLE_EPS_DEG = 0.6
RATIO_FB_STABLE_COUNT = 2
RATIO_FB_EXTRA_OPEN_DEG = 12.0
RATIO_FB_EXTRA_CLOSE_DEG = 18.0
SETTINGS_PATH = Path(__file__).with_suffix(".settings.json")
POS_SCALE_DEFAULT = 26.5


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def add_crc(frame_wo_crc: bytes) -> bytes:
    return frame_wo_crc + struct.pack("<H", crc16_modbus(frame_wo_crc))


def bytes_to_hex(data: bytes) -> str:
    return " ".join(f"{x:02X}" for x in data)


def hex_to_bytes(text: str) -> bytes:
    text = text.strip().replace(" ", "")
    if len(text) % 2:
        raise ValueError("Hex 长度必须为偶数")
    return bytes.fromhex(text)


def mb_read_holding(slave: int, addr: int, qty: int) -> bytes:
    return add_crc(struct.pack(">BBHH", slave, 0x03, addr, qty))


def mb_write_single(slave: int, addr: int, value: int) -> bytes:
    return add_crc(struct.pack(">BBHH", slave, 0x06, addr, value & 0xFFFF))


def mb_write_multi(slave: int, addr: int, regs: list[int]) -> bytes:
    qty = len(regs)
    payload = b"".join(struct.pack(">H", r & 0xFFFF) for r in regs)
    header = struct.pack(">BBHHB", slave, 0x10, addr, qty, qty * 2)
    return add_crc(header + payload)


def expected_len_from_prefix(prefix: bytes) -> int | None:
    if len(prefix) < 2:
        return None
    func = prefix[1]
    if func & 0x80:
        return 5
    if func in (0x06, 0x10):
        return 8
    if func == 0x03:
        if len(prefix) < 3:
            return None
        return 3 + prefix[2] + 2
    return None


def verify_crc(resp: bytes) -> bool:
    if len(resp) < 3:
        return False
    return struct.unpack("<H", resp[-2:])[0] == crc16_modbus(resp[:-2])


def regs_from_read03(resp: bytes) -> list[int]:
    if len(resp) < 5:
        raise ValueError("响应太短")
    if resp[1] & 0x80:
        raise ValueError(f"异常响应 func=0x{resp[1]:02X}, code=0x{resp[2]:02X}")
    if resp[1] != 0x03:
        raise ValueError(f"不是 0x03 响应: func=0x{resp[1]:02X}")
    bytecount = resp[2]
    data = resp[3:3 + bytecount]
    if len(data) != bytecount or bytecount % 2 != 0:
        raise ValueError("寄存器数据长度错误")
    return [struct.unpack(">H", data[i:i + 2])[0] for i in range(0, bytecount, 2)]


def s32_from_regs(reg_hi: int, reg_lo: int) -> int:
    value = (reg_hi << 16) | reg_lo
    return struct.unpack(">i", struct.pack(">I", value))[0]


def i32_to_regs(value: int) -> list[int]:
    return list(struct.unpack(">HH", struct.pack(">i", int(value))))


def rad_to_deg(rad: float) -> float:
    return rad * 57.29577951308232


def deg_to_rad(deg: float) -> float:
    return deg / 57.29577951308232


def deg_to_mdeg_regs(deg: float) -> list[int]:
    return i32_to_regs(int(round(deg * 1000.0)))


def deg_to_mrad_regs(deg: float) -> list[int]:
    return i32_to_regs(int(round(deg_to_rad(deg) * 1000.0)))


def nm_to_regs_x1000(val: float) -> int:
    return int(round(val * 1000.0)) & 0xFFFF


def signed_u16_from_float_x1000(val: float) -> int:
    return struct.unpack(">H", struct.pack(">h", int(round(val * 1000.0))))[0]


def signed_u16_to_float_x1000(reg: int) -> float:
    return struct.unpack(">h", struct.pack(">H", reg))[0] / 1000.0


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("夹爪 Modbus")
        self.root.minsize(1080, 760)

        self.ser: serial.Serial | None = None
        self.rx_lock = threading.Lock()
        self.manual_io_pending = threading.Event()

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.slave_var = tk.StringVar(value="1")
        self.timeout_var = tk.StringVar(value="0.4")

        self.live_status_var = tk.StringVar(value="实时反馈: 未连接")
        self.safe_status_var = tk.StringVar(value="SAFE:\nflags=0x0000  sum=0  max=0  OK")
        self.pressure_var = tk.StringVar(value="触觉: sum=--  max=--  status=--")
        self.pressure_detail_var = tk.StringVar(value="触觉分区: 等待反馈")
        self.pressure_detail_latest = {}
        self.sum_th_var = tk.StringVar(value="2500")
        self.max_th_var = tk.StringVar(value="200")
        self.alarm_latched = False
        self._poll_fail_count = 0
        self._safe_poll_stop = threading.Event()
        self._safe_poll_thread = None

        self.motor_err_var = tk.StringVar(value="电机: --")
        self.debug_var = tk.StringVar(value="AS: --")
        self.torque_text_var = tk.StringVar(
            value=(
                "TorqueEst:\n"
                "raw=--  filt=--  fric=--\n"
                "ext_tau=--  contact_eff=--  inst_contact=--  grip_hold=--"
            )
        )

        self.motor_err_code = 0
        self.motor_err_latched = False
        self.motor_err_alarmed = False

        self.motor_pos_latest = 0.0
        self.motor_cmd_latest = 0.0
        self.motor_raw_latest = 0.0
        self.motor_pmax_latest = 0.0
        self.motor_clamp_latest = 0
        self.goal_status_latest = 0
        self.scale_latest = POS_SCALE_DEFAULT

        self._ratio_busy = False

        self.v_var = tk.StringVar(value="5")
        self.p_var = tk.StringVar(value="30")
        self.kp_var = tk.StringVar(value="0.2")
        self.kd_var = tk.StringVar(value="0.2")
        self.tff_var = tk.StringVar(value="0")

        self.ratio_var = tk.StringVar(value="50")
        self.open_deg_var = tk.StringVar(value="-29.74")
        self.close_deg_var = tk.StringVar(value="0")
        self.raw_hex_var = tk.StringVar()
        self.dm_pmax_var = tk.StringVar(value="12.5")

        self.est_th_var = tk.StringVar(value="0.08")
        self.est_b_var = tk.StringVar(value="0.0000465141")
        self.est_tc_pos_var = tk.StringVar(value="0.250000")
        self.est_tc_neg_var = tk.StringVar(value="0.120000")
        self.est_ts_pos_var = tk.StringVar(value="0.120000")
        self.est_ts_neg_var = tk.StringVar(value="0.080000")
        self.est_vsign_var = tk.StringVar(value="0.04")
        self.est_dir_var = tk.StringVar(value="1")
        self.est_base_open_var = tk.StringVar(value="-29.74")
        self.est_base_close_var = tk.StringVar(value="0.00")
        self.est_grip_th_var = tk.StringVar(value="0.12")
        self.est_raw_regs: list[int] | None = None

        self._load_ui_calibration_settings()

        self._build_ui()
        self.refresh_ports()

    def _load_ui_calibration_settings(self):
        try:
            data = json.loads(SETTINGS_PATH.read_text(encoding="utf-8"))
            open_deg = float(data.get("open_deg", self.open_deg_var.get()))
            close_deg = float(data.get("close_deg", self.close_deg_var.get()))
        except Exception:
            return
        self.open_deg_var.set(f"{open_deg:.2f}")
        self.close_deg_var.set(f"{close_deg:.2f}")
        self.est_base_open_var.set(f"{open_deg:.2f}")
        self.est_base_close_var.set(f"{close_deg:.2f}")

    def _save_ui_calibration_settings(
        self,
        open_deg: float | None = None,
        close_deg: float | None = None,
    ):
        try:
            if open_deg is None:
                open_deg = float(self.open_deg_var.get())
            if close_deg is None:
                close_deg = float(self.close_deg_var.get())
            data = {
                "open_deg": round(open_deg, 2),
                "close_deg": round(close_deg, 2),
            }
            SETTINGS_PATH.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        except Exception as exc:
            self.log(f"保存端点配置失败: {exc}")

    def _sync_estimator_base_from_calibration(self, reason: str = "同步力矩BASE") -> bool:
        try:
            open_deg = float(self.open_deg_var.get())
            close_deg = float(self.close_deg_var.get())
        except Exception as exc:
            self.log(f"{reason}失败: 全开/全关角无效 ({exc})")
            return False

        self.est_base_open_var.set(f"{open_deg:.2f}")
        self.est_base_close_var.set(f"{close_deg:.2f}")

        if not self.ser or not self.ser.is_open:
            return False

        regs = deg_to_mrad_regs(open_deg) + deg_to_mrad_regs(close_deg)
        resp = self._send_and_recv(mb_write_multi(self._slave(), REG_EST_BASE_OPEN_H, regs))
        if not resp:
            self.log(f"{reason}失败: 设备无响应")
            return False

        if self.est_raw_regs is not None and len(self.est_raw_regs) >= EST_READ_QTY:
            self.est_raw_regs[13:17] = regs

        self.log(f"{reason}: BASEOPEN={open_deg:.2f}°, BASECLOSE={close_deg:.2f}°")
        return True

    def _build_ui(self):
        frm = ttk.Frame(self.root, padding=10)
        frm.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        frm.columnconfigure(0, weight=1)
        frm.rowconfigure(4, weight=1)

        conn = ttk.LabelFrame(frm, text="串口连接", padding=8)
        conn.grid(row=0, column=0, sticky="ew")
        conn.columnconfigure(9, weight=1)

        ttk.Button(conn, text="刷新串口", command=self.refresh_ports).grid(row=0, column=0, padx=4)
        ttk.Label(conn, text="端口").grid(row=0, column=1)
        self.port_cb = ttk.Combobox(conn, textvariable=self.port_var, width=18, state="readonly")
        self.port_cb.grid(row=0, column=2, padx=4)
        ttk.Label(conn, text="波特率").grid(row=0, column=3)
        ttk.Entry(conn, textvariable=self.baud_var, width=10).grid(row=0, column=4, padx=4)
        ttk.Label(conn, text="Slave").grid(row=0, column=5)
        ttk.Entry(conn, textvariable=self.slave_var, width=5).grid(row=0, column=6, padx=4)
        ttk.Label(conn, text="超时(s)").grid(row=0, column=7)
        ttk.Entry(conn, textvariable=self.timeout_var, width=6).grid(row=0, column=8, padx=4)
        self.btn_connect = ttk.Button(conn, text="连接", command=self.connect)
        self.btn_connect.grid(row=0, column=9, padx=6, sticky="e")
        self.btn_disconnect = ttk.Button(conn, text="断开", command=self.disconnect, state="disabled")
        self.btn_disconnect.grid(row=0, column=10, padx=6, sticky="e")
        ttk.Label(conn, textvariable=self.live_status_var, anchor="w").grid(
            row=1, column=0, columnspan=11, padx=4, pady=(4, 0), sticky="ew"
        )

        from tactile_ui import TactilePanel
        self.tactile = TactilePanel(self, conn)

        quick = ttk.LabelFrame(frm, text="常用指令", padding=8)
        quick.grid(row=1, column=0, sticky="ew", pady=8)
        for i in range(12):
            quick.columnconfigure(i, weight=1)

        ttk.Button(quick, text="MIT模式(0x0011=0)", command=self.cmd_mode_mit).grid(row=0, column=0, padx=4, pady=4, sticky="ew")
        ttk.Button(quick, text="单位配置(0x0014=0x000B)", command=self.cmd_unit_cfg).grid(row=0, column=1, padx=4, pady=4, sticky="ew")
        ttk.Button(quick, text="使能(0x0012=1)", command=self.cmd_enable).grid(row=0, column=2, padx=4, pady=4, sticky="ew")
        ttk.Button(quick, text="失能(0x0012=0)", command=self.cmd_disable).grid(row=0, column=3, padx=4, pady=4, sticky="ew")

        ttk.Button(quick, text="读位置(0x0100)", command=self.cmd_read_pos).grid(row=1, column=0, padx=4, pady=4, sticky="ew")
        ttk.Button(quick, text="读温度(0x0104)", command=self.cmd_read_temp).grid(row=1, column=1, padx=4, pady=4, sticky="ew")
        ttk.Button(quick, text="读持久化(0x0018)", command=self.cmd_read_persist).grid(row=1, column=2, padx=4, pady=4, sticky="ew")
        ttk.Button(quick, text="保存零点(0x0016=1)", command=self.cmd_save_zero).grid(row=1, column=3, padx=4, pady=4, sticky="ew")
        ttk.Button(quick, text="保存电机零点(0x0017=1)", command=self.cmd_save_motor_zero).grid(row=1, column=4, padx=4, pady=4, sticky="ew")

        ttk.Button(quick, text="一键标定闭合零点", command=self.cmd_oneclick_calib_close_zero).grid(row=2, column=0, padx=4, pady=4, sticky="ew")
        ttk.Button(quick, text="记录当前为全开角", command=self.cmd_capture_open_deg).grid(row=2, column=1, padx=4, pady=4, sticky="ew")
        ttk.Button(quick, text="读安全状态(0x0106)", command=self.cmd_read_safe).grid(row=2, column=2, padx=4, pady=4, sticky="ew")
        ttk.Button(quick, text="解锁(0x0034=0xA55A)", command=self.cmd_unlock_safe).grid(row=2, column=3, padx=4, pady=4, sticky="ew")

        ttk.Label(quick, textvariable=self.safe_status_var, anchor="w", justify="left", wraplength=980).grid(
            row=3, column=0, columnspan=12, padx=4, pady=(2, 6), sticky="ew"
        )
        ttk.Label(quick, textvariable=self.pressure_var, anchor="w", justify="left").grid(
            row=4, column=0, columnspan=12, padx=4, pady=(0, 4), sticky="ew"
        )
        ttk.Label(
            quick,
            textvariable=self.pressure_detail_var,
            anchor="w",
            justify="left",
            wraplength=980,
        ).grid(row=5, column=0, columnspan=12, padx=4, pady=(0, 4), sticky="ew")
        ttk.Label(quick, text="Sum阈值").grid(row=6, column=0, padx=4, pady=2, sticky="e")
        ttk.Entry(quick, textvariable=self.sum_th_var, width=10).grid(row=6, column=1, padx=4, pady=2, sticky="w")
        ttk.Label(quick, text="Max阈值").grid(row=6, column=2, padx=4, pady=2, sticky="e")
        ttk.Entry(quick, textvariable=self.max_th_var, width=10).grid(row=6, column=3, padx=4, pady=2, sticky="w")
        ttk.Button(quick, text="写安全阈值(0x0030/0x0032)", command=self.cmd_write_safe_th).grid(row=6, column=4, padx=4, pady=2, sticky="ew")

        motor_err_frm = ttk.LabelFrame(quick, text="电机故障报警", padding=4)
        motor_err_frm.grid(row=7, column=0, columnspan=12, padx=4, pady=(4, 2), sticky="ew")
        for i in range(6):
            motor_err_frm.columnconfigure(i, weight=1)

        self.motor_err_label = tk.Label(
            motor_err_frm,
            textvariable=self.motor_err_var,
            anchor="w",
            justify="left",
            font=("TkDefaultFont", 10, "bold"),
            fg="green",
            bg="#F0F0F0",
            relief="sunken",
            padx=6,
            pady=2,
        )
        self.motor_err_label.grid(row=0, column=0, columnspan=4, padx=4, pady=2, sticky="ew")
        ttk.Button(motor_err_frm, text="读电机错误(0x0103)", command=self.cmd_read_motor_err).grid(row=0, column=4, padx=4, pady=2, sticky="ew")
        ttk.Button(motor_err_frm, text="清除错误(0x0013=1)", command=self.cmd_clear_motor_fault).grid(row=0, column=5, padx=4, pady=2, sticky="ew")

        self.debug_label = tk.Label(
            motor_err_frm,
            textvariable=self.debug_var,
            anchor="w",
            justify="left",
            relief="sunken",
            bg="#F8F8F8",
            padx=6,
            pady=4,
        )
        self.debug_label.grid(row=1, column=0, columnspan=6, padx=4, pady=(2, 4), sticky="ew")

        ttk.Label(motor_err_frm, text="PMAX(rad)").grid(row=2, column=0, padx=4, pady=2, sticky="e")
        ttk.Entry(motor_err_frm, textvariable=self.dm_pmax_var, width=10).grid(row=2, column=1, padx=4, pady=2, sticky="w")
        ttk.Button(motor_err_frm, text="写 PMAX 并保存", command=self.cmd_write_dm_pmax_save).grid(row=2, column=2, columnspan=2, padx=4, pady=2, sticky="ew")

        torque_frm = ttk.LabelFrame(frm, text="力矩接触估算 / 夹紧保持", padding=8)
        torque_frm.grid(row=2, column=0, sticky="ew")
        for i in range(8):
            torque_frm.columnconfigure(i, weight=1)

        tk.Label(torque_frm, textvariable=self.torque_text_var, anchor="w", justify="left", bg="#F8F8F8", padx=6, pady=4).grid(
            row=0, column=0, columnspan=8, sticky="ew", padx=4, pady=(0, 4)
        )

        ttk.Label(torque_frm, text="Th(Nm)").grid(row=1, column=0, sticky="e")
        ttk.Entry(torque_frm, textvariable=self.est_th_var, width=10).grid(row=1, column=1, sticky="w")
        ttk.Label(torque_frm, text="B").grid(row=1, column=2, sticky="e")
        ttk.Entry(torque_frm, textvariable=self.est_b_var, width=12).grid(row=1, column=3, sticky="w")
        ttk.Label(torque_frm, text="Tc+").grid(row=1, column=4, sticky="e")
        ttk.Entry(torque_frm, textvariable=self.est_tc_pos_var, width=10).grid(row=1, column=5, sticky="w")
        ttk.Label(torque_frm, text="Tc-").grid(row=1, column=6, sticky="e")
        ttk.Entry(torque_frm, textvariable=self.est_tc_neg_var, width=10).grid(row=1, column=7, sticky="w")

        ttk.Label(torque_frm, text="Ts+").grid(row=2, column=0, sticky="e")
        ttk.Entry(torque_frm, textvariable=self.est_ts_pos_var, width=10).grid(row=2, column=1, sticky="w")
        ttk.Label(torque_frm, text="Ts-").grid(row=2, column=2, sticky="e")
        ttk.Entry(torque_frm, textvariable=self.est_ts_neg_var, width=10).grid(row=2, column=3, sticky="w")
        ttk.Label(torque_frm, text="Vsign").grid(row=2, column=4, sticky="e")
        ttk.Entry(torque_frm, textvariable=self.est_vsign_var, width=10).grid(row=2, column=5, sticky="w")
        ttk.Label(torque_frm, text="Dir").grid(row=2, column=6, sticky="e")
        ttk.Entry(torque_frm, textvariable=self.est_dir_var, width=6).grid(row=2, column=7, sticky="w")

        ttk.Label(torque_frm, text="BaseOpen(deg)").grid(row=3, column=0, sticky="e")
        ttk.Entry(torque_frm, textvariable=self.est_base_open_var, width=10).grid(row=3, column=1, sticky="w")
        ttk.Label(torque_frm, text="BaseClose(deg)").grid(row=3, column=2, sticky="e")
        ttk.Entry(torque_frm, textvariable=self.est_base_close_var, width=10).grid(row=3, column=3, sticky="w")
        ttk.Label(torque_frm, text="GripTh(Nm)").grid(row=3, column=4, sticky="e")
        ttk.Entry(torque_frm, textvariable=self.est_grip_th_var, width=10).grid(row=3, column=5, sticky="w")

        ttk.Button(torque_frm, text="Read", command=self.cmd_read_estimator).grid(row=4, column=6, padx=4, pady=2, sticky="ew")
        ttk.Button(torque_frm, text="Write", command=self.cmd_write_estimator).grid(row=4, column=7, padx=4, pady=2, sticky="ew")

        goal = ttk.LabelFrame(frm, text="目标设置(0x0020 起，共 8 个寄存器: V,P,Kp,Kd,TFF)", padding=8)
        goal.grid(row=3, column=0, sticky="ew", pady=8)
        for i in range(10):
            goal.columnconfigure(i, weight=1)

        ttk.Label(goal, text="V(°/s)").grid(row=0, column=0)
        ttk.Entry(goal, textvariable=self.v_var, width=8).grid(row=0, column=1, padx=4)
        ttk.Label(goal, text="P(°)").grid(row=0, column=2)
        ttk.Entry(goal, textvariable=self.p_var, width=8).grid(row=0, column=3, padx=4)
        ttk.Label(goal, text="Kp").grid(row=0, column=4)
        ttk.Entry(goal, textvariable=self.kp_var, width=8).grid(row=0, column=5, padx=4)
        ttk.Label(goal, text="Kd").grid(row=0, column=6)
        ttk.Entry(goal, textvariable=self.kd_var, width=8).grid(row=0, column=7, padx=4)
        ttk.Label(goal, text="TFF").grid(row=0, column=8)
        ttk.Entry(goal, textvariable=self.tff_var, width=8).grid(row=0, column=9, padx=4)

        ttk.Button(goal, text="发送目标", command=self.cmd_send_goal).grid(row=1, column=0, columnspan=10, pady=6, sticky="ew")

        ttk.Label(goal, text="夹爪开合度(%)").grid(row=2, column=0)
        ttk.Entry(goal, textvariable=self.ratio_var, width=8).grid(row=2, column=1, padx=4)
        ttk.Label(goal, text="全开角(°)").grid(row=2, column=2)
        ttk.Entry(goal, textvariable=self.open_deg_var, width=8).grid(row=2, column=3, padx=4)
        ttk.Label(goal, text="全闭角(°)").grid(row=2, column=4)
        ttk.Entry(goal, textvariable=self.close_deg_var, width=8).grid(row=2, column=5, padx=4)
        ttk.Button(goal, text="按开合度发送", command=self.cmd_send_ratio).grid(row=2, column=6, columnspan=4, padx=4, sticky="ew")

        raw = ttk.LabelFrame(frm, text="原始 Hex 发送", padding=8)
        raw.grid(row=4, column=0, sticky="nsew")
        raw.columnconfigure(1, weight=1)
        raw.rowconfigure(1, weight=1)

        ttk.Label(raw, text="Hex").grid(row=0, column=0)
        ttk.Entry(raw, textvariable=self.raw_hex_var).grid(row=0, column=1, sticky="ew", padx=6)
        ttk.Button(raw, text="发送", command=self.cmd_send_raw).grid(row=0, column=2, padx=4)

        logfrm = ttk.LabelFrame(raw, text="日志", padding=8)
        logfrm.grid(row=1, column=0, columnspan=3, sticky="nsew", pady=(8, 0))
        logfrm.rowconfigure(0, weight=1)
        logfrm.columnconfigure(0, weight=1)

        self.txt = tk.Text(logfrm, height=18, wrap="none")
        self.txt.grid(row=0, column=0, sticky="nsew")
        yscroll = ttk.Scrollbar(logfrm, orient="vertical", command=self.txt.yview)
        yscroll.grid(row=0, column=1, sticky="ns")
        self.txt.configure(yscrollcommand=yscroll.set)

    def log(self, text: str):
        ts = time.strftime("%H:%M:%S")
        self.txt.insert("end", f"[{ts}] {text}\n")
        self.txt.see("end")

    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def connect(self):
        if self.ser:
            return
        port = self.port_var.get()
        if not port:
            messagebox.showerror("错误", "请选择串口")
            return
        try:
            baud = int(self.baud_var.get())
            timeout = float(self.timeout_var.get())
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=timeout,
            )
            self._poll_fail_count = 0
            self.live_status_var.set("实时反馈: 已连接，等待首帧 Modbus 反馈")
            self.log(f"已连接: {port} @ {baud}")
            self.btn_connect.configure(state="disabled")
            self.btn_disconnect.configure(state="normal")
            self._start_safe_poll()
        except Exception as exc:
            self.ser = None
            messagebox.showerror("连接失败", str(exc))

    def disconnect(self):
        if self.tactile.active.is_set():
            self.tactile.stop.set()
            self.log("Please wait for matrix STOP/drain before disconnecting.")
            return
        self._stop_safe_poll()
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.live_status_var.set("实时反馈: 未连接")
        self.btn_connect.configure(state="normal")
        self.btn_disconnect.configure(state="disabled")
        self.log("已断开")

    def _slave(self) -> int:
        return int(self.slave_var.get())

    def _pump_ui_while_waiting(self):
        try:
            self.root.update_idletasks()
            self.root.update()
        except tk.TclError:
            pass

    def _send_and_recv(self, req: bytes, quiet: bool = False) -> bytes | None:
        if (self.tactile.active.is_set() and len(req) >= 6 and req[1] in (6, 16)
                and int.from_bytes(req[2:4], "big") + (int.from_bytes(req[4:6], "big") if req[1] == 16 else 1) > 0x150):
            self.log("Matrix registers are owned by the recorder during capture.")
            return None
        if not self.ser:
            if not quiet:
                messagebox.showwarning("提示", "请先连接串口")
            return None
        if self.manual_io_pending.is_set():
            if not quiet:
                self.log("上一条手动指令仍在等待响应。")
            return None
        self.manual_io_pending.set()
        try:
            with self.rx_lock:
                time.sleep(0.00175)
                serial_timeout = self.ser.timeout
                self.ser.reset_input_buffer()
                self.ser.write(req)
                self.ser.flush()
                if not quiet:
                    self.log(f"TX: {bytes_to_hex(req)}")
                    self._pump_ui_while_waiting()

                try:
                    self.ser.timeout = min(0.05, max(0.01, float(self.timeout_var.get())))
                    buf = b""
                    t0 = time.time()
                    timeout = float(self.timeout_var.get()) + 1.0
                    while time.time() - t0 < timeout:
                        chunk = self.ser.read(1)
                        if chunk:
                            buf += chunk
                            exp = expected_len_from_prefix(buf)
                            if exp is not None and len(buf) >= exp:
                                resp = buf[:exp]
                                if not quiet:
                                    if verify_crc(resp):
                                        self.log(f"RX: {bytes_to_hex(resp)}")
                                    else:
                                        self.log(f"RX(BAD CRC): {bytes_to_hex(resp)}")
                                return resp
                        if not quiet:
                            self._pump_ui_while_waiting()
                    if not quiet:
                        self.log("RX: <timeout/no response>")
                    return None
                finally:
                    self.ser.timeout = serial_timeout
        finally:
            self.manual_io_pending.clear()

    def _send_and_recv_nolog(self, req: bytes) -> bytes | None:
        """Return b"" if deferred (nothing sent), None on an exchange failure."""
        port = self.ser
        if not port or self.manual_io_pending.is_set():
            return b""
        if not self.rx_lock.acquire(blocking=False):
            return b""
        try:
            # A manual command may have arrived while acquiring the lock.
            if self.manual_io_pending.is_set():
                return b""
            serial_timeout = port.timeout
            try:
                timeout = min(0.25, float(self.timeout_var.get()))
                port.timeout = min(0.01, timeout)
                time.sleep(0.00175)
                port.reset_input_buffer()
                port.write(req)
                port.flush()
                buf = b""
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    chunk = port.read(1)
                    if not chunk:
                        continue
                    buf += chunk
                    exp = expected_len_from_prefix(buf)
                    if exp is not None and len(buf) >= exp:
                        return buf[:exp]
                return None
            finally:
                port.timeout = serial_timeout
        except Exception:
            return None
        finally:
            self.rx_lock.release()

    def _start_safe_poll(self):
        self._safe_poll_stop.clear()
        if self._safe_poll_thread and self._safe_poll_thread.is_alive():
            return
        self._safe_poll_thread = threading.Thread(target=self._safe_poll_loop, daemon=True)
        self._safe_poll_thread.start()

    def _stop_safe_poll(self):
        self._safe_poll_stop.set()
        self._safe_poll_thread = None

    def _safe_poll_loop(self):
        poll_steps = (
            self._poll_safe_feedback,
            self._poll_pressure_feedback_front,
            self._poll_pressure_feedback_back,
            self._poll_motor_feedback,
            self._poll_torque_feedback,
            self._poll_as_motor_feedback,
        )
        poll_index = 0
        if self._safe_poll_stop.wait(LIVE_POLL_PERIOD_S):
            return
        while not self._safe_poll_stop.is_set():
            if self.ser and not self.manual_io_pending.is_set():
                poll_steps[poll_index]()
                poll_index = (poll_index + 1) % len(poll_steps)
            if self._safe_poll_stop.wait(LIVE_POLL_PERIOD_S):
                break

    def _poll_read_regs(self, addr: int, qty: int) -> list[int] | None:
        slave = self._slave()
        resp = self._send_and_recv_nolog(mb_read_holding(slave, addr, qty))
        if resp == b"":
            # Matrix/manual traffic owns the port; no reply was expected.
            # Preserve previous real failures, but do not count a skipped poll.
            return None
        try:
            if resp is None:
                raise ValueError("timeout or serial error")
            if not verify_crc(resp) or resp[0] != slave:
                raise ValueError("CRC or slave mismatch")
            regs = regs_from_read03(resp)
            if len(regs) != qty or len(resp) != 5 + qty * 2:
                raise ValueError("register count mismatch")
        except Exception:
            self._poll_fail_count += 1
            if self._poll_fail_count >= 4:
                self.root.after(
                    0,
                    self.live_status_var.set,
                    "实时反馈: 连续读取失败（超时或无效回复），检查485通信",
                )
            return None
        self._poll_fail_count = 0
        self.root.after(0, self.live_status_var.set, "实时反馈: Modbus 正常")
        return regs

    def _poll_safe_feedback(self):
        regs = self._poll_read_regs(REG_SAFE_FLAGS, 4)
        if regs:
            flags = regs[0]
            sum_v = ((regs[1] << 16) | regs[2]) & 0xFFFFFFFF
            self.root.after(0, self._update_safe_status, flags, sum_v, regs[3])

    def _poll_motor_feedback(self):
        regs = self._poll_read_regs(REG_FB_ERR, 3)
        if regs:
            self.root.after(0, self._update_motor_err_status, regs[0] & 0x0F, regs[1], regs[2])

    def _poll_pressure_feedback_front(self):
        qty = 3 * PRESSURE_FEATURE_REGS
        regs = self._poll_read_regs(REG_FB_PRESS_BASE, qty)
        if regs:
            self.root.after(0, self._update_pressure_feature_group, 0, regs)

    def _poll_pressure_feedback_back(self):
        qty = 3 * PRESSURE_FEATURE_REGS
        addr = REG_FB_PRESS_BASE + qty
        regs = self._poll_read_regs(addr, qty)
        if regs:
            self.root.after(0, self._update_pressure_feature_group, 3, regs)

    def _poll_torque_feedback(self):
        regs = self._poll_read_regs(REG_FB_TOR_RAW_H, 11)
        if regs:
            self.root.after(0, self._update_torque_feedback_from_regs, regs)

    def _poll_as_motor_feedback(self):
        regs = self._poll_read_regs(REG_FB_AS_ABS_H, 12)
        if regs:
            self.root.after(0, self._update_as_motor_feedback_from_regs, regs)

    def _update_torque_feedback_from_regs(self, regs: list[int]):
        tor_raw = s32_from_regs(regs[0], regs[1]) / 1000.0
        tor_filt = s32_from_regs(regs[2], regs[3]) / 1000.0
        tor_ext = s32_from_regs(regs[4], regs[5]) / 1000.0
        contact_eff = s32_from_regs(regs[6], regs[7]) / 1000.0
        contact_flag = regs[8]
        tor_fric = s32_from_regs(regs[9], regs[10]) / 1000.0
        torque_hold = bool(self.goal_status_latest & (1 << 8))

        self.torque_text_var.set(
            "TorqueEst:\n"
            f"raw={tor_raw:.4f} N*m  filt={tor_filt:.4f} N*m  fric={tor_fric:.4f} N*m\n"
            f"ext_tau={tor_ext:.4f} N*m  contact_eff={contact_eff:.4f} N*m  "
            f"inst_contact={'YES' if contact_flag else 'NO'}  "
            f"grip_hold={'YES' if torque_hold else 'NO'}"
        )

    def _update_as_motor_feedback_from_regs(self, regs: list[int]):
        as_abs = s32_from_regs(regs[0], regs[1]) / 1000.0
        as_abs_deg = rad_to_deg(as_abs)
        as_ok = regs[2] & 0x0001

        motor_pos = s32_from_regs(regs[3], regs[4]) / 1000.0
        motor_cmd = s32_from_regs(regs[5], regs[6]) / 1000.0
        motor_raw = s32_from_regs(regs[7], regs[8]) / 1000.0
        motor_pmax = regs[9] / 1000.0
        motor_clamp = regs[10] & 0x0001
        goal_status = regs[11]

        self.motor_pos_latest = motor_pos
        self.motor_cmd_latest = motor_cmd
        self.motor_raw_latest = motor_raw
        self.motor_pmax_latest = motor_pmax
        self.motor_clamp_latest = motor_clamp
        self.goal_status_latest = goal_status

        torque_hold = bool(goal_status & (1 << 8))
        hold_new = bool(goal_status & (1 << 9))
        as_range_fault = bool(goal_status & (1 << 10))
        first_soft = bool(goal_status & (1 << 11))

        self.debug_var.set(
            f"AS: abs={as_abs_deg:.2f} deg  status={'OK' if as_ok else 'ERR'}\n"
            f"MotorDbg: pos={motor_pos:.3f} rad  cmd={motor_cmd:.3f} rad  raw={motor_raw:.3f} rad  "
            f"pmax={motor_pmax:.3f} rad  clamp={'YES' if motor_clamp else 'NO'}\n"
            f"GoalDbg: status=0x{goal_status:04X}  valid={1 if (goal_status & (1 << 0)) else 0}  "
            f"bias={1 if (goal_status & (1 << 1)) else 0}  en={1 if (goal_status & (1 << 2)) else 0}  "
            f"latch={1 if (goal_status & (1 << 3)) else 0}  block={1 if (goal_status & (1 << 4)) else 0}  "
            f"applied={1 if (goal_status & (1 << 5)) else 0}  as_bias={1 if (goal_status & (1 << 6)) else 0}  "
            f"saved_bias={1 if (goal_status & (1 << 7)) else 0}  torque_hold={1 if torque_hold else 0}  "
            f"hold_new={1 if hold_new else 0}  as_range_fault={1 if as_range_fault else 0}  "
            f"first_soft={1 if first_soft else 0}"
        )

        if motor_pmax > 0.9:
            self.dm_pmax_var.set(f"{motor_pmax + 0.2:.1f}")

    def _update_pressure_feature_group(self, first_region: int, regs: list[int]):
        for offset in range(0, len(regs), PRESSURE_FEATURE_REGS):
            region = first_region + (offset // PRESSURE_FEATURE_REGS)
            if region >= len(PRESSURE_REGION_NAMES):
                break
            feat = regs[offset:offset + PRESSURE_FEATURE_REGS]
            if len(feat) < PRESSURE_FEATURE_REGS:
                break
            self.pressure_detail_latest[PRESSURE_REGION_NAMES[region]] = {
                "cal": feat[0] & 0x0001,
                "contact": 1 if (feat[0] & 0x0002) else 0,
                "sum": ((feat[1] << 16) | feat[2]) & 0xFFFFFFFF,
                "max": feat[3],
                "area": feat[4],
            }
        self._render_pressure_feature_details()

    def _render_pressure_feature_details(self):
        lines = []
        groups = (
            ("TOTAL", "BE_SUM"),
            ("BE_C", "BE_D", "BE_E"),
            ("LE_C",),
        )
        for names in groups:
            parts = []
            for name in names:
                feat = self.pressure_detail_latest.get(name)
                if not feat:
                    parts.append(f"{name}: --")
                    continue
                parts.append(
                    f"{name}:{'ON' if feat['contact'] else 'OFF'} "
                    f"cal={feat['cal']} sum={feat['sum']} max={feat['max']} area={feat['area']}"
                )
            lines.append(" | ".join(parts))
        self.pressure_detail_var.set("触觉分区:\n" + "\n".join(lines))

    def _update_feedback_from_regs(self, regs: list[int]):
        pos_mrad = s32_from_regs(regs[0], regs[1])
        pos_rad = pos_mrad / 1000.0
        pos_deg = rad_to_deg(pos_rad)

        err_code = regs[3] & 0x0F
        mos_temp = regs[4]
        rotor_temp = regs[5]
        self._update_motor_err_status(err_code, mos_temp, rotor_temp)

        flags = regs[6]
        sum_v = ((regs[7] << 16) | regs[8]) & 0xFFFFFFFF
        max_v = regs[9]
        self._update_safe_status(flags, sum_v, max_v)

        tor_raw = s32_from_regs(regs[10], regs[11]) / 1000.0
        tor_filt = s32_from_regs(regs[12], regs[13]) / 1000.0
        tor_ext = s32_from_regs(regs[14], regs[15]) / 1000.0
        contact_eff = s32_from_regs(regs[16], regs[17]) / 1000.0
        contact_flag = regs[18]
        tor_fric = s32_from_regs(regs[19], regs[20]) / 1000.0

        as_abs = s32_from_regs(regs[21], regs[22]) / 1000.0
        as_abs_deg = rad_to_deg(as_abs)
        as_ok = regs[23] & 0x0001

        motor_pos = s32_from_regs(regs[24], regs[25]) / 1000.0
        motor_cmd = s32_from_regs(regs[26], regs[27]) / 1000.0
        motor_raw = s32_from_regs(regs[28], regs[29]) / 1000.0
        motor_pmax = regs[30] / 1000.0
        motor_clamp = regs[31] & 0x0001
        goal_status = regs[32]
        fric_raw = s32_from_regs(regs[33], regs[34]) / 1000.0
        baseline_tau = s32_from_regs(regs[35], regs[36]) / 1000.0
        contact_tau_raw = s32_from_regs(regs[37], regs[38]) / 1000.0
        contact_tau_net = s32_from_regs(regs[39], regs[40]) / 1000.0
        vel_est = s32_from_regs(regs[41], regs[42]) / 1000.0
        pos_est = s32_from_regs(regs[43], regs[44]) / 1000.0

        self.motor_pos_latest = motor_pos
        self.motor_cmd_latest = motor_cmd
        self.motor_raw_latest = motor_raw
        self.motor_pmax_latest = motor_pmax
        self.motor_clamp_latest = motor_clamp
        self.goal_status_latest = goal_status

        torque_hold = bool(goal_status & (1 << 8))
        hold_new = bool(goal_status & (1 << 9))
        as_range_fault = bool(goal_status & (1 << 10))
        first_soft = bool(goal_status & (1 << 11))

        goal_parts = [
            f"status=0x{goal_status:04X}",
            f"valid={1 if (goal_status & (1 << 0)) else 0}",
            f"bias={1 if (goal_status & (1 << 1)) else 0}",
            f"en={1 if (goal_status & (1 << 2)) else 0}",
            f"latch={1 if (goal_status & (1 << 3)) else 0}",
            f"block={1 if (goal_status & (1 << 4)) else 0}",
            f"applied={1 if (goal_status & (1 << 5)) else 0}",
            f"as_bias={1 if (goal_status & (1 << 6)) else 0}",
            f"saved_bias={1 if (goal_status & (1 << 7)) else 0}",
            f"torque_hold={1 if torque_hold else 0}",
            f"hold_new={1 if hold_new else 0}",
            f"as_range_fault={1 if as_range_fault else 0}",
            f"first_soft={1 if first_soft else 0}",
        ]
        try:
            open_deg = float(self.open_deg_var.get())
            close_deg = float(self.close_deg_var.get())
        except Exception:
            open_deg = -35.35
            close_deg = 0.0
        scale = self.scale_latest if abs(self.scale_latest) > 1e-6 else POS_SCALE_DEFAULT
        open_motor_est = motor_pos + scale * (deg_to_rad(open_deg) - pos_rad)
        close_motor_est = motor_pos + scale * (deg_to_rad(close_deg) - pos_rad)
        open_margin = motor_pmax - abs(open_motor_est)
        close_margin = motor_pmax - abs(close_motor_est)
        ideal_zero_at_motor = 0.5 * (open_motor_est + close_motor_est)
        zero_hint = "OK"
        if open_margin < 0.3 or close_margin < 0.3:
            zero_hint = "CHECK"

        self.debug_var.set(
            f"AS: abs={as_abs_deg:.2f} deg  status={'OK' if as_ok else 'ERR'}\n"
            f"MotorDbg: pos={motor_pos:.3f} rad  cmd={motor_cmd:.3f} rad  raw={motor_raw:.3f} rad  "
            f"pmax={motor_pmax:.3f} rad  clamp={'YES' if motor_clamp else 'NO'}\n"
            f"ZeroFit: open={open_motor_est:.3f} rad margin={open_margin:.3f}  "
            f"close={close_motor_est:.3f} rad margin={close_margin:.3f}  "
            f"center={ideal_zero_at_motor:.3f} rad {zero_hint}\n"
            f"GoalDbg: {' '.join(goal_parts)}\n"
            f"EstDbg: vel={vel_est:.3f} rad/s  pos={pos_est:.3f} rad  baseline={baseline_tau:.4f} N*m"
        )

        self.torque_text_var.set(
            "TorqueEst:\n"
            f"raw={tor_raw:.4f} N*m  filt={tor_filt:.4f} N*m  fric_raw={fric_raw:.4f} N*m  fric={tor_fric:.4f} N*m\n"
            f"ext_tau={tor_ext:.4f} N*m  contact_raw={contact_tau_raw:.4f} N*m  contact_net={contact_tau_net:.4f} N*m\n"
            f"contact_eff={contact_eff:.4f} N*m  "
            f"inst_contact={'YES' if contact_flag else 'NO'}  "
            f"grip_hold={'YES' if torque_hold else 'NO'}"
        )

        if motor_pmax > 0.9:
            self.dm_pmax_var.set(f"{motor_pmax + 0.2:.1f}")

    def _update_safe_status(self, flags: int, sum_v: int, max_v: int):
        latched = bool(flags & 0x0001)
        sum_over = bool(flags & 0x0002)
        max_over = bool(flags & 0x0004)
        self.safe_status_var.set(
            "SAFE:\n"
            f"flags=0x{flags:04X}  sum={sum_v}  max={max_v}  "
            f"{'LATCHED' if latched else 'OK'}"
            f"{'  SUM_OVER' if sum_over else ''}"
            f"{'  MAX_OVER' if max_over else ''}"
        )
        self.pressure_var.set(
            f"触觉: sum={sum_v}  max={max_v}  "
            f"status={'LATCHED' if latched else 'OK'}"
            f"{'  SUM_OVER' if sum_over else ''}"
            f"{'  MAX_OVER' if max_over else ''}"
        )
        if latched and not self.alarm_latched:
            self.alarm_latched = True
            messagebox.showerror(
                "压力保护触发",
                f"压力超过阈值，电机已失能。\nflags=0x{flags:04X}\nsum={sum_v}, max={max_v}\n请先减小压力后再点击“解锁”。",
            )
        if not latched:
            self.alarm_latched = False

    def _update_motor_err_status(self, err_code: int, mos_temp: int, rotor_temp: int):
        self.motor_err_code = err_code
        err_desc = MOTOR_ERR_CODES.get(err_code, f"未知(0x{err_code:X})")
        is_fault = err_code in MOTOR_ERR_FAULT_SET
        self.motor_err_var.set(f"电机: ERR=0x{err_code:X} ({err_desc})  MOS={mos_temp}°C  Rotor={rotor_temp}°C")
        if is_fault:
            self.motor_err_label.config(fg="white", bg="#CC0000")
            self.motor_err_latched = True
        else:
            self.motor_err_label.config(fg="green", bg="#F0F0F0")
            if self.motor_err_latched:
                self.motor_err_latched = False
                self.motor_err_alarmed = False

        if is_fault and not self.motor_err_alarmed:
            self.motor_err_alarmed = True
            self.log(f"*** 电机故障报警 *** ERR=0x{err_code:X}: {err_desc}  MOS={mos_temp}°C  Rotor={rotor_temp}°C")
            messagebox.showerror(
                "电机故障报警",
                f"检测到电机故障\n\n错误码: 0x{err_code:X}\n故障描述: {err_desc}\nMOS温度: {mos_temp}°C\n"
                f"Rotor温度: {rotor_temp}°C\n\n电机力矩已切断。\n请排查故障原因后点击“清除错误”。",
            )

    def _guard_if_latched(self) -> bool:
        if self.motor_err_latched:
            messagebox.showwarning(
                "电机故障保护",
                f"当前电机处于故障保护 (ERR=0x{self.motor_err_code:X}: {MOTOR_ERR_CODES.get(self.motor_err_code, '未知')})\n"
                "必须先点击“清除错误”排除故障后才能继续。",
            )
            return True
        if self.alarm_latched:
            messagebox.showwarning(
                "已触发压力保护",
                "压力超过阈值，电机已失能。\n必须先写解锁寄存器 (0x0034=0xA55A) 才能继续。",
            )
            return True
        return False

    def cmd_mode_mit(self):
        self._send_and_recv(mb_write_single(self._slave(), REG_MODE, 0x0000))

    def cmd_unit_cfg(self):
        self._send_and_recv(mb_write_single(self._slave(), REG_UNIT_CFG, UNIT_CFG_P_MDEG_V_DEG))

    def cmd_enable(self):
        if self._guard_if_latched():
            return
        self._send_and_recv(mb_write_single(self._slave(), REG_ENABLE, 0x0001))

    def cmd_disable(self):
        self._send_and_recv(mb_write_single(self._slave(), REG_ENABLE, 0x0000))

    def cmd_read_pos(self):
        pos = self._read_pos_deg(do_log=True)
        return pos

    def cmd_read_temp(self):
        resp = self._send_and_recv(mb_read_holding(self._slave(), REG_FB_MOS_TEMP, 2))
        if not resp:
            return
        try:
            regs = regs_from_read03(resp)
            self.log(f"解析: MOS温度 = {regs[0]} °C, Rotor温度 = {regs[1]} °C")
        except Exception as exc:
            self.log(f"解析失败: {exc}")

    def cmd_read_safe(self):
        resp = self._send_and_recv(mb_read_holding(self._slave(), REG_SAFE_FLAGS, 4))
        if not resp:
            return
        try:
            regs = regs_from_read03(resp)
            self._update_safe_status(regs[0], ((regs[1] << 16) | regs[2]) & 0xFFFFFFFF, regs[3])
        except Exception as exc:
            messagebox.showerror("解析失败", str(exc))

    def cmd_unlock_safe(self):
        self._send_and_recv(mb_write_single(self._slave(), REG_SAFE_UNLOCK, SAFE_UNLOCK_VALUE))
        self.cmd_read_safe()

    def cmd_write_safe_th(self):
        try:
            sum_th = int(self.sum_th_var.get())
            max_th = int(self.max_th_var.get())
        except Exception as exc:
            messagebox.showerror("参数错误", str(exc))
            return
        regs = [(sum_th >> 16) & 0xFFFF, sum_th & 0xFFFF, max_th & 0xFFFF]
        self._send_and_recv(mb_write_multi(self._slave(), REG_SAFE_SUM_TH_H, regs))
        self.cmd_read_safe()

    def cmd_save_zero(self):
        self._send_and_recv(mb_write_single(self._slave(), REG_SAVE_ZERO, 0x0001))
        try:
            self.cmd_read_persist()
            self.cmd_read_pos()
        except Exception:
            pass

    def cmd_save_motor_zero(self):
        self._send_and_recv(mb_write_single(self._slave(), REG_SAVE_MOTOR_ZERO, 0x0001))
        self.log("已发送：保存电机当前位置为电机零点 (0x0017=1)。")
        try:
            self.cmd_read_persist()
            self.cmd_read_pos()
        except Exception:
            pass

    def cmd_oneclick_calib_close_zero(self):
        if self._guard_if_latched():
            return
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("未连接", "请先连接串口。")
            return
        self.log("一键标定：请确认夹爪已完全闭合且静止。")
        try:
            self.cmd_mode_mit()
            self.cmd_unit_cfg()
        except Exception:
            pass
        _ = self._read_pos_deg(do_log=True)
        self._send_and_recv(mb_write_single(self._slave(), REG_SAVE_ZERO, 0x0001))
        time.sleep(0.15)
        try:
            self.cmd_read_persist()
        except Exception:
            pass
        post = self._read_pos_deg(do_log=True)
        self.close_deg_var.set("0")
        self.est_base_close_var.set("0.00")
        self.p_var.set("0")
        self.ratio_var.set("100")
        self._save_ui_calibration_settings(close_deg=0.0)
        self._sync_estimator_base_from_calibration("闭合标定同步力矩BASE")
        self.log("已完成：闭合=0°，零点已保存。下一步：张开到最大后点击“记录当前为全开角”。")
        if post is not None and abs(post) > 5.0:
            messagebox.showwarning(
                "零点偏差提示",
                f"标定后读到的位置仍为 {post:.2f}°（期望接近 0°）。\n建议重新闭合到位后再点一次“一键标定”。",
            )

    def cmd_capture_open_deg(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("未连接", "请先连接串口。")
            return
        pos = self._read_pos_deg(do_log=True)
        if pos is None:
            return
        self.open_deg_var.set(f"{pos:.2f}")
        self.est_base_open_var.set(f"{pos:.2f}")
        self.ratio_var.set("0")
        self._save_ui_calibration_settings(open_deg=pos)
        self._sync_estimator_base_from_calibration("全开角同步力矩BASE")
        self.log(f"已记录：全开角(0%端点) = {pos:.2f}°")

    def _read_pos_deg(self, do_log: bool = True) -> float | None:
        req = mb_read_holding(self._slave(), REG_FB_POS_H, 2)
        resp = self._send_and_recv(req) if do_log else self._send_and_recv_nolog(req)
        if not resp:
            return None
        try:
            regs = regs_from_read03(resp)
            pos_mrad = s32_from_regs(regs[0], regs[1])
            pos_rad = pos_mrad / 1000.0
            pos_deg = rad_to_deg(pos_rad)
            if do_log:
                self.log(f"解析: 位置 = {pos_mrad} mrad = {pos_rad:.3f} rad = {pos_deg:.2f}°")
            return pos_deg
        except Exception as exc:
            self.log(f"解析失败: {exc}")
            return None

    def cmd_read_persist(self):
        resp = self._send_and_recv(mb_read_holding(self._slave(), REG_PERSIST_VALID, 8))
        if not resp:
            return
        try:
            regs = regs_from_read03(resp)
            valid = regs[0]
            zero_abs_mrad = s32_from_regs(regs[1], regs[2])
            scale_x1000 = regs[3]
            self.scale_latest = scale_x1000 / 1000.0
            last_save_st = regs[4]
            save_ok_cnt = regs[5]
            bias_mrad = s32_from_regs(regs[6], regs[7])
            self.log("解析: 持久化状态")
            self.log(f"  VALID={valid}")
            self.log(f"  ZERO_ABS={zero_abs_mrad} mrad ({rad_to_deg(zero_abs_mrad / 1000.0):.2f}°)  # 零点绝对角(AS5048A)")
            self.log(f"  SCALE={scale_x1000}/1000 = {scale_x1000 / 1000.0:.3f}")
            self.log(f"  LAST_SAVE_ST=0x{last_save_st:04X}")
            self.log(f"  SAVE_OK_CNT={save_ok_cnt}")
            self.log(f"  BIAS={bias_mrad} mrad ({rad_to_deg(bias_mrad / 1000.0):.2f}°)")
        except Exception as exc:
            self.log(f"解析失败: {exc}")

    def _send_goal_values(self, slave: int, V: int, p_deg: float, kp: float, kd: float, tff: int):
        regs = []
        regs += i32_to_regs(V)
        regs += deg_to_mdeg_regs(p_deg)
        regs += [int(round(kp * 100.0)) & 0xFFFF, int(round(kd * 100.0)) & 0xFFFF]
        regs += i32_to_regs(tff)
        self._send_and_recv(mb_write_multi(slave, REG_V_DES_H, regs))

    def cmd_send_goal(self):
        if self._guard_if_latched():
            return
        try:
            V = int(float(self.v_var.get()))
            p_deg = float(self.p_var.get())
            kp = float(self.kp_var.get())
            kd = float(self.kd_var.get())
            tff = int(float(self.tff_var.get()))
        except Exception:
            messagebox.showerror("参数错误", "请检查 V/P/Kp/Kd/TFF")
            return
        self._send_goal_values(self._slave(), V, p_deg, kp, kd, tff)

    def _clamp_comp_cmd(self, p_cmd_deg: float) -> float:
        try:
            open_deg = float(self.open_deg_var.get())
            close_deg = float(self.close_deg_var.get())
            lo = min(open_deg, close_deg) - RATIO_FB_EXTRA_OPEN_DEG
            hi = max(open_deg, close_deg) + RATIO_FB_EXTRA_CLOSE_DEG
        except Exception:
            lo = -60.0
            hi = 20.0
        return max(lo, min(hi, p_cmd_deg))

    def _ui_sleep(self, seconds: float):
        time.sleep(seconds)

    def _read_pos_after_settle(
        self,
        slave: int,
        timeout_s: float | None = None,
        stable_eps_deg: float | None = None,
    ) -> float | None:
        start = time.time()
        last = None
        stable_count = 0
        timeout = timeout_s if timeout_s is not None else RATIO_FB_STABLE_TIMEOUT_S
        stable_eps = stable_eps_deg if stable_eps_deg is not None else RATIO_FB_STABLE_EPS_DEG
        while True:
            self._ui_sleep(RATIO_FB_STABLE_INTERVAL_S)
            pos = self._read_pos_deg(do_log=False)
            if pos is None:
                return None
            if last is not None:
                delta = abs(pos - last)
                if delta <= stable_eps:
                    stable_count += 1
                    if stable_count >= RATIO_FB_STABLE_COUNT:
                        return pos
                else:
                    stable_count = 0
            last = pos
            if time.time() - start >= timeout:
                return pos

    def _read_pos_until_target(self, slave: int, target_deg: float, timeout_s: float) -> tuple[float | None, bool]:
        start = time.time()
        last = None
        in_tol_count = 0
        while True:
            self._ui_sleep(RATIO_FB_STABLE_INTERVAL_S)
            pos = self._read_pos_deg(do_log=False)
            if pos is None:
                return None, False
            last = pos
            if abs(target_deg - pos) <= RATIO_FB_TOL_DEG:
                in_tol_count += 1
                if in_tol_count >= RATIO_FB_STABLE_COUNT:
                    return pos, True
            else:
                in_tol_count = 0
            if time.time() - start >= timeout_s:
                return last, False

    def _send_ratio_worker(self, ratio: float, open_deg: float, close_deg: float, slave: int, V: int, kp: float, kd: float, tff: int):
        try:
            fb_target_deg = open_deg + (ratio / 100.0) * (close_deg - open_deg)
            p_deg = self._clamp_comp_cmd(fb_target_deg)
            clamped = abs(p_deg - fb_target_deg) > 1e-6
            self.root.after(0, self.p_var.set, f"{p_deg:.2f}")
            self.log(
                f"开合度 {ratio:.1f}% -> 目标反馈={fb_target_deg:.2f}°, 下发P={p_deg:.2f}°"
                f"{' (target limit)' if clamped else ''}"
            )
            self.motor_clamp_latest = 0
            self._send_goal_values(slave, V, p_deg, kp, kd, tff)

            span_deg = abs(close_deg - open_deg)
            move_timeout = max(RATIO_FB_STABLE_TIMEOUT_S, span_deg / max(abs(V), 1.0) + 4.0)
            fb_deg, reached = self._read_pos_until_target(slave, fb_target_deg, move_timeout)
            if fb_deg is not None:
                err = fb_target_deg - fb_deg
                if self.motor_clamp_latest:
                    self.log(
                        "开合度结束：电机侧已限幅 "
                        f"(raw={self.motor_raw_latest:.3f} rad, "
                        f"cmd={self.motor_cmd_latest:.3f} rad, "
                        f"pmax={self.motor_pmax_latest:.3f} rad)，实际={fb_deg:.2f}°, 误差={err:.2f}°"
                    )
                elif reached:
                    self.log(f"开合度完成：实际={fb_deg:.2f}°, 误差={err:.2f}°")
                else:
                    self.log(f"开合度超时：实际={fb_deg:.2f}°, 误差={err:.2f}°")
        finally:
            self._ratio_busy = False

    def cmd_send_ratio(self):
        if self._guard_if_latched():
            return
        if self._ratio_busy:
            self.log("开合度动作正在执行，请等待完成。")
            return
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("未连接", "请先连接串口。")
            return
        try:
            ratio = max(0.0, min(100.0, float(self.ratio_var.get())))
            open_deg = float(self.open_deg_var.get())
            close_deg = float(self.close_deg_var.get())
            slave = self._slave()
            V = int(float(self.v_var.get()))
            kp = float(self.kp_var.get())
            kd = float(self.kd_var.get())
            tff = int(float(self.tff_var.get()))
        except Exception:
            self.log("参数读取失败，请检查开合度/全开角/全闭角/V/Kp/Kd/TFF")
            return
        self._save_ui_calibration_settings(open_deg, close_deg)
        self._sync_estimator_base_from_calibration("开合度发送前同步力矩BASE")

        self._ratio_busy = True
        threading.Thread(
            target=self._send_ratio_worker,
            args=(ratio, open_deg, close_deg, slave, V, kp, kd, tff),
            daemon=True,
        ).start()

    def cmd_read_motor_err(self):
        resp = self._send_and_recv(mb_read_holding(self._slave(), REG_FB_ERR, 3))
        if not resp:
            return
        try:
            regs = regs_from_read03(resp)
            err_code = regs[0] & 0x0F
            mos_temp = regs[1]
            rotor_temp = regs[2]
            err_desc = MOTOR_ERR_CODES.get(err_code, f"未知(0x{err_code:X})")
            self.log(f"电机状态: ERR=0x{err_code:X} ({err_desc})  MOS={mos_temp}°C  Rotor={rotor_temp}°C")
            self._update_motor_err_status(err_code, mos_temp, rotor_temp)
        except Exception as exc:
            self.log(f"解析失败: {exc}")

    def cmd_clear_motor_fault(self):
        self.log(">>> 发送清除电机错误指令 (0x0013=1)...")
        resp = self._send_and_recv(mb_write_single(self._slave(), REG_CLEAR_FAULT, 0x0001))
        if not resp:
            self.log("清错指令发送失败或无响应")
            return
        self.log("等待电机错误状态刷新中... (1s)")
        time.sleep(1.0)
        self.cmd_read_motor_err()

    def cmd_read_estimator(self):
        resp = self._send_and_recv(mb_read_holding(self._slave(), REG_EST_TOR_TH, EST_READ_QTY))
        if not resp:
            return
        try:
            regs = regs_from_read03(resp)
            self.est_raw_regs = regs[:]
            self.est_th_var.set(f"{regs[0] / 1000.0:.4f}")
            self.est_b_var.set(f"{regs[4] / 1000000.0:.10f}")
            self.est_tc_pos_var.set(f"{signed_u16_to_float_x1000(regs[5]):.6f}")
            self.est_tc_neg_var.set(f"{signed_u16_to_float_x1000(regs[6]):.6f}")
            self.est_ts_pos_var.set(f"{signed_u16_to_float_x1000(regs[7]):.6f}")
            self.est_ts_neg_var.set(f"{signed_u16_to_float_x1000(regs[8]):.6f}")
            self.est_vsign_var.set(f"{regs[9] / 1000.0:.4f}")
            dir_val = struct.unpack(">h", struct.pack(">H", regs[10]))[0]
            self.est_dir_var.set(str(dir_val))
            self.est_base_open_var.set(f"{rad_to_deg(s32_from_regs(regs[13], regs[14]) / 1000.0):.2f}")
            self.est_base_close_var.set(f"{rad_to_deg(s32_from_regs(regs[15], regs[16]) / 1000.0):.2f}")
            self.est_grip_th_var.set(f"{regs[17] / 1000.0:.4f}")
            self.log("已读取力矩估算参数。")
        except Exception as exc:
            self.log(f"读取力矩估算参数失败: {exc}")

    def cmd_write_estimator(self):
        try:
            if self.est_raw_regs is None:
                self.cmd_read_estimator()
            if self.est_raw_regs is None:
                return
            regs = self.est_raw_regs[:]
            regs[0] = int(round(float(self.est_th_var.get()) * 1000.0)) & 0xFFFF
            regs[4] = int(round(float(self.est_b_var.get()) * 1000000.0)) & 0xFFFF
            regs[5] = signed_u16_from_float_x1000(float(self.est_tc_pos_var.get()))
            regs[6] = signed_u16_from_float_x1000(float(self.est_tc_neg_var.get()))
            regs[7] = signed_u16_from_float_x1000(float(self.est_ts_pos_var.get()))
            regs[8] = signed_u16_from_float_x1000(float(self.est_ts_neg_var.get()))
            regs[9] = int(round(float(self.est_vsign_var.get()) * 1000.0)) & 0xFFFF
            regs[10] = struct.unpack(">H", struct.pack(">h", int(self.est_dir_var.get())))[0]
            regs[13:15] = deg_to_mrad_regs(float(self.est_base_open_var.get()))
            regs[15:17] = deg_to_mrad_regs(float(self.est_base_close_var.get()))
            regs[17] = int(round(float(self.est_grip_th_var.get()) * 1000.0)) & 0xFFFF
        except Exception as exc:
            messagebox.showerror("参数错误", f"请检查力矩估算参数输入: {exc}")
            return
        self._send_and_recv(mb_write_multi(self._slave(), REG_EST_TOR_TH, regs))
        self.est_raw_regs = regs
        self.log("已写入力矩估算参数。")

    def cmd_write_dm_pmax_save(self):
        try:
            pmax = float(self.dm_pmax_var.get())
            if pmax < 1.0 or pmax > 20.0:
                messagebox.showwarning("范围错误", "PMAX 建议范围为 1.0~20.0 rad。")
                return
            pmax_mrad = int(round(pmax * 1000.0))
        except Exception as exc:
            messagebox.showerror("参数错误", str(exc))
            return
        req = mb_write_multi(self._slave(), REG_DM_PMAX_MRAD, [pmax_mrad & 0xFFFF, 3])
        resp = self._send_and_recv(req)
        if resp:
            self.log(f"已写入电机 PMAX={pmax:.3f} rad 并请求保存。请断电重启后读取 MotorDbg pmax 确认。")

    def cmd_send_raw(self):
        text = self.raw_hex_var.get().strip()
        if not text:
            return
        try:
            req = hex_to_bytes(text)
        except Exception as exc:
            messagebox.showerror("Hex错误", str(exc))
            return
        self._send_and_recv(req)


def main():
    root = tk.Tk()
    app = App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
