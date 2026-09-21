"""Tkinter USB-RS485 updater for the STM32G431 gripper Bootloader."""

from __future__ import annotations

import pathlib
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - user-facing startup error
    raise SystemExit("缺少pyserial，请先执行：python -m pip install pyserial") from exc

from gripper_ota_protocol import (
    OtaError,
    SerialBootClient,
    STATE_NAMES,
    describe_hello,
    enter_bootloader_from_application,
    validate_application,
    validate_hello,
    wait_for_bootloader,
)


class GripperOtaTool:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("夹爪G431 OTA工具（USB-RS485直连）")
        self.root.geometry("920x670")
        self.serial_port = None
        self.busy = False
        self.cancel_event = threading.Event()

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.gripper_var = tk.StringVar(value="1")
        self.file_var = tk.StringVar()
        self.major_var = tk.StringVar(value="1")
        self.minor_var = tk.StringVar(value="15")
        self.patch_var = tk.StringVar(value="0")
        self.auto_enter_var = tk.BooleanVar(value=True)
        self.auto_reboot_var = tk.BooleanVar(value=True)
        self.connection_var = tk.StringVar(value="未连接")
        self.progress_var = tk.DoubleVar(value=0.0)
        self.progress_text_var = tk.StringVar(value="0 / 0 bytes")

        self._build_ui()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)

        connection = ttk.LabelFrame(outer, text="串口连接", padding=8)
        connection.pack(fill="x")
        ttk.Label(connection, text="端口").grid(row=0, column=0, padx=4, pady=4)
        self.port_combo = ttk.Combobox(
            connection, textvariable=self.port_var, width=18, state="readonly"
        )
        self.port_combo.grid(row=0, column=1, padx=4, pady=4)
        ttk.Button(connection, text="刷新", command=self.refresh_ports).grid(
            row=0, column=2, padx=4, pady=4
        )
        ttk.Label(connection, text="波特率").grid(row=0, column=3, padx=4, pady=4)
        ttk.Entry(connection, textvariable=self.baud_var, width=10).grid(
            row=0, column=4, padx=4, pady=4
        )
        ttk.Label(connection, text="夹爪编号").grid(
            row=0, column=5, padx=4, pady=4
        )
        ttk.Combobox(
            connection,
            textvariable=self.gripper_var,
            values=("1", "2"),
            width=6,
            state="readonly",
        ).grid(
            row=0, column=6, padx=4, pady=4
        )
        self.connect_button = ttk.Button(
            connection, text="连接", command=self.toggle_connection
        )
        self.connect_button.grid(row=0, column=7, padx=8, pady=4)
        ttk.Label(connection, textvariable=self.connection_var).grid(
            row=0, column=8, padx=8, pady=4, sticky="w"
        )

        firmware = ttk.LabelFrame(outer, text="Application固件", padding=8)
        firmware.pack(fill="x", pady=(10, 0))
        ttk.Entry(firmware, textvariable=self.file_var).grid(
            row=0, column=0, columnspan=8, padx=4, pady=4, sticky="ew"
        )
        ttk.Button(firmware, text="选择BIN", command=self.choose_file).grid(
            row=0, column=8, padx=4, pady=4
        )
        firmware.columnconfigure(0, weight=1)

        labels = (
            ("版本Major", self.major_var, 0),
            ("Minor", self.minor_var, 2),
            ("Patch", self.patch_var, 4),
        )
        for text, variable, column in labels:
            ttk.Label(firmware, text=text).grid(row=1, column=column, padx=4, pady=4)
            ttk.Entry(firmware, textvariable=variable, width=7).grid(
                row=1, column=column + 1, padx=4, pady=4
            )

        options = ttk.Frame(outer)
        options.pack(fill="x", pady=(8, 0))
        ttk.Checkbutton(
            options,
            text="未发现Bootloader时，自动通过Modbus让Application进入Bootloader",
            variable=self.auto_enter_var,
        ).pack(side="left")
        ttk.Checkbutton(
            options,
            text="升级成功后自动重启Application",
            variable=self.auto_reboot_var,
        ).pack(side="left", padx=(18, 0))

        actions = ttk.Frame(outer)
        actions.pack(fill="x", pady=(10, 0))
        self.enter_button = ttk.Button(
            actions, text="进入Bootloader", command=self.on_enter_bootloader
        )
        self.enter_button.pack(side="left", padx=4)
        self.query_button = ttk.Button(
            actions, text="读取Bootloader状态", command=self.on_query
        )
        self.query_button.pack(side="left", padx=4)
        self.update_button = ttk.Button(
            actions, text="开始OTA升级", command=self.on_update
        )
        self.update_button.pack(side="left", padx=4)
        self.cancel_button = ttk.Button(
            actions, text="取消", command=self.on_cancel, state="disabled"
        )
        self.cancel_button.pack(side="left", padx=4)

        progress_frame = ttk.Frame(outer)
        progress_frame.pack(fill="x", pady=(12, 0))
        self.progress = ttk.Progressbar(
            progress_frame, variable=self.progress_var, maximum=100.0
        )
        self.progress.pack(side="left", fill="x", expand=True)
        ttk.Label(progress_frame, textvariable=self.progress_text_var, width=22).pack(
            side="left", padx=(8, 0)
        )

        log_frame = ttk.LabelFrame(outer, text="日志", padding=6)
        log_frame.pack(fill="both", expand=True, pady=(10, 0))
        self.log_text = tk.Text(log_frame, height=22, wrap="word")
        scrollbar = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        self.log(
            "提示：首次烧入Bootloader后可直接升级；Application正常运行时，"
            "工具会先写0x0012=0，再按顺序写OTA Key。"
        )

    def log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")

        def append() -> None:
            self.log_text.insert("end", f"[{timestamp}] {message}\n")
            self.log_text.see("end")

        self.root.after(0, append)

    def refresh_ports(self) -> None:
        ports = [item.device for item in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def choose_file(self) -> None:
        path = filedialog.askopenfilename(
            title="选择从0x08006000链接生成的Application BIN",
            filetypes=(("Binary firmware", "*.bin"), ("All files", "*.*")),
        )
        if path:
            self.file_var.set(path)

    def toggle_connection(self) -> None:
        if self.serial_port is not None:
            self.disconnect()
            return
        if not self.port_var.get():
            messagebox.showerror("连接失败", "请选择串口")
            return
        try:
            self.serial_port = serial.Serial(
                port=self.port_var.get(),
                baudrate=int(self.baud_var.get()),
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,
                write_timeout=1.0,
            )
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
        except Exception as exc:
            self.serial_port = None
            messagebox.showerror("连接失败", str(exc))
            return
        self.connection_var.set(f"已连接 {self.port_var.get()}")
        self.connect_button.configure(text="断开")
        self.log(f"串口已连接：{self.port_var.get()} @ {self.baud_var.get()} 8N1")

    def disconnect(self) -> None:
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            finally:
                self.serial_port = None
        self.connection_var.set("未连接")
        self.connect_button.configure(text="连接")
        self.log("串口已断开")

    def _require_serial(self):
        if self.serial_port is None or not self.serial_port.is_open:
            raise OtaError("请先连接USB-RS485串口")
        return self.serial_port

    def _set_busy(self, busy: bool) -> None:
        self.busy = busy
        state = "disabled" if busy else "normal"
        for button in (
            self.enter_button,
            self.query_button,
            self.update_button,
        ):
            button.configure(state=state)
        self.connect_button.configure(state=state)
        self.cancel_button.configure(state="normal" if busy else "disabled")

    def _run_worker(self, operation) -> None:
        if self.busy:
            return
        try:
            self._require_serial()
        except OtaError as exc:
            messagebox.showerror("无法执行", str(exc))
            return
        self.cancel_event.clear()
        self._set_busy(True)

        def worker() -> None:
            try:
                operation()
            except Exception as exc:
                self.log(f"失败：{exc}")
                self.root.after(
                    0,
                    lambda error=str(exc): messagebox.showerror("操作失败", error),
                )
            finally:
                self.root.after(0, lambda: self._set_busy(False))

        threading.Thread(target=worker, daemon=True).start()

    def _boot_client(self) -> SerialBootClient:
        return SerialBootClient(self._require_serial(), log=self.log)

    def _discover_or_enter(self, auto_enter: bool, slave: int):
        client = self._boot_client()
        try:
            session_id, hello = wait_for_bootloader(client, timeout=0.7)
            self.log("已直接发现Bootloader：" + describe_hello(hello))
            return client, session_id, hello
        except OtaError:
            if not auto_enter:
                raise OtaError("当前没有发现Bootloader；请复位板卡或允许自动进入")

        self.log(f"未发现Bootloader，向Application从站{slave}发送安全进入命令")
        enter_bootloader_from_application(self._require_serial(), slave)
        self.log("Application已确认进入请求，等待软件复位和Bootloader启动")
        session_id, hello = wait_for_bootloader(client, timeout=3.0)
        self.log("Bootloader已响应：" + describe_hello(hello))
        return client, session_id, hello

    def on_enter_bootloader(self) -> None:
        try:
            gripper = int(self.gripper_var.get(), 0)
        except ValueError as exc:
            messagebox.showerror("参数错误", str(exc))
            return

        def operation() -> None:
            _client, _session, hello = self._discover_or_enter(True, gripper)
            validate_hello(hello, gripper)
            self.log("进入Bootloader成功")

        self._run_worker(operation)

    def on_query(self) -> None:
        try:
            gripper = int(self.gripper_var.get(), 0)
        except ValueError as exc:
            messagebox.showerror("参数错误", str(exc))
            return

        def operation() -> None:
            client, session, hello = self._discover_or_enter(False, gripper)
            validate_hello(hello, gripper)
            query = client.query(session, 1)
            state = STATE_NAMES.get(query.state, str(query.state))
            self.log(
                f"QUERY：state={state}, image_size={query.image_size}, "
                f"image_crc32=0x{query.image_crc32:08X}, "
                f"metadata_seq={query.metadata_sequence}"
            )

        self._run_worker(operation)

    def on_update(self) -> None:
        try:
            firmware_path = pathlib.Path(self.file_var.get())
            gripper = int(self.gripper_var.get(), 0)
            version = (
                int(self.major_var.get(), 0),
                int(self.minor_var.get(), 0),
                int(self.patch_var.get(), 0),
            )
            auto_enter = bool(self.auto_enter_var.get())
            auto_reboot = bool(self.auto_reboot_var.get())
        except ValueError as exc:
            messagebox.showerror("参数错误", str(exc))
            return

        def operation() -> None:
            if not firmware_path.is_file():
                raise OtaError("请选择有效的Application BIN文件")
            image = validate_application(firmware_path.read_bytes())
            self.log(
                f"固件检查通过：{image.size}字节，CRC32=0x{image.crc32:08X}，"
                f"MSP=0x{image.initial_msp:08X}，Reset=0x{image.reset_handler:08X}"
            )

            client, _hello_session, hello = self._discover_or_enter(
                auto_enter, gripper
            )
            validate_hello(hello, gripper)

            def progress(done: int, total: int) -> None:
                percent = (done * 100.0 / total) if total else 0.0
                self.root.after(0, lambda: self.progress_var.set(percent))
                self.root.after(
                    0, lambda: self.progress_text_var.set(f"{done} / {total} bytes")
                )

            session_id, query = client.update(
                image,
                gripper,
                version,
                progress=progress,
                cancelled=self.cancel_event.is_set,
            )
            self.log(
                f"升级成功：Metadata={STATE_NAMES.get(query.state, query.state)}，"
                f"CRC32=0x{query.image_crc32:08X}"
            )
            if auto_reboot:
                self.log("发送REBOOT，目标将复位并启动Application")
                client.reboot(session_id, 0xFFFE)
                self.log("REBOOT已确认；约0.5秒后可使用普通Modbus PC_Tool连接")

        self._run_worker(operation)

    def on_cancel(self) -> None:
        if self.busy:
            self.cancel_event.set()
            self.log("已请求取消；工具会在当前数据块结束后发送ABORT")

    def on_close(self) -> None:
        self.cancel_event.set()
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    GripperOtaTool(root)
    root.mainloop()


if __name__ == "__main__":
    main()
