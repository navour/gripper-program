"""Tk integration: one existing serial port and transaction lock, manual controls first."""
import queue
import threading
import time
import sys
from pathlib import Path
from tkinter import filedialog, ttk, messagebox

class TactilePanel:
    def __init__(self, app, parent):
        self.app = app
        self.active = threading.Event()
        self.stop = threading.Event()
        self.messages = queue.Queue()
        self.closing = False
        ttk.Button(parent,text='开始矩阵录制',command=self.start).grid(row=2,column=0,columnspan=2,pady=4)
        ttk.Button(parent,text='恢复录制',command=lambda:self.start(True)).grid(row=2,column=2,pady=4)
        ttk.Button(parent,text='停止并保存 H5',command=self.stop.set).grid(row=2,column=3,columnspan=2,pady=4)
        ttk.Button(parent,text='查看矩阵文件',command=self.view_file).grid(row=3,column=0,columnspan=2,pady=4)
        self.label = ttk.Label(parent,text='矩阵: 未录制')
        self.label.grid(row=2,column=5,columnspan=6,sticky='w')
        app.root.after(100,self._poll)
        app.root.protocol('WM_DELETE_WINDOW',self.close_window)

    def view_file(self):
        from tactile_viewer import open_viewer
        open_viewer(self.app.root)

    def start(self,resume=False):
        if self.active.is_set():
            return
        if not self.app.ser:
            messagebox.showwarning('矩阵录制','请先连接夹爪串口'); return
        try:
            slave = self.app._slave()
            if slave not in (1,2): raise ValueError('Slave 必须为 1 或 2')
            import h5py, numpy
        except ImportError as exc:
            python = Path(sys.executable)
            if python.name.lower() == 'pythonw.exe':
                python = python.with_name('python.exe')
            requirements = Path(__file__).resolve().with_name('requirements-tactile.txt')
            command = f'& "{python}" -m pip install --user -r "{requirements}"'
            self.app.log(f'安装录制依赖（PowerShell）：{command}')
            messagebox.showerror('矩阵录制依赖缺失',
                f'当前 Python 缺少录制依赖：{exc}\n\n'
                f'Python：{sys.executable}\n\n'
                f'请在 PowerShell 执行日志中的安装命令，然后重新打开上位机。')
            return
        except Exception as exc:
            messagebox.showerror('矩阵录制',str(exc)); return
        if resume:
            journal = filedialog.askopenfilename(title='选择原录制日志',filetypes=[('恢复日志','*.wal.sqlite')])
            if not journal: return
            path = journal[:-len('.wal.sqlite')]
        else:
            from tactile_recorder import default_output_path
            path = str(default_output_path(slave))
        self.app.log(f'矩阵保存路径：{path}')
        self.stop.clear(); self.active.set()
        self.label.configure(text='矩阵: 准备录制')
        serial_port = self.app.ser
        threading.Thread(target=self._worker,args=(serial_port,slave,path,resume),daemon=True).start()

    def _worker(self,serial_port,slave,path,resume):
        from tactile_protocol import MatrixClient, SerialTransport
        from tactile_recorder import Recorder,capture
        transport = SerialTransport(serial_port)
        def exchange(request):
            deadline = time.monotonic()+2
            while True:
                if not self.app.manual_io_pending.is_set() and self.app.rx_lock.acquire(timeout=0.005):
                    if self.app.manual_io_pending.is_set():
                        self.app.rx_lock.release()
                    else:
                        try:
                            time.sleep(0.00175)
                            return transport(request)
                        finally:
                            self.app.rx_lock.release()
                if time.monotonic()>deadline:
                    raise TimeoutError('控制指令持续占用总线，矩阵录制无法获得服务')
                time.sleep(0.001)
        recorder = None
        try:
            recorder = Recorder(path)
            capture(MatrixClient(exchange,slave),recorder,self.stop,resume,notify=self.messages.put)
            self.messages.put('采集结束，正在核验 HDF5')
        except Exception as exc:
            self.messages.put(f'录制未完整完成: {exc}；保留日志后可恢复')
        finally:
            if recorder is not None:
                try:
                    recorder.close()
                    self.messages.put(f'HDF5 已生成: {path}（完整性见 complete 属性）')
                except Exception as exc:
                    self.messages.put(f'保存失败: {exc}')
            self.messages.put(None)

    def _poll(self):
        try:
            while True:
                text = self.messages.get_nowait()
                if text is None:
                    self.active.clear()
                else:
                    self.label.configure(text=text)
                    self.app.log(text)
        except queue.Empty:
            pass
        if self.closing and not self.active.is_set():
            self.app.disconnect(); self.app.root.destroy(); return
        self.app.root.after(100,self._poll)

    def close_window(self):
        self.closing = True
        self.stop.set()
        if self.active.is_set():
            self.label.configure(text='矩阵: 停止并保存后关闭窗口')
