"""Read-only tactile HDF5 viewer. Does not open serial ports or send commands."""
import argparse
import json
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

class MatrixViewer:
    def __init__(self, window):
        self.window = window
        self.file = None
        self.groups = {}
        self.dataset = None
        self.window.title('触觉矩阵文件查看器（只读）')
        self.window.minsize(940, 540)
        self.path_text = tk.StringVar(value='请选择已停止录制的 .h5 文件')
        self.session = tk.StringVar()
        self.index = tk.StringVar(value='1')
        self.info = tk.StringVar()
        bar = ttk.Frame(window, padding=8)
        bar.pack(fill='x')
        ttk.Button(bar, text='打开 H5 文件', command=self.choose).pack(side='left')
        ttk.Label(bar, textvariable=self.path_text, wraplength=780).pack(side='left', padx=8)
        nav = ttk.Frame(window, padding=8)
        nav.pack(fill='x')
        self.sessions = ttk.Combobox(nav, textvariable=self.session, state='readonly', width=34)
        self.sessions.pack(side='left')
        self.sessions.bind('<<ComboboxSelected>>', self.select_session)
        ttk.Button(nav, text='上一帧', command=lambda:self.move(-1)).pack(side='left', padx=5)
        ttk.Label(nav, text='帧号').pack(side='left')
        self.spin = ttk.Spinbox(nav, from_=1, to=1, textvariable=self.index, width=9, command=self.show)
        self.spin.pack(side='left')
        self.spin.bind('<Return>', self.show)
        ttk.Button(nav, text='跳转', command=self.show).pack(side='left', padx=5)
        ttk.Button(nav, text='下一帧', command=lambda:self.move(1)).pack(side='left')
        ttk.Label(window, textvariable=self.info, padding=8, wraplength=920).pack(fill='x')
        matrices = ttk.Frame(window, padding=8)
        matrices.pack(fill='both', expand=True)
        self.panels, self.cells = [], []
        for region in range(4):
            panel = ttk.LabelFrame(matrices, text=f'区域 {region}', padding=5)
            panel.grid(row=0, column=region, padx=4, sticky='nsew')
            matrices.columnconfigure(region, weight=1)
            self.panels.append(panel)
            cells=[]
            for col in range(4):
                ttk.Label(panel, text=f'列{col}').grid(row=0,column=col,pady=4)
            for row in range(8):
                line=[]
                for col in range(4):
                    value=tk.StringVar(value='—')
                    ttk.Label(panel,textvariable=value,width=7,anchor='center',relief='solid',padding=(2,6)).grid(row=row+1,column=col,sticky='nsew')
                    line.append(value)
                cells.append(line)
            self.cells.append(cells)
        ttk.Label(window,text='显示传感器原始计数；每块按行排列为 8×4，物理朝向需按实物确认。',padding=8).pack(fill='x')
        window.protocol('WM_DELETE_WINDOW',self.close)

    def choose(self):
        path=filedialog.askopenfilename(parent=self.window,title='打开矩阵文件',filetypes=[('HDF5','*.h5')])
        if path:self.open_file(path)

    def open_file(self, path):
        candidate=None
        try:
            import h5py
            if str(path).endswith('.partial.h5'):
                raise ValueError('请先停止录制，打开最终 .h5；异常中断时先从日志恢复。')
            candidate=h5py.File(path,'r')
            groups={}
            for device in ('gripper1','gripper2'):
                if device not in candidate:continue
                for session,group in candidate[device].items():
                    if 'raw' in group:
                        raw=group['raw']
                        if raw.ndim!=4 or raw.shape[1:]!=(4,8,4):
                            raise ValueError(f'{raw.name} 不是 [帧数,4,8,4] 格式')
                        if len(raw):groups[f'{device}/{session}']=group
            if not groups:raise ValueError('文件中没有可查看的矩阵帧')
            if self.file is not None:self.file.close()
            self.file,self.groups=candidate,groups
            candidate=None
            self.path_text.set(str(Path(path).resolve()))
            self.sessions['values']=list(groups)
            self.session.set(next(iter(groups)))
            self.select_session()
        except Exception as exc:
            if candidate is not None:candidate.close()
            messagebox.showerror('无法打开矩阵文件',str(exc),parent=self.window)

    def select_session(self, event=None):
        if self.session.get() not in self.groups:return
        self.dataset=self.groups[self.session.get()]
        self.spin.configure(to=len(self.dataset['raw']))
        self.index.set('1')
        self.show()

    def move(self, delta):
        if self.dataset is None:return
        try:index=int(self.index.get())
        except ValueError:index=1
        self.index.set(str(min(len(self.dataset['raw']),max(1,index+delta))))
        self.show()

    def show(self, event=None):
        if self.dataset is None:return
        try:
            group=self.dataset
            index=int(self.index.get())-1
            count=len(group['raw'])
            if not 0<=index<count:raise ValueError(f'帧号应在 1 到 {count} 之间')
            raw=group['raw'][index]
            regions=json.loads(group.attrs.get('regions','["区域0","区域1","区域2","区域3"]'))
            for region in range(4):
                self.panels[region].configure(text=regions[region])
                for row in range(8):
                    for col in range(4):self.cells[region][row][col].set(str(int(raw[region,row,col])))
            flags=int(group['sample_flags'][index])
            cal=int(group['cal_mask'][index])
            seq=int(group['sequence'][index])
            be=int(group['be_ms'][index]);le=int(group['le_ms'][index])
            complete=bool(group.attrs.get('complete',False))
            drops=group.attrs.get('drop_count','未知')
            self.info.set(f'第 {index+1}/{count} 帧；采集序号 {seq}；BE={be} ms，LE={le} ms\n'
                f'本轮 BE 成功={bool(flags&1)}，LE 成功={bool(flags&2)}；flags=0x{flags:04X}；校准掩码=0x{cal:04X}\n'
                f'文件记录的完整状态={complete}；丢帧数={drops}（本查看器显示内容；完整校验用 --verify）')
        except Exception as exc:
            messagebox.showerror('无法显示矩阵',str(exc),parent=self.window)

    def close(self):
        if self.file is not None:self.file.close();self.file=None
        self.window.destroy()

def open_viewer(parent):
    window=tk.Toplevel(parent)
    viewer=MatrixViewer(window)
    window.after(0,viewer.choose)
    return viewer

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('file',nargs='?')
    args=parser.parse_args()
    root=tk.Tk()
    viewer=MatrixViewer(root)
    if args.file:root.after(0,lambda:viewer.open_file(args.file))
    root.mainloop()

if __name__=='__main__':main()
