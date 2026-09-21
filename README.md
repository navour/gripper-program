# gripper-program

双夹爪 STM32G431 固件、USB-RS485 上位机、原始触觉矩阵采集与 HDF5 存储工具。

## 目录

| 目录 | 内容 |
| --- | --- |
| `5_夹爪总程序id1slave1` | 夹爪 1 固件及上位机，Modbus 从站地址 1 |
| `5_夹爪总程序id2slave2` | 夹爪 2 固件及上位机，Modbus 从站地址 2 |
| `Bootloader_G431` | 引导程序工程 |
| `OTA_PC_Tool` | 固件升级工具及发布固件 |
| `tactile_tests` | 矩阵协议、存储、固件历史模块及反馈轮询回归测试 |
| `matrix_validation` | 回归测试使用的编译参考文件及验证记录 |

## USB-485 上位机

在对应夹爪的 `PC_Tool` 目录运行：

```powershell
python -m pip install -r requirements-tactile.txt
python 3_gripper.py
```

连接实际 COM 口，选择 115200 波特率及对应从站地址。矩阵录制自动按日期时间命名；停止保存后，可通过“查看矩阵文件”查看 HDF5 数据。

使用方法、寄存器、录制完整性校验和实机验收步骤见 [USB485矩阵采集代码修改说明.md](USB485矩阵采集代码修改说明.md)。设计方案见 [双夹爪原始矩阵逐帧采集与HDF5存储方案.md](双夹爪原始矩阵逐帧采集与HDF5存储方案.md)。

## 固件与测试

Keil 工程位于各固件目录的 `MDK-ARM` 下。烧录方式及地址应按对应工程和部署文档确认。

在仓库根目录执行软件回归测试：

```powershell
python -m pip install -r tactile_tests/requirements.txt
python -m unittest discover -s tactile_tests -v
```

固件仿真测试使用 `matrix_validation` 中的参考 AXF；修改固件后需重新构建并更新参考文件。自动化测试不能替代 USB-485、电机控制与断电等实机验收。

本地 Python 环境、编辑器缓存、录制数据、个人校准设置和临时备份由 `.gitignore` 排除，仍保留在本机。源码依赖及原有许可证文件随工程保留。
