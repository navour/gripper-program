# 夹爪 OTA 首次部署、Keil 烧录与验收归档

## 1. 文档定位

本文记录夹爪 1/2 从旧的 `0x08000000` 裸 Application 首次迁移到“Bootloader + `0x08006000` Application”布局时的手工操作，以及以后使用 Keil/ST-Link 烧录时必须保持的设置。文件名沿用原夹爪 2 部署文档，以避免既有引用失效；内容现覆盖两只夹爪。

日常 OTA 不需要重复本流程。已完成首次部署的设备优先使用 [机器人以太网工具](../robot-program0827OTA/夹爪OTA以太网工具/README.md)；台架直连和串口恢复见 [USB-RS485 工具](OTA_PC_Tool/README.md)。

## 2. 工程和角色对应

| 项目 | 夹爪 1 | 夹爪 2 |
|---|---|---|
| Application 工程 | `5_夹爪总程序id1slave1/MDK-ARM/JIAZHUA_RTOS.uvprojx` | `5_夹爪总程序id2slave2/MDK-ARM/JIAZHUA_RTOS.uvprojx` |
| Target | `JIAZHUA_RTOS` | `JIAZHUA_RTOS` |
| Role / Modbus 从站 | 1 / 1 | 2 / 2 |
| 主控串口 | UART7 | UART8 |
| OTA BIN | `OTA_PC_Tool/gripper1_app.bin` | `OTA_PC_Tool/gripper2_app.bin` |
| 普通调试工具 | 各自工程 `PC_Tool/3_gripper.py` | 各自工程 `PC_Tool/3_gripper.py` |

公共 Bootloader 工程为：

```text
Bootloader_G431/Bootloader_G431/MDK-ARM/Bootloader_G431.uvprojx
```

开发版公共 Bootloader 默认 `GRIPPER_DEVICE_ROLE=0`，可用于两只夹爪。0 表示 Bootloader 未锁角色，不是 Modbus 从站 0。若构建时定义了 `GRIPPER_DEVICE_ROLE=1/2`，产物必须明确标记且只用于对应设备。

界面选择夹爪编号只决定目标 UART、Modbus 地址和 Manifest Role，不会改写 BIN 内部的从站、Role 或版本。必须选择对应工程生成的 BIN，不能改名混用。

## 3. 首次部署前准备

### 3.1 安全条件

- 机器人停机，夹爪空载，人员和物品离开运动范围；
- 电机保持失能；
- 调整 SWD/RS485 接线前断电；
- 台架接 USB-RS485 时断开机器人原 RS485 主站，避免两个主站并联；
- 供电稳定，ST-Link 连接 SWDIO、SWCLK、GND、VTref，建议同时接 NRST；
- 关闭普通 PC Tool、串口助手和 OTA 工具，避免串口或目标被并发访问。

### 3.2 双份备份校准页

每只夹爪只能使用自己的校准备份。用 STM32CubeProgrammer 读取：

```text
Address: 0x0801F800
Length : 0x800（2048 字节）
```

分别从芯片读取两次，保存为类似 `gripper1_calib_1.bin` 和 `gripper1_calib_2.bin`，不要只复制同一个文件。比较：

```powershell
fc.exe /b ".\gripper1_calib_1.bin" ".\gripper1_calib_2.bin"
```

两份都必须为 2048 字节且无差异。建议同时记录校准有效标志、零点、比例/bias 和迁移前正常运动现象，并额外导出完整 128 KiB Flash 作为只读留档。

如果误擦校准页，只能把同一只夹爪自己的备份写回 `0x0801F800`，只擦该页并 Verify。不要用另一只夹爪的校准数据，也不要用旧全片镜像恢复校准，因为全片镜像会覆盖新 Bootloader、Application 和 Metadata。

## 4. Keil 固定配置

工程设置已经写入 `.uvprojx/.uvoptx` 和 Scatter，但首次在新电脑打开、Keil 曾长期保持工程打开、CubeMX 重新生成或工具链升级后，必须逐项核对。

### 4.1 Bootloader Target

| 页面 | 项目 | 值 |
|---|---|---|
| Device | MCU | `STM32G431CBTx` |
| Target | IROM1 | Start `0x08000000`，Size `0x00006000` |
| Target | IRAM1 | Start `0x20000000`，Size `0x00007FE0` |
| C/C++ | Defines | `USE_HAL_DRIVER,STM32G431xx`；角色锁定版另加 `GRIPPER_DEVICE_ROLE=1/2` |
| Linker | Scatter | `Bootloader_G431.sct` |
| Output | HEX | 开启 Create HEX File |
| User | After Build | 生成 `Bootloader_G431/bootloader_g431.bin` |

Scatter 把代码限制在前 22 KiB，并把 `0x08005800~0x08005FFF` 固定为首次部署许可页。最后 32 字节 SRAM 不参与链接。任何 Load Region 溢出都必须修复代码，禁止扩大到整片 Flash 绕过。

### 4.2 Application Target

夹爪 1/2 的设置相同，只有代码中的 Role、从站和导出文件名不同：

| 页面 | 项目 | 值 |
|---|---|---|
| Device | MCU | `STM32G431CBTx` |
| Target | IROM1 | Start `0x08006000`，Size `0x00019000` |
| Target | IRAM1 | Start `0x20000000`，Size `0x00007FE0` |
| C/C++ | Define | 保留原宏，并含 `APP_VECT_TAB_OFFSET=0x00006000U` |
| C/C++ | Include Paths | 包含 `../User/OTA` |
| Linker | Scatter | `JIAZHUA_RTOS/JIAZHUA_RTOS.sct`，LR/ER 起点必须为 `0x08006000` |
| Project Group | OTA | `gripper_ota_app.c` 参与构建 |

夹爪 1 After Build：

```text
fromelf.exe --bin --output .\JIAZHUA_RTOS\gripper1_app.bin .\JIAZHUA_RTOS\JIAZHUA_RTOS.axf
fromelf.exe --bin --output ..\..\OTA_PC_Tool\gripper1_app.bin .\JIAZHUA_RTOS\JIAZHUA_RTOS.axf
```

夹爪 2 把文件名改为 `gripper2_app.bin`。Rebuild 后必须看到 0 Error 和两条 fromelf 成功；导出失败时不得继续使用公共工具目录中时间较旧的 BIN。

### 4.3 Debug 设置

Bootloader 和 Application 两个工程分别核对：

1. `Options for Target -> Debug` 选择 ST-Link；
2. Settings 中 Port=`SW`，SW Device=`ARM CoreSight SW-DP`；
3. Connect=`Normal`，Reset=`SYSRESETREQ`；
4. 勾选 `Reset after Connect`，不勾选 `Stop after Reset`；
5. 首次连接建议 SWD Clock 1~4 MHz；连接困难时使用 `Connect under Reset` 并接 NRST；
6. 正式下载使用 `Flash -> Download`，不要在下载后进入 Debug 并让 CPU 停在断点。

如果 Keil 下载后 Bootloader HELLO 和 Application Modbus 都无响应，而重新上电恢复，通常是 CPU 仍处于 HALT 或复位释放设置不正确。完全关闭 Keil 且不保存旧的内存设置，重新打开工程后复核上述项目。

### 4.4 Flash Download 设置

1. Utilities 选择与 Debug 页一致的 ST-Link Target Driver；
2. Flash Algorithm 使用 `STM32G43x-4x_128.FLM`，Start=`0x08000000`，Size=`0x00020000`；
3. Download Function 选择 `Erase Sectors`；
4. 勾选 `Program`、`Verify`、`Reset and Run`；
5. 禁止选择 `Erase Full Chip`，除非已经明确批准整片擦除并完成校准恢复准备；
6. 不修改 RDP、WRP、BOOT_LOCK 等 Option Bytes。

Flash Algorithm 的范围是探针能力范围，不是当前固件写入范围。实际写入地址由 AXF/HEX 和 Scatter 决定，不要为了 Application 把算法起点改成 `0x08006000`。

## 5. Rebuild 后检查

### 5.1 Bootloader

- AXF/HEX/BIN/MAP 均已生成；
- `LR_IROM1` 从 `0x08000000` 开始，代码区终点不超过 `0x08005800`；
- `ER_FACTORY` 固定于 `0x08005800`；
- RAM 终点不超过 `0x20007FE0`；
- BIN 不超过 `0x6000`；
- Boot 模块实际参与链接。

### 5.2 Application

- RESET/向量表位于 `0x08006000`；
- Load Region 不超过 `0x0801F000`；
- RAM 终点不超过 `0x20007FE0`；
- BIN 不超过 `0x19000`；
- 首字 MSP 位于 G431 SRAM，Reset Handler 带 Thumb 位且位于 Application 区；
- OTA 身份签名存在；
- 公共 `OTA_PC_Tool/gripperN_app.bin` 的修改时间是本次构建时间。

Target 页上方的通用 CPU 描述可能仍显示芯片总 IROM；最终以具体 IROM1/IRAM1、Scatter、MAP 和 BIN 四项一致为准。

## 6. 首次 ST-Link 双烧

旧 Application 从 `0x08000000` 启动，第一次写 Bootloader 会覆盖其入口，因此 Bootloader 和重定位 Application 必须作为同一次受控迁移准备好。

1. 完成校准页双份备份；
2. Rebuild 公共 G431 Bootloader 和目标夹爪 Application；
3. 按第 5 节检查产物；
4. 关闭串口工具，在 Keil 打开 Bootloader 工程，执行 `Flash -> Download`；
5. 确认分区擦除、Program、Verify 成功；此时夹爪业务不运行是正常现象；
6. 关闭/切换到目标夹爪 Application 工程，再执行 `Flash -> Download`；
7. 确认 Reset and Run 后 CPU 正常释放；Bootloader 从复位入口运行并跳转 Application；
8. 重新读取 `0x0801F800~0x0801FFFF`，与部署前备份逐字节比较；
9. 不要主动保存零点或重新标定来“验证”，这些操作会改变校准页，破坏对比依据。

首次部署必须按“Bootloader -> 对应 Application”的顺序。以后若已有 VALID Metadata 却又通过 ST-Link 直接覆盖 Application，Metadata 的旧 CRC 可能使 Bootloader 拒绝启动。优先使用生成好的离线恢复 HEX，或重新执行受控的 Bootloader/Application/Metadata 恢复流程，不要只覆盖 BIN 后反复断电尝试。

## 7. USB-RS485 直连验收

接入单只夹爪，115200、8N1。先用对应工程的普通 `3_gripper.py` 验证 Modbus：

| 寄存器 | 预期值 |
|---:|---|
| `0x0001` / `0x00F5` | `0x010F`（1.15） |
| `0x00F3` | `0x0100`（Boot 1.0） |
| `0x00F4` | `0x0007`（当前 OTA 能力） |
| `0x00F6` | `0`（Patch） |
| `0x00F7` | `1`（READY） |
| `0x00F8` | `1`（Layout） |
| `0x00F9` | 夹爪 1 为 1；夹爪 2 为 2 |

随后关闭普通工具，运行：

```powershell
cd .\OTA_PC_Tool
python -m pip install -r .\requirements.txt
python .\gripper_ota_tool.py
```

选择正确 COM、夹爪编号、`gripperN_app.bin` 和 `1.15.0`，保留两个自动选项，执行一次完整 OTA。成功后断开 OTA 工具，用普通工具复查版本、READY、Role 和校准状态。

直连工具不会执行 H723 的单位恢复。普通工具发送位置前，先失能、设置 MIT，并配置 `0x0014=0x000B`，再在空载条件下低速小范围使能测试。READY/OTA 能力只证明程序入口正常，不能替代反馈、压力、编码器、限位和机械运动验收。

## 8. 机器人以太网验收

1. 断开 USB-RS485 主站，安全断电后把夹爪 1 接 UART7、夹爪 2 接 UART8 对应的 RS485；
2. A/B/GND 和供电正确；
3. H723 运行包含夹爪 OTA 与单位恢复的固件，连接标识应为 `20260909-unit1`；
4. 电脑有线网卡为 `192.168.0.20`，工具监听 `0.0.0.0:5001`；
5. 先 QUERY 确认 `IDLE`，再选择目标夹爪和对应 BIN；
6. 等待 READY、块 ACK、RESULT 和 BOOT_OK；
7. BOOT_OK 必须是目标夹爪，且版本/Role 正确、`control_ready=true`、`unit_cfg=11`、`mode=0`、`motor_enabled=false`；
8. 先确认另一只夹爪不受影响，再恢复业务上位机，在安全空载条件下发送新的低速命令。

如果停在 ENTER_BOOT，检查 UART/从站和首次部署；停在 BEGIN/DATA/END，检查 G431 下游状态；RESTORE_CONTROL 失败而 RESULT 已成功时，镜像通常已经有效，先用 CANCEL 重试控制参数恢复，不要立即重刷。

## 9. ST-Link 救援

仓库已有夹爪 1 救援包目录 `OTA_PC_Tool/STLink_Recovery_Gripper1`：

- `gripper1_stlink_app_recovery.hex`：Application/Metadata 恢复，Bootloader 正常时使用；
- `gripper1_stlink_full_recovery.hex`：Bootloader 也损坏时使用；
- 两者均排除 `0x0801F800~0x0801FFFF` 校准页。

用 STM32CubeProgrammer 选择扇区擦除和 Verify，禁止 Full Chip Erase。救援包的版本、大小和 CRC 以同目录 `gripper1_recovery_info.txt` 为准。

如需为夹爪 2 或新版本生成救援包：

```powershell
cd .\OTA_PC_Tool
python .\gripper_stlink_recovery.py .\gripper2_app.bin --role 2 --version 1.15.0 --output-dir .\STLink_Recovery_Gripper2
```

生成后检查 info、HEX 地址范围，并连板 Verify。版本更新时必须重新生成，不能沿用旧 Metadata。

## 10. 验收记录模板

每只实物建议保留下列记录：

| 项目 | 记录内容 |
|---|---|
| 设备身份 | 序列号、夹爪编号、Role、硬件版本 |
| 校准备份 | 两份文件名、SHA256/比较结果、读取时间 |
| Bootloader | Git/归档版本、BIN CRC、Role 锁定状态 |
| Application | 版本、BIN 文件、大小、CRC32、构建时间 |
| Keil | 编译器、Device Pack、0 Error/已审核 Warning |
| 首次烧录 | Erase Sectors、Program/Verify、校准页对比 |
| 直连验证 | HELLO/QUERY/完整 OTA、Modbus 与运动结果 |
| 以太网验证 | relay_rev、目标 UART、RESULT/BOOT_OK、单位恢复 |
| 故障注入 | 断线、取消、重传、断电点和恢复结果 |

代码或版本更新后，大小和 CRC 会变化，不应把历史值当作固定协议参数；固定门禁是地址范围、Role、版本一致性和成功判据。
