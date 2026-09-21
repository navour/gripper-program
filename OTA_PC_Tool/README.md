# USB-RS485 直连夹爪 OTA 工具操作归档

本文档用于台架调试、远程链路故障恢复和单只夹爪维护。机器人整机上的日常升级优先使用[以太网工具](../../robot-program0827OTA/夹爪OTA以太网工具/README.md)；首次 ST-Link 迁移及 Keil 设置见[首次部署与烧录归档](../夹爪2_OTA部署与测试.md)。

## 1. 工具用途

本工具用于电脑通过USB-RS485直接更新单只STM32G431夹爪：

```text
电脑 -> USB-RS485 -> 夹爪Application/Bootloader
```

工具支持两种起始状态：

1. Application正在运行：工具先用Modbus失能电机并请求进入Bootloader，再传输固件；
2. Bootloader正在运行：工具直接通过Bootloader二进制协议传输固件。

工具一次只连接一只夹爪。普通业务上位机和 OTA 工具不能同时打开同一个串口。

## 2. 启动方法

在`OTA_PC_Tool`目录执行：

```powershell
python -m pip install -r requirements.txt
python gripper_ota_tool.py
```

USB-RS485适配器必须支持自动收发方向控制，串口格式固定为`115200, 8N1`。

## 3. 界面字段和选项

### 3.1 串口连接

| 界面项 | 允许值 | 含义和使用要求 |
|---|---|---|
| 端口 | 电脑识别到的COM口 | 选择USB-RS485对应端口，例如`COM6` |
| 刷新 | 按钮 | 重新扫描COM口；不向夹爪发送数据 |
| 波特率 | `115200` | 当前Application和Bootloader均固定为115200，不要改成其他值 |
| 夹爪编号 | 下拉选择`1`或`2` | 统一决定Application的Modbus从站ID和OTA Manifest中的Role：夹爪1两者均为1，夹爪2两者均为2 |
| 连接/断开 | 按钮 | 打开或关闭电脑串口；显示“已连接”只表示COM口打开，不表示夹爪已经回应 |

连接前必须关闭`3_gripper.py`、串口助手和其他占用同一COM口的软件。

### 3.2 Application固件

| 界面项 | 允许值 | 含义和使用要求 |
|---|---|---|
| 固件路径 | `*.bin` | 必须选择Keil中`JIAZHUA_RTOS` Target生成的纯Application BIN |
| 选择BIN | 按钮 | 选择固件文件；选择文件本身不会开始升级 |
| 版本Major | `0~65535` | 主版本号，例如`1.15.0`中的`1` |
| Minor | `0~65535` | 次版本号，例如`1.15.0`中的`15` |
| Patch | `0~65535` | 修订号，例如`1.15.0`中的`0` |

版本号会写入OTA Metadata，目前不会从BIN中自动推导。发布固件时必须确保界面版本与Application代码中的实际版本一致。

两只夹爪的Keil工程Rebuild后，都会同时更新各自工程输出目录和本目录的BIN：夹爪1为`gripper1_app.bin`，夹爪2为`gripper2_app.bin`。Build Output中应同时出现两条`fromelf`成功命令。当前两者版本均为1.15.0，文件不能混用，选择编号不会改变BIN内的从站地址。

选择“夹爪编号=1”时，工具用Modbus从站ID 1请求Application进入Bootloader，并给OTA清单填写Role 1；选择2时两处同时改为2，不再分别填写。

Bootloader日志中的`role=0`不表示上位机选择了0。它表示当前开发版Bootloader没有锁定左右夹爪角色，可以接受夹爪1或夹爪2固件。量产Bootloader可分别锁定为Role 1和Role 2。

### 3.3 自动操作选项

#### 未发现Bootloader时，自动通过Modbus让Application进入Bootloader

- 勾选：点击“开始OTA升级”后，工具先发送Bootloader `HELLO`；没有回应时，再向Application发送安全进入命令；
- 不勾选：工具只连接已经处于Bootloader的夹爪，Application正在运行时会直接报错；
- 该选项只控制“开始OTA升级”；单独点击“进入Bootloader”时始终会尝试通过Modbus自动进入。

Application安全进入命令依次为：

| Modbus寄存器 | 写入值 | 作用 |
|---:|---:|---|
| `0x0012` | `0` | 电机失能 |
| `0x00F0` | `0x4F54` | OTA Key1 |
| `0x00F1` | `0x4131` | OTA Key2 |
| `0x00F2` | `1` | 请求写BootMailbox并软件复位 |

Key1到OTA控制命令必须在1秒内完成。Application只有在电机已经失能且Key顺序正确时才接受请求。

#### 升级成功后自动重启Application

- 勾选：`END`校验和`QUERY`复核成功后，工具发送`REBOOT`，约0.5秒后运行新Application；
- 不勾选：固件已变为`VALID`，但夹爪继续停留在Bootloader，便于读取状态；需要关闭OTA工具后复位夹爪才能运行Application。

正常升级建议同时勾选这两个选项。

## 4. 操作按钮

### 4.1 进入Bootloader

该按钮只切换运行状态，不擦除Application，也不发送固件：

1. 先尝试发送`HELLO`发现Bootloader；
2. 若Application正在运行，则按3.3中的Modbus顺序失能电机并请求复位；
3. 等待Bootloader回应并校验Product、Flash布局和所选夹爪编号对应的Role。

操作成功后夹爪停留在Bootloader，普通`3_gripper.py`暂时无法通信，这是正常现象。

### 4.2 读取Bootloader状态

该按钮只发送`HELLO + QUERY`，不会擦除、写入或复位夹爪。它要求夹爪已经处于Bootloader；Application正在运行时不会自动切换。

日志字段含义：

| 字段 | 含义 |
|---|---|
| `state` | 当前Metadata/下载状态 |
| `image_size` | 已提交Application的实际字节数 |
| `image_crc32` | 已提交Application的整包CRC32 |
| `metadata_seq` | Metadata追加记录序号 |
| `next_offset` | 当前下载会话下一段数据的偏移量 |

`state`取值：

| 状态 | 含义 | 后续动作 |
|---|---|---|
| `EMPTY` | 没有有效OTA记录 | 可以开始升级 |
| `DOWNLOADING` | 已经BEGIN但没有完整提交 | 重新执行完整OTA |
| `VALID` | Application大小和CRC记录有效 | 可以复位启动Application，也可以再次升级 |
| `INVALID` | 升级取消、失败或镜像校验不通过 | 重新执行完整OTA |

### 4.3 开始OTA升级

这是唯一会执行完整Application擦写的按钮。实际流程为：

```text
本地检查BIN
  -> HELLO/必要时让Application进入Bootloader
  -> BEGIN并写DOWNLOADING Metadata
  -> 擦除Application分区
  -> DATA逐块写入并读回
  -> END校验向量表和整包CRC32
  -> 写VALID Metadata
  -> QUERY复核
  -> 可选REBOOT
```

发送前工具检查：

- BIN不超过`0x19000`，即100 KiB；
- 初始MSP位于允许的SRAM范围；
- Reset Handler带Thumb位且位于`0x08006000`开始的Application范围；
- BIN包含夹爪OTA身份签名；
- Product、布局、所选夹爪编号对应的Role与Bootloader兼容；
- 本地计算整包CRC32。

旧的、链接在`0x08000000`的Application BIN会被拒绝，不能绕过检查发送。

### 4.4 取消

“取消”只在操作进行中可用。升级传输期间点击后，工具会等待当前数据块结束，再发送`ABORT`。

取消不是暂停，也不支持下次从原进度续传。只要`BEGIN`已经成功，旧Application就可能已经被擦除；取消后夹爪会停留在Bootloader，必须重新点击“开始OTA升级”并从头完整下载。

## 5. 推荐的正常升级流程

以下流程适用于已经完成首次部署、Application 当前可以正常运行的任一夹爪：

1. 关闭普通`3_gripper.py`和其他串口软件；
2. 连接电脑、USB-RS485和单只夹爪；
3. 启动本工具，点击“刷新”，选择正确COM口；
4. 保持波特率`115200`，按实物选择夹爪编号`1`或`2`；
5. 点击“连接”，确认日志出现串口已连接；
6. 点击“选择BIN”，夹爪1选择`gripper1_app.bin`，夹爪2选择`gripper2_app.bin`；
7. Major/Minor/Patch填写与固件一致的版本，例如`1/15/0`；
8. 勾选“自动通过Modbus进入Bootloader”和“升级成功后自动重启Application”；
9. 点击“开始OTA升级”，不要在进度完成前断电、拔线或启动其他串口工具；
10. 等待日志出现`Metadata=VALID`和`REBOOT已确认`；
11. 等待约0.5秒，点击“断开”并关闭OTA工具；
12. 启动普通`3_gripper.py`，验证版本、开合、压力、编码器和校准数据。

工具会根据夹爪编号同时选择对应的Modbus从站和Manifest Role。尚未迁移到OTA布局的旧实物必须先完成ST-Link首次部署；校准备份、Keil设置、首次双烧和验证详见 [首次部署、Keil烧录与验收](../夹爪2_OTA部署与测试.md)。不能把夹爪1 BIN改名后用于夹爪2，当前工具不从BIN自动识别左右角色。

直连OTA重启后，普通`3_gripper.py`发送位置前仍需配置`0x0014=0x000B`；本直连工具不执行机器人主控的单位恢复流程。夹爪2目录的普通工具默认从站已设为2。

## 6. 夹爪已经处于Bootloader时的流程

如果普通Modbus已经无法连接，但Bootloader仍能回应：

1. 关闭其他串口软件并连接OTA工具；
2. 可先点击“读取Bootloader状态”；
3. 选择夹爪编号、正确BIN和版本；
4. 点击“开始OTA升级”；
5. 工具发现Bootloader后会直接从`BEGIN`重新下载，不需要Application参与。

即使状态为`DOWNLOADING`或`INVALID`，也可以重新执行完整升级。

## 7. 日志判断和故障处理

### 7.1 升级成功

至少应看到：

```text
固件检查通过
Bootloader已响应（或已直接发现Bootloader）
BEGIN：擦除Application区
END：校验向量表和整包CRC32
升级成功：Metadata=VALID
REBOOT已确认
```

### 7.2 开始时出现HELLO超时

如果随后出现“未发现Bootloader，向Application从站发送安全进入命令”，这是正常探测过程：工具先检查夹爪是否已经在Bootloader，Application运行时不会回应Bootloader `HELLO`。

如果安全进入命令也失败，检查：

- 夹爪编号是否正确；
- 是否有其他软件占用串口；
- USB-RS485的A/B线、地线和方向控制；
- Application是否仍能通过普通Modbus通信；
- 电机是否能够被写入失能状态。

### 7.3 DATA或END失败

不要反复复位期待旧程序恢复。单分区升级开始后旧Application可能已经被擦除，应保持供电和RS485连接，重新点击“开始OTA升级”从BEGIN完整传输。

### 7.4 升级成功但普通工具连接不上

依次检查：

1. 日志是否确实出现`Metadata=VALID`；
2. 是否勾选自动重启，或是否已经手动复位；
3. OTA工具是否已经断开串口；
4. 普通工具从站ID是否与夹爪一致；
5. 是否至少等待了Bootloader的约0.5秒启动窗口。

### 7.5 Keil烧Application后两个工具都无响应，重新上电又恢复

这不是OTA工具的协议错误。若Bootloader的`HELLO`和Application的Modbus同时没有任何字节返回，说明Keil下载结束后CPU没有被正确复位并释放；CPU处于HALT时两套固件都不可能回应。

夹爪1 Application工程已经把ST-Link设置统一为Bootloader和机器人工程使用的配置。使用前先完全关闭Keil并重新打开`JIAZHUA_RTOS.uvprojx`，然后检查：

1. `Options for Target -> Debug -> Settings -> Debug`：Port=`SW`，SW Device=`ARM CoreSight SW-DP`；
2. Connect=`Normal`，Reset=`SYSRESETREQ`；
3. 勾选`Reset after Connect`，不勾选`Stop after Reset`；
4. `Flash Download`页选择`Erase Sectors`并勾选`Program / Verify / Reset and Run`；
5. 使用`Flash -> Download`，完成后不要进入Debug会话，也不需要重新上电。

如果Keil窗口在工程文件修改期间一直开着，必须先关闭且不要保存旧设置，再重新打开；否则Keil可能把内存中的旧ST-Link参数覆盖回`.uvoptx`。

## 8. 安全边界

- `BEGIN`成功后才擦除Application；
- `END`完成整包CRC和向量校验后才提交`VALID`；
- 失败或取消时Bootloader继续保留，可以重新下载；
- OTA不会擦除`0x08000000 ~ 0x08005FFF`的Bootloader；
- OTA不会擦除`0x0801F800 ~ 0x0801FFFF`的校准页；
- 单分区方案保证升级失败后仍能重新下载，不保证旧Application继续运行。

## 9. 相关文档

- [夹爪 OTA 归档总览](../../夹爪OTA技术归档.md)
- [G431 单分区 OTA 技术归档](../夹爪单分区OTA技术方案.md)
- [首次部署、Keil 烧录与验收](../夹爪2_OTA部署与测试.md)
- [H723 流式转发技术归档](../../robot-program0827OTA/机器人主控_夹爪OTA流式转发技术方案.md)
