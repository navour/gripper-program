# 夹爪 G431 单分区 OTA 技术归档

## 1. 文档定位

本文记录 STM32G431 夹爪设备端 OTA 的最终实现，包括资源布局、启动链路、协议、Flash 提交规则和恢复边界。首次烧录和 Keil 操作见 [夹爪 OTA 首次部署、Keil 烧录与验收](夹爪2_OTA部署与测试.md)，上位机操作见 [USB-RS485 直连工具](OTA_PC_Tool/README.md) 和 [机器人以太网工具](../robot-program0827OTA/夹爪OTA以太网工具/README.md)。

归档基线：STM32G431CBT6，Bootloader 协议 v1，Flash 布局 v1，夹爪 1/2 Application 版本 1.15.0。

## 2. 设计目标与取舍

G431 具有 128 KiB Flash。现有夹爪 Application 加上 Bootloader、Metadata 和独立校准页后，没有足够空间保存两个完整应用镜像，因此采用单 Application 分区：

```text
固定 Bootloader + 单个 Application + OTA Metadata + Calibration
```

此方案保证的是“升级失败后仍能通过 Bootloader 重新下载，且校准页不被 OTA 擦除”，不保证“失败后旧 Application 仍可运行”。BEGIN 成功后旧 Application 可能立即失效，所以升级期间必须保持执行机构失能，失败时进入恢复状态。

主要安全原则如下：

- Bootloader、Application、Metadata、Calibration 采用页对齐硬边界；
- 擦除前校验产品、布局、地址、大小、角色和 Bootloader 最低版本；
- 先持久化 `DOWNLOADING`，再擦除 Application；
- 每段 Flash 写入并读回成功后才回复提交偏移；
- END 完成向量表和整包 CRC32 校验后才写 `VALID`；
- 只有 `VALID` 镜像或受控的首次 ST-Link 镜像可以启动；
- Bootloader 和校准页不在 OTA 擦写范围内。

## 3. Flash 与 SRAM 布局

### 3.1 Flash 布局

| 区域 | 起始地址 | 大小 | 页范围 | 用途 |
|---|---:|---:|---:|---|
| Bootloader 代码 | `0x08000000` | `0x5800`（22 KiB） | 0~10 | 启动、协议、擦写、校验和跳转 |
| 首次部署许可页 | `0x08005800` | `0x0800`（2 KiB） | 11 | ST-Link 首次部署兼容标记 |
| Application | `0x08006000` | `0x19000`（100 KiB） | 12~61 | 夹爪业务固件 |
| OTA Metadata | `0x0801F000` | `0x0800`（2 KiB） | 62 | 下载状态与镜像清单 |
| Calibration | `0x0801F800` | `0x0800`（2 KiB） | 63 | 零点、比例、偏置等校准数据 |

页大小为 `0x800`，Flash 最小编程粒度为 8 字节。`Application` 的结束地址正好等于 Metadata 起始地址；所有边界由 `gripper_ota_layout.h` 的编译期检查约束。

首次部署许可页用于解决“通过 ST-Link 直接烧入重定位 Application 时没有匹配的 VALID Metadata”的启动问题。Application 内嵌完整身份签名；每次明确重刷 Bootloader 都会重新置位许可，Bootloader 在许可有效、向量合法且身份签名完整时允许受控启动，即使页中仍有旧 Metadata。下一次合法 BEGIN 在写 DOWNLOADING 和擦 Application 之前先消费该许可，使中断的 OTA 不能回退到只检查身份/向量的启动路径。量产 WRP 策略必须把此页的生命周期考虑在内。

### 3.2 SRAM 布局

| 区域 | 地址/大小 | 说明 |
|---|---|---|
| Application/Bootloader 可链接 SRAM | `0x20000000` / `0x7FE0` | 末地址不超过 `0x20007FE0` |
| BootMailbox | `0x20007FE0` / `0x20` | Application 向 Bootloader 传递会话和进入原因 |

BootMailbox 为 32 字节：magic、session ID、reason、magic 反码、前 16 字节 CRC32 和 3 个保留字。Application 先清 magic，再写完整内容，最后写 magic 并执行系统复位；Bootloader 校验并消费后清除，避免普通复位被误认为升级请求。

## 4. 软件组成与职责

### 4.1 Bootloader

代码位于 `Bootloader_G431/Bootloader_G431/Device/Boot`：

| 模块 | 职责 |
|---|---|
| `gripper_ota_layout.h` | Flash/SRAM 边界、产品号、布局/协议/Boot 版本和角色宏 |
| `boot_main.c` | 启动判定与 HELLO/BEGIN/DATA/END/QUERY/ABORT/REBOOT 状态机 |
| `boot_protocol.c` | USART2 半双工帧收发、SOF 同步、CRC16、PA1 方向控制 |
| `boot_flash.c` | Application 擦除、8 字节编程、末段补 `0xFF` 和逐段读回 |
| `boot_metadata.c` | 64 字节追加记录、记录 CRC 和最后提交标记 |
| `boot_image_verify.c` | MSP、Reset Handler、身份签名和整包 CRC32 检查 |
| `boot_jump.c` | Mailbox 处理、中断/外设清理、VTOR/MSP 设置和跳转 |

Bootloader 使用 USART2：PA2 TX、PA3 RX、PA1 RS485 DE，115200、8N1。PA1 发送前拉高，等待 UART TC 后拉低。

### 4.2 Application

夹爪 1/2 工程均包含 `User/OTA/gripper_ota_app.c/h` 和相同的布局头。Application 负责：

- 在固件中保留产品、布局、应用基址的身份签名；
- 通过 Modbus 提供版本、Bootloader 版本、能力、READY、布局和 Role；
- 校验安全进入序列；
- 生成非零 session ID，写 BootMailbox，并在 Modbus 成功回显发送完成后复位；
- 把向量表重定位到 `0x08006000`。

两套工程不可互换：

| 工程 | Role | Modbus 从站 | 发布文件 |
|---|---:|---:|---|
| `5_夹爪总程序id1slave1` | 1 | 1 | `OTA_PC_Tool/gripper1_app.bin` |
| `5_夹爪总程序id2slave2` | 2 | 2 | `OTA_PC_Tool/gripper2_app.bin` |

## 5. 启动判定

复位后 Bootloader 的逻辑顺序为：

1. 初始化最小硬件，读取并校验 BootMailbox；
2. 扫描 Metadata，获取最后一条已提交且记录 CRC 正确的状态；
3. Mailbox 有效、Metadata 为 `DOWNLOADING/INVALID`、镜像无效或收到 Boot 协议时，留在 Bootloader；
4. 对可启动镜像开放约 500 ms 握手窗口；窗口内收到协议帧则继续留在 Bootloader；
5. Metadata 为 `VALID` 且向量/CRC 合法时跳转 Application；
6. 没有 OTA 记录时，仅允许满足首次部署许可与身份签名的 Application 启动；
7. 任何校验失败都不跳转，保持 RS485 恢复能力。

跳转前关闭中断、清理 SysTick/NVIC、设置 `SCB->VTOR=0x08006000`、装载 Application MSP，最后跳转到 Reset Handler。Application 自身也通过 `APP_VECT_TAB_OFFSET=0x00006000U` 设置向量偏移。

## 6. Application 进入 Bootloader

### 6.1 Modbus 寄存器

| 寄存器 | 访问 | 值/含义 |
|---:|---|---|
| `0x00F0` | 写 | Key1=`0x4F54` |
| `0x00F1` | 写 | Key2=`0x4131` |
| `0x00F2` | 写 | `1` 请求进入 Bootloader |
| `0x00F3` | 读 | Bootloader 版本，当前 `0x0100` |
| `0x00F4` | 读 | OTA 能力位，当前 mailbox/boot protocol/single slot |
| `0x00F5` | 读 | Application Major/Minor |
| `0x00F6` | 读 | Application Patch |
| `0x00F7` | 读 | Application READY，正常启动后为 1 |
| `0x00F8` | 读 | Flash Layout Version，当前 1 |
| `0x00F9` | 读 | Application Role，夹爪 1 为 1、夹爪 2 为 2 |

安全进入顺序必须是：

```text
0x0012 = 0           # 电机失能
0x00F0 = 0x4F54      # Key1
0x00F1 = 0x4131      # Key2
0x00F2 = 1           # 请求复位到 Bootloader
```

Key1 到 CTRL 必须在 1 秒内完成，并使用三次独立的 Modbus 功能码 `0x06` 写入。功能码 `0x10` 批量覆盖该范围会被拒绝。电机未失能、Key 顺序错误、超时或重复请求均不进入 Bootloader。

## 7. RS485 Bootloader 协议

### 7.1 帧格式

所有多字节字段使用小端，帧尾使用 Modbus CRC16，最大 payload 为 192 字节。

| 偏移 | 长度 | 字段 |
|---:|---:|---|
| 0 | 2 | SOF=`0x47 0x42`（`GB`） |
| 2 | 1 | Protocol Version=1 |
| 3 | 1 | Command；回复为请求命令 OR `0x80` |
| 4 | 4 | Session ID，BEGIN 时必须非 0 |
| 8 | 2 | Sequence |
| 10 | 2 | 请求 flags / 回复 status |
| 12 | 4 | 请求 offset / 回复 next offset |
| 16 | 2 | Payload Length，0~192 |
| 18 | N | Payload |
| 18+N | 2 | CRC16/Modbus，覆盖此前全部字节 |

### 7.2 命令

| 命令 | 值 | 作用 | 是否写 Flash |
|---|---:|---|---|
| HELLO | `0x01` | 读取产品、布局、分区、版本、Role、能力和会话 | 否 |
| BEGIN | `0x02` | 校验 136 字节 Manifest，写 DOWNLOADING 并擦 Application | 是 |
| DATA | `0x03` | 按 offset 写入并读回，回复 next offset | 是 |
| END | `0x04` | 校验向量表/整包 CRC，提交 VALID | 是 |
| QUERY | `0x05` | 查询状态、会话、进度、镜像和 Metadata 序号 | 否 |
| ABORT | `0x06` | 终止当前下载并记录 INVALID | 是 |
| REBOOT | `0x07` | 回复后延时复位 | 否 |

DATA 采用 stop-and-wait：正常帧的 offset 必须等于 `next_offset`，sequence 必须连续；非最后段的 offset 和长度需按 8 字节对齐。ACK 丢失时可以原样重发上一段：若 Flash 内容与重复 payload 一致，Bootloader 重复确认同一 next offset；内容不同则返回读回错误。不得在未确认时盲目发送下一 offset。

### 7.3 主要状态码

`0x0000` 表示成功。维护中重点关注：`BAD_FRAME_CRC`、`BAD_SESSION`、`BAD_SEQUENCE`、`BAD_OFFSET`、`BAD_LENGTH`、`WRONG_PRODUCT`、`WRONG_ROLE`、`IMAGE_TOO_LARGE`、`ERASE_FAILED`、`PROGRAM_FAILED`、`READBACK_FAILED`、`IMAGE_CRC_FAILED`、`INVALID_VECTOR`、`APP_NOT_VALID`、`BUSY` 和 `UNSUPPORTED`。完整枚举以 `boot_protocol.h` 为准。

## 8. Manifest 与完整性

BEGIN payload 固定为 136 字节：

| 偏移 | 长度 | 字段 | 当前约束 |
|---:|---:|---|---|
| 0 | 4 | Magic | `0x474D4947`（GIMG） |
| 4 | 2 | Format | 1 |
| 6 | 2 | Header Size | 136 |
| 8 | 4 | Product ID | `0x0000D431` |
| 12 | 1 | Role | 0、1 或 2 |
| 13 | 1 | Hardware Revision | 记录，当前不强制匹配 |
| 14 | 2 | Flags | 当前必须为 0 |
| 16/18/20 | 各 2 | Major/Minor/Patch | 写入 Metadata |
| 22 | 2 | Min Boot Version | 当前 Boot 为 `0x0100` |
| 24 | 4 | Layout Version | 1 |
| 28 | 4 | Image Base | `0x08006000` |
| 32 | 4 | Image Size | `8~0x19000` |
| 36 | 4 | Image CRC32 | CRC-32/ISO-HDLC |
| 40 | 32 | SHA-256 | 当前填 0，未验证 |
| 72 | 64 | Signature | 当前填 0，未验证 |

当前是 CRC 完整性版本，不是带身份认证的安全启动版本。只要 flags 声明未支持能力，Bootloader 返回 `UNSUPPORTED`，不会静默忽略。量产前需要实现签名、公钥保护和密钥生命周期管理。

## 9. Metadata 与提交语义

Metadata 位于独立 2 KiB 页，使用 64 字节追加记录。记录包含 state、role、sequence、session、product、layout、image size/CRC、版本、flags、next offset、记录 CRC 和 commit marker。

状态含义：

| 状态 | 含义 | 启动/恢复行为 |
|---|---|---|
| EMPTY | 无已识别记录 | 仅可能走首次部署兼容启动 |
| DOWNLOADING | BEGIN 已提交，下载未完成 | 禁止启动，重新 BEGIN 完整下载 |
| VALID | 镜像、向量、CRC 均通过 | 允许启动 |
| INVALID | 取消、写入或校验失败 | 禁止启动，重新 BEGIN 完整下载 |

写记录时先写前 56 字节主体，最后写包含记录 CRC 和 commit marker 的 8 字节 doubleword。掉电留下的未提交尾记录不会被当成有效状态。Metadata 页满时由实现擦除 Metadata 页并从首槽继续写入；Calibration 页不参与回收。

## 10. Flash 擦写边界

- BEGIN 先成功追加 DOWNLOADING，再擦除 Application 的 50 页；
- 地址检查限定在 `0x08006000~0x0801EFFF`，不接受跨区写入；
- 每次编程 8 字节，最后不足 8 字节使用 `0xFF` 补齐，但整包 CRC 只覆盖实际镜像长度；
- 每段写后读回比较；只有读回一致才回复新 next offset；
- END 重新从 Flash 计算整个 BIN 的 CRC32，并验证初始 MSP 与 Reset Handler；
- Bootloader 区、Metadata 之外的校准页永远不在 Application 擦除函数范围内。

## 11. 失败与恢复

| 失败时点 | 设备状态 | 正确处理 |
|---|---|---|
| BEGIN 前失败 | 旧 Application 通常仍有效 | 取消维护态或重新发起 |
| BEGIN 后中断 | DOWNLOADING/恢复模式 | 从 BEGIN 和 offset 0 重发完整 BIN |
| DATA CRC/序号/偏移错误 | next offset 不推进 | 修正或原样重发当前帧 |
| ACK 丢失 | Flash 可能已写入 | QUERY 或原样重发，不发送下一段 |
| END 校验失败 | INVALID/恢复模式 | 重新完整下载 |
| VALID 后未启动 | 镜像已提交，启动或业务确认失败 | 直连 QUERY，检查版本/向量/READY，必要时重刷 |
| RS485 不通 | Bootloader 仍保留但不可远程访问 | 修复线路，最终使用 SWD 救援 |

不支持掉电后的断点续传。QUERY 的 next offset 只用于诊断当前通电会话；复位或断电后必须重新 BEGIN。

## 12. Keil 构建门禁

### 12.1 Bootloader

- Device：`STM32G431CBTx`；
- IROM1：Start `0x08000000`，Size `0x6000`；
- IRAM1：Start `0x20000000`，Size `0x7FE0`；
- Scatter：`MDK-ARM/Bootloader_G431.sct`；
- 代码执行区不得超过 `0x5800`，许可页固定在 `0x08005800`；
- 开发版未定义 `GRIPPER_DEVICE_ROLE` 时为 Role 0；角色锁定版本定义为 1 或 2；
- Rebuild 后检查 AXF/HEX/BIN/MAP，BIN 不超过 `0x6000`。

### 12.2 Application

- IROM1：Start `0x08006000`，Size `0x19000`；
- IRAM1：Start `0x20000000`，Size `0x7FE0`；
- Define 包含 `APP_VECT_TAB_OFFSET=0x00006000U`；
- Scatter 的 `LR_IROM1/ER_IROM1` 从 `0x08006000` 开始；
- Rebuild 后 `fromelf` 同时更新工程输出目录和 `OTA_PC_Tool/gripperN_app.bin`；
- MAP 中 RESET/向量表位于 `0x08006000`，BIN 不超过 `0x19000`。

界面中显示的通用 CPU IROM 描述可能仍是芯片总容量，最终以 Target 页内存区、Scatter、MAP 和 BIN 四者为准。

## 13. 量产增强项

当前公共 Bootloader 默认 Role 0，且只做 CRC32 完整性检查。量产前应完成：

- Bootloader 分角色构建或写入受保护的出厂角色；
- 数字签名验证和公钥保护；
- Bootloader 页 WRP、RDP 和 Option Bytes 恢复流程；
- 低压、随机断电、噪声、Metadata 回收与至少 100 次循环升级；
- 校准备份/恢复工具和生产烧录记录闭环。

保护策略必须先在可报废样机上验证。尤其不能在首次部署许可仍需消费时盲目保护最后一个 Bootloader 页。
