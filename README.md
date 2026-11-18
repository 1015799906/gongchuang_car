# GCS 物流搬运机器人 — 智能物流搬运赛项

面向 **2026 年广东省大学生工创大赛 · 智能物流搬运赛项**的双端机器人系统：**STM32F407 下位机固件** + **RDK X5 上位机视觉**。

整车由两块计算单元协同工作：

- **`firmware/` STM32F407 下位机**：麦克纳姆轮底盘运动学与四轮速度环 PID 闭环、CAN 总线驱动的 ZDT 闭环步进电机（升降 / 平推）、三路舵机云台与物块盘、电加热控制，并通过串口与上位机交互、编排整套比赛流程。
- **`host/` RDK X5 上位机**：OpenCV 视觉识别（二维码 / 物块 / 色环 / 码垛），双摄像头分工，按命令码触发对应视觉任务，把 9 字节 ASCII 结果回传下位机。

> 完整文档（协议、调车流程、PID 整定、故障分析等 19 篇）见 [`docs/`](docs/)。

---

## 1. 项目结构

```text
gongchuang_car/
├── firmware/                     # STM32F407 下位机固件（CMake + STM32Cube HAL）
│   ├── Core/                     #   应用层源码
│   │   ├── Inc/                  #     race_chassis.h / race_task.h / host_proto.h / hal_zdt.h / imu.h ...
│   │   └── Src/                  #     main.c / race_chassis.c / race_task.c / host_proto.c / hal_zdt.c ...
│   ├── Drivers/                  #   STM32F4xx HAL + CMSIS
│   ├── cmake/                    #   CubeMX 生成的构建脚本与工具链文件
│   ├── CMakeLists.txt            #   工程入口（CubeMX 生成，可手改）
│   ├── CMakePresets.json
│   ├── STM32f407_wuliu.ioc       #   CubeMX 工程文件（引脚/时钟/外设配置）
│   ├── startup_stm32f407xx.s
│   ├── STM32F407XX_FLASH.ld
│   ├── cfg.sh / build.sh / flash.sh   # 配置 / 编译 / 烧录
│   └── host_probe.ps1, host_task_probe.py  # 上位机协议联调脚本
├── host/                         # RDK X5 上位机视觉（Python + OpenCV）
│   ├── host_main.py              #   主程序：命令分发 + 各视觉任务
│   ├── 原视觉参考（别删）.py      #   早期参考实现，保留对照
│   ├── HOST_SERVICE.md           #   上位机服务部署与调试速查
│   ├── gcs-host                  #   /etc/init.d 启动脚本
│   └── run_*.bat / run_*.ps1     #   Windows 本机预览启动脚本
├── docs/                         # 项目文档
│   ├── 命题/                     #   赛题与评分标准 PDF
│   └── *.md                      #   19 篇中文文档（见第 8 节索引）
├── .gitignore
└── README.md
```

> 本仓库只收录**比赛主固件**（`wuliu` 版本）。开发过程中的 `wuliu_debug`、`motor_test`、`motor_test_backup` 分支工程未纳入版本库。

---

## 2. 系统架构

```text
        ┌─────────────────────────────────────────────────────────┐
        │        RDK X5 上位机 (Linux, Python 3.10 + OpenCV)      │
        │  host_main.py                                           │
        │   - 命令分发：1/2/a/3/…/d 触发不同视觉任务              │
        │   - 二维码摄像头 /dev/video_xia0                        │
        │   - 上方摄像头   /dev/video_shang0                      │
        │   - 回传 0xFF + 9 字节 ASCII + 0xFE                     │
        └───────────────────────┬─────────────────────────────────┘
                                │ USART1  PA9/PA10  115200 8N1
                                │ /dev/ttyUSB0
        ┌───────────────────────┴─────────────────────────────────┐
        │              STM32F407 下位机（168MHz 级 Cortex-M4）      │
        │   - host_proto.c  上位机帧协议（0xFF…0xFE，9 字节载荷） │
        │   - race_task.c   比赛流程编排（四圈 11 步）            │
        │   - race_chassis.c 麦轮运动学 / 速度环 / 航向环         │
        │   - hal_zdt.c     ZDT 闭环步进 CAN 驱动                 │
        │   - imu.c         JY901B 姿态（WIT SDK）                │
        │   - race_io.c     开关 / LED / 加热等 IO                │
        └───┬──────────────┬───────────────┬──────────────────────┘
            │ CAN1         │ TIM2 PWM      │ UART5
            │ 500 kbps     │ 50 Hz ×3      │ 115200
            ▼              ▼               ▼
   4 × ZDT 闭环步进   3 × 舵机云台     调试终端
   (轮 / 升降 / 平推) (圆台 / 物块盘 / 爪)
```

- **上位机 ↔ 下位机**：USART1，115200 8N1。下位机发**单字节命令**触发拍照，上位机回 `0xFF + 9 字节 ASCII + 0xFE`。
- **下位机 ↔ 执行器**：CAN1 @ 500 kbps 驱动 ZDT 闭环步进；TIM2 输出 50 Hz PWM 驱动 3 路 DS 系列舵机。
- **调试口**：UART5（PC12/PD2），115200，单字符命令（`SJ` / `PUSH` / `ACT` / `YT` / `PID` 等）。

---

## 3. 硬件接线

配置源：`firmware/STM32f407_wuliu.ioc`、`Core/Src/usart.c`、`can.c`、`tim.c`、`gpio.c`。

| 外设 | 接口 | 引脚 | 参数 | 用途 |
| --- | --- | --- | --- | --- |
| 上位机通信 | USART1 | PA9 / PA10 | 115200 8N1 | 9 字节帧协议（`HOST_UART`） |
| 调试终端 | UART5 | PC12 / PD2 | 115200 8N1 | 单字符命令 + 可读日志（`DBG_UART`） |
| IMU 通信 | USART2 | PD5 / PD6 | 115200 8N1 | WitMotion 姿态传感器（WIT SDK） |
| 串口屏 | USART6 | PC6 / PC7 | 9600 8N1 | 串口显示屏 |
| 电机总线 | CAN1 | PA11(RX) / PA12(TX) | 500 kbps（Prescaler 7, BS1 9TQ, BS2 2TQ） | ZDT 闭环步进 ×6 |
| 舵机 PWM | TIM2 CH2/CH3/CH4 | PA1 / PA2 / PA3 | 50 Hz（Prescaler 83, Period 19999） | CH2 夹爪 / CH3 置物盘 / CH4 云台 |
| 加热 PWM | TIM3 CH1 | PA6 | 500 Hz（Prescaler 83, Period 1999） | 电加热片 `race_heat_set()` |
| 控制周期中断 | TIM4 | 内部 | Prescaler 839, Period 4999（≈20 Hz） | PID 节拍 `race_chassis_tim4_callback()` |
| 辅助定时 | TIM5 | 内部 | Prescaler 839, Period 49999 | 长周期计时 |
| 状态输出 | GPIO | PA8 | 推挽输出 | LED / 状态指示 |
| 输入检测 | GPIO | PC9 / PE4 | 输入 | 启动开关等 |
| 调试/烧录 | SWD | PA13 / PA14 | — | CMSIS-DAP |

---

## 4. 通信协议

### 4.1 上位机 ↔ 下位机（USART1）

帧格式（上位机 → 下位机）：

```text
0xFF | 9 字节 ASCII 载荷 | 0xFE      （共 11 字节，载荷不足 9 字节时补 0x00）
```

- 握手包：`987654321`
- 识别失败包：`000000004`
- 坐标结果：`{x:04}{y:04}{color}`（示例 `095505191`）
- 下位机 → 上位机：**单字节文本命令，不补换行**（上位机用 `pyserial.readline()` + `timeout=0.1s` 取回，这是已验证行为，不要擅自加 `\n`）。

下位机发出的任务命令（触发上位机拍照识别，摘自 `race_task.c` 与 `docs/视觉通信与比赛流程分析.md`）：

| 命令 | 触发任务 | 回传内容 |
| --- | --- | --- |
| `0x31` | 读取物块颜色序列 | 6 个物块颜色码（1/2/3） |
| `0x32` | 物块坐标定位 | `xxxxxyyyyc` 坐标 + 颜色 |
| `0x33` | 圆台颜色码 | 圆台颜色序列 |
| `0x35` / `0x36` | 圆台二/三层精定位 | 9 字节坐标包 |
| `0x37` / `0x38` | 色环一次 / 二次定标 | 环心坐标 + 颜色 |

> 完整命令字、字段定义与 ACK 机制见 [`docs/视觉通信与比赛流程分析.md`](docs/视觉通信与比赛流程分析.md) 与 [`docs/串口命令参考手册.md`](docs/串口命令参考手册.md)。

### 4.2 下位机 ↔ 步进电机（CAN1）

- 物理层：CAN1，500 kbps，**扩展帧**；仲裁域 `ExtId = (motor_addr << 8) | pack`，`data[0]` 为命令码。
- 应用层：ZDT（张大头）闭环步进协议，封装在 `Core/Src/hal_zdt.c`：使能 `0xF3`、速度模式 `0xF6`、位置模式 `0xFD`、急停 `0xFE`、回零等；回读用 `0x27/0x35/0x36/0x3B`（转速 / 相位电流 / 位置状态）。
- 地址分配：`1–4` 四个麦克纳姆轮，`5` 升降，`6` 平推。
- 收到 CAN 帧后在中断里入环形缓冲（`race_chassis_can_rx_callback`，深度 16），主循环弹出处理，避免 ISR 内做重活。

---

## 5. 主要功能模块

### 5.1 底盘：麦轮运动学 + 三层闭环

- `car_move(v_x, v_y, w)`：底盘速度基元，按麦轮运动学解算四轮转速。
- `car_move_distance_x/y()`、`car_move_distance()`、`car_move1~4()`：带标定系数的定距移动（前进 / 侧移 / 斜向）。
- 四轮**速度环**（闭环在 MCU 内实现，非驱动器内）：`zdt_read(addr, ZDT_S_VEL)` 回读轮速 → 每轮独立 PI 修正 → `wheel_speed_pid_command()` → `zdt_vel()` 下发。默认增益 `Kp 0.35 / Ki 0.10 / Kd 0.00`，节拍 50 ms，积分限幅 300、单次修正限幅 80；反馈超过 200 ms 未更新时**自动退化为开环**，避免失速锁死。
- **航向环**：`PID_move(v_x, v_y, w, time, target_angle, pid_choose)` 以 IMU 角度为反馈闭环锁定朝向，内置 5 组预置参数（见 7 节）。
- 里程/距离系数在 `car_move_distance_x`（10.5）、`car_move_distance_y`（10.3）中，需按实测校准。

### 5.2 升降与平推

- `shengjiang_control()` / `pingtui_control()`：位置模式 + 加速度控制，带当前位置查询与软件零点（`*_current_pos` / `*_set_current_pos`）。
- `race_shengjiang_homezero()`：无传感器回零（`zdt_home_cfg_sensorless`），靠机械限位与堵转电流判定零点。
- 偏差问题的现象、根因与解法见 [`docs/升降电机偏差问题分析.md`](docs/升降电机偏差问题分析.md)。

### 5.3 三轴舵机云台与物块盘

| 执行器 | 关键常量 | 值 |
| --- | --- | --- |
| 圆台舵机 | `YUANTAI_ANGLE_OFFSET` / `YUANTAI_DEFAULT_DEG` | 7.9° / 0.0° |
| 物块盘三工位 | `WUKUAI_POS1/2/3_DEG` | 0° / 135° / 270° |
| 爪子 开 / 合 | `ZHUASHOU_OPEN_DEG` / `ZHUASHOU_CLOSE_DEG` | 50° / 25° |
| 舵机 PWM 步进 | `YUANTAI_PWM_STEP` / `WUKUAI_PWM_STEP` | 10 @20ms / 20 @10ms |

- `set_yuantai_Angle()`、`set_wukuaipingtai_Angle()`、`set_zhuashou_Angle()` 提供**缓慢渐入**的角度设定，避免舵机瞬间冲击。
- `move_all()` / `move_all1()`：升降 + 平推 + 云台三轴协同的复合动作。

### 5.4 比赛流程编排（`race_task_run`）

`Core/Src/race_task.c` 把整场比赛拆成四个阶段，配合上位机视觉逐段推进：

```text
① 读任务顺序    race_stage_read_task_order        （取物块颜色序列）
② 执行器准备    race_stage_prepare_actuators      （平推归位 / 升降归零 / 加热开）
③ 第一平台取料  race_stage_pick_first_platform_blocks
④ 第一圈色环    race_stage_first_ring_task012
⑤ 第二平台取料  race_stage_pick_second_platform_blocks
⑥ 第二圈色环    race_stage_second_ring_task012
⑦ 第三圈色环    race_stage_third_ring_task345
⑧ 第四圈色环    race_stage_fourth_ring_task345
⑨ 收尾停车      race_stage_finish_drive + race_finish_stop
```

- 任务号 0–5 对应 6 个物块的目标色环；`transfer_position_wukuai()` / `transfer_position_sehuan()` 负责把视觉坐标转成底盘位移。
- 上位机返回颜色码 4（未识别）时，下位机会**重发命令重试**，最多循环若干次。
- 上位机断链保护：`race_host_timeout_stop()` 超时自动停车。

### 5.5 上位机视觉（`host/host_main.py`）

| 能力 | 函数 | 说明 |
| --- | --- | --- |
| 二维码识别 | `run_erweima()` | 任务码（形如 `132+312`）解析，下摄像头 |
| 物块定位 | `run_wukuaiyuanxin_1/2/A()`、`run_wukuaiyuanxin_xuanzequyu()` | 圆形轮廓 + 颜色分类，上摄像头 |
| 色环定标 | `run_sehuanyuanxin2()`、`..._centered()` | 一次粗定位 + 二次环心精定位 |
| 码垛识别 | `run_maduoyuanxin2()`、`..._centered()` | 多层堆叠场景 |
| 边界线检测 | `detect_boundary_line()` | 场地边界辅助纠偏 |
| 命令分发 | `class HostSerialCommandHandler` | 串口单字节命令 → 视觉任务 → 9 字节回包 |

- 二维码识别走 **pyzbar 优先、`cv2.QRCodeDetector` 降级**的双通道，并做**连续 3 帧结果一致**才确认，抑制误码。
- 物块与色环采用 **HoughCircles 粗定位 + HSV `inRange` 颜色判定 + 形态学去噪** 的两阶段策略：先粗定位靠近，再二次"centered"精定位。
- 三个工位的颜色用**排除法**确定（已知两色反推第三色），降低单一色域误判的影响。
- 通信口：`GCS_HOST_SERIAL=/dev/ttyUSB0`，`GCS_HOST_BAUD=115200`。
- 摄像头：`GCS_QR_CAMERA=/dev/video_xia0`、`GCS_UPPER_CAMERA=/dev/video_shang0`，后端 `v4l2`。
- 支持 `--qr / --ring / --ring-centered / --maduo / --maduo-centered` 单独预览，配合 `--dry-run-serial` 可脱离下位机调试。
- 详见 [`host/HOST_SERVICE.md`](host/HOST_SERVICE.md)。

---

## 6. 构建与烧录

### 6.1 下位机固件（CMake + Ninja + arm-none-eabi-gcc）

工具链（STM32CubeCLT 自带）：

```text
cmake      4.2.3+st.1
ninja      1.13.2+st.1
arm-gcc    14.3.1+st.2  (arm-none-eabi-gcc / g++)
```

三个脚本依次执行（WSL 或 Git Bash 下运行，内部通过 `wslpath`/`cygpath` 转换路径）：

```bash
cd firmware
./cfg.sh      # 生成构建目录 build/win-debug（Debug + Ninja + 自定义工具链文件）
./build.sh    # 编译，产物 build/win-debug/STM32f407_wuliu.elf
./flash.sh    # OpenOCD + CMSIS-DAP(SWD, 4000kHz) 烧录，verify + reset
```

- 构建目录与工具链路径可用环境变量覆盖：`BUILD_DIR`、`CMAKE_EXE_WIN`、`GCC_BIN_WIN`、`NINJA_EXE_WIN`、`OPENOCD_EXE`、`OPENOCD_SCRIPTS`。
- 也可以在 STM32CubeIDE / VS Code + CubeMX 里直接打开 `STM32f407_wuliu.ioc`。

### 6.2 上位机视觉（RDK X5 / Linux）

```bash
# 依赖：Python 3.10+、opencv-python、pyserial
python3 host_main.py --self-test      # 自检
python3 host_main.py --camera-test    # 摄像头连通性
python3 host_main.py                  # 正式运行（等待下位机命令）

# 安装为开机自启服务
cp gcs-host /etc/init.d/ && chmod +x /etc/init.d/gcs-host
/etc/init.d/gcs-host start
```

Windows 本机调试用 `run_qr_preview_windows.bat`、`run_ring_preview_windows.bat` 等脚本（避免 PowerShell profile 干扰）。

---

## 7. 关键参数速查

| 项 | 值 |
| --- | --- |
| 上位机串口 | USART1, 115200 8N1 |
| 调试串口 | UART5, 115200 8N1 |
| 电机总线 | CAN1, 500 kbps |
| 舵机 PWM | 50 Hz（TIM2，脉宽 500–2500 µs） |
| 加热 PWM | 500 Hz（TIM3） |
| PID 节拍 | TIM4，Prescaler 839 / Period 4999（≈20 Hz） |
| 航向环默认参数 | `kp 1.5, ki 0.001, kd 2.0`，`maxOutput 180` |
| 航向环预置组 0 | `kp 2.0, ki 0.0, kd 0.8` |
| 航向环预置组 1 | `kp 3.3, ki 0.0, kd 1.8`，`maxOutput 30` |
| 航向环预置组 2 | `kp 5.0, ki 0.0, kd 5.0` |
| 航向环预置组 3 | `kp 2.0, ki 0.05, kd 0.4` |
| 航向环预置组 4 | `kp 2.0, ki 0.07, kd 0.4` |
| 距离标定系数 | 前进 10.5 / 侧移 10.3（需实测校准） |
| 升降行程 / 脉冲系数 | 0–60 单位 / 80 脉冲每单位 |
| 平推行程 / 脉冲系数 | 0–65 单位 / 25.465 脉冲每单位 |
| CAN 帧格式 | 扩展帧，`ExtId = (addr << 8) OR pack` |
| 电机地址 | 1–4 四轮 / 5 升降 / 6 平推 |
| 轮速环增益 | Kp 0.35 / Ki 0.10 / Kd 0.00（节拍 50 ms，反馈超时 200 ms） |
| 看门狗 | IWDG / WWDG **均未启用**（`hal_conf.h` 中相关宏被注释） |
| 爪子 开 / 合 | 50° / 25° |
| 物块盘工位 | 0° / 135° / 270° |
| 圆台角度偏置 | 7.9° |

---

## 8. 文档索引

| 文档 | 说明 |
| --- | --- |
| [使用说明文档.md](docs/使用说明文档.md) | 环境搭建、构建烧录、逐模块操作与调试命令 |
| [项目核心代码解析文档.md](docs/项目核心代码解析文档.md) | GCS 赛项核心代码逐层解析 |
| [函数参考手册.md](docs/函数参考手册.md) | 下位机固件 + 上位机 Python 全函数参考 |
| [代码阅读指南_六文件函数解析.md](docs/代码阅读指南_六文件函数解析.md) | 六个核心文件逐函数阅读指南 |
| [视觉通信与比赛流程分析.md](docs/视觉通信与比赛流程分析.md) | 通信架构、协议帧、完整比赛流程 |
| [串口命令参考手册.md](docs/串口命令参考手册.md) | 调车串口命令速查（SJ / PUSH / ACT / YT / PID） |
| [DEBUG_COMMANDS.md](docs/DEBUG_COMMANDS.md) | 现场调试命令速查 |
| [调车流程.md](docs/调车流程.md) | 从零到可跑车的完整调车流程与标定步骤 |
| [PID调车指南.md](docs/PID调车指南.md) | PID 参数整定方法与实测记录 |
| [速度环位置环角度环分析.md](docs/速度环位置环角度环分析.md) | 三层闭环的原理、参数与调试 |
| [四轮速度闭环控制分析.md](docs/四轮速度闭环控制分析.md) | 四轮速度环实现细节 |
| [麦轮方向分析.md](docs/麦轮方向分析.md) | 麦克纳姆轮方向与运动学约定（修正版） |
| [IMU噪声分析.md](docs/IMU噪声分析.md) | JY901B 噪声辨识与滤波取舍 |
| [升降电机偏差问题分析.md](docs/升降电机偏差问题分析.md) | 升降位置漂移的根因与解决方案 |
| [动作组调试指南.md](docs/动作组调试指南.md) | 复合动作组的设计与调试 |
| [PUSH_HOME完整代码链.md](docs/PUSH_HOME完整代码链.md) | 平推回零的完整调用链剖析 |
| [上电行为模拟.md](docs/上电行为模拟.md) | 三版固件的上电行为对照 |
| [教学文档_从零掌握智能物流小车.md](docs/教学文档_从零掌握智能物流小车.md) | 面向新人的系统级教学材料 |
| [组会汇报_智能物流搬运机器人项目学习总结.md](docs/组会汇报_智能物流搬运机器人项目学习总结.md) | 项目阶段总结与复盘 |
| [命题/](docs/命题) | 赛题说明与评分标准（PDF） |

---

## 附：已知限制与阅读提示

1. **`motor_ack_test.c`（约 3000 行）与 `chassis_calib.c` 未纳入编译**（`CMakeLists.txt` 的源码清单里没有它们）。因此 [`docs/DEBUG_COMMANDS.md`](docs/DEBUG_COMMANDS.md) 与 [`docs/串口命令参考手册.md`](docs/串口命令参考手册.md) 里 `STATUS` / `ACT` / `PUSH` / `SJ` 一类交互命令，**对当前比赛主固件不生效** —— 它们属于电机测试固件（`motor_test` 分支）的调试入口。
2. `Core/Inc/chassis_calib.h` 只有函数声明、**没有标定常量**，不要把它当作参数表使用；实际标定值分散在 `race_chassis.c` 的宏与 `docs/调车流程.md` 的记录表里。
3. `docs/` 下部分文档写于不同开发阶段，个别参数存在互相矛盾（典型如上位机串口波特率被某篇文档写成 9600，而代码与 `.ioc` 均为 115200）。**本 README 一律以源码为准**。
4. `.ioc` 中 TIM2_CH4 的初始脉宽记为 838，`tim.c` 实际为 1500 —— 以源码为准；重新用 CubeMX 生成代码时注意别被覆盖。
5. 轮径、轮距、减速比、每圈脉冲数在工程中**没有常量定义**，只有"距离 → 脉冲"的经验系数，换车或换轮后必须重新标定（见 [`docs/调车流程.md`](docs/调车流程.md)）。
6. `race_task.c` 中的场地距离（910 / 1700 / 1850 mm 等）为**硬编码**，换场地需重新测量调整。

---

## 附：安全提示

- 舵机、升降、平推运动前请先确认**机械零点与运动方向**，方向反了立即断电（详见调车流程）。
- 加热片工作时温度很高，调试时不要触摸，测试前先确认 `ACT_HEAT_COMPARE` 标定值。
- 底盘移动命令下发前确保场地无人、车体线缆不会缠绕。
