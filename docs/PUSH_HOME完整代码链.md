● PUSH HOME 完整代码链

  串口输入 "PUSH HOME"
  │
  ├─[1] handle_line()                     ← 第2717行
  │     starts_with(cmd, "PUSH ")
  │     → handle_pingtui(cmd + 5)
  │
  ├─[2] handle_pingtui()                  ← 第1821行
  │     starts_with(cursor, "HOME")        ← 第1979行
  │     cursor += 4
  │     确认后面无多余字符
  │
  │     → zdt_home_trigger(6, 2, false)    ← 第1988行，就这一行！
  │         ↓  [hal_zdt.c 第250行]
  │         CAN发送: addr=6, cmd=0x9A, mode=2
  │         平推电机开始向限位方向全速运动
  │
  │     打印 "# PUSH HOME addr=6 mode=2 sensorless"
  │     return  ← 不等结果，直接返回！

  和 SJ HOMEZERO 的关键区别

  ┌───────────────┬────────────────────────────┬────────────────────────────┐
  │               │        SJ HOMEZERO         │         PUSH HOME          │
  ├───────────────┼────────────────────────────┼────────────────────────────┤
  │ 清除堵转      │ ✅ zdt_clear_stall(5)      │ ❌ 无                      │
  ├───────────────┼────────────────────────────┼────────────────────────────┤
  │ 使能电机      │ ✅ zdt_en(5, true)         │ ❌ 无（假设已使能）        │
  ├───────────────┼────────────────────────────┼────────────────────────────┤
  │ 配置回零参数  │ ✅ zdt_home_cfg_sensorless │ ❌ 无（用默认/之前配置的） │
  ├───────────────┼────────────────────────────┼────────────────────────────┤
  │ 触发回零      │ ✅ zdt_home_trigger(5, 2)  │ ✅ zdt_home_trigger(6, 2)  │
  ├───────────────┼────────────────────────────┼────────────────────────────┤
  │ 等待完成      │ ✅ wait_home_idle(5, 10s)  │ ❌ 不等！发完就返回        │
  ├───────────────┼────────────────────────────┼────────────────────────────┤
  │ 清除位置      │ ✅ zdt_clear_pos(5)        │ ❌ 无                      │
  ├───────────────┼────────────────────────────┼────────────────────────────┤
  │ 软件位置=0    │ ✅ set_current_pos(0)      │ ❌ 无                      │
  ├───────────────┼────────────────────────────┼────────────────────────────┤
  │ 报告成功/失败 │ ✅ done/timeout_or_fail    │ ❌ 不报告                  │
  └───────────────┴────────────────────────────┴────────────────────────────┘

  PUSH HOME 是"发射后不管"——命令发出去了，电机开始转，但 STM32 不跟踪它是否成功、什么时候停。

  平推完整操作应该这样

  PUSH EN            ← 先使能
  PUSH HCFG 0        ← 配回零参数（方向0, 800mA, 60ms）
  PUSH HOME          ← 触发回零（电机开始动）

#####等几秒，观察电机停止...

  PUSH CLEAR         ← 手动清零位置

  或者直接用位置命令驱动：

  PUSH EN
  PUSH ZERO          ← 标零
  PUSH 5 400 60      ← 小幅度测试
  PUSH 0 400 60      ← 回收

  关键常量

  PINGTUI_ADDR                  = 6       ← CAN地址
  PINGTUI_MIN_POS               = -20     ← 位置下限(mm)
  PINGTUI_MAX_POS               = 65      ← 位置上限(mm)
  PINGTUI_DEFAULT_SPEED         = 1000    ← 默认速度
  PINGTUI_DEFAULT_ACC           = 120     ← 默认加速度
  ZDT_HOME_PINGTUI_CURRENT_MA   = 800     ← 碰撞电流阈值(mA)
  ZDT_HOME_PINGTUI_TIME_MS      = 60      ← 碰撞持续时间(ms)
  ZDT_HOME_VEL                  = 30      ← 回零运动速度(RPM)
  ZDT_HOME_SENSORLESS_VEL       = 300     ← 传感器less判定速度
  ZDT_HOME_TIMEOUT_MS           = 10000   ← 超时(10s)

  完整代码链（5层调用）

  串口输入 "SJ HOMEZERO"
  │
  ├─[1] handle_line()                    ← 第2717行，顶层命令分发
  │     starts_with(cmd, "SJ ")
  │     → handle_shengjiang(cmd + 3)
  │
  ├─[2] handle_shengjiang()              ← 第1461行，SJ 子命令解析
  │     starts_with(cursor, "HOMEZERO")
  │     解析可选参数：dir, current_ma, time_ms, timeout_ms
  │     默认值：
  │       dir=1              ← 方向（第1558行）
  │       current_ma=600     ← 碰撞电流阈值mA（第1559行，宏第65行）
  │       time_ms=40         ← 碰撞持续时间ms（第1560行，宏第66行）
  │       timeout_ms=10000   ← 总超时ms（第1561行，宏第61行）
  │
  │     参数校验（第1571-1574行）：
  │       dir ∈ [0,1]
  │       current_ma ∈ [400, 3000]
  │       time_ms ∈ [20, 200]
  │       timeout_ms ∈ [1000, 30000]
  │
  │     打印 "# SJ HOMEZERO START mode=2 dir=1 current=600 time=40 timeout=10000"
  │     → shengjiang_homezero_run(dir, current_ma, time_ms, timeout_ms, &status)
  │
  ├─[3] shengjiang_homezero_run()        ← 第734行，回零执行
  │     │
  │     ├── zdt_clear_stall(5)           ← 清除地址5堵转保护
  │     ├── delay_ms1(20)
  │     ├── zdt_en(5, true, false)       ← 使能升降电机
  │     ├── delay_ms1(50)
  │     │
  │     ├── zdt_home_cfg_sensorless(5, dir, current_ma, time_ms)
  │     │   └→ zdt_home_params_write(5, 不保存, mode=2, dir,
  │     │         回零速度=30, 超时=10000,
  │     │         传感器less速度=300, current_ma, time_ms,
  │     │         不自动回零)
  │     │         ↓  [hal_zdt.c 第288行]
  │     │         CAN发送: addr=5, cmd=0x4C, 20字节配置
  │     │
  │     ├── delay_ms1(20)
  │     ├── zdt_home_trigger(5, 2, false)  ← 触发回零，mode=2
  │     │         ↓  [hal_zdt.c 第250行]
  │     │         CAN发送: addr=5, cmd=0x9A, mode=2
  │     │         电机开始全速向限位方向运动！
  │     │
  │     ├── wait_home_idle(5, timeout_ms, &status)
  │     │   │
  │     │   └──[4] wait_home_idle()       ← 第711行，轮询等待
  │     │         while (未超时) {
  │     │           zdt_read(5, ZDT_S_ORG)    ← CAN查询回零状态寄存器0x3B
  │     │           delay_ms1(80)
  │     │           pop_home_status(5, &status)
  │     │           │
  │     │           └──[5] pop_home_status() ← 第691行
  │     │                 从CAN环形缓冲区找 addr=5, data[0]=0x3B 的帧
  │     │                 提取 status = data[1]
  │     │                 │
  │     │                 if (status & GOING) == 0  → 回零完成！
  │     │                 if (status & FAIL)  != 0  → 回零失败！
  │     │           │
  │     │           delay_ms1(120)
  │     │         }
  │     │         超时 → 返回 false, status=0x04(GOING)
  │     │
  │     ├── 如果失败 → zdt_home_stop_repeat(5)  ← 强制中止+连发3次停止
  │     │
  │     ├── delay_ms1(120)
  │     ├── zdt_clear_pos(5)              ← 驱动器位置脉冲清零
  │     ├── delay_ms1(20)
  │     ├── shengjiang_set_current_pos(0)  ← 软件位置=0
  │     │
  │     └── 返回 true/false
  │
  ├─[2] 打印结果
  │     ok=true  → "# SJ HOMEZERO done status=03"
  │     ok=false → "# SJ HOMEZERO timeout_or_fail status=04"
  │
  └── 不管成功失败，位置都已清零

  关键常量

  SHENGJIANG_ADDR              = 5       ← CAN地址
  ZDT_HOME_VEL                 = 30      ← 回零运动速度(RPM)
  ZDT_HOME_TIMEOUT_MS          = 10000   ← 总超时(ms)
  ZDT_HOME_SENSORLESS_VEL      = 300     ← 传感器less判定速度(RPM)
  ZDT_HOME_SHENGJIANG_CURRENT_MA = 600   ← 碰撞电流阈值(mA)
  ZDT_HOME_SHENGJIANG_TIME_MS  = 40      ← 碰撞持续时间(ms)
  ZDT_HOME_STATUS_REPLY        = 0x3B    ← 状态回复命令码
  ZDT_HOME_STATUS_GOING        = 0x04    ← bit2=运动中
  ZDT_HOME_STATUS_FAIL         = 0x08    ← bit3=失败

  状态码含义

  status=03 = ENCODER_READY|TABLE_READY  → 成功，已就绪
  status=04 = GOING                       → 超时，电机还在转
  status=08 = FAIL                        → 失败（堵转异常等）