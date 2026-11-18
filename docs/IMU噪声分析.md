# IMU 噪声分析

---

## 一、数据怎么来的

```
IMU 模块(串口) → USART2 中断 → WitSerialDataIn() → SDK 解析
  → sReg[Yaw] (=Yaw角原始值)
  → SensorDataUpdata(): s_cDataUpdate |= ANGLE_UPDATE

imu_scan() 每 50ms 被 llm_apply_control() 调用一次：
  if (s_cDataUpdate == 0) → 直接 return，fAngle 保持旧值！
  fAngle[2] = sReg[Yaw] / 32768 × 180  → 无任何滤波
```

## 二、发现的三个问题

### 问题1：无滤波，裸数据直接给 PID

`imu.c` 第 119~121 行：

```c
fAcc[i]   = (float)sReg[AX + i] / 32768.0f * 16.0f;
fGyro[i]  = (float)sReg[GX + i] / 32768.0f * 2000.0f;
fAngle[i] = (float)sReg[Roll + i] / 32768.0f * 180.0f;
```

直接转换，不走任何低通滤波、中值滤波、死区。IMU 原始数据的 ±0.5° 噪声直接喂给 PID。

### 问题2：D 项放大噪声

`race_chassis.c` 第 377 行：

```c
dout = (pid->error - pid->lastError) * pid->kd;
```

50ms 采样周期 + 未滤波的 Yaw → 噪声的 D 分量被放大。即使 Yaw 只抖了 0.5°，配合 kd=0.4：

```
dout = 0.5 × 0.4 = 0.2  →  不是很大
```

但如果 IMU 更新不同步（见问题3），缺帧后再收到新数据的那个瞬间，`lastError` 是 100ms 前的值，新 error 突然跳 2~3°，D 项就是十几个单位的尖刺。

### 问题3：IMU 更新频率 ≠ PID 采样频率

你的 CSV 数据里**每两行数值相同**：

```
1006118, -123.387, -168.387, ...
1006131, -123.387, -168.387, ...  ← 13ms 后完全相同的 feedback！
1006167, -123.387, -168.387, ...
1006217, -123.387, -168.387, ...
1006267, -123.387, -168.387, ...
```

数据实际上 100ms 才换一次，中间三次 `imu_scan()` 全被静默跳过（`s_cDataUpdate==0`），PID 用旧数据算了三个周期。100ms 后突然新数据到了，error 一跳，D 项一个尖刺。

## 三、验证方法

**输入 `IMU` 十次，看 Yaw 值抖不抖：**

连续敲 10 次 `IMU`，每次记下 yaw 值。比较 10 个值的波动幅度。如果在 ±0.5° 以内 → IMU 本身没问题，是采样不同步造成的。如果波动超过 ±2° → IMU 模块本身噪声大。

**输入 `IMUREAD` 看数据频率：**

查 IMU 返回的原始数据寄存器，看更新率是多少 Hz。

**原地不动输入 `LLM ON 0 1`，看 CSV 数据：**

```bash
SET P:0.06 I:0 D:0
LLM ON 0 1        ← 只转1°，角度环不发力（P×1°=0.06，小于死区输出）

# 如果车完全不动，CSV 里的 feedback 还在 ±2° 上下抖 → IMU 噪声大
# 如果车完全不动，feedback 稳如死狗 → IMU 没问题，问题在其他环节
```

## 四、如果是 IMU 噪声，怎么修

### 方案A：加一阶低通滤波（最简单）

在 `llm_apply_control` 里，用当前反馈值替代原始值：

```c
// 改前:
imu_scan(fAcc, fGyro, fAngle);
s_llm_feedback_yaw = fAngle[2];

// 改后: 加一阶低通滤波
static float filtered_yaw = 0;
imu_scan(fAcc, fGyro, fAngle);
filtered_yaw = filtered_yaw * 0.7f + fAngle[2] * 0.3f;  // α=0.3
s_llm_feedback_yaw = filtered_yaw;
```

α 越小滤波越重（响应越慢）。0.3 是合理起点。

### 方案B：减小采样不同步

`imu_scan` 返回 0 时不更新 PID，避免 D 项用旧数据：

```c
imu_scan(fAcc, fGyro, fAngle);
if (imu_update_count() > prev_update_count) {
    s_llm_feedback_yaw = fAngle[2];  // 只有新数据才更新反馈
    PID_Calc(...);
    prev_update_count = imu_update_count();
}
// 旧数据不做 PID，保持上次输出
```

### 方案C：降 kd，去 D 项

把 `SET P:0.08 I:0.02 D:0`（无 D 项）。用 I 代替 D 消残差——I 项对噪声不敏感，不会放大尖刺。代价是响应稍慢。

## 五、结论

有可能不是 IMU 硬件问题，而是**裸数据+采样不同步+D项放大**的组合作案。先验证步骤三（LLM ON 0 1 不动看反馈抖不抖），确认根因再修。如果是 IMU 模块本身噪声大（±3°以上），换模块；如果是采样不同步，用方案B+方案A。如果只是 D 项放大，直接用方案C（降 kd 或去 kd）。
