# 自瞄（TF 树 + 实拍 PnP 绝对角）—— 可用状态说明

> 2026-10-12 打通。**这页是结论，别凭记忆改，尤其看第 3 节"别改回去的地方"。**

## 1. 它到底是怎么工作的

```
仿真 AimCapture 出图 ─┐
                      ├─ 同一个 UDP 包（41001，第0片带 36 字节相机位姿）
                      │
YOLO + 灯条精修 ──► PnP(ippe) ──► pos_c ──► peer 换算成"绝对角"
                      │
            41003 绝对角包（useAbsolute=1, hasTarget, absYaw, absPitch）
                      │
仿真 WLVehiclePawn::AimNetStep 绝对角通道 ──► MouseYawRateCmd / Gimbal.PitchTarget
                      │
                      └─ MouseYawRateCmd 在**同一帧内**被 follow 控制器消费（Rate += -MouseYawRateCmd）
                         ⇒ 每帧写、每帧用。它每帧初被清零（WLVehiclePawn.cpp:2671），所以只能帧内生效。
```

**不是视觉伺服。** 视觉端只提供角度；速度环、限幅、伺服都在仿真里。

## 2. 怎么跑

```
1) 双击 Tools\bias_window.bat        （会自动杀掉旧 peer，开一个调参窗口）
2) 进关，按住右键对目标
3) 看 peer 窗口的 [tuner] 行 + 编辑器日志的 WLAIMGATE
```

调偏置：`←→` yaw∓0.1°、`↑↓` pitch±0.1°、`PgUp/PgDn` pitch±1.0°、
`b` 自动配平、`r` 清零、`y` 存盘、`?` 帮助。

**判据（三个数）**
| 日志 | 期望 |
|---|---|
| `WLAIMGATE ... fresh=1` | 绝对角通道在执行（`fresh=0` = 门关着，云台不会动） |
| `WLAIMGATE ... yawT/pitchT` | 数字在变 = 云台在跟 |
| `WLAIMTRUE ... err=` | 收敛到 0 附近（实测 0.12°） |

## 3. ⚠ 别改回去的地方（每条都是踩过的坑）

| 位置 | 现在的写法 | 为什么 |
|---|---|---|
| `WLVehiclePawn.cpp` 绝对角通道的**门** | `bTargetFresh` 用**指令包里的 `hasTarget`** 刷新时间戳（`AimAbsTargetTime`） | 曾用 `AimNetHasTargetIsFresh()`（0.4s 无目标判据）⇒ **永远 fresh=0 ⇒ 整块跳过 ⇒ 云台完全不动** ✗ |
| 同处 `MouseYawRateCmd` | `+= YawVel`（正=左转需求） | 曾写 `+= -YawVel` ⇒ 正反馈 ⇒ 单向甩飞 ✗ |
| `WLAimNet.cpp` `SendFrame` | 第 0 片 JPEG 后附 **36 字节 `FCamPose`** | 图像与相机位姿分两个包发 ⇒ 云台一转整帧偏几十度 ✗ |
| `plugyou_peer.py` 用相机位姿 | **帧内携带的位姿 + 检测那一刻的快照** | 直接读"最新帧"位姿 ⇒ 等于把时间差引回来 ✗ |
| `plugyou_peer.py` PnP 求解器 | **`ippe`** | `qcqp`/`diff` 只约束方向，深度不可观 ⇒ 测距 +39.8% ✗ |
| `plugyou_peer.py` 偏置 | `AUTOBIAS` 直接叠加到 `target_yaw/pitch`（**不要**经过 `TF_FILT`） | 曾有一层 `TF_FILT` 一阶滤波(0.35)+门限(20°/12°) ⇒ 按一下方向键只挪 0.035° ⇒ "改了没反应" ✗ |
| peer 控制台输出 | 只有**一条** `_live()` 行，且**留 1 列不写满** | ① 写满列宽会折行 ⇒ 每刷一次多滚一行 ② 两处 printf 互相覆盖也会滚 ✗ |

## 4. 相关文件

| 文件 | 作用 |
|---|---|
| `Tools/plugyou_peer.py` | 视觉桥：收帧、检测、PnP、算绝对角、发指令；含调偏置窗口 |
| `Tools/bias_window.bat` / `.py` | 启动器（自动清旧 peer + 开调参窗口） |
| `Tools/bias_saved.json` | 偏置存档（按 `y` 生成，启动自动载入） |
| `Source/.../WLAimNet.h/.cpp` | 帧协议（含 36 字节相机位姿） |
| `Source/.../WLVehiclePawn.cpp` | 绝对角通道、`WLAIMGATE`/`WLAIMCMD`/`WLAIMTRUE` 诊断日志 |

## 5. 调试用的日志（都在编辑器 Output Log / Saved/Logs）

| 日志 | 用途 |
|---|---|
| `WLAIMGATE` | 绝对角通道有没有执行 + 门的状态 + 云台目标角 |
| `WLAIMCMD` | 收到的原始指令 + engaged + 云台角（不受 engaged 影响，每秒一条） |
| `WLAIMTRUE` | 视觉给的角 vs 真值（`err` 就是残差）—— 判"谁错"最快的一条 |
| `[tuner]`（peer 窗口） | 偏置 / 下发角 / 像素误差 / 残差 d，实时一行 |

## 6. 弹道补偿 / 稳态误差积分 / 检测抗扰（2026-10-12 补）

### 6.1 弹道补偿（**默认开**）

仿真 `WLPellet.h`：17 mm 凝胶弹、3.2 g、**初速 23 m/s**、**二次空气阻力**、g=9.81。
23 m/s 下坠很凶，必须按距离抬枪：

| 距离 | 1 m | 3 m | 6 m | 10 m | 20 m |
|---|---|---|---|---|---|
| 抬枪角 | 0.54° | 1.66° | 3.48° | **6.10°** | 13.94° |

实现：`plugyou_peer.py` 的 `ballistic_superelevation()`，按 PnP 斜距实时算并叠到 pitch。
可调 `--v0` `--cd` `--ballistic-scale`；`--no-ballistic` 关闭。
**注意**：视觉端 `data.yaml` 里配的是 `speed: 25.0`，与仿真 23 m/s 不一致 —— 补偿按仿真算。

### 6.2 稳态误差积分（**默认开**）

现象（用户实测）：**两车都不动，从左边跟过去和从右边跟过去停在不同的角度**。
根因：整条回路是**纯比例**控制 —— peer 端 `误差×增益`，仿真端
`Rate += -MouseYawRateCmd`（`WLVehiclePawn.cpp:2557`）。有摩擦时推不动就停，
残余误差取决于接近方向。

实现：`INTEG` —— 对**闭环角度残差**（目标角 − 炮口当前世界朝向）慢积分后叠加到目标角。
只在 `|px_err| < 90` 时才积分（确保咬住目标）。`--no-integ` 关闭，
`--integ-rate` / `--integ-limit` / `--integ-pxgate` 可调。

> 注：**pitch 从不同位置跟本来就会不同**（距离变 ⇒ 弹道抬枪角变），这是对的，不是稳态误差。

### 6.3 检测跳变闸门（打弹时灯条被干扰）

现象：打弹时弹丸（荧光绿胶囊）飞过画面/擦灯条 ⇒ 某帧的框突然跳位或尺寸突变 ⇒ 云台抖。
实现：检测主循环里加连续性判据 —— 中心位移 > 220 px 或尺寸变化 > 70% 就**丢弃该帧检测**，
让 tracker 沿用上一帧观测。日志 `[det] reject jumped frame ... total rejected=N`。
实测抓到过 `d_ctr=663px d_size=117%`。

---

## 7. 已知还没做的


- **相机→枪口偏移补偿**（`--comp 1`）：帧里已带偏移量，但默认关着。恒定偏置用 `AUTOBIAS` 压平就够了。
- **`WLAIMCMD` / `WLAIMGATE` 是取证日志**，稳定后可以删（留着也不碍事，每秒 1 条）。
- 那批 `tf_check/tf_proof/tf_final/tf_full/real_frame_check` 等分析脚本里有几版结论是**错的**
  （用合成数据自证的循环论证），只留作过程记录，别当依据。
