# 架构

> 系统由什么组成，各部分边界在哪，什么稳定、什么变化。

## 一、一句话

**core 定义契约与数学，sim 提供世界，custom 是你的方案，entrance 把三者装配起来。**

## 二、四个目录

```
core/       契约 + 数学。不认识 sim / custom。
sim/        世界。物理、传感器、电控、渲染。
custom/     方案 + 组件。你写代码的地方。
entrance/   装配。唯一同时认识 sim 和 custom 的地方。
```

**依赖方向是单向的**：

```
       entrance
         /  \
        ↓    ↓
      sim    custom
        \    /
         ↓  ↓
         core
```

- `core` 不 import `sim` / `custom`
- `sim` 不 import `custom`
- `custom` 不 import `sim`
- **只有 `entrance` 同时认识两边**

这条边界是整个架构最重要的一条。它保证：**学生改 `custom/` 下的任何东西，不会弄坏仿真器；反过来也一样。**

## 三、core —— 契约 + 数学

core 不知道自己会为谁服务。它定义"消息长什么样"和"扩展点长什么样"，仅此而已。

### 3.1 消息契约

`core/bus/dataflow/` 定义系统里流动的全部消息类型：

| 消息 | 方向 | 含义 |
|---|---|---|
| `DetectionFrame` | sim → custom | 一帧检测（只有 2D 图像点） |
| `SelfState` | sim → custom | 一帧自车状态（云台/相机/枪口位姿） |
| `Observation` | custom 内部 | 一帧位姿观测（3D 位置 + 朝向） |
| `Prediction` | custom → sim | 一帧目标预测（未来时刻的位置，供渲染画点） |
| `ShotSolution` | custom → sim | 自瞄的火控指令（角度 + 前馈 + fire / mode / key） |
| `DrawText` | 任意 → 渲染 | 调试文本 |
| `PipelineStats` | custom → 渲染 | custom 循环的运行时度量（fps） |

每个 dataclass 的 docstring 里写着**字段的语义约定**——比如 `Prediction.armors[0]` 是"未来时刻"的位置，不是当前；`ShotSolution.fire` 是自瞄的推荐，不是电控的裁决。这些约定比字段本身更重要。

### 3.2 扩展点契约

`core/interfaces/` 定义两个扩展点：

- **`Tracker`** —— 一份完整的 MOT 方案。`step(obs, self_state) -> List[Prediction]`。
- **`PoseSolver`** —— 帧级位姿解算。`solve(frame, self_state) -> Observation`。

两者都继承 `Registry`，用 `@register_*` 装饰器注册，在启动界面里被挑选。

### 3.3 数学工具

`core/algorithms/` 放**不做方案假设的代码**。判据：

> 这段代码假定了一个从检测到发射的固定流程的某一段吗？
> 假定 → 搬 custom；不假定 → 留这里。

- `math/` —— 角度、变换、相交、四元数、五次多项式（纯函数 + 纯数学类）
- `filters/` —— KF / EKF / UKF
- `control/` —— PID / MPC

**不含**：PnP 的具体实现、弹道解算器——它们做了方案假设（"检测必须经过 PnP"、"弹道要单独算"），所以住 custom。

### 3.4 领域实体

`core/entities/` 定义世界里的东西：`Robot` / `Chassis` / `Gimbal` / `Armor` / `Bullet` / `Rune`。

**这些是 sim 的模型，custom 不接触它们。** custom 处理的是总线上的消息（`Observation` / `Prediction` / `ShotSolution`），它看不到实体。

## 四、sim —— 世界

sim 提供"自瞄算法所面对的环境"。它包含：

- **`simulator.py`** —— 装配全部 manager，驱动主循环
- **`comm.py`（`SimComm`）** —— sim 侧的通信模块
- **`managers/`** —— 各子系统：motion / robot / device / bullet / sensor / control / keyboard
- **`render/`** —— 渲染（主视图 / 相机视图 / 信息面板 / 曲线面板）
- **`clock.py`** —— 仿真时钟

sim 不关心你写什么 tracker。它只做两件事：

1. **产生输入**：每帧 `sensor_manager` 出检测 + `collect_self_state()` 出状态，通过 `SimComm.publish_frame` 发到总线。
2. **消费输出**：每帧 `control_manager.update` 从 `SimComm.latest_control()` 取最新 `ShotSolution`，驱动云台并发弹；`render_manager` 订阅 `Observation` / `Prediction` 做可视化。

### 4.1 sim 与 custom 的异步关系

**两个循环，各自跑，通过总线交换最新值。**

```
sim 循环（sim_fps = 200）           custom 循环（target_fps = 300）
─────────────────────              ────────────────────────────
physics / motion / sensor            keeper 按 fps 叫 pipeline.tick
  ↓ publish_frame                        ↓ latest_inputs
[public_bus]                            [public_bus]
  ↑ latest_control                       ↑ publish_control
control / render                     pose_solver → tracker → shot_solver
```

**关键点**：谁快谁慢都不影响对方——慢的一侧读到的永远是"最新那一份"，不是积压的旧值。

这靠 `SimComm` / `CustomComm` 的**单槽缓存**实现（一个变量，不是队列）：sim 每出一帧检测就覆盖 `CustomComm._latest_frame`，custom 每次 tick 拿到的就是"最新一帧"。

**为什么不是队列**：自瞄只关心最新帧。队列会让"sim 出帧比 custom 消费快"时延迟累积——用 100ms 前的检测去打现在的目标，是错的。

## 五、custom —— 方案 + 组件

**这是你写代码的地方。** custom 分两层：

### 5.1 schemes —— 方案

一份**完整的 MOT 实现**。包含：

- 一个 `Tracker` 实现（方案主体）
- 一个 `PoseSolver` 实现（帧级解算）
- 方案内部私有代码：模型、目标池、射击决策

```
custom/schemes/tjurm_scheme/
  tracker.py            注册为 "multi_target"
  pose_solver.py        注册为 "tjurm"
  pool.py               目标池（方案私有）
  shot_solver.py        一发解：model→select→traj 迭代（方案私有）
  models/               EKF 模型（方案私有）
  mpc/                  MPC 扩展（方案私有，默认关闭）
```

**方案之间不共享代码。** 你想写第二份方案，就复制一份新目录、改内部、换 `@register_*` 的名字。

### 5.2 components —— 组件

**跨方案共享的库。** 是"预判学生可能用到的工具"，不是抽象层：

```
custom/components/
  armor_solvers/       单板 PnP 求解器（diff / qcqp / ippe）
  traj_solvers/        弹道解算器
  armor_selectors/     选板策略
  shooters/            射击决策
```

判据：**能脱离任何具体 scheme 独立使用吗？** 能 → 这里；不能 → scheme 私有。

**组件不是接口。** 它们是普通类——不注册、不出现在启动界面、方案内部自己 `import`、自己 `new`。

### 5.3 pipeline —— custom 内编排

`custom/pipeline.py` 做两件事：

```python
def tick():
    frame, self_state = comm.latest_inputs()
    obs = pose_solver.solve(frame, self_state)         # 帧级解算
    preds = tracker.step(obs, self_state)               # 方案主体
    for pred in preds:
        publish(pred/<type>, pred)                      # 分发到总线
    # tracker 内部自己发 ShotSolution 到 CONTROL
```

**pipeline 只认识 `Tracker` 和 `PoseSolver` 两个接口。** 它不知道方案内部有几个模型、几个组件、怎么组织。

**火控（`ShotSolution`）是方案自由**：tracker 内部自己决定要不要发、什么时候发、算得对不对。pipeline 不参与——它不认识 `ShotSolution`。

## 六、entrance —— 装配

唯一同时认识 sim 和 custom 的地方。它做四件事：

- **`cli.py`** —— 启动准备（stdio、schema）+ 分发
- **`host.py`** —— 宿主循环：选择 → 跑一局 → 返回 → 再选择
- **`selector.py`** —— 两列选择界面（Tracker / PoseSolver）
- **`session.py`** —— 一局游戏：装总线、创建两个 comm、boot custom、跑 sim 主循环
- **`keeper.py`** —— custom 循环的节拍器

### 6.1 keeper

**keeper 不碰数据。** 它只做一件事：按 `target_fps` 叫 `pipeline.tick()`。

- 它订阅 `RESET`（sim 复位后通知 custom）
- 起一个线程，每 `1/target_fps` 调一次 `tick`
- 异常时报死（打 stderr + 将来会打调试面板）

**它曾经管的那些事——缓冲、丢帧、消息转发——现在都归 `SimComm` / `CustomComm`。** 数据的进出归 comm，节拍归 keeper。

## 七、稳定 vs 变化

**稳定的（core 定义的契约）：**

1. **消息格式** —— `DetectionFrame` / `SelfState` / `Observation` / `Prediction` / `ShotSolution` 的字段 + 语义 + 坐标系约定。
2. **时序** —— sim 一帧发一次输入，custom 一帧最多发一次输出，`timestamp` 一致。
3. **方案粒度** —— 一个方案 = 一个注册的 `Tracker`。

**变化的（custom 自由决定）：**

- 一份方案分几段
- 方案内部怎么组织
- 用哪个 PnP、哪个弹道、哪个选板
- 火控判据（`ShotSolution.fire` / `fire_mode` / `fire_key` 怎么算）
- 是否完全不用 PnP（比如端到端网络直接出 `Prediction`——只要你能拿到 `DetectionFrame` 就合法）

## 八、为什么这样分

### 8.1 为什么 core 不认识 sim / custom

如果 `core.algorithms.pose` 里有 PnP 实现，那它就隐含地假定"存在一个从检测到发射的固定流程"。学生想换掉整条链路时，这段代码就变成了负担——他得绕过它而不是替换它。

把"契约"和"具体流程"分开，让 core 只做前者。

### 8.2 为什么 sim 与 custom 互不认识

如果 sim 认识 custom，那"加一个新 tracker"就要改 sim；如果 custom 认识 sim，那"改一个 sim 内部细节"就可能破坏 custom。

**两边通过 `core.bus` 的消息和 `core.interfaces` 的接口相遇**——这是唯一接触面。任何一边单独重构，另一边不受影响。

### 8.3 为什么用总线而不是直接传参

**异步。** sim 与 custom 是两个独立循环，各自跑自己的帧率。总线提供"一个循环发、另一个循环收"的解耦——发的人不知道谁在听，收的人不知道谁在说。

如果直接传参，就必须有"谁调谁"的关系——那两边就绑死了。

### 8.4 为什么 schemes 和 components 分两层

**方案是变化的、组件是预判的。**

- 方案：一份完整实现。学生写自己的方案时，从零开始或者 copy 一份——他改的东西和他写的代码在同一个目录里。
- 组件：所有方案可能都用得上的工具。它们是**库**——不是抽象层、不做接口、不被选择界面感知。

如果只分一层（比如把 PnP 放 components，方案里 import 它），那"加一个新方案"和"改一个公共工具"的影响范围就分不开——改一个工具可能影响多个方案。

分两层之后，影响范围清楚：**改 scheme 只影响自己；改 component 可能影响所有用了它的 scheme。**

## 九、一句话

**分层是因为变化速率不同。**

- 消息契约最稳定——一年也不会改一次
- sim 中等——物理、渲染可以改，但接口稳定
- custom 里的 components 次之——工具可以加、可以换
- custom 里的 schemes 最活跃——一份方案可以彻底重写

**每一层的边界，都是"变化速率的边界"。**