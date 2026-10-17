# 写你自己的 Tracker

> 从一帧观测到一帧预测：Tracker 是什么、怎么跑、怎么一步步做出你想要的 MOT。

## 一、Tracker 是什么

**一句话**：Tracker 是"一帧观测 → 一帧预测"的函数。

```
Observation + SelfState  ──→  Tracker.step  ──→  List[Prediction]
```

- **输入**：`Observation`（一帧位姿观测）、`SelfState`（这一帧自车状态）
- **输出**：`Prediction` 列表——**第一条**是"该打谁"，其余是候选

**就这一个契约。** Tracker 内部怎么组织，从零写还是一个 EKF、要不要多目标、要不要选板、要不要做火控——都是你的自由。

## 二、输入长什么样

### Observation

一帧解算结果。`obs.obs_armors` 是 `ArmorObservation` 列表——每一块被看到的装甲板。

```python
@dataclass
class ArmorObservation:
    entity_type: EntityType     # 什么车（Hero / Infantry / ... / Rune）
    rel_pos: np.ndarray         # 位置 (3,)，G 系或 W 系（看 cfg.sim.frame）
    rel_rpy: np.ndarray         # 朝向 (3,)，绝对系
    armor_size: str             # "small" / "large"
    image_points_2d: np.ndarray # (4,2) 像素四角点，顺序 LT / RT / RB / LB
    lit: bool                   # 只有符用：这块灯臂是否点亮
```

**要点**：

- `rel_pos` 是**装甲板**的位置，不是车的中心。一块车上几块板就有几个 `ArmorObservation`。
- `rel_rpy[2]` 是板在**绝对系**下的偏航（不是相对相机的角度）。
- 一帧可以看到同一辆车的**多块板**（比如 Hero 正对你时能看到 2 块）。

### SelfState

这一帧自车状态。常用字段：

```python
self_state.chassis.pos      # 底盘中心（世界系）
self_state.gimbal.pos       # 云台中心（世界系）
self_state.gimbal.rpy       # 云台朝向（yaw 在这，pitch 正值为低头）
self_state.muzzle.pos       # 枪口位置
self_state.camera.K          # 相机内参矩阵
```

**G 系基准是云台中心**——如果你的 tracker 要做"从枪口看目标"的角度计算，记得把目标位置减掉 `gimbal.pos`（在 G 系下不用减，它已经是相对量了）。

## 三、输出长什么样

### Prediction

```python
@dataclass
class Prediction:
    target_type: TargetType    # ROBOT / OUTPOST / RUNE
    center: np.ndarray         # 目标中心 (3,)
    armors: List[np.ndarray]   # 各装甲板位置，第一条是你要打的
    timestamp: float           # 必须等于这一帧 obs.timestamp
    is_tracking: bool          # 是否在跟踪
    aim_rate: Optional[np.ndarray] = None  # 瞄准点角速度 (2,)
    gimbal_target: Optional[GimbalTarget] = None  # MPC 前馈建议（默认不用）
```

**要点**：

- **`armors[0]` 是"未来时刻"的位置，不是当前**。你应该按 `total_delay`（弹丸飞行时间 + 延迟）前推，然后把这个位置填进 `armors[0]`。
- `timestamp` 必须和输入的 `obs.timestamp` 一致。
- 返回空列表 = "这一帧没有可用目标"。
- **fps 不在 Prediction 里**——它是 pipeline 层的度量，走独立的 `PIPELINE_STATS` 消息，你不需要管。
- **火控信息不在 Prediction 里**——`fire` / `fire_mode` 走 `ShotSolution`（见下）。

### ShotSolution

**这是一个独立的、可选的输出**——只在你"要做弹道、要让云台动"的时候才发。它在 `CONTROL` 通道上，**驱动云台的唯一路径**。

```python
@dataclass
class ShotSolution:
    yaw: float           # 绝对系下的云台 yaw
    pitch: float         # 抬头为正（电控下发时会取负）
    yaw_rate: float = 0.0    # 瞄点视线的角速度，给电控做前馈
    pitch_rate: float = 0.0
    fire: bool = False       # 自瞄的推荐："这一帧该不该扣扳机"
    fire_mode: str = "auto"  # "auto"（连发）/ "single"（单发）
    fire_key: Optional[int] = None  # 自瞄估计的"当前瞄的板/灯臂编号"
    timestamp: float = 0.0
```

**要点**：

- **`pitch` 抬头为正**——你只管发抬头为正的 `pitch`，电控会取负。
- **`yaw_rate` / `pitch_rate` 是前馈**——没有前馈，云台 PID 会滞后于瞄点轨迹。
- **`fire` 是"推荐"，不是"裁决"**——它说的是"自瞄认为该打了"，不是"云台到位了"。电控还会自己判跟踪误差、冷却、用户开关。
- **`fire_mode` / `fire_key` 见 `bus_and_messages.md` §6.5。**
- **`timestamp` 必须与产生它的那条 `Prediction` 一致**——这是配对键。

**如果你只发 `Prediction` 不发 `ShotSolution`**——云台不会动。`Prediction` 只给渲染画点。

## 四、最小可跑的 Tracker

不看默认实现，从零开始。**目标**：让云台指向最近的那块板。

```python
# custom/schemes/my_scheme/my_tracker.py
import math
import numpy as np

from core.interfaces.tracker import Tracker, register_tracker
from core.bus import public_bus
from core.bus.dataflow import Prediction, ShotSolution
from core.bus.messages import PublicMessages
from core.entities.property.entity_types import EntityType, TargetType
from config import cfg


@register_tracker("my_first")
class MyFirstTracker(Tracker):
    description = "我的第一个跟踪器：挑最近的板直接打"

    def __init__(self):
        self.frame = str(cfg.sim.frame).upper()

    def step(self, obs, self_state):
        # 1) 没有观测？什么都不发
        if obs is None or not obs.obs_armors:
            return []

        # 2) 挑一块最近/最中心的板（这里简化：就挑第一块）
        armor = obs.obs_armors[0]

        # 3) 按 entity_type 分类
        if armor.entity_type == EntityType.Rune:
            tt = TargetType.RUNE
        elif armor.entity_type == EntityType.Outpost:
            tt = TargetType.OUTPOST
        else:
            tt = TargetType.ROBOT

        # 4) 直接拿观测位置当预测（先不做提前量）
        pred = Prediction(
            target_type=tt,
            center=armor.rel_pos.copy(),
            armors=[armor.rel_pos.copy()],
            timestamp=obs.timestamp,
            is_tracking=True,
        )

        # 5) 发弹道解——这一步让云台真的动起来
        #    用观测位置直接算云台该指的 yaw/pitch。
        aim = np.asarray(armor.rel_pos, float)
        # G 系下 rel_pos 已经是相对云台中心；W 系下要先减 gimbal.pos
        if self.frame == "W":
            aim = aim - np.asarray(self_state.gimbal.pos, float)
        dx, dy, dz = aim
        yaw = math.atan2(dy, dx)
        pitch_up = math.atan2(dz, math.hypot(dx, dy))
        solution = ShotSolution(
            yaw=yaw,
            pitch=pitch_up,
            fire=True,                 # 先无脑 fire，电控自己会做物理检查
            fire_mode="auto",
            timestamp=obs.timestamp,
        )
        public_bus().publish(PublicMessages.CONTROL, solution)

        return [pred]
```

**注册**：`@register_tracker("my_first")` 会自动把它挂进注册表。

**使用**：`python main.py`，选择界面里就会多一行 `my_first`。按 `KP9` 开自瞄、`KP8` 开自动开火——云台会指向板，开火受电控的冷却和跟踪误差节制。

**你现在能做什么**：

- 打静止靶——云台会指向板的位置
- 打缓慢移动的靶——因为没做提前量，会滞后，但能跟

**你现在不能做什么**：

- 打多个目标——你没有选择逻辑，永远打"第一块"
- 打高速目标——没有前推，云台永远慢一拍
- 打小陀螺——一块板转到看不见时，你不知道该换另一块

**这就是升级路径的起点。**

## 五、升级路径

**每一步只改几十行，但解决一类新问题。** 你不需要一次全做完——按需升级。

### 第 1 步：多目标筛选（"打哪辆"）

现在的 tracker 只看"第一块板"。场地上有几辆车时，你会永远打列表里第一块——不是最该打的。

**做什么**：给每块板算一个"优先级"，挑最高的那个。

最简单的优先级：**离画面中心近的**（`image_points_2d` 的均值到主点的距离）。

```python
def _priority(armor):
    # 越靠画面中心越优先（本项目主点即像面原点）
    pts = armor.image_points_2d
    u, v = pts.mean(axis=0)
    return float(np.hypot(u, v))

best = min(obs.obs_armors, key=_priority)
```

**你学到**：Tracker 的第一层不是"精度"，是"选谁"。

### 第 2 步：位置平滑（"打稳定"）

观测带噪声（`pixel_noise_sigma > 0` 时），云台会跟着抖。

**做什么**：用滑动窗口平均位置。

```python
from collections import deque

def __init__(self):
    self.history = deque(maxlen=10)

def step(self, obs, self_state):
    ...
    self.history.append(armor.rel_pos.copy())
    smoothed = np.mean(np.stack(self.history), axis=0)
    return [Prediction(center=smoothed, armors=[smoothed], ...)]
```

**你学到**：Tracker 的第二层是"滤波"。滑动平均是最简单的滤波器——但它对运动的目标会滞后。

### 第 3 步：前推（"打提前量"）

子弹飞过去要时间。如果只瞄目标当前位置，打出去会落后。

**做什么**：估计目标速度，把瞄点按 `total_delay` 前推。

```python
def __init__(self):
    self.prev_pos = None
    self.prev_t = None

def step(self, obs, self_state):
    ...
    pos = armor.rel_pos
    t = obs.timestamp

    if self.prev_pos is not None and t > self.prev_t:
        dt = t - self.prev_t
        velocity = (pos - self.prev_pos) / dt
    else:
        velocity = np.zeros(3)

    # 假设子弹飞 0.2s
    total_delay = 0.2
    aim_point = pos + velocity * total_delay

    self.prev_pos = pos.copy()
    self.prev_t = t

    return [Prediction(center=aim_point, armors=[aim_point], ...)]
```

**你学到**：Tracker 的核心难点是**时间**——观测是过去的，枪口要指向未来。

### 第 4 步：换 EKF（"打机动目标"）

差分速度只能处理匀速运动。目标如果变速、转圈，差分速度就崩了。

**做什么**：上 EKF。状态向量 `[x, y, z, vx, vy, vz]`——位置 + 速度。用观测更新，用过程模型预测。

平台自带 `core.algorithms.filters.extended_kalman`，参考默认实现里的 `models/armor_model.py`。

**你学到**：这是自瞄从"能打"到"能打稳"的分水岭。

### 第 5 步：多目标（"同时跟踪几辆"）

现在的 tracker 只有一个状态。场地上有三辆车时，观测交替到达，滤波器会来回跳。

**做什么**：给每辆车一个独立的 EKF；用**马氏距离**把观测匹配到已有目标；新目标没见过就新建；目标消失久了就淘汰。

这就是完整的 MOT。参考 `custom/schemes/tjurm_scheme/pool.py`——但先自己写一版最简的，再看它。

**你学到**：多目标的核心是**数据关联**——观测本身不带身份，你得自己判断"这一帧看到的是刚才那辆还是新来的"。

### 第 6 步：选板（"打哪块"）

一辆车有几块板。你从正面看它，哪块板朝向你就打哪块——但你不知道"哪块板朝向自己"。

**做什么**：从跟踪器的状态里算出每一块板的位置（需要**车辆模型**——车绕 Z 转、板的半径和角度），挑一块朝向自车的打。

参考 `custom/components/armor_selectors/tjurm_armor_selector.py`。

**你学到**：这是自瞄从"能打"到"打得准"的最后一环。

### 第 7 步：火控（"什么时候开火"）

现在你只是无脑发 `fire=True`，让电控自己节流。真自瞄要判"现在该不该打"——板是不是朝向自己、灯臂有没有亮、大陀螺时是不是瞄着虚拟点等。

**做什么**：把 `fire` 从 `True` 改成"真的判断"。三种目标三套判据：

- **车**：板法向朝向自车 / 板接近虚拟点
- **前哨**：板法向朝向自车
- **符**：有灯臂点亮

参考 `custom/components/shooters/tjurm_shooter.py`。

**你学到**：火控是"该不该打"，不是"能不能打"。后者归电控。

---

**升级路径不是一次走完的。** 每一步都是一份能跑的 tracker；每一步解决一个具体问题；每一步的代码量在几十行。

## 六、注册与选择

注册用装饰器：

```python
@register_tracker("my_first", aliases=["mine"])
class MyFirstTracker(Tracker):
    ...
```

- `"my_first"` 是主名——启动界面里显示这个
- `aliases` 是可选的别名，命令行里可以用

**选择**：启动界面的 Tracker 列里选你注册的名字。

**`discover()` 会自动找到你**：`custom/__init__.py` 会扫描 `custom/` 下的所有模块，import 它们——只要你在 `custom/` 的任何位置写一个 `@register_tracker(...)`，启动时就会被认到，不用改任何其他文件。

**重名会报错**：如果两个 tracker 用了同一个主名（或别名），注册时立刻报错，报错信息指向两个类的模块路径。**别怕重名——报错是帮你的。**

## 七、常见问题

### 我的 tracker 注册了但没生效？

检查：

1. 类继承 `Tracker`（不是 `PoseSolver`）
2. 装饰器拼写是 `@register_tracker`（不是 `register_solver`）
3. 文件在 `custom/` 目录下（`discover()` 只扫 `custom/`）
4. 启动界面的 Tracker 列里翻到底——如果列表太长可能要往下滚

### 我的预测点数对不上目标？

最常见的原因：**坐标系**。

- `cfg.sim.frame = "G"`：`obs.obs_armors[i].rel_pos` 是**相对云台中心**的位置——你算角度或距离时不需要再减自车位置。
- `cfg.sim.frame = "W"`：`rel_pos` 是**绝对世界坐标**——你要减 `self_state.gimbal.pos` 才能得到"相对云台"的量。

**检查方法**：写一行 `print(armor.rel_pos, self_state.gimbal.pos)`，看两个值是否在同一个量级。如果差一个底盘或云台的高度，说明你混淆了两者。

### 云台在动，但打得不准？

可能的原因，按频率排：

1. **没做提前量**：飞过去 0.2s，目标也移动了 0.2s 的距离。
2. **提前量方向错**：你的速度估计反了（观测位置波动、差分公式方向错）。
3. **用错了坐标**：G 系下的相对量当成绝对量用了。
4. **时间戳不对**：`solution.timestamp` 和 `obs.timestamp` 不一致——下游会拒绝配对。

### 我能不能一次发多条 Prediction？

**能。** `Prediction.target_type` 决定发到哪条 `pred/<type>` 通道。但**火控只看 `ShotSolution`**——你发几条 Prediction 都不影响发弹。

### 我要不要看一眼 `total_delay`？

**要。** `total_delay` 是"从扣扳机到子弹命中目标"的总时间——它由弹道飞行时间加上系统延迟组成。你的 tracker 里做前推时要用它。

**怎么算**：默认实现里由弹道迭代给出（`TrajectorySolver.solve` 返回飞行时间 + `fire_delay`）。你如果不做弹道迭代，可以用一个固定值（比如 `0.2s`）先跑通。

### 我的 Prediction 发了但云台不动？

**Prediction 不驱动云台。** 云台看的是 `ShotSolution`——它在 `CONTROL` 通道上。

**最小可跑 Tracker 那一段已经示范了怎么发 `ShotSolution`。** 检查：

1. 你的 tracker 里有没有 `public_bus().publish(PublicMessages.CONTROL, solution)`？
2. `solution.timestamp` 和这一帧的 `obs.timestamp` 一致吗？
3. 电控的 `reason` 字段是什么？—— 在信息面板看 `reason`。

### 我的 `fire=True` 但子弹不出膛？

按顺序检查：

1. **用户开关**：KP9（`auto_aiming`）、KP8（`auto_fire`）都开了吗？
2. **跟踪误差**：电控有 `track_tol_deg`（默认 0.3°）——云台当前指向和 `solution.yaw` / `solution.pitch` 的差小于它才发。
3. **冷却**：距上一发超过 `cooldown`（0.1s）了吗？
4. **单发判据**：`fire_mode="single"` 时——如果 `fire_key` 非 None（符），看 `fire_key` 是否变；如果为 None（车虚拟点），看 `fire` 是否 0→1。

这些 `reason` 都能在信息面板或日志里看到。

## 八、一句话

**Tracker 是"一帧进一帧出"的函数。**

**从最简单的一行代码开始，按需升级——每一次升级解决一类具体问题，每一次都是一份能跑的 tracker。**

**不要试图一次写完"完整的"自瞄。** 从静止靶开始，一步步加。