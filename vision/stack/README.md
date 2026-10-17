# PlugYourRM

> 可插拔的 RoboMaster 自瞄跟踪器仿真系统 —— 给社团新生学自瞄用。

## 这是什么

一个**仿真平台**，把 RM 自瞄系统里三层核心算法拆成可插拔的组件，让新生在不接触真车、不调试真实相机的情况下，把自瞄的每一个环节都亲手写一遍：

- **位姿解算** —— 从四个像素点解出装甲板的三维位姿（PnP）
- **跟踪器** —— 把一帧帧离散的观测串成对目标的连续估计（EKF / 多目标关联 / 选板）
- **弹道解算** —— 从打击点到云台角度与飞行时间

平台负责物理、传感器、云台伺服、渲染；你只写算法。

## 它不是什么

- **不是完整比赛系统**。没有裁判系统、没有通信协议、没有真车接口。
- **不是"帮你写好自瞄"**。默认 tracker 只是一个能跑的 demo，从 EKF 到选板都可以换掉。
- **不是纯视觉项目**。相机是仿真的——你看到的是完美的针孔投影 + 像素量化 + 可选噪声，不用标定、不用对齐、不用处理真实世界的脏数据。

## 快速开始

### 环境

Python 3.10+，依赖 `numpy` / `pyyaml` / `scipy` / `pygame`：

```bash
pip install -e .
```

### 运行

```bash
python main.py
```

### 你会看到

1. **选择界面**：两列 —— `Tracker`（方案）和 `PoseSolver`（位姿解算）。按 `Enter` 逐列确认，进入游戏。
2. **仿真窗口**：左侧俯视图（跟随受控车），右侧相机视图 + 信息面板。
3. **自瞄**：
   - `KP9` —— 开/关自动瞄准（云台自动指向目标）
   - `KP8` —— 开/关自动开火
   - `R` —— 复位

### 常用按键

| 键 | 功能 |
|---|---|
| `WASD` | 底盘平移（场地轴） |
| `A/D` | 底盘慢转，`Z/C` 快转 |
| `Q/E` | 云台偏航，`W/X` 俯仰（自瞄关闭时） |
| `Enter + 1/2/3/4` | 创建 Hero / Infantry / Sentry / Engineer |
| `Enter + KP1/KP2` | 创建 Outpost / Rune |
| `Backspace + 数字` | 删除对应实体 |
| `KP9` / `KP8` | 自动瞄准 / 自动开火 |
| `KP0` | 手动连发（不受自瞄约束，试弹道用） |
| `M` / `KP4` | 曲线面板开关 / 切换预设 |
| `R` / `Space` | 复位 / 急停 |
| `ESC` | 退出当前局，回到选择界面 |

## 系统结构

三层，加一个装配层：

```
core/       契约 + 数学。不认识 sim / custom。
  bus/        消息定义 + 发布订阅
  interfaces/ Tracker / PoseSolver 基类
  entities/   领域实体（车、云台、装甲板、弹丸）
  algorithms/ 纯数学 + 无方案假设的算法

sim/        世界。物理、传感器、电控、渲染、键盘。
  comm.py     sim 侧通信模块
  simulator.py
  managers/   各子系统
  render/     渲染层

custom/     方案 + 组件。你写代码的地方。
  schemes/    完整 MOT 方案（Tracker + PoseSolver 实现）
  components/ 可复用的库（选板、弹道、PnP 算法）
  pipeline.py custom 内编排
  comm.py     custom 侧通信模块

entrance/   装配。唯一同时认识 sim 和 custom 的地方。
```

**依赖方向**：`core` 不认识 `sim` / `custom`；`sim` 与 `custom` **互相不认识**——两边靠 `core.bus` 的消息和 `core.interfaces` 的接口相遇。

**数据流**（一帧）：

```
sensor 出检测 ──→ SimComm ──→ [public_bus] ──→ CustomComm
                                                    │
                                                    ↓
                                              pipeline.tick
                                                    │
                              ┌─────────────────────┼─────────────────────┐
                              ↓                     ↓                     ↓
                        PoseSolver.solve     Tracker.step          （可选）Trajectory
                              ↓                     ↓
                        Observation           Prediction
                              └─────────────────────┤
                                                    ↓
                                              [public_bus.pred/*]
                                                    │
                              ┌─────────────────────┴─────────────────────┐
                              ↓                                           ↓
                        RenderManager                              ControlManager
                        （画预测点）                                （驱动云台 + 开火）
```

## 写你自己的自瞄

三条扩展路径，按难度排：

### 1. 换一个 PnP 算法

在 `custom/components/armor_solvers/` 下新开一个模块（照抄 `ippe.py` / `qcqp.py` / `diff.py` 中最近的一个），实现

```python
estimate(*, camera_k, armor_size, image_points_2d,
         rot_camera_to_abs_camera, mount_pitch=0.0) -> (pos, rpy) | (None, None)
```

再在 `__init__.py` 的 `_SOLVERS` 和 `get_solver` 里登记。没有抽象基类。

### 2. 换一个 Tracker

复制 `custom/schemes/tjurm_scheme/` 到一个新目录，改 `tracker.py`，用 `@register_tracker("my_name")` 注册。启动界面就会多一列选项。

### 3. 从零写一个最小 Tracker

不看默认实现，只按 `core/interfaces/tracker.py` 的契约走：

```python
from core.interfaces.tracker import Tracker, register_tracker
from core.bus.dataflow import Prediction

@register_tracker("my_first")
class MyFirstTracker(Tracker):
    description = "我的第一个跟踪器"

    def step(self, obs, self_state):
        if obs is None or not obs.obs_armors:
            return []
        # 就拿第一块观测板，直接当作预测点
        armor = obs.obs_armors[0]
        return [Prediction(
            target_type=...,      # 按 entity_type 分类
            center=armor.rel_pos,
            armors=[armor.rel_pos],
            timestamp=obs.timestamp,
            is_tracking=True,
        )]
```

**一帧观测进，一帧预测出**——这是 Tracker 的全部契约。

详细的写作指南见 `docs/writing_your_tracker.md`。

## 项目结构

```
PlugYourRM/
├── main.py
├── pyproject.toml
├── README.md
├── docs/                     # 全部文档
├── config/                   # data.yaml + 生成物
│   ├── data/data.yaml
│   ├── generator.py
│   ├── loader.py
│   ├── reader.py
│   └── schema.py             # 生成物，勿手改
├── core/                     # 契约 + 数学
│   ├── bus/
│   ├── interfaces/
│   ├── entities/
│   └── algorithms/
├── sim/                      # 世界
│   ├── comm.py
│   ├── simulator.py
│   ├── managers/
│   └── render/
├── custom/                   # 方案 + 组件（你写这里）
│   ├── schemes/
│   │   └── tjurm_scheme/
│   ├── components/
│   │   ├── armor_solvers/
│   │   ├── armor_selectors/
│   │   ├── traj_solvers/
│   │   └── shooters/
│   ├── pipeline.py
│   └── comm.py
└── entrance/                 # 装配
```

## 文档

| 文档 | 内容 |
|---|---|
| `docs/architecture.md` | 分层、边界、稳定 vs 变化 |
| `docs/writing_your_tracker.md` | 从零写一个 Tracker |
| `docs/coordinate_systems.md` | 坐标系系统 |
| `docs/bus_and_messages.md` | 总线、消息契约、异步模型 |

## 已知限制

诚实列出：

- **弹道解算是默认实现**，用了带阻力的数值迭代，不是解析解。理论上可换。
- **`multi_target` 是唯一的完整方案**。写第二个方案时才能看出"schemes / components 分离"是否真的够用。
- **MPC 默认关闭**。`custom/schemes/tjurm_scheme/mpc/` 里的代码保留着，但 `enable_mpc=False`——学生想用要自己打开并调试。
- **能量机关（Rune）的解算没有做**。传感器给靶心位置，`RuneModel` 只用位置不用姿态——十环靶没有板面方向可解，这是设计选择。
- **没有单元测试**。回归靠手动跑仿真。这是最应该补的一块。
- **错误可见性有限**。`EventBus.publish` 的回调异常打到 stderr，不显示在游戏里——游戏里看到"卡住但没报错"时，记得看终端。

## 许可 / 反馈

社团内部使用。问题直接找作者。

---

**下一步**：读 `docs/architecture.md`，然后 `docs/writing_your_tracker.md`，然后打开 `custom/schemes/tjurm_scheme/tracker.py` 看看默认实现。