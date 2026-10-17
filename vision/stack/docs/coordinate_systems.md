# 坐标系系统

> 自瞄算法面对的第一类坑不是数学，是坐标系。这篇文档说清平台里有哪些坐标系、它们的关系、以及你写代码时会遇到哪些。

## 一、总览

系统里有这些坐标系：

| 名字                | 定义 | 原点 | 方向 |
|-------------------|---|---|---|
| `world`  (W)      | 世界系 | 场地原点 | 绝对惯性 |
| `gimbal` (G)      | 云台系 | 云台中心（yaw 轴上） | 绝对惯性 |
| `camera` (C1)     | 相机系 | 相机光心 | 相机自身朝向（FLU 轴序） |
| `camera` (Crdf)   | 相机光学系 | 相机光心 | 相机自身朝向（RDF 轴序） |
| `muzzle` (M)      | 枪口系 | 枪口 | 枪口自身朝向 |
| `pixel` (Pix)     | 像素坐标 | 图像左上 | — |
| `normalized` (N)  | 归一化像平面 | 主点 | 随表达轴序定（RDF 或 FLU） |
| `abs_camera` (C') | 绝对相机系 | 相机光心 | 绝对惯性 |
| `abs_armor` (A')  | 绝对板系 | 板中心 | 绝对惯性 |
| `armor`   (A)     | 装甲板系 | 板中心 | 板面法线为 x 轴 |

**C1 和 Crdf 不是两个系**——它们是同一个相机系的两种轴序表达，见第六节。

## 二、核心决定：G 系就是云台系

**G 系 = 云台坐标系。原点在云台中心（yaw 轴上），方向与 world 一致（绝对惯性方向）。**

这条决定否决了三个常见理解：

- **G 系不是底盘系。** 底盘不构成一个坐标系。
- **G 系不跟随车头方向。** 它是惯性系，方向永远与 world 一致。跟随车头的是"yaw 后系"（云台内部的中间系，不对外暴露）。
- **G 系原点在云台中心，不是底盘中心。** 所有 G 系相对位置都以云台中心为零点。

**为什么 G 是云台不是底盘？**

云台承载的坐标系转换职能比底盘多——相机挂在云台上、枪口挂在云台上、弹道解算在云台系里做。底盘只决定"云台在世界里的哪个位置"。

**G 系和底盘的关系**：云台跟着底盘平移、但旋转独立。底盘只在运动学上决定"云台在世界的哪里"，它自己不是坐标系。

## 三、坐标链

**坐标系之间的关系只在链上相邻节点之间成立。**

```
物理链（由机械结构、电机角、相机投影决定）：

    world
      ↕  平移（云台在世界里的位置）
    gimbal
      ↕  yaw / pitch / pitch_offset / mount 链（电机角 + 结构常数）
    camera ─── muzzle

world ↔ camera（用 camera 的绝对位姿一步到位，与 world ↔ gimbal 同构）：

    world ↔ camera（FLU）

相机几何（由相机内参决定）：

    camera ↔ pixel ↔ normalized

PnP 数学链（像素 ↔ C' 的换算归单板求解器，见 custom/components/armor_solvers/）：

    abs_camera ↔ gimbal（方向一致，只差平移）
    abs_camera ↔ abs_armor（A'，方向与 world 一致、原点在板中心）
    abs_armor ↔ armor
```

**没有"任意两个坐标系之间算变换"这回事。** 只有链上相邻节点之间的变换。

**每一段对应现实中一个能拿到的量：**

- `world ↔ gimbal`：运动模块维护的 `gimbal.world_pos`
- `world ↔ camera`：相机自己的 `world_pos` / `world_rpy`（一步到位，不经过 gimbal）
- `gimbal ↔ camera`：电机 yaw / pitch 角 + 机械结构常数
- `gimbal ↔ muzzle`：同上
- `camera ↔ pixel`：相机内参 K
- `pixel ↔ C'`：单板求解器内部处理（解出 C' 位姿后由平台做 C' → G 平移）
- `abs_camera ↔ gimbal`：相机光心在 gimbal 下的位置
- `abs_camera ↔ abs_armor`：板中心在 C' 下的位置
- `abs_armor ↔ armor`：板的安装俯仰和绝对偏航

## 四、写 Tracker 时会遇到哪些

**你写 tracker 时，接触的坐标系只有两个：G 和 W。**

`cfg.sim.frame` 决定你收到的 `Observation.rel_pos` 用哪个：

- **`frame = "G"`（默认）** —— `rel_pos` 相对**云台中心**。
- **`frame = "W"`** —— `rel_pos` 是绝对世界坐标。

**`rel_rpy` 两系相同** —— 都是绝对系下的朝向（因为 G 系方向与世界一致）。

### 4.1 做角度计算时的常见错误

假设你要算"从云台看向目标"的 yaw：

```python
# ❌ 错：G 系下还减 gimbal.pos
d = armor.rel_pos - self_state.gimbal.pos

# ✓ 对：G 系下 rel_pos 已是相对量，直接用
d = armor.rel_pos
```

```python
# ✓ 对：W 系下要先减到云台
d = armor.rel_pos - self_state.gimbal.pos
```

**判断方法**：看 `cfg.sim.frame`。G 系用相对量，W 系先减再算。

### 4.2 云台指向的正方向约定

- **yaw**：绕 Z 轴，从 +X 转向 +Y 为正。
- **pitch**：**正值为低头**。

你在 `ShotSolution` 里填的 `pitch` 用的是**抬头为正**——和云台 `set_target` 的约定相反，下发时取负：

```python
# 计算时用"抬头为正"
pitch_up = math.atan2(dz, hypot(dx, dy))     # 抬头为正

# ShotSolution 里也填抬头为正 —— 电控下发时会取负
```

`sim/managers/control_manager.py` 负责取负这一步——你只管发抬头为正的 `pitch`。

### 4.3 `total_delay` 与前推

你要在 `Prediction.armors[0]`（和 `ShotSolution.yaw/pitch` 用的瞄点）里填"未来时刻"的位置：

```python
aim_point = current_pos + velocity * total_delay
```

`total_delay` 是"从扣扳机到子弹命中"的总时间（弹道飞行 + 系统延迟）。它由弹道解算给出。简单起见，你可以先写死 `0.2s`。

### 4.4 云台跟随的是"枪口"

`aim_rate` 和开火门限都是相对**枪口**算的，不是相对云台中心。因为枪口和云台中心有一个安装偏置（配置里的 `muzzle_mount_pos`）。

**如果你不做提前量**，`aim_rate` 不填也能跑——电控没有前馈就是纯 PID 跟踪，会滞后一点。

## 五、写 PoseSolver 时会遇到哪些

`PoseSolver.solve(frame, self_state)` 里涉及整条 PnP 数学链。

### 5.1 相机内参 K

`self_state.camera.K` 是一个 3×3 矩阵，但它的形式是：

```
K = [[f, 0, 0],
     [0, f, 0],
     [0, 0, 1]]
```

**主点 (cx, cy) = (0, 0)**，即图像**中心**是原点，不是左上角。这是项目的约定——相机投影的像素坐标以图像中心为基准。

你的 PnP 求解器必须接受这个约定。

### 5.2 相机姿态

`self_state.camera.rpy` 是相机在**世界系**下的姿态。换成旋转矩阵：

```python
R_camera_to_abs_camera = rpy_to_R(camera.rpy)
```

这个矩阵把**相机系下的方向**转到**C' 系（绝对相机系）**。

**为什么需要 C'？** 因为板在 A' 里的姿态可以写成 `Rz(psi) · Ry(theta_0)`——这个简单形式**只在绝对方向系里成立**。在相机系里，板的姿态会混入云台的 yaw/pitch。

**C' 与 C1（相机系）同原点、只差姿态。** 像素 ↔ C' 的换算归单板求解器内部处理，不经过 transform 层。

### 5.3 PnP 的输入和输出

输入：

- `image_points_2d`：四个像素点（LT / RT / RB / LB）
- `K`：内参
- `rot_camera_to_abs_camera`：相机绝对姿态（`R^{C'}_{C1}`）
- `mount_pitch`：板的安装俯仰（平地假设下 = 装配倾角）

输出：

- `pos_C_prime`：板中心在 **C' 系**下的位置
- `rpy_C_prime`：板在 **C' 系**下的欧拉角

**注意：出口是 C'，不是相机系。** 平台后续会做 C' → G 的平移。

### 5.4 增强 PnP 需要的额外输入

如果你想做"用关节角增强"的 PnP（比如利用板在绝对系下的安装角度先验），你需要：

- `rot_camera_to_abs_camera`：即 `M = R^{C'}_{C1}`，直接从 `rpy_to_R(camera.rpy)` 拿
- `mount_pitch`：即 `theta_0`，从 `cfg.robot.armor_tilt` / `cfg.outpost.armor_tilt` 拿（平地假设）

**注意**：`mount_pitch` 的语义是"板面在绝对系下的安装俯仰"，**不是**自车 pitch。原实现里加了 `self_state.chassis.rpy[1]`，那是错的——它把自车姿态混进去了。

## 六、FLU / RDF 轴序

相机系内部有两个轴序，它们**不是坐标系**，只是同一坐标系里的两种轴序表达：

- **FLU**（前-左-上）：相机系的默认表达。`x` 朝前，`y` 朝左，`z` 朝上。
- **RDF**（右-下-前）：**光学系**的表达。`x` 朝右，`y` 朝下，`z` 朝前（深度方向）。

两者之间的旋转由 `core.algorithms.math.transform` 承担，分两个维度层级：

**三维向量**——`vec3d_FLU2RDF` / `vec3d_RDF2FLU`：

```python
from core.algorithms.math.transform import (
    vec3d_FLU2RDF, vec3d_RDF2FLU, FLU2RDF, RDF2FLU,
)

# FLU 的 3D 向量转 RDF
v_rdf = vec3d_FLU2RDF(v_flu)

# 或者用矩阵
v_rdf = FLU2RDF() @ v_flu
```

**归一化坐标**——`vec2d_FLU2RDF` / `vec2d_RDF2FLU`：

```python
from core.algorithms.math.transform import vec2d_FLU2RDF, vec2d_RDF2FLU

# RDF 归一化坐标 -> FLU 归一化坐标
n_flu = vec2d_RDF2FLU(n_rdf)
```

**`vec2d_` 派生自 `vec3d_`**——它内部先补深度把归一化坐标升成射线，用 `vec3d_` 换轴序，再除回深度。**轴序公式只在 `vec3d_` 一处**。

**你写 PnP 时会遇到**：像素坐标是 RDF 的，板角点在世界系是 FLU 的。换轴序就是换轴序，不需要把它当成"换坐标系"。

## 七、transform.py 提供的函数

`core/algorithms/math/transform.py` 是坐标系变换的唯一入口。它按"链上每一段"提供函数：

```python
# 通用数学
rpy_to_R(rpy)                    # 欧拉角 -> 旋转矩阵
R_to_rpy(R)                      # 旋转矩阵 -> 欧拉角
rpyt_to_T(rpy, t)                # 位姿 -> 4x4 齐次变换

# 轴序（3D 向量 / 归一化坐标）
vec3d_FLU2RDF(v) / vec3d_RDF2FLU(v)
vec2d_FLU2RDF(v) / vec2d_RDF2FLU(v)
FLU2RDF() / RDF2FLU()            # 矩阵形式

# world ↔ gimbal
world_to_gimbal(pos, R, *, gimbal_pos_world)
gimbal_to_world(pos, R, *, gimbal_pos_world)

# world ↔ camera（用 camera 的绝对位姿一步到位）
world_to_camera_FLU(pos, R, *, camera_pos_world, camera_rpy_world)
camera_FLU_to_world(pos, R, *, camera_pos_world, camera_rpy_world)

# gimbal ↔ camera（需要电机角 + 机械常数）
gimbal_to_camera(pos, R, *, yaw, pitch, pitch_offset,
                 camera_mount_pos, camera_mount_rpy)
camera_to_gimbal(...)

# gimbal ↔ muzzle
gimbal_to_muzzle(...)
muzzle_to_gimbal(...)

# camera ↔ normalized（3D 点与归一化坐标，都在 RDF）
camera_RDF_to_normalized(pos)

# pixel ↔ normalized（RDF）
pixel_to_normalized_RDF(pixel, *, K)
normalized_RDF_to_pixel(normalized, *, K)

# C' ↔ gimbal
abs_camera_to_gimbal(pos, R, *, camera_pos_gimbal)
gimbal_to_abs_camera(pos, R, *, camera_pos_gimbal)

# C' ↔ A'
abs_camera_to_abs_armor(pos, R, *, armor_pos_abs_camera)
abs_armor_to_abs_camera(pos, R, *, armor_pos_abs_camera)

# A' ↔ armor
armor_to_abs_armor(pos, R, *, psi, theta_0)
abs_armor_to_armor(pos, R, *, psi, theta_0)
```

**命名约定**：`<from>_to_<to>(pos, R, *, ...) -> (pos', R')`。参数名带坐标系后缀（`gimbal_pos_world` 是"云台在世界下的位置"）。**涉及 camera 系或归一化的函数，名字里带 FLU / RDF 说清轴序**。

**不提供复合变换。** 你要 `world → camera_RDF`，自己拼 `vec3d_FLU2RDF(world_to_camera_FLU(...))`。没有一步到位的 `world_to_camera_RDF`——两条路径会引入浮点差异。

**像素 ↔ C' 的换算也不在这里**——它归单板求解器（`custom/components/armor_solvers/`），因为"要不要换轴序"、"要不要造虚拟像素"是各求解器自己的方案选择。

## 八、底盘、云台、相机、枪口的关系

物理上是一条两级关节链：

```
底盘（位置参考，不是坐标系）
  │  平移 gimbal_mount_pos（结构常数）
  ▼
G 系（云台系，原点在 yaw 轴上）
  │  绕 z 轴旋转 yaw（电机角）
  ▼
yaw 后系（中间系，不暴露）
  │  平移 pitch_offset
  ▼
pitch 轴交点
  │  绕 y 轴旋转 pitch（电机角）
  ▼
pitch 后系
  │
  ├── 平移 camera_mount_pos + 旋转 camera_mount_rpy → camera
  └── 平移 muzzle_mount_pos → muzzle
```

**中间节点（yaw 后系、pitch 轴交点、pitch 后系）不对外暴露** —— 它们是 `gimbal_to_camera` 内部的中间量。

**`pitch_offset` 属于 yaw 后系** —— 只受 yaw 影响，不受 pitch 影响。用完整姿态 `R(yaw, pitch)` 转 `pitch_offset` 会让俯仰把相机/枪口带偏（Hero 在 pitch 20° 时约 3.4 cm，45° 时约 7.1 cm）。

**`world ↔ camera` 是一步到位的**：camera 自己知道它在世界里的绝对位姿（`world_pos` / `world_rpy`），所以不需要走 `world → gimbal → camera` 两段。这和 `world_to_gimbal` 同构——不是复合变换。

## 九、常见问题

### 我的观测位置和目标的真实位置差一个量级？

十有八九是混淆了 G 系和 W 系，或者忘了减 `gimbal.pos`。

**检查方法**：

```python
print(f"obs={armor.rel_pos}, gimbal={self_state.gimbal.pos}, frame={cfg.sim.frame}")
```

- `frame="G"`：`armor.rel_pos` 应该在 0.5 米到 20 米量级。
- `frame="W"`：`armor.rel_pos` 的数值可能十几米，`gimbal.pos` 也差不多。

### 我做了一个简单 PnP 但解出的位姿反的？

最常见原因是**相机系内部约定**。

本平台的相机投影是：

```
u = f * x / z,  v = f * y / z
```

其中 `(x, y, z)` 是光轴系下的点（RDF：右-下-前），`z` 是深度。

**这与 OpenCV 的 `cv2.solvePnP` 不同** —— 后者用 x 为深度、y 向下、z 向前。如果你照抄 OpenCV 的实现，板的位置会被解到"相机后面"。

参考 `custom/components/armor_solvers/ippe.py` 里的 `RDF2FLU` 用法——它把 PnP 求解出的 RDF 结果换成 FLU 输出。

### `_mount_pitch` 到底是自车的 pitch 还是目标的 pitch？

**目标的**。平地假设下目标的 pitch = 0，所以 `theta_p = 装配倾角`。

自车的 pitch 只影响"相机看向哪里"，不影响"板装在哪里"。原实现里把自车 pitch 加进去是错的——平地场景下自车 pitch ≈ 0 所以碰巧能跑，一旦自车在坡上就会偏。

### 我能不能不用 transform.py，自己写变换？

**技术上可以，但你会在坐标系约定上翻车。** transform.py 里的每一个函数都对应链上的一段，它们的约定（FLU / RDF、pitch 正方向、C' 与 G 的方向一致）是自洽的。自己写一份等价的很可能会漏掉某个符号。

**如果确实要自己写**：至少 import 它提供的 `rpy_to_R` 和 `vec3d_FLU2RDF` 作为基础。

## 十、一句话

**坐标系是链状结构，不是任意转换。**

**G 系是云台系，不是底盘系。它的方向与 world 一致，所以 G ↔ W 只差平移。**

**C1 / Crdf 是相机系的两种轴序，不是两个系；C' / A' 是 PnP 的数学构造，只在 PnP 内部存在。**

**你写 tracker 时只需要记住一件事：`cfg.sim.frame` 决定 `rel_pos` 用不用减 `gimbal.pos`。**