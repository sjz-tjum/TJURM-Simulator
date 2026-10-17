# 裁判系统（UE C++）

从 UE4.27 工程 `RM2019Sim` 抽出的裁判/对局源码。

| 文件 | 内容 |
|---|---|
| `WLRefereeSystem.h/.cpp` | 血量 / 热量 / 电容 / 弹量 的存储与恢复、击打结算、对局规则 |
| `WLPVPArena.h/.cpp` | PVP 对局：出生点、复活、胜负判定、场地规则 |
| `WLAimNet.h/.cpp` | 视觉链路：UDP 41001–41004 收发、帧/状态包编码解码 |

## 依赖

- UE 4.27
- 工程内其他模块：`WLVehiclePawn`（车辆/云台/发射）、`WLGimbal`、`WLPellet`

## 待办（TODO）

- [ ] 移植 Arena-1（Rust）的裁判规则：RMUC 2027/2026 规则集、控制点判定、科技等级、哨兵姿态
- [ ] 参考 Arena-1 的地图/场地资源
