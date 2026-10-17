# Arena-1（Rust）裁判系统 → UE4.27 C++ 移植方案

上游：<https://github.com/RM-Arena/Arena-1>（Rust 引擎 + Python 策略）
目标：把我们 UE 工程里的裁判/对局（`WLRefereeSystem` / `WLPVPArena`）对齐到 Arena-1 的 RMUC 2027 规则。

## 一、上游代码地图

| Rust 文件 | 大小 | 内容 |
|---|---|---|
| `simulation/src/lib.rs` | 78 KB | 核心数据结构 + `Engine` 主循环：`Spec/Terrain/Zone/Arena/Robot/Base/Team/ZoneState/Projectile/Action` |
| `simulation/src/official.rs` | 112 KB | 官方规则（伤害/热量/功率/复活/经济/经验/超速/判罚） |
| `simulation/src/season2027.rs` | 95 KB | 2027 赛季流程（控制点/科技等级/哨兵姿态/机库/空中） |
| `simulation/src/physics.rs` | 103 KB | 运动/弹道/碰撞物理 |
| `simulation/src/penalties.rs` | 11 KB | 判罚 |
| `simulation/src/pickup.rs` | 6 KB | 搬运/取放 |
| `config/rules.json` | 370 KB | **全部数值参数**（match/damage/heat/power/heal/respawn/economy/experience/overspeed/specs/robot_designs/maps/seasons） |
| `docs/RULES_2027_IMPLEMENTATION.md` | 6 KB | 规则实现说明 |
| `docs/RULES_IMPLEMENTATION_AUDIT.md` | 38 KB | 实现审计（偏差清单） |

## 二、模块映射（Rust → 我们的 C++）

| Rust | C++（本仓库 / UE 工程） | 状态 |
|---|---|---|
| `lib.rs::Robot` + `specs` | `referee/arena/ArenaReferee.h::FRobotReferee` + `FRobotSpec` | ✅ 已移植 |
| `official.rs` 伤害/热量/功率/复活/经验/超速 | 同上：`ApplyDamage / ApplyShotHeat / SettleHeat / SettlePower / TickRespawnProgress / AddXp / ApplyOverspeed17` | ✅ 已移植（核心公式） |
| `official.rs` 经济 | `FRobotReferee::AddCoins/ConsumeCoins`、`FArenaReferee::PeriodCoins` | ✅ 已移植 |
| `lib.rs::Engine` 时间轴 | `FArenaReferee::Tick`（Prep→SelfCheck→Countdown→Running→Finished） | ✅ 基础版 |
| `season2027.rs` 控制点/科技/哨兵 | `FArenaReferee::FTeamState`（ControlPercent/TechLevel） | 🚧 骨架 |
| `season2027.rs` 基地/前哨结构 | 待建 `FStructureState`（血量/控制度/伤害结算） | ⬜ 待做 |
| `lib.rs::Zone`/`ZoneState` | 待建 `FZoneState`（占点/控制度积分） | ⬜ 待做 |
| `penalties.rs` 判罚 | 待建 `FPenaltySystem`（超速/围栏/异常） | 🚧 超速已做 |
| `physics.rs` 弹道 | UE 已有 `WLPellet`（17mm/3.2g/23m/s 二次阻力） | ✅ 用现有的 |
| `pickup.rs` 搬运 | 暂不需要（步兵 17mm 自瞄场景） | ⬜ 可选 |
| `maps`/`terrain_geometry` | 参考 `config/rules.json` 的 `maps` + `terrain_geometry` | ⬜ 地图移植 |

## 三、实施阶段

### 阶段 1（已完成 ✅）
- 规则常量全量落地（`ArenaReferee.h` 顶部 `Rules` 命名空间，数值逐条对应 `rules.json`）
- 单机裁判状态机：伤害（取整/弱点 ×1.5/装甲间隔）、热量（上限锁火/永久附加热）、
  功率（能量/缓冲/断电 cutoff/增强阈值）、复活（普通/瞬时、无敌时长）、经验升级、超速判罚
- 对局时间轴 + 经济（周期金币表）
- 胜负基础判定

### 阶段 2（下一步）
- `FStructureState`：基地 / 前哨 血量与控制点（`official.structures`）
- `FZoneState`：中央高地 / 能量机关龙门 / 制高点 的占点与控制度积分
- 接入 UE：`WLRefereeSystem` 持有 `FArenaReferee`，`WLPVPArena` 每帧 `Tick`

### 阶段 3
- `season2027.rs`：科技等级 1~4（重型 42mm 解锁）、哨兵三姿态、机库/空中机器人
- `penalties.rs`：判罚表
- 地图：按 `rules.json` 的 `maps` / `terrain_geometry` 在 UE 关卡里重建

## 四、与上游的差异（需要拍板）

1. **规格数值**：`rules.json` 的 `specs` 是 Arena-1 的**简化版**（步兵 hp 200 / 弹速 25 m/s / 热量 40），
   而官方手册的装甲板血量、弹速与此不同。我们 UE 仿真的真值（`WLPellet`：17mm、3.2 g、v0=23 m/s、二次阻力）
   更接近实物 ⇒ **比赛规则用 official，物理参数用我们自己的**。
2. **热量/功率**：官方是"热量上限锁火 + 能量缓冲/断电"双系统；Arena-1 简化层只用了热量。移植时两套都保留，
   由开关选择。
3. **`rules.json` 的 `official.heat.permanent_boundary`** 写的是"at_or_above_Q2_per_figure_5_1"（要看手册图 5.1），
   当前实现按"降到 0 解锁"处理，需按手册核对。

## 五、验证

- `simulation/src/*_tests.rs`（上游测试）是最好的对照：`damage_tests.rs` / `mechanism_tests.rs` / `rule_audit_tests.rs`
- 移植后建议同样写一份 `ArenaRefereeTest.cpp` 逐条对照上游测试用例
