# 接入 UE 工程（3 处改动）

## 1) WLRefereeSystem.h —— 持有裁判核心

```cpp
#include "ArenaReferee.h"
#include "ArenaStructures.h"

// 在 WLRefereeSystem 类里加：
ArenaReferee2027::FArenaMatch ArenaMatch;

// 初始化（BeginPlay 或 对局开始）：
ArenaMatch.RegisterBase(0);
ArenaMatch.RegisterBase(1);
ArenaMatch.RegisterOutpost(0);
ArenaMatch.RegisterOutpost(1);
// 占点区（坐标来自 rules.json -> maps.rmuc2027.zones）：
// ArenaMatch.Zones.push_back(FZoneState{ "center_high", EZoneKind::HighGround, x, y, r });
```

## 2) WLVehiclePawn（或 WLRefereeSystem::Tick）—— 每帧推进

```cpp
ArenaMatch.TickMatch(DeltaTime);
```

## 3) 开火/受击事件 —— 把物理结果喂给裁判

```cpp
// 开火：热量 + 弹道判定
if (!RobotRef.ApplyShotHeat(ShellMm, HeatPerShot)) { /* 锁火 */ }

// 命中：伤害（17mm=20 / 42mm=200 / 弱点 x1.5）
const float Dmg = RobotRef.ApplyDamage(NowS, ShellMm, SpeedMps, bWeakspot);

// 命中基地/前哨：
ArenaMatch.DamageStructure(ArenaMatch.TeamBaseIndex[EnemyTeam], Dmg);

// 占点：每帧把区域内双方人数送进去
ArenaMatch.SetZoneOccupancy(ZoneIdx, N0, N1, DeltaTime);
```

## 说明

- 两个头文件**只依赖标准 C++**，可直接被 UE 模块 include（不需要 UCLASS/UPROPERTY）
- 若要暴露给蓝图，再包一层 UCLASS 包装器
- 数值常量在 Rules 命名空间，改规则只改那里
