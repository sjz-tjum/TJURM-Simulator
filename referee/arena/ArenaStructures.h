// ArenaStructures.h —— 阶段2：基地/前哨/堡垒 + 占点区（控制度）
// 来源：config/rules.json -> official.structures / maps[rmuc2027].zones
//       simulation/src/season2027.rs（基地开放、前哨转子时序）
//       simulation/src/lib.rs（Zone / ZoneState）
//
// 依赖：ArenaReferee.h（规则常量 + 单机裁判状态）
#pragma once

#include "ArenaReferee.h"

#include <string>
#include <vector>

namespace ArenaReferee2027
{

// ============================================================================
// 1) 结构体（基地 / 前哨 / 堡垒）—— official.structures
// ============================================================================
namespace Rules
{
    inline constexpr float kBaseHp = 5000.0f;
    inline constexpr float kOutpostHp = 1500.0f;
    inline constexpr bool  bBaseInvulnerableWhileOutpostAlive = true;
    inline constexpr float kBaseOpensAtHpLte = 2000.0f;
    inline constexpr float kRebuildLossStepHp = 1000.0f;
    inline constexpr float kRebuildHp = 750.0f;
    inline constexpr float kRebuildHoldHeroInfantryS = 10.0f;
    inline constexpr float kRebuildHoldEngineerS = 5.0f;
    inline constexpr float kRebuildDisabledAtS = 300.0f;
    inline constexpr float kOutpostAccelerationS = 5.0f;
    inline constexpr float kOutpostStopAfterS = 180.0f;
    inline constexpr float kOutpostReturnInitialS = 10.0f;
}

enum class EStructureKind : uint8_t { Base, Outpost, Fortress, Supply };

struct FStructureState
{
    EStructureKind Kind = EStructureKind::Base;
    float Hp = 5000.0f;
    float MaxHp = 5000.0f;
    float ShieldHp = 0.0f;          // 护盾（若规则有）
    bool bAlive = true;
    bool bInvulnerable = false;

    // 重建（基地被打掉后的 rebuild 规则）
    bool bRebuilding = false;
    float RebuildHoldS = 0.0f;      // 需要英雄/步兵或工程持续占位的时间
    int32_t RebuildStage = 0;       // 每损失 1000 hp 退一档

    EStructureKind GetKind() const { return Kind; }

    float TakeDamage(float Dmg)
    {
        if (bInvulnerable || !bAlive) return 0.0f;
        if (ShieldHp > 0.0f)
        {
            const float Absorb = std::min(ShieldHp, Dmg);
            ShieldHp -= Absorb;
            Dmg -= Absorb;
        }
        Hp = std::max(0.0f, Hp - Dmg);
        if (Hp <= 0.0f) { Hp = 0.0f; bAlive = false; }
        // 基地每掉 1000 hp 触发一级重建损失（official.rebuild_loss_step_hp）
        if (Kind == EStructureKind::Base)
        {
            RebuildStage = (int32_t)std::floor((MaxHp - Hp) / Rules::kRebuildLossStepHp);
        }
        return Dmg;
    }

    // 基地是否可被攻击（前哨活着时基地无敌）
    void UpdateInvulnerability(bool bEnemyOutpostAlive)
    {
        if (Kind != EStructureKind::Base) { bInvulnerable = false; return; }
        if (Rules::bBaseInvulnerableWhileOutpostAlive && bEnemyOutpostAlive)
        {
            bInvulnerable = true;
        }
        else
        {
            bInvulnerable = (Hp > Rules::kBaseOpensAtHpLte);   // 只有血量 <=2000 才允许继续打
        }
    }

    // 重建进度（需要持续占位）
    void TickRebuild(float Dt, bool bHeldByHeroInfantry, bool bHeldByEngineer)
    {
        if (!bRebuilding) return;
        const float Need = (bHeldByEngineer && !bHeldByHeroInfantry)
            ? Rules::kRebuildHoldEngineerS : Rules::kRebuildHoldHeroInfantryS;
        if (bHeldByHeroInfantry || bHeldByEngineer) RebuildHoldS += Dt;
        if (RebuildHoldS >= Need)
        {
            RebuildHoldS = 0.0f;
            bRebuilding = false;
            Hp = std::min(MaxHp, Hp + Rules::kRebuildHp);
            bAlive = Hp > 0.0f;
        }
    }
};

// ============================================================================
// 2) 占点区（中央高地 / 能量机关龙门 / 制高点 / 基地区）—— maps.zones + lib.rs::Zone
// ============================================================================
enum class EZoneKind : uint8_t { Base, Fortress, ControlPoint, Rune, HighGround, Custom };

struct FZoneState
{
    std::string Id;
    EZoneKind Kind = EZoneKind::ControlPoint;
    float X = 0.0f, Y = 0.0f, Radius = 1.0f;
    int32_t OwnerTeam = -1;            // -1 中立
    float ControlPercent = 0.0f;       // 0..100 控制度
    float CaptureRatePerS = 25.0f;     // 占领速度（%/s，可调）
    int32_t OccupyingTeam = -1;
    bool bContested = false;

    // 每个 tick：n0/n1 = 双方在该区域内的机器人数
    void Tick(float Dt, int32_t n0, int32_t n1)
    {
        bContested = (n0 > 0 && n1 > 0);
        if (bContested) return;                    // 双方都有人 ⇒ 不推进
        const int32_t Team = (n0 > 0) ? 0 : (n1 > 0 ? 1 : -1);
        if (Team < 0) return;

        if (OwnerTeam >= 0 && OwnerTeam != Team)
        {
            // 先中和
            ControlPercent = std::max(0.0f, ControlPercent - CaptureRatePerS * Dt);
            if (ControlPercent <= 0.0f) { OwnerTeam = -1; }
        }
        else
        {
            ControlPercent = std::min(100.0f, ControlPercent + CaptureRatePerS * Dt);
            if (ControlPercent >= 100.0f)
            {
                OwnerTeam = Team;
                ControlPercent = 100.0f;
            }
        }
    }
};

// ============================================================================
// 3) 对局扩展（结构 + 占点区） 继承阶段1的 FArenaReferee
// ============================================================================
struct FArenaMatch : public FArenaReferee
{
    std::vector<FStructureState> Structures;   // [0..] 基地/前哨/堡垒
    std::vector<FZoneState>      Zones;

    // 队伍结构索引（按规则：每队 1 基地 + 1 前哨 + 堡垒若干）
    int32_t TeamBaseIndex[2] = {-1, -1};
    int32_t TeamOutpostIndex[2] = {-1, -1};

    void RegisterBase(int32_t Team, float MaxHp = Rules::kBaseHp)
    {
        FStructureState S; S.Kind = EStructureKind::Base; S.Hp = S.MaxHp = MaxHp;
        TeamBaseIndex[Team] = (int32_t)Structures.size();
        Structures.push_back(S);
    }

    void RegisterOutpost(int32_t Team, float MaxHp = Rules::kOutpostHp)
    {
        FStructureState S; S.Kind = EStructureKind::Outpost; S.Hp = S.MaxHp = MaxHp;
        TeamOutpostIndex[Team] = (int32_t)Structures.size();
        Structures.push_back(S);
    }

    bool IsOutpostAlive(int32_t Team) const
    {
        const int32_t I = TeamOutpostIndex[Team];
        return I >= 0 && Structures[I].bAlive;
    }

    // 伤害入口：目标结构
    float DamageStructure(int32_t Index, float Dmg)
    {
        if (Index < 0 || Index >= (int32_t)Structures.size()) return 0.0f;
        return Structures[Index].TakeDamage(Dmg);
    }

    void TickMatch(float Dt)
    {
        FArenaReferee::Tick(Dt);

        for (int32_t T = 0; T < 2; ++T)
        {
            // 基地无敌：对方前哨还活着 ⇒ 基地无敌；否则血量 <=2000 后可打
            const int32_t BI = TeamBaseIndex[T];
            if (BI >= 0)
            {
                Structures[BI].UpdateInvulnerability(IsOutpostAlive(1 - T));
            }
        }

        // 占点区控制度 ⇒ 队伍 ControlPercent
        for (auto& Z : Zones)
        {
            if (Z.Kind != EZoneKind::ControlPoint && Z.Kind != EZoneKind::HighGround) continue;
            Z.Tick(Dt, 0, 0);   // 具体人数由 UE 侧每帧 SetZoneOccupancy 传入
        }
        UpdateControlPercent();
    }

    // UE 侧每帧把"区域内双方机器人数"送进来
    void SetZoneOccupancy(int32_t ZoneIndex, int32_t n0, int32_t n1, float Dt)
    {
        if (ZoneIndex < 0 || ZoneIndex >= (int32_t)Zones.size()) return;
        Zones[ZoneIndex].Tick(Dt, n0, n1);
    }

    void UpdateControlPercent()
    {
        for (int32_t T = 0; T < 2; ++T)
        {
            int32_t N = 0, Owned = 0;
            for (const auto& Z : Zones)
            {
                if (Z.Kind != EZoneKind::ControlPoint && Z.Kind != EZoneKind::HighGround) continue;
                ++N;
                if (Z.OwnerTeam == T) ++Owned;
            }
            Teams[T].ControlPercent = (N > 0) ? (100.0f * Owned / N) : 0.0f;
        }
    }

    // 胜负（覆盖阶段1）：基地被摧毁 > 控制度 > 比分
    int32_t Winner() const
    {
        const bool bAlive0 = (TeamBaseIndex[0] >= 0) ? Structures[TeamBaseIndex[0]].bAlive : true;
        const bool bAlive1 = (TeamBaseIndex[1] >= 0) ? Structures[TeamBaseIndex[1]].bAlive : true;
        if (bAlive0 != bAlive1) return bAlive0 ? 0 : 1;
        if (Teams[0].ControlPercent != Teams[1].ControlPercent)
            return Teams[0].ControlPercent > Teams[1].ControlPercent ? 0 : 1;
        return Teams[0].Score >= Teams[1].Score ? 0 : 1;
    }
};

} // namespace ArenaReferee2027
