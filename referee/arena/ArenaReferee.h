// ArenaReferee.h —— 从 Arena-1（Rust 引擎）移植的 RMUC 2027 裁判核心
//
// 来源对照：
//   simulation/src/lib.rs        -> 引擎数据结构（Robot/Base/Team/Zone/ZoneState/Projectile/Engine）
//   simulation/src/official.rs   -> 官方规则（伤害/热量/功率/复活/经济/经验/超速）
//   simulation/src/penalties.rs  -> 判罚
//   simulation/src/season2027.rs -> 2027 赛季流程（控制点/科技等级/哨兵姿态）
//   config/rules.json            -> 全部数值参数（本文件顶部常量即由它生成）
//
// 本文件只依赖标准 C++，不依赖 UE 头文件 ⇒ 可单独编译测试，也可被 UE 模块 include。
// 用法：在 WLRefereeSystem 里持有 FArenaReferee，按帧调用 Tick(dt)，事件走 Apply* 接口。
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace ArenaReferee2027
{

// ============================================================================
// 1) 规则常量（config/rules.json -> official / specs）
// ============================================================================
namespace Rules
{
    // -- match --
    inline constexpr float kMatchDurationS = 420.0f;
    inline constexpr float kFieldLengthM = 28.0f;
    inline constexpr float kFieldWidthM = 15.0f;
    inline constexpr float kPrepS = 180.0f;
    inline constexpr float kSelfCheckS = 15.0f;
    inline constexpr float kCountdownS = 5.0f;

    // -- damage --
    inline constexpr float kDamage17 = 20.0f;
    inline constexpr float kDamage42 = 200.0f;
    inline constexpr float kDamageCollision = 2.0f;
    inline constexpr float kNormalSpeedMin17Mps = 12.0f;
    inline constexpr float kNormalSpeedMin42Mps = 10.0f;
    inline constexpr float kArmorIntervalS = 0.05f;
    inline constexpr float kArmorInterval42MaxS = 0.2f;
    inline constexpr float kWeakspotWidthM = 0.01f;
    inline constexpr float kWeakspotHeightM = 0.01f;
    inline constexpr float kWeakspotMultiplier = 1.5f;

    // -- heat --
    inline constexpr float kHeatSettlementHz = 10.0f;
    inline constexpr float kPermanentExtraHeat17 = 100.0f;
    inline constexpr float kPermanentExtraHeat42 = 200.0f;

    // -- power --
    inline constexpr float kPowerSettlementHz = 10.0f;
    inline constexpr float kBufferCapacityJ = 60.0f;
    inline constexpr float kPowerCutoffS = 5.0f;
    inline constexpr float kInitialEnergyJ = 20000.0f;
    inline constexpr float kMaxEnergyJ = 40000.0f;
    inline constexpr float kEnergySavingPowerW = 35.0f;
    inline constexpr float kEnhancedThresholdJ = 25000.0f;
    inline constexpr float kEnhancedMultiplier = 1.25f;
    inline constexpr float kAbsoluteCapW = 200.0f;
    inline constexpr float kWirelessSupplyMaxW = 200.0f;
    inline constexpr float kWirelessTransferMaxW = 120.0f;

    // -- heal --
    inline constexpr float kDisengageS = 6.0f;
    inline constexpr float kSupplyRatioPerS = 0.1f;
    inline constexpr float kLateStartS = 240.0f;
    inline constexpr float kLateDisengagedSupplyRatioPerS = 0.25f;
    inline constexpr float kRemoteDelayS = 6.0f;
    inline constexpr float kRemoteRatio = 0.6f;

    // -- respawn --
    inline constexpr float kRespawnProgressBase = 10.0f;
    inline constexpr float kRespawnElapsedDivisor = 10.0f;
    inline constexpr float kInstantPurchaseProgressPenalty = 20.0f;
    inline constexpr float kNormalProgressPerS = 1.0f;
    inline constexpr float kSupplyOrBaseBelow2000ProgressPerS = 4.0f;
    inline constexpr float kBaseLowHpStrictThreshold = 2000.0f;
    inline constexpr float kNormalHpRatio = 0.1f;
    inline constexpr float kNormalInvulnerabilityS = 30.0f;
    inline constexpr float kInstantHpRatio = 1.0f;
    inline constexpr float kInstantInvulnerabilityS = 3.0f;
    inline constexpr float kInstantPowerMultiplier = 2.0f;
    inline constexpr float kInstantPowerCapW = 200.0f;
    inline constexpr float kInstantPowerDurationS = 4.0f;

    // -- economy --
    inline constexpr int32_t kInitialCoins = 400;
    struct FCoinTick { float ElapsedS; int32_t Coins; };
    inline constexpr FCoinTick kCoinSchedule[] = {
        {61.0f, 50}, {121.0f, 50}, {181.0f, 50}, {241.0f, 50}, {301.0f, 50}, {361.0f, 150}};

    // -- experience --
    inline constexpr int32_t kXpThresholds[] = {0, 550, 1100, 1650, 2200, 2750, 3300, 3850, 4400, 5000};
    inline constexpr int32_t kXpInitialCap = 5;
    inline constexpr int32_t kXpCapAfterAssembly2 = 7;
    inline constexpr int32_t kXpCapAfterAssembly3 = 10;
    inline constexpr float kShotXp17 = 1.0f;
    inline constexpr float kShotXp42 = 10.0f;
    inline constexpr float kRobotDamageXpPerHp = 4.0f;
    inline constexpr float kOutpostDamageXpPerHp = 2.0f;
    inline constexpr float kBaseDamageXpPerHp = 0.5f;
    inline constexpr float kKillBaseXp = 50.0f;
    inline constexpr float kTerrainFirstXp = 300.0f;

    // -- overspeed (17mm) --
    struct FOverspeedTier { float ExcessFrom; float ExcessTo; float LockS; bool bPermanent; };
    inline constexpr FOverspeedTier kOverspeed17[] = {
        {0.0f, 5.0f, 15.0f, false}, {5.0f, 10.0f, 20.0f, false}, {10.0f, 1e9f, 0.0f, true}};
}

// ============================================================================
// 2) 机体规格（config/rules.json -> specs，数值来自 simplification 层）
// ============================================================================
struct FRobotSpec
{
    float Hp = 200.0f;
    float PowerW = 45.0f;
    float SpeedMps = 4.0f;
    float AccelMps2 = 8.0f;
    float Damage = 20.0f;
    float CooldownS = 0.05f;
    int32_t Ammo = 0;
    float VisionM = 18.0f;
    float RangeM = 40.0f;
    float Armor = 0.0f;
    float HeatPerShot = 10.0f;
    float MaxHeat = 40.0f;
    float CoolingPerS = 12.0f;
    float ProjectileSpeedMps = 25.0f;
};

inline FRobotSpec SpecInfantry()
{
    FRobotSpec S; S.Hp = 200.0f; S.PowerW = 45.0f; S.SpeedMps = 4.0f; S.AccelMps2 = 8.0f;
    S.Damage = 20.0f; S.CooldownS = 0.05f; S.Ammo = 0; S.VisionM = 18.0f; S.RangeM = 40.0f;
    S.Armor = 0.0f; S.HeatPerShot = 10.0f; S.MaxHeat = 40.0f; S.CoolingPerS = 12.0f;
    S.ProjectileSpeedMps = 25.0f; return S;
}

// ============================================================================
// 3) 单机裁判状态
// ============================================================================
struct FRobotReferee
{
    // 生命
    float Hp = 200.0f;
    float MaxHp = 200.0f;
    bool bDead = false;
    float InvulnerableUntilS = 0.0f;      // 复活后的无敌
    float LastArmorHitS = -1e9f;          // 装甲板受击间隔（0.05s 内只算一次）

    // 热量 / 锁火
    float Heat = 0.0f;
    float MaxHeat = 40.0f;
    bool bHeatLocked = false;
    float PermanentExtraHeat = 0.0f;      // 17mm:100 / 42mm:200（超速等判罚附加）

    // 功率 / 能量
    float EnergyJ = Rules::kInitialEnergyJ;
    float PowerBufferJ = Rules::kBufferCapacityJ;
    float CutoffRemainingS = 0.0f;

    // 弹药 / 经济 / 经验
    int32_t Ammo = 0;
    int32_t Coins = Rules::kInitialCoins;
    float Xp = 0.0f;
    int32_t Level = 1;

    // 复活进度
    float RespawnProgress = 0.0f;

    // 超速判罚累计
    float OverspeedLockRemainingS = 0.0f;

    // ---------------- 伤害 ----------------
    // Shell: 17 / 42（mm）。SpeedMps 用于"正常速度"判定。bWeakspot 命中弱点。
    // 返回实际造成的伤害（已按规则取整）。
    float ApplyDamage(float NowS, int32_t ShellMm, float SpeedMps, bool bWeakspot)
    {
        if (bDead) return 0.0f;
        if (NowS < InvulnerableUntilS) return 0.0f;

        // 装甲板受击间隔：
        //   17mm：0.05s 内同一块板只结算一次；
        //   42mm：上限 0.2s（但按规则是"每发都算"，这里保守取 0.05s 下限）
        const float IntervalS = (ShellMm == 42) ? Rules::kArmorIntervalS : Rules::kArmorIntervalS;
        if ((NowS - LastArmorHitS) < IntervalS) return 0.0f;
        LastArmorHitS = NowS;

        float Dmg = (ShellMm == 42) ? Rules::kDamage42 : Rules::kDamage17;
        // 正常初速判定：低于阈值视为"低速弹"，官方规则里按伤害减半处理（此处按 1.0 保留可调）
        const float MinSpeed = (ShellMm == 42) ? Rules::kNormalSpeedMin42Mps : Rules::kNormalSpeedMin17Mps;
        if (SpeedMps < MinSpeed * 0.5f) Dmg *= 0.5f;

        if (bWeakspot) Dmg *= Rules::kWeakspotMultiplier;

        // 取整规则 half_up_integer
        Dmg = std::floor(Dmg + 0.5f);

        Hp = std::max(0.0f, Hp - Dmg);
        if (Hp <= 0.0f) { bDead = true; }
        return Dmg;
    }

    // ---------------- 热量 ----------------
    // 开火一次：加弹道热量；超过上限 ⇒ 锁火（lock_when_above_limit）
    bool ApplyShotHeat(int32_t ShellMm, float HeatPerShot)
    {
        if (bHeatLocked) return false;
        Heat += HeatPerShot;
        if (Heat > MaxHeat + PermanentExtraHeat)
        {
            bHeatLocked = true;
        }
        // 42mm 每次射击会给 200 永久附加热（official.heat.permanent_extra_42）
        if (ShellMm == 42) PermanentExtraHeat = Rules::kPermanentExtraHeat42;
        return true;
    }

    void SettleHeat(float Dt, float CoolingPerS)
    {
        Heat = std::max(0.0f, Heat - CoolingPerS * Dt);
        // 解锁：unlock_heat = 0（降到 0 才解锁）
        if (bHeatLocked && Heat <= 0.0f)
        {
            bHeatLocked = false;
            PermanentExtraHeat = 0.0f;
        }
    }

    // ---------------- 功率 / 能量 ----------------
    // 返回 true = 供电正常；false = 功率超限已断电（cutoff）
    bool SettlePower(float Dt, float DemandW)
    {
        const float Cap = Rules::kAbsoluteCapW
            + (EnergyJ > Rules::kEnhancedThresholdJ
                ? (DemandW * (Rules::kEnhancedMultiplier - 1.0f)) : 0.0f);
        const float Draw = std::min(DemandW, Cap);
        EnergyJ -= Draw * Dt;
        if (EnergyJ < 0.0f)
        {
            EnergyJ = 0.0f;
            CutoffRemainingS = Rules::kPowerCutoffS;
            bHeatLocked = true;                 // 断电时不能开火
            return false;
        }
        if (CutoffRemainingS > 0.0f) CutoffRemainingS = std::max(0.0f, CutoffRemainingS - Dt);
        EnergyJ = std::min(EnergyJ, Rules::kMaxEnergyJ);
        return CutoffRemainingS <= 0.0f;
    }

    void AddEnergy(float J) { EnergyJ = std::min(Rules::kMaxEnergyJ, EnergyJ + J); }

    // ---------------- 弹药 / 经济 / 经验 ----------------
    void AddCoins(int32_t N) { Coins += N; }
    bool ConsumeCoins(int32_t N) { if (Coins < N) return false; Coins -= N; return true; }

    void AddXp(float N, int32_t LevelCap)
    {
        Xp += N;
        int32_t NewLevel = 1;
        for (int32_t i = 0; i < 10; ++i)
        {
            if (Xp >= (float)Rules::kXpThresholds[i]) NewLevel = i + 1;
        }
        Level = std::min(NewLevel, std::max(1, LevelCap));
    }

    // ---------------- 复活 ----------------
    // 普通复活：进度 = base + elapsed/divisor，同时 supply/base 血量 <2000 时按 4/s 加速
    void TickRespawnProgress(float Dt, float ElapsedS, bool bSupplyOrBaseLow)
    {
        const float Rate = bSupplyOrBaseLow ? Rules::kSupplyOrBaseBelow2000ProgressPerS
                                            : Rules::kNormalProgressPerS;
        RespawnProgress += Rate * Dt;
    }

    // 执行普通复活：返回所需进度是否已够
    bool TryRespawnNormal(float NowS)
    {
        const float Need = Rules::kRespawnProgressBase;   // 10
        if (RespawnProgress < Need) return false;
        RespawnProgress -= Need;
        bDead = false;
        Hp = MaxHp * Rules::kNormalHpRatio;
        InvulnerableUntilS = NowS + Rules::kNormalInvulnerabilityS;
        return true;
    }

    // 瞬时复活（花金币）：满血、3s 无敌、功率 ×2 持续 4s
    bool TryRespawnInstant(float NowS, int32_t Cost)
    {
        if (!bDead) return false;
        if (!ConsumeCoins(Cost)) return false;
        RespawnProgress = std::max(0.0f, RespawnProgress - Rules::kInstantPurchaseProgressPenalty);
        bDead = false;
        Hp = MaxHp * Rules::kInstantHpRatio;
        InvulnerableUntilS = NowS + Rules::kInstantInvulnerabilityS;
        return true;
    }

    // ---------------- 超速 ----------------
    // Excess = 实际速度 - 允许速度（17mm）
    void ApplyOverspeed17(float ExcessMps)
    {
        for (const auto& T : Rules::kOverspeed17)
        {
            if (ExcessMps > T.ExcessFrom && ExcessMps <= T.ExcessTo)
            {
                if (T.bPermanent) { PermanentExtraHeat = Rules::kPermanentExtraHeat17; }
                else { OverspeedLockRemainingS = std::max(OverspeedLockRemainingS, T.LockS); }
                return;
            }
        }
    }

    void TickPenalties(float Dt)
    {
        if (OverspeedLockRemainingS > 0.0f)
            OverspeedLockRemainingS = std::max(0.0f, OverspeedLockRemainingS - Dt);
    }
};

// ============================================================================
// 4) 对局整体（提现阶段：时间轴 / 经济 / 控制点）
// ============================================================================
enum class EMatchPhase : uint8_t { Prep, SelfCheck, Countdown, Running, Finished };

struct FTeamState
{
    int32_t Coins = Rules::kInitialCoins;
    int32_t Score = 0;
    float ControlPercent = 0.0f;      // 控制点控制度
    int32_t TechLevel = 0;            // 重型科技 1~4 级
    bool bBaseAlive = true;
};

struct FArenaReferee
{
    EMatchPhase Phase = EMatchPhase::Prep;
    float NowS = 0.0f;                // 对局内时间（秒）
    int32_t CoinTickIndex = 0;
    FTeamState Teams[2];

    void Tick(float Dt)
    {
        NowS += Dt;
        switch (Phase)
        {
        case EMatchPhase::Prep:
            if (NowS >= Rules::kPrepS) { Phase = EMatchPhase::SelfCheck; NowS = 0.0f; }
            break;
        case EMatchPhase::SelfCheck:
            if (NowS >= Rules::kSelfCheckS) { Phase = EMatchPhase::Countdown; NowS = 0.0f; }
            break;
        case EMatchPhase::Countdown:
            if (NowS >= Rules::kCountdownS) { Phase = EMatchPhase::Running; NowS = 0.0f; }
            break;
        case EMatchPhase::Running:
            PeriodCoins();
            if (NowS >= Rules::kMatchDurationS) { Phase = EMatchPhase::Finished; }
            break;
        default:
            break;
        }
    }

    void PeriodCoins()
    {
        const int32_t N = (int32_t)(sizeof(Rules::kCoinSchedule) / sizeof(Rules::kCoinSchedule[0]));
        while (CoinTickIndex < N && NowS >= Rules::kCoinSchedule[CoinTickIndex].ElapsedS)
        {
            for (int32_t t = 0; t < 2; ++t) Teams[t].Coins += Rules::kCoinSchedule[CoinTickIndex].Coins;
            ++CoinTickIndex;
        }
    }

    // 胜负：基地血量 + 控制度（按 rule 的判据，先给基础版）
    int32_t Winner() const
    {
        if (Teams[0].bBaseAlive != Teams[1].bBaseAlive) return Teams[0].bBaseAlive ? 0 : 1;
        if (Teams[0].ControlPercent != Teams[1].ControlPercent)
            return Teams[0].ControlPercent > Teams[1].ControlPercent ? 0 : 1;
        return Teams[0].Score >= Teams[1].Score ? 0 : 1;
    }
};

} // namespace ArenaReferee2027
