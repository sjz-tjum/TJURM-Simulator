// WLRefereeSystem.h
//
// 裁判系统（Referee System）—— 对局中"和车有关的一切数据"的唯一持有者。
//
// 为什么要有它（用户 2026-10-09 的需求）：
//   "车的血量数据不再由车本身记录，而是由一个裁判系统记录，热量数据和电容也是，其实和车有关
//    的信息都由这个记录。车辆血条清零后会直接下力无法操纵，10s 后复活后只有 20% 的血量和 50%
//    的最高速度，必须回到回血点才能重置和回血。"
//
// 所以：
//   * 车（AWLVehiclePawn）**不再有** Health / Heat / Capacitor 成员，只有转发接口
//     （GetHealth() / SetHeat() / GetCapacitorPct() ...），读写都落到这里的那条 FWLCombatantRecord；
//   * 记录由本 Actor 持有、在服务器上写入、整表复制给客户端（HUD/结算/观战都读它）；
//   * 死亡、复活、限速、回血点结算、战绩都在 RefereeStep 里结算 —— 一个地方，不散落在车里。
//
// 权威模型：本项目的对局逻辑是"服务器权威"（listen server）。车端只做表现与输入，数值一律问这里。
// 单机（standalone）时本地就是服务器，所以自检/GroundQA 那套流程照常工作。

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GameFramework/PlayerState.h"
#include "WLCombat.h"
#include "WLRefereeSystem.generated.h"

class APawn;
class AWLVehiclePawn;

/** 玩家状态：队伍 + 选的角色 + 对局编号。三者都要复制，UI 与出生逻辑都读它。 */
UCLASS()
class RM2019SIM_API AWLPvPPlayerState : public APlayerState
{
	GENERATED_BODY()

public:
	UPROPERTY(Replicated, BlueprintReadOnly, Category = "WL|PVP") EWLTeam Team = EWLTeam::None;
	UPROPERTY(Replicated, BlueprintReadOnly, Category = "WL|PVP") FName CharacterId;
	UPROPERTY(Replicated, BlueprintReadOnly, Category = "WL|PVP") int32 CombatantId = 0;

	virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;
};

UCLASS()
class RM2019SIM_API AWLRefereeSystem : public AActor
{
	GENERATED_BODY()

public:
	AWLRefereeSystem();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;

	/** 找当前世界的裁判系统（没有就返回 nullptr）。客户端也用它读记录。 */
	static AWLRefereeSystem* Get(UWorld* World);

	// ============================================================ 规则参数（服务器）
	/** 血条清零后多久复活（用户要求 10 s） */
	UPROPERTY(EditAnywhere, Category = "WL|PVP") float RespawnDelaySec = 10.0f;
	/** 复活时恢复到最大血量的比例（用户要求 20%） */
	UPROPERTY(EditAnywhere, Category = "WL|PVP") float RespawnHealthPct = 0.20f;
	/** 复活后的最高速度倍率（用户要求 50%），回到回血点才恢复 1.0 */
	UPROPERTY(EditAnywhere, Category = "WL|PVP") float RespawnSpeedScale = 0.50f;
	/** 在回血点里需要停多久才开始回血（秒）。0 = 立刻 */
	UPROPERTY(EditAnywhere, Category = "WL|PVP") float HealDwellSec = 1.0f;
	/** 回血点治疗速度（HP/秒）；<=0 表示"直接回满"（demo 默认，规则最简单也最好懂） */
	UPROPERTY(EditAnywhere, Category = "WL|PVP") float HealPerSec = 0.0f;
	/** 每队上限（5v5 = 5） */
	UPROPERTY(EditAnywhere, Category = "WL|PVP") int32 MaxPerTeam = 5;

	// ============================================================ 注册 / 查询
	/** 服务器：登记一台车（出生时由 pawn 自己调用）。返回记录引用。 */
	FWLCombatantRecord& RegisterCombatant(APawn* Pawn, EWLTeam Team, FName CharacterId, const FString& InPlayerName);
	void UnregisterCombatant(APawn* Pawn);

	FWLCombatantRecord* FindByPawn(const APawn* Pawn);
	const FWLCombatantRecord* FindByPawn(const APawn* Pawn) const;
	FWLCombatantRecord* FindById(int32 Id);
	const FWLCombatantRecord* FindById(int32 Id) const;
	/** 车队里人数较少的一队（自动平衡用） */
	EWLTeam SuggestTeam() const;
	int32 CountTeam(EWLTeam T) const;
	/** 队伍得分 = 击杀 * 100 + 伤害 */
	float ScoreOf(EWLTeam T) const;

	// ============================================================ 事件（服务器权威）
	/** 命中：扣血、记战绩、把血条清零的车判死 */
	void ApplyDamage(int32 Id, float Amount, const FString& Module, AActor* DamageInstigator);
	void SetHealth(int32 Id, float Value);
	/** 热量：车端的热/散热 ODE 仍由车计算（它才知道扳机状态），数值记录在这里 */
	void AddHeat(int32 Id, float Amount);
	void SetHeat(int32 Id, float Value);
	void SetOverheated(int32 Id, bool bValue);
	void SetCapacitorPct(int32 Id, float Pct);
	/** 电容消耗（加速）：返回实际消耗量 */
	float ConsumeCapacitor(int32 Id, float Amount);
	void KillCombatant(int32 Id, const FString& Reason);
	void RespawnCombatant(int32 Id);
	/** 回血点进出（由 AWLHealZone 在服务器上通知） */
	void SetOnHealPad(int32 Id, bool bInside);

	/** 队伍出生点（由地图/竞技场构建器填），复活时按最少使用挑选 */
	void SetTeamSpawns(EWLTeam Team, const TArray<FTransform>& Spawns);
	/** 把一台车放到它队伍的出生点（复活用）。服务器调用。 */
	void TeleportToTeamSpawn(int32 Id);

	/** 是否已经出局（血条清零，等待复活）—— 车端用它切断操控（"下力无法操纵"） */
	bool IsDisabled(int32 Id) const;

	// ============================================================ 每帧结算（服务器）
	void RefereeStep(float Dt);

	/** -WLPvPSmoke：不依赖网络/弹丸，直接对规则做断言（死亡/10 s 复活/20% 血/50% 速度/回血点重置） */
	int32 RunRuleSelfTest();

	// ============================================================ 数据
	/** 全部战斗记录。整表复制：demo 规模（<=10 台车）下最省事也最不容易出错。 */
	UPROPERTY(ReplicatedUsing = OnRep_Records, BlueprintReadOnly, Category = "WL|PVP")
	TArray<FWLCombatantRecord> Records;

	UFUNCTION()
	void OnRep_Records();

	/** 回血点耗时（服务器内部） */
	float NowSeconds = 0.0f;

private:
	/** 记录 -> 车（只在服务器上有意义） */
	UPROPERTY(Transient) TMap<int32, TWeakObjectPtr<APawn>> PawnById;
	int32 NextCombatantId = 1;
	TMap<EWLTeam, TArray<FTransform>> TeamSpawns;
	TMap<EWLTeam, int32> TeamSpawnCursor;

	FWLCombatantRecord* FindMutable(int32 Id);
};
