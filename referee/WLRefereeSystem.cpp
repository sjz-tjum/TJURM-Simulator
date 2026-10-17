// WLRefereeSystem.cpp —— 对局规则的唯一实现处。

#include "WLRefereeSystem.h"
#include "WLVehiclePawn.h"

#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerState.h"
#include "Net/UnrealNetwork.h"

// ============================================================================ player state
void AWLPvPPlayerState::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
	Super::GetLifetimeReplicatedProps(OutLifetimeProps);
	DOREPLIFETIME(AWLPvPPlayerState, Team);
	DOREPLIFETIME(AWLPvPPlayerState, CharacterId);
	DOREPLIFETIME(AWLPvPPlayerState, CombatantId);
}

// ============================================================================ referee
AWLRefereeSystem::AWLRefereeSystem()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;
	// 对局数据必须无条件送到每个客户端：HUD、结算、观战都靠它
	bReplicates = true;
	bAlwaysRelevant = true;
	SetReplicateMovement(false);
}

void AWLRefereeSystem::BeginPlay()
{
	Super::BeginPlay();
	NowSeconds = 0.0f;
	UE_LOG(LogTemp, Display,
		TEXT("WLREF|referee up: respawn=%.1fs @%.0f%% hp & %.0f%% speed | healpad dwell=%.1fs heal=%s | maxPerTeam=%d"),
		RespawnDelaySec, RespawnHealthPct * 100.0f, RespawnSpeedScale * 100.0f,
		HealDwellSec, (HealPerSec > 0.0f) ? TEXT("gradual") : TEXT("instant"), MaxPerTeam);
}

void AWLRefereeSystem::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
	Super::GetLifetimeReplicatedProps(OutLifetimeProps);
	DOREPLIFETIME(AWLRefereeSystem, Records);
}

AWLRefereeSystem* AWLRefereeSystem::Get(UWorld* World)
{
	if (!World)
	{
		return nullptr;
	}
	for (TActorIterator<AWLRefereeSystem> It(World); It; ++It)
	{
		return *It;
	}
	return nullptr;
}

void AWLRefereeSystem::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (!HasAuthority())
	{
		return;                       // 客户端只读
	}
	RefereeStep(DeltaSeconds);
}

// ---------------------------------------------------------------- 注册 / 查询
FWLCombatantRecord& AWLRefereeSystem::RegisterCombatant(APawn* Pawn, EWLTeam Team, FName CharacterId, const FString& InPlayerName)
{
	const FWLCharacterDef& Def = FWLCharacterRegistry::Get(CharacterId.IsNone() ? FWLCharacterRegistry::DefaultCharacterId() : CharacterId);

	// 同一台车重复注册（重生、possess 切换）就复用原记录：id 与战绩不能丢
	if (Pawn)
	{
		for (FWLCombatantRecord& R : Records)
		{
			if (PawnById.Contains(R.CombatantId) && PawnById[R.CombatantId].Get() == Pawn)
			{
				return R;
			}
		}
	}

	FWLCombatantRecord R;
	R.CombatantId = NextCombatantId++;
	R.Team = (Team == EWLTeam::None) ? SuggestTeam() : Team;
	R.CharacterId = Def.Id;
	R.PlayerName = InPlayerName;
	R.MaxHealth = Def.MaxHealth;
	R.Health = Def.MaxHealth;
	R.HeatMax = Def.HeatMax;
	R.Heat = 0.0f;
	R.Capacitor = Def.CapacitorMax;
	R.SpeedScale = 1.0f;
	R.bNeedsReset = false;
	R.bDead = false;
	R.RespawnRemain = 0.0f;

	Records.Add(R);
	if (Pawn)
	{
		PawnById.Add(R.CombatantId, Pawn);
	}
	UE_LOG(LogTemp, Display,
		TEXT("WLREF|registered #%d team=%d char=%s name='%s' hp=%.0f (roster=%d)"),
		R.CombatantId, (int32)R.Team, *Def.Id.ToString(), *InPlayerName, R.Health, Records.Num());
	return Records.Last();
}

void AWLRefereeSystem::UnregisterCombatant(APawn* Pawn)
{
	if (!Pawn)
	{
		return;
	}
	for (int32 i = Records.Num() - 1; i >= 0; --i)
	{
		const int32 Id = Records[i].CombatantId;
		if (PawnById.Contains(Id) && PawnById[Id].Get() == Pawn)
		{
			PawnById.Remove(Id);
			// 记录保留（战绩/结算要用），只解绑车：把 id 标记为无主
			UE_LOG(LogTemp, Display, TEXT("WLREF|unregistered pawn for #%d (record kept)"), Id);
			return;
		}
	}
}

FWLCombatantRecord* AWLRefereeSystem::FindMutable(int32 Id)
{
	for (FWLCombatantRecord& R : Records)
	{
		if (R.CombatantId == Id)
		{
			return &R;
		}
	}
	return nullptr;
}

FWLCombatantRecord* AWLRefereeSystem::FindById(int32 Id) { return FindMutable(Id); }

const FWLCombatantRecord* AWLRefereeSystem::FindById(int32 Id) const
{
	for (const FWLCombatantRecord& R : Records)
	{
		if (R.CombatantId == Id)
		{
			return &R;
		}
	}
	return nullptr;
}

FWLCombatantRecord* AWLRefereeSystem::FindByPawn(const APawn* Pawn)
{
	if (!Pawn)
	{
		return nullptr;
	}
	for (const TPair<int32, TWeakObjectPtr<APawn>>& KV : PawnById)
	{
		if (KV.Value.Get() == Pawn)
		{
			return FindMutable(KV.Key);
		}
	}
	return nullptr;
}

const FWLCombatantRecord* AWLRefereeSystem::FindByPawn(const APawn* Pawn) const
{
	return const_cast<AWLRefereeSystem*>(this)->FindByPawn(Pawn);
}

int32 AWLRefereeSystem::CountTeam(EWLTeam T) const
{
	int32 N = 0;
	for (const FWLCombatantRecord& R : Records)
	{
		if (R.Team == T)
		{
			++N;
		}
	}
	return N;
}

EWLTeam AWLRefereeSystem::SuggestTeam() const
{
	// 5v5：人少的一队优先；满员就进红队（demo 不踢人）
	const int32 Red = CountTeam(EWLTeam::Red);
	const int32 Blue = CountTeam(EWLTeam::Blue);
	if (Red >= MaxPerTeam && Blue >= MaxPerTeam)
	{
		return EWLTeam::Red;
	}
	return (Red <= Blue) ? EWLTeam::Red : EWLTeam::Blue;
}

float AWLRefereeSystem::ScoreOf(EWLTeam T) const
{
	float S = 0.0f;
	for (const FWLCombatantRecord& R : Records)
	{
		if (R.Team == T)
		{
			S += R.Kills * 100.0f + R.DamageDealt;
		}
	}
	return S;
}

// ---------------------------------------------------------------- 事件
void AWLRefereeSystem::ApplyDamage(int32 Id, float Amount, const FString& Module, AActor* DamageInstigator)
{
	if (!HasAuthority() || Amount <= 0.0f)
	{
		return;
	}
	FWLCombatantRecord* R = FindMutable(Id);
	if (!R || R->bDead)
	{
		return;
	}
	const float Before = R->Health;
	R->Health = FMath::Max(0.0f, R->Health - Amount);
	R->LastHitTime = NowSeconds;

	// 战绩记在射手身上（弹丸的 Owner 就是那台车）
	if (const AWLVehiclePawn* Shooter = Cast<AWLVehiclePawn>(DamageInstigator))
	{
		if (FWLCombatantRecord* SR = FindByPawn(Shooter))
		{
			if (SR->CombatantId != R->CombatantId)
			{
				SR->DamageDealt += (Before - R->Health);
			}
		}
	}

	UE_LOG(LogTemp, Display, TEXT("WLREF|#%d %s hit for %.0f -> %.0f/%.0f hp"),
		Id, *Module, Before - R->Health, R->Health, R->MaxHealth);

	if (R->Health <= 0.0f)
	{
		KillCombatant(Id, TEXT("armour destroyed"));
	}
}

void AWLRefereeSystem::SetHealth(int32 Id, float Value)
{
	if (!HasAuthority())
	{
		return;
	}
	if (FWLCombatantRecord* R = FindMutable(Id))
	{
		R->Health = FMath::Clamp(Value, 0.0f, R->MaxHealth);
		if (R->Health <= 0.0f && !R->bDead)
		{
			KillCombatant(Id, TEXT("health set to 0"));
		}
	}
}

void AWLRefereeSystem::AddHeat(int32 Id, float Amount)
{
	if (FWLCombatantRecord* R = FindMutable(Id))
	{
		R->Heat = FMath::Clamp(R->Heat + Amount, 0.0f, R->HeatMax);
		if (R->Heat >= R->HeatMax)
		{
			R->bOverheated = true;
		}
	}
}

void AWLRefereeSystem::SetHeat(int32 Id, float Value)
{
	if (FWLCombatantRecord* R = FindMutable(Id))
	{
		R->Heat = FMath::Clamp(Value, 0.0f, R->HeatMax);
		R->bOverheated = (R->Heat >= R->HeatMax);
	}
}

void AWLRefereeSystem::SetOverheated(int32 Id, bool bValue)
{
	if (FWLCombatantRecord* R = FindMutable(Id))
	{
		R->bOverheated = bValue;
	}
}

void AWLRefereeSystem::SetCapacitorPct(int32 Id, float Pct)
{
	if (FWLCombatantRecord* R = FindMutable(Id))
	{
		R->Capacitor = FMath::Clamp(Pct, 0.0f, 1.0f);
	}
}

float AWLRefereeSystem::ConsumeCapacitor(int32 Id, float Amount)
{
	FWLCombatantRecord* R = FindMutable(Id);
	if (!R || Amount <= 0.0f)
	{
		return 0.0f;
	}
	const float Used = FMath::Min(R->Capacitor, Amount);
	R->Capacitor -= Used;
	return Used;
}

void AWLRefereeSystem::KillCombatant(int32 Id, const FString& Reason)
{
	if (!HasAuthority())
	{
		return;
	}
	FWLCombatantRecord* R = FindMutable(Id);
	if (!R || R->bDead)
	{
		return;
	}
	R->bDead = true;
	R->Health = 0.0f;
	++R->Deaths;
	R->RespawnRemain = RespawnDelaySec;
	R->SpeedScale = 0.0f;              // 下力：完全不能动
	R->bAtHealPad = false;
	R->HealProgress = 0.0f;
	UE_LOG(LogTemp, Display, TEXT("WLREF|#%d DESTROYED (%s) - respawn in %.1f s"),
		Id, *Reason, RespawnDelaySec);
}

void AWLRefereeSystem::RespawnCombatant(int32 Id)
{
	if (!HasAuthority())
	{
		return;
	}
	FWLCombatantRecord* R = FindMutable(Id);
	if (!R)
	{
		return;
	}
	const FWLCharacterDef& Def = FWLCharacterRegistry::Get(R->CharacterId);
	R->MaxHealth = Def.MaxHealth;
	R->bDead = false;
	R->RespawnRemain = 0.0f;
	R->Health = Def.MaxHealth * FMath::Clamp(RespawnHealthPct, 0.05f, 1.0f);
	R->SpeedScale = RespawnSpeedScale;
	R->bNeedsReset = true;             // 必须回回血点才能恢复速度上限并回满
	R->Heat = 0.0f;
	R->bOverheated = false;
	R->Capacitor = Def.CapacitorMax;
	R->HealProgress = 0.0f;

	TeleportToTeamSpawn(Id);
	UE_LOG(LogTemp, Display,
		TEXT("WLREF|#%d RESPAWNED at %.0f%% hp (%.0f/%.0f), speed x%.2f - must reach a heal pad to reset"),
		Id, RespawnHealthPct * 100.0f, R->Health, R->MaxHealth, R->SpeedScale);
}

void AWLRefereeSystem::SetOnHealPad(int32 Id, bool bInside)
{
	FWLCombatantRecord* R = FindMutable(Id);
	if (!R)
	{
		return;
	}
	if (R->bAtHealPad != bInside)
	{
		UE_LOG(LogTemp, Display, TEXT("WLREF|#%d heal pad %s"), Id, bInside ? TEXT("ENTER") : TEXT("EXIT"));
	}
	R->bAtHealPad = bInside;
	if (!bInside)
	{
		R->HealProgress = 0.0f;
	}
}

// ---------------------------------------------------------------- 出生点
void AWLRefereeSystem::SetTeamSpawns(EWLTeam Team, const TArray<FTransform>& Spawns)
{
	TeamSpawns.Add(Team, Spawns);
	TeamSpawnCursor.Add(Team, 0);
}

void AWLRefereeSystem::TeleportToTeamSpawn(int32 Id)
{
	FWLCombatantRecord* R = FindMutable(Id);
	if (!R)
	{
		return;
	}
	const TArray<FTransform>* Spawns = TeamSpawns.Find(R->Team);
	if (!Spawns || Spawns->Num() == 0)
	{
		return;                        // 地图没给出生点：原地复活（不会被静默忽略）
	}
	int32& Cursor = TeamSpawnCursor.FindOrAdd(R->Team);
	const FTransform T = (*Spawns)[Cursor % Spawns->Num()];
	++Cursor;

	if (const TWeakObjectPtr<APawn>* P = PawnById.Find(Id))
	{
		if (APawn* Pawn = P->Get())
		{
			Pawn->SetActorTransform(T, false, nullptr, ETeleportType::TeleportPhysics);
			// 车端的仿真状态必须跟着一起复位，否则轮子/姿态会留在原地
			if (AWLVehiclePawn* V = Cast<AWLVehiclePawn>(Pawn))
			{
				V->OnRefereeRespawn(T);
			}
		}
	}
}

bool AWLRefereeSystem::IsDisabled(int32 Id) const
{
	const FWLCombatantRecord* R = FindById(Id);
	return R && (R->bDead || R->RespawnRemain > 0.0f);
}

// ---------------------------------------------------------------- 每帧结算
void AWLRefereeSystem::RefereeStep(float Dt)
{
	NowSeconds += Dt;
	for (FWLCombatantRecord& R : Records)
	{
		// 1) 复活计时
		if (R.bDead)
		{
			R.RespawnRemain = FMath::Max(0.0f, R.RespawnRemain - Dt);
			if (R.RespawnRemain <= 0.0f)
			{
				RespawnCombatant(R.CombatantId);
			}
			continue;                       // 死的车不参与回血
		}

		// 2) 回血点：站住 HealDwellSec 秒后生效。**只有这里能回血**（没有脱战自动回血）
		if (!R.bAtHealPad)
		{
			R.HealProgress = 0.0f;
			continue;
		}
		R.HealProgress = FMath::Min(HealDwellSec, R.HealProgress + Dt);
		if (R.HealProgress < HealDwellSec)
		{
			continue;
		}
		if (HealPerSec > 0.0f)
		{
			R.Health = FMath::Min(R.MaxHealth, R.Health + HealPerSec * Dt);
		}
		else
		{
			R.Health = R.MaxHealth;          // demo：到点直接回满
		}
		if (R.bNeedsReset && R.Health >= R.MaxHealth)
		{
			R.bNeedsReset = false;
			R.SpeedScale = 1.0f;
			UE_LOG(LogTemp, Display, TEXT("WLREF|#%d RESET at heal pad: full hp + speed restored"), R.CombatantId);
		}
	}
}

void AWLRefereeSystem::OnRep_Records()
{
	// 客户端拿到整表副本。UI 直接读 Records；这里只做一次轻量日志方便联调。
	UE_LOG(LogTemp, Verbose, TEXT("WLREF|client received %d record(s)"), Records.Num());
}

// ============================================================================ 规则自检
int32 AWLRefereeSystem::RunRuleSelfTest()
{
	// 直接对规则做断言：不依赖网络、不依赖弹丸、不依赖地图。把用户的四条要求逐条变红/变绿。
	int32 Fail = 0;
	auto Check = [&Fail](const TCHAR* What, bool bOk, const FString& Detail)
	{
		UE_LOG(LogTemp, Display, TEXT("WLPvP|rule %-34s -> %s  %s"), What,
			bOk ? TEXT("PASS") : TEXT("FAIL"), *Detail);
		if (!bOk)
		{
			++Fail;
		}
	};

	FWLCombatantRecord& R = RegisterCombatant(nullptr, EWLTeam::Red, FWLCharacterRegistry::DefaultCharacterId(), TEXT("smoke"));
	const int32 Id = R.CombatantId;
	const float MaxHP = R.MaxHealth;

	Check(TEXT("spawn at full hp"), FMath::IsNearlyEqual(R.Health, MaxHP, 0.01f),
		FString::Printf(TEXT("hp=%.1f/%.1f speed=%.2f"), R.Health, MaxHP, R.SpeedScale));

	// 打死
	ApplyDamage(Id, MaxHP + 50.0f, TEXT("smoke"), nullptr);
	const FWLCombatantRecord* D = FindById(Id);
	Check(TEXT("hp 0 -> disabled + 10 s respawn"), D && D->bDead && FMath::IsNearlyEqual(D->RespawnRemain, RespawnDelaySec, 0.01f),
		D ? FString::Printf(TEXT("dead=%d remain=%.2f speed=%.2f"), D->bDead ? 1 : 0, D->RespawnRemain, D->SpeedScale) : TEXT("no record"));

	// 9.5 s 后仍未复活
	RefereeStep(0.95f * RespawnDelaySec);
	D = FindById(Id);
	Check(TEXT("still dead before 10 s"), D && D->bDead,
		D ? FString::Printf(TEXT("remain=%.2f"), D->RespawnRemain) : TEXT("no record"));

	// 10 s 到 -> 复活：20% 血、50% 速度、需要回血点
	RefereeStep(0.10f * RespawnDelaySec + 0.05f);
	D = FindById(Id);
	const bool bHpOk = D && FMath::IsNearlyEqual(D->Health, MaxHP * RespawnHealthPct, 0.5f);
	const bool bSpdOk = D && FMath::IsNearlyEqual(D->SpeedScale, RespawnSpeedScale, 0.001f);
	Check(TEXT("respawn: 20% hp"), bHpOk, D ? FString::Printf(TEXT("hp=%.1f (want %.1f)"), D->Health, MaxHP * RespawnHealthPct) : TEXT("no record"));
	Check(TEXT("respawn: 50% max speed"), bSpdOk, D ? FString::Printf(TEXT("speed=%.2f"), D->SpeedScale) : TEXT("no record"));
	Check(TEXT("respawn: reset required"), D && D->bNeedsReset && !D->bDead, D ? FString::Printf(TEXT("needsReset=%d"), D->bNeedsReset ? 1 : 0) : TEXT("no record"));

	// 回血点之外不许回血（demo 里唯一的治疗来源）
	RefereeStep(3.0f);
	D = FindById(Id);
	Check(TEXT("no healing away from a pad"), D && FMath::IsNearlyEqual(D->Health, MaxHP * RespawnHealthPct, 0.5f),
		D ? FString::Printf(TEXT("hp=%.1f"), D->Health) : TEXT("no record"));

	// 进回血点 -> 满血 + 速度恢复
	SetOnHealPad(Id, true);
	RefereeStep(HealDwellSec + 0.05f);
	D = FindById(Id);
	Check(TEXT("heal pad: full hp + speed reset"), D && FMath::IsNearlyEqual(D->Health, MaxHP, 0.5f)
		&& FMath::IsNearlyEqual(D->SpeedScale, 1.0f, 0.001f) && !D->bNeedsReset,
		D ? FString::Printf(TEXT("hp=%.1f speed=%.2f needsReset=%d"), D->Health, D->SpeedScale, D->bNeedsReset ? 1 : 0) : TEXT("no record"));

	// 离开回血点后再受伤：不能自愈
	SetOnHealPad(Id, false);
	ApplyDamage(Id, 50.0f, TEXT("smoke"), nullptr);
	const float AfterHit = FindById(Id) ? FindById(Id)->Health : -1.0f;
	RefereeStep(2.0f);
	D = FindById(Id);
	Check(TEXT("no regen after leaving the pad"), D && FMath::IsNearlyEqual(D->Health, AfterHit, 0.5f),
		D ? FString::Printf(TEXT("hp=%.1f"), D->Health) : TEXT("no record"));

	UE_LOG(LogTemp, Display, TEXT("WLPvP|RULES %s (%d failure(s))"), Fail == 0 ? TEXT("ALL PASS") : TEXT("FAILED"), Fail);
	UnregisterCombatant(nullptr);
	Records.RemoveAll([Id](const FWLCombatantRecord& X) { return X.CombatantId == Id; });
	return Fail;
}
