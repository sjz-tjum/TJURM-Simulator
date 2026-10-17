// WLPVPArena.cpp —— 26 m x 18 m 中心对称竞技场 + 回血点。

#include "WLPVPArena.h"
#include "WLRefereeSystem.h"
#include "WLVehiclePawn.h"

#include "Components/BoxComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/DirectionalLight.h"
#include "Engine/SkyLight.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/SkyLightComponent.h"
#include "GameFramework/PlayerStart.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Engine/World.h"
#include "EngineUtils.h"

// ============================================================================ 回血点
AWLHealZone::AWLHealZone()
{
	PrimaryActorTick.bCanEverTick = false;
	bReplicates = true;

	Pad = CreateDefaultSubobject<UBoxComponent>(TEXT("Pad"));
	SetRootComponent(Pad);
	Pad->SetBoxExtent(FVector(PadHalfSizeCm, PadHalfSizeCm, 120.0f));
	Pad->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
	Pad->SetCollisionResponseToAllChannels(ECR_Ignore);
	Pad->SetCollisionResponseToChannel(ECC_Pawn, ECR_Overlap);
	Pad->SetGenerateOverlapEvents(true);
	Pad->SetHiddenInGame(true);

	Marker = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Marker"));
	Marker->SetupAttachment(Pad);
	Marker->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Marker->SetRelativeScale3D(FVector(PadHalfSizeCm * 2.0f / 100.0f, PadHalfSizeCm * 2.0f / 100.0f, 0.08f));
}

void AWLHealZone::SetPadSize(float HalfSizeCm)
{
	PadHalfSizeCm = HalfSizeCm;
	if (Pad)
	{
		Pad->SetBoxExtent(FVector(HalfSizeCm, HalfSizeCm, 120.0f));
	}
	if (Marker)
	{
		Marker->SetRelativeScale3D(FVector(HalfSizeCm * 2.0f / 100.0f, HalfSizeCm * 2.0f / 100.0f, 0.08f));
	}
}

void AWLHealZone::BeginPlay()
{
	Super::BeginPlay();
	Referee = AWLRefereeSystem::Get(GetWorld());
	if (Pad)
	{
		Pad->OnComponentBeginOverlap.AddDynamic(this, &AWLHealZone::OnPadBegin);
		Pad->OnComponentEndOverlap.AddDynamic(this, &AWLHealZone::OnPadEnd);
	}
	if (Marker)
	{
		if (UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube")))
		{
			Marker->SetStaticMesh(Cube);
			if (UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial")))
			{
				UMaterialInstanceDynamic* MID = UMaterialInstanceDynamic::Create(Base, this);
				MID->SetVectorParameterValue(TEXT("Color"), PadColour);
				Marker->SetMaterial(0, MID);
			}
		}
	}
	UE_LOG(LogTemp, Display, TEXT("WLARENA|heal pad at (%.0f,%.0f) r=%.0f team=%d"),
		GetActorLocation().X, GetActorLocation().Y, PadHalfSizeCm, (int32)OwnerTeam);
}

void AWLHealZone::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
}

void AWLHealZone::OnPadBegin(UPrimitiveComponent*, AActor* Other, UPrimitiveComponent*, int32, bool, const FHitResult&)
{
	if (!Referee || !Other)
	{
		return;
	}
	if (AWLVehiclePawn* V = Cast<AWLVehiclePawn>(Other))
	{
		if (const FWLCombatantRecord* R = Referee->FindByPawn(V))
		{
			Referee->SetOnHealPad(R->CombatantId, true);
		}
	}
}

void AWLHealZone::OnPadEnd(UPrimitiveComponent*, AActor* Other, UPrimitiveComponent*, int32)
{
	if (!Referee || !Other)
	{
		return;
	}
	if (AWLVehiclePawn* V = Cast<AWLVehiclePawn>(Other))
	{
		if (const FWLCombatantRecord* R = Referee->FindByPawn(V))
		{
			Referee->SetOnHealPad(R->CombatantId, false);
		}
	}
}

// ============================================================================ 竞技场
AWLPVPArena::AWLPVPArena()
{
	PrimaryActorTick.bCanEverTick = false;
	bReplicates = false;                 // 服务器生成一次，几何本身不需要复制（碰撞盒会随关卡/客户端构建）
	// 说明：几何是服务器生成的；局域网客户端进入时由 GameMode 在客户端本地重建同一张表
	// （Build 是纯函数式的，同样的输入必然得到同样的地图），比复制 30 个 actor 更省事也更稳。
}

void AWLPVPArena::BeginPlay()
{
	Super::BeginPlay();
	if (HasAuthority())
	{
		Build();
	}
}

const TArray<FTransform>& AWLPVPArena::GetTeamSpawns(EWLTeam Team) const
{
	if (Team == EWLTeam::Red)
	{
		return SpawnsRed;
	}
	if (Team == EWLTeam::Blue)
	{
		return SpawnsBlue;
	}
	static const TArray<FTransform> Empty;
	return Empty;
}

AStaticMeshActor* AWLPVPArena::SpawnBox(const FVector& Centre, const FVector& SizeCm, const FLinearColor& Colour, const TCHAR* Tag)
{
	UWorld* W = GetWorld();
	if (!W)
	{
		return nullptr;
	}
	FActorSpawnParameters SP;
	SP.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	AStaticMeshActor* A = W->SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass(),
		FTransform(FRotator::ZeroRotator, Centre), SP);
	if (!A)
	{
		return nullptr;
	}
	A->SetMobility(EComponentMobility::Movable);
	UStaticMeshComponent* MC = A->GetStaticMeshComponent();
	if (UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube")))
	{
		MC->SetStaticMesh(Cube);
	}
	MC->SetWorldScale3D(SizeCm / 100.0f);      // Cube 是 100 cm 边长
	MC->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	MC->SetCollisionProfileName(TEXT("BlockAll"));
	MC->SetCollisionResponseToAllChannels(ECR_Block);
	if (UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial")))
	{
		UMaterialInstanceDynamic* MID = UMaterialInstanceDynamic::Create(Base, A);
		MID->SetVectorParameterValue(TEXT("Color"), Colour);
		MC->SetMaterial(0, MID);
	}
	if (Tag)
	{
		A->Tags.Add(FName(Tag));
	}
	Spawned.Add(A);
	return A;
}

AWLHealZone* AWLPVPArena::SpawnPad(const FVector2D& CentreXY, EWLTeam Team)
{
	UWorld* W = GetWorld();
	if (!W)
	{
		return nullptr;
	}
	FActorSpawnParameters SP;
	SP.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	AWLHealZone* Z = W->SpawnActor<AWLHealZone>(AWLHealZone::StaticClass(),
		FTransform(FRotator::ZeroRotator, FVector(CentreXY.X, CentreXY.Y, 10.0f)), SP);
	if (Z)
	{
		Z->OwnerTeam = Team;
		Pads.Add(Z);
	}
	return Z;
}

void AWLPVPArena::SpawnTeamStart(const FVector& Loc, EWLTeam Team, int32 Index)
{
	UWorld* W = GetWorld();
	if (!W)
	{
		return;
	}
	FActorSpawnParameters SP;
	SP.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	APlayerStart* S = W->SpawnActor<APlayerStart>(APlayerStart::StaticClass(),
		FTransform(FRotator(0.0f, (Team == EWLTeam::Red) ? 0.0f : 180.0f, 0.0f), Loc), SP);
	if (!S)
	{
		return;
	}
	S->PlayerStartTag = (Team == EWLTeam::Red) ? FName(TEXT("Red")) : FName(TEXT("Blue"));
	S->Tags.Add(S->PlayerStartTag);
	if (Team == EWLTeam::Red)
	{
		SpawnsRed.Add(S->GetActorTransform());
	}
	else
	{
		SpawnsBlue.Add(S->GetActorTransform());
	}
}

void AWLPVPArena::EnsureLighting()
{
	// ---- 比赛场地照明（用户要求 2026-10-09）："场地内不能有阴影存在，且光照不能太亮"。
	// 做法：三盏**不投影**的平行光（一主两补，互成 120 度）把场地铺匀，亮度总和压在 4.6；
	// "不投影"是关键 —— 有影子的话车和掩体会在地面上拖黑块，既难看又会让人误判距离。
	// 这里只在对局地图里生成（AWLPVPArena 只在 RM_PVP 里 Build），不会动测试场的灯光。
	UWorld* W = GetWorld();
	if (!W)
	{
		return;
	}
	bool bHasSun = false;
	for (TActorIterator<ADirectionalLight> It(W); It; ++It)
	{
		bHasSun = true;
		break;
	}
	if (!bHasSun)
	{
		struct FLightSpec { float Pitch; float Yaw; float Intensity; };
		static const FLightSpec Specs[] =
		{
			{ -62.0f,  -35.0f, 2.2f },      // 主光：接近垂直，压住整体对比
			{ -26.0f,   85.0f, 1.2f },      // 补光 1
			{ -26.0f, -155.0f, 1.2f },      // 补光 2（与补光 1 相对，消灭单侧死黑）
		};
		FActorSpawnParameters SP;
		SP.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		for (const FLightSpec& S : Specs)
		{
			if (ADirectionalLight* L = W->SpawnActor<ADirectionalLight>(ADirectionalLight::StaticClass(),
				FTransform(FRotator(S.Pitch, S.Yaw, 0.0f), FVector(0.0f, 0.0f, 1200.0f)), SP))
			{
				if (ULightComponent* LC = L->GetLightComponent())
				{
					LC->SetIntensity(S.Intensity);
					LC->SetCastShadows(false);                 // <<< 场地里不留影子
					LC->SetMobility(EComponentMobility::Movable);
				}
			}
		}
		UE_LOG(LogTemp, Display,
			TEXT("WLARENA|lighting: 3 shadowless directional lights (2.2/1.2/1.2) - no shadows inside the arena by design"));
	}
	bool bHasSky = false;
	for (TActorIterator<ASkyLight> It(W); It; ++It)
	{
		bHasSky = true;
		break;
	}
	if (!bHasSky)
	{
		FActorSpawnParameters SP;
		SP.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		if (ASkyLight* Sky = W->SpawnActor<ASkyLight>(ASkyLight::StaticClass(), FTransform::Identity, SP))
		{
			if (USkyLightComponent* SC = Sky->GetLightComponent())
			{
				SC->SetIntensity(0.6f);                        // 一点点环境光，别把暗部提成灰
				SC->SetCastShadows(false);
				SC->SetMobility(EComponentMobility::Movable);
			}
		}
	}
}

int32 AWLPVPArena::Build()
{
	if (bBuilt)
	{
		return Spawned.Num();
	}
	bBuilt = true;
	UWorld* W = GetWorld();
	if (!W)
	{
		return 0;
	}

	// ---- 关卡里已经有场地了吗？（用户导入的真实 RM 地图有 1.2 万个 actor）
	// 有的话就**只补对局要素**（回血点 / 队伍出生点），不再往上面叠程序化生成的竞技场 ——
	// 否则真实地图上会凭空多出一圈围墙和一堆掩体。
	int32 ExistingMeshes = 0;
	for (TActorIterator<AStaticMeshActor> It(W); It; ++It)
	{
		if (++ExistingMeshes > 50)
		{
			break;
		}
	}
	const bool bLevelAlreadyBuilt = (ExistingMeshes > 50);
	if (bLevelAlreadyBuilt)
	{
		UE_LOG(LogTemp, Display,
			TEXT("WLARENA|the level already has geometry (%d+ static mesh actors) - skipping the procedural arena, keeping only pads + spawns"),
			ExistingMeshes);
	}

	if (!bLevelAlreadyBuilt)
	{
	EnsureLighting();

	// ---------------------------------------------------------------- 地面与围墙
	SpawnBox(FVector(0.0f, 0.0f, -10.0f), FVector(HalfLengthCm * 2.0f, HalfWidthCm * 2.0f, 20.0f),
		FLinearColor(0.16f, 0.17f, 0.19f), TEXT("PVP_Floor"));
	// 四周围墙：高 3 m，挡视线也挡子弹
	SpawnBox(FVector(HalfLengthCm + WallThickCm * 0.5f, 0.0f, WallHeightCm * 0.5f),
		FVector(WallThickCm, HalfWidthCm * 2.0f + WallThickCm * 2.0f, WallHeightCm), FLinearColor(0.22f, 0.23f, 0.26f), TEXT("PVP_Wall_XP"));
	SpawnBox(FVector(-HalfLengthCm - WallThickCm * 0.5f, 0.0f, WallHeightCm * 0.5f),
		FVector(WallThickCm, HalfWidthCm * 2.0f + WallThickCm * 2.0f, WallHeightCm), FLinearColor(0.22f, 0.23f, 0.26f), TEXT("PVP_Wall_XN"));
	SpawnBox(FVector(0.0f, HalfWidthCm + WallThickCm * 0.5f, WallHeightCm * 0.5f),
		FVector(HalfLengthCm * 2.0f, WallThickCm, WallHeightCm), FLinearColor(0.22f, 0.23f, 0.26f), TEXT("PVP_Wall_YP"));
	SpawnBox(FVector(0.0f, -HalfWidthCm - WallThickCm * 0.5f, WallHeightCm * 0.5f),
		FVector(HalfLengthCm * 2.0f, WallThickCm, WallHeightCm), FLinearColor(0.22f, 0.23f, 0.26f), TEXT("PVP_Wall_YN"));

	// ---------------------------------------------------------------- 中央地形跨越
	// 中间一座 7 m x 9 m 的高台（顶面 55 cm），两侧各一条 5 m 长的坡道上去。
	// 台子左右各留 4.5 m 的平地绕行路线 —— 走上面是捷径也是靶子，这是这张图的核心选择。
	SpawnBox(FVector(0.0f, 0.0f, PlatformHeightCm * 0.5f), FVector(700.0f, 900.0f, PlatformHeightCm),
		FLinearColor(0.28f, 0.30f, 0.34f), TEXT("PVP_Platform"));
	{
		const float RampLen = 500.0f;
		const float PitchDeg = FMath::RadiansToDegrees(FMath::Atan2(PlatformHeightCm, RampLen));
		const float RampCentreX = 350.0f + RampLen * 0.5f;
		for (int32 S = 0; S < 2; ++S)
		{
			const float Sign = (S == 0) ? +1.0f : -1.0f;
			UWorld* WW = GetWorld();
			FActorSpawnParameters SP;
			SP.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
			AStaticMeshActor* R = WW->SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass(),
				FTransform(FRotator(Sign * PitchDeg, 0.0f, 0.0f),
					FVector(Sign * RampCentreX, 0.0f, PlatformHeightCm * 0.5f)), SP);
			if (R)
			{
				R->SetMobility(EComponentMobility::Movable);
				UStaticMeshComponent* MC = R->GetStaticMeshComponent();
				if (UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube")))
				{
					MC->SetStaticMesh(Cube);
				}
				MC->SetWorldScale3D(FVector(RampLen / 100.0f, 4.0f, 0.20f));
				MC->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
				MC->SetCollisionProfileName(TEXT("BlockAll"));
				R->Tags.Add(FName(TEXT("PVP_Ramp")));
				Spawned.Add(R);
			}
		}
	}

	// ---------------------------------------------------------------- 掩体（一个象限定义，四个象限镜像）
	struct FCover { FVector2D XY; FVector2D SizeXY; float Height; float ZCentre; const TCHAR* Tag; };
	static const FCover Cover[] =
	{
		{ FVector2D(500.0f, 480.0f),  FVector2D(60.0f, 320.0f), 130.0f, 65.0f,  TEXT("PVP_CoverWall")  },
		{ FVector2D(700.0f, 250.0f),  FVector2D(140.0f, 140.0f),140.0f, 70.0f,  TEXT("PVP_CoverBox")   },
		{ FVector2D(260.0f, 700.0f),  FVector2D(320.0f, 60.0f), 70.0f,  35.0f,  TEXT("PVP_CoverLow")   },
		{ FVector2D(820.0f, 700.0f),  FVector2D(220.0f, 220.0f),45.0f,  22.5f,  TEXT("PVP_Ledge45")    },
		{ FVector2D(470.0f, 140.0f),  FVector2D(90.0f, 90.0f),  200.0f, 100.0f, TEXT("PVP_Pillar")     },
	};
	for (const FCover& C : Cover)
	{
		for (int32 Q = 0; Q < 4; ++Q)
		{
			const float Sx = (Q & 1) ? -1.0f : +1.0f;
			const float Sy = (Q & 2) ? -1.0f : +1.0f;
			SpawnBox(FVector(Sx * C.XY.X, Sy * C.XY.Y, C.ZCentre),
				FVector(C.SizeXY.X, C.SizeXY.Y, C.Height),
				FLinearColor(0.34f, 0.33f, 0.30f), C.Tag);
		}
	}

	}   // ---- end of "the level was empty, so build the procedural arena"

	// ================================================================ 全场景碰撞修复
	// 用户 2026-10-09："除了测试地图以外，其他的地图障碍都有碰撞问题，轮子会陷进去"。
	//
	// 实测他导入的真实地图（Content/模拟器地图.umap，12217 个 actor）：**2273 个静态网格里有
	// 2267 个根本没有简单碰撞**。没有简单碰撞、又没有复杂碰撞回退时，网格在物理上等于没有碰撞面，
	// 车就会直接穿进去/陷进去。Python 在 4.27 里改不了这个属性（EditorStaticMeshLibrary 没有
	// set_collision_complexity，三个候选属性名全部失败），但 C++ 可以直接写 mesh 资产的
	// ECollisionTraceFlag，所以在开局这里统一处理：
	//   * 每个静态网格组件：碰撞打开 + BlockAll；
	//   * 每个用到的网格资产：改成"用复杂碰撞当简单碰撞"（碰撞面 = 看得见的面）；
	// 只改内存里的运行时状态，不动磁盘上的资产。
	{
		TSet<UStaticMesh*> Seen;
		int32 Comps = 0;
		for (TActorIterator<AActor> It(W); It; ++It)
		{
			TArray<UStaticMeshComponent*> MCs;
			It->GetComponents<UStaticMeshComponent>(MCs);
			for (UStaticMeshComponent* MC : MCs)
			{
				if (!MC)
				{
					continue;
				}
				++Comps;
				MC->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
				MC->SetCollisionProfileName(TEXT("BlockAll"));
				if (UStaticMesh* Mesh = MC->GetStaticMesh())
				{
					if (!Seen.Contains(Mesh))
					{
						Seen.Add(Mesh);
						// 注意：这个属性在 UBodySetup 上，不在 UStaticMesh 上
						// （实测 C2039: 'CollisionTraceFlag' is not a member of 'UStaticMesh'）。
						// 就地包含头文件：这个 .cpp 原本不需要 BodySetup。
						#include "PhysicsEngine/BodySetup.h"
						if (UBodySetup* BS = Mesh->GetBodySetup())
						{
							BS->CollisionTraceFlag = ECollisionTraceFlag::CTF_UseComplexAsSimple;
						}
					}
				}
			}
		}
		UE_LOG(LogTemp, Display,
			TEXT("WLARENA|collision pass: %d component(s), %d unique mesh(es) -> complex-as-simple + BlockAll"),
			Comps, Seen.Num());
	}

	// ---------------------------------------------------------------- 回血点（对称 4 个）
	SpawnPad(FVector2D(-950.0f, 0.0f), EWLTeam::Red);      // 红方后场
	SpawnPad(FVector2D(+950.0f, 0.0f), EWLTeam::Blue);     // 蓝方后场
	SpawnPad(FVector2D(0.0f, -600.0f), EWLTeam::None);     // 中场（中立）
	SpawnPad(FVector2D(0.0f, +600.0f), EWLTeam::None);

	// ---------------------------------------------------------------- 出生点（每队 3 个，5v5 时循环使用）
	for (int32 i = 0; i < 3; ++i)
	{
		const float Y = -300.0f + i * 300.0f;              // -300 / 0 / +300
		SpawnTeamStart(FVector(-1100.0f, Y, 29.67f), EWLTeam::Red, i);
		SpawnTeamStart(FVector(+1100.0f, Y, 29.67f), EWLTeam::Blue, i);
	}

	// ---------------------------------------------------------------- 出生点交给裁判系统（复活用）
	if (AWLRefereeSystem* Ref = AWLRefereeSystem::Get(W))
	{
		Ref->SetTeamSpawns(EWLTeam::Red, SpawnsRed);
		Ref->SetTeamSpawns(EWLTeam::Blue, SpawnsBlue);
	}

	int32 Pairs = 0, Bad = 0;
	VerifySymmetry(Pairs, Bad);
	UE_LOG(LogTemp, Display,
		TEXT("WLARENA|built %d actor(s) | %s x %s cm | %d heal pad(s) | %d spawn(s)/team | symmetric pairs=%d bad=%d -> %s"),
		Spawned.Num(), *FString::FromInt((int32)(HalfLengthCm * 2.0f / 100.0f)),
		*FString::FromInt((int32)(HalfWidthCm * 2.0f / 100.0f)), Pads.Num(), SpawnsRed.Num(), Pairs, Bad,
		(Bad == 0) ? TEXT("SYMMETRIC") : TEXT("ASYMMETRIC *** "));
	return Spawned.Num();
}

bool AWLPVPArena::VerifySymmetry(int32& OutPairs, int32& OutBad) const
{
	OutPairs = 0;
	OutBad = 0;
	for (const AStaticMeshActor* A : Spawned)
	{
		if (!A)
		{
			continue;
		}
		const FVector L = A->GetActorLocation();
		if (L.X <= 1.0f)
		{
			continue;                       // 只看 x>0 的一半，另一半应该有孪生体
		}
		const FVector Mirror(-L.X, -L.Y, L.Z);
		bool bFound = false;
		for (const AStaticMeshActor* B : Spawned)
		{
			if (!B || B == A)
			{
				continue;
			}
			if (FVector::Dist(B->GetActorLocation(), Mirror) < 1.0f)
			{
				const FVector SA = A->GetActorScale3D();
				const FVector SB = B->GetActorScale3D();
				if (FVector::Dist(SA, SB) < 0.01f)
				{
					bFound = true;
				}
				break;
			}
		}
		++OutPairs;
		if (!bFound)
		{
			++OutBad;
			UE_LOG(LogTemp, Warning, TEXT("WLARENA|asymmetric block '%s' at (%.0f,%.0f,%.0f) has no mirrored twin"),
				*A->GetName(), L.X, L.Y, L.Z);
		}
	}
	return OutBad == 0;
}
