// WLPVPArena.h
//
// PVP 试玩地图（26 m x 18 m，中心对称）+ 回血点。
//
// 用户要求（2026-10-09）："设计一个对称的地图，有掩体有地形跨越，地图长 26m，宽 18m，要求有回血点"。
//
// 做法：**程序化生成**，不是手搭关卡。原因很实际：
//   * 对称是可验证的 —— 几何按"半场"定义一次，四个象限镜像生成，生成后逐块检查镜像孪生体，
//     不对称会直接报出来（手搭的地图只能靠眼睛看）；
//   * 可 diff、可复现 —— 地图就是一张表（尺寸/位置/颜色），改数值就有新版，不需要美术资产；
//   * 与项目既有做法一致 —— QA 台架本来就是用 SpawnBox 程序化搭出来的（WLGameMode::EnsureTestRange）。
//
// 坐标：竞技场中心 = 世界原点。X 为长边（±1300 = ±13 m），Y 为短边（±900 = ±9 m），地面顶面 Z = 0。

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "WLCombat.h"
#include "WLPVPArena.generated.h"

class UBoxComponent;
class AStaticMeshActor;
class AWLRefereeSystem;

/**
 * 回血点。一个带触发体积的场地标记：车停进去（服务器判定）-> 通知裁判系统。
 * 队伍中立（双方都能用），位置由竞技场构建器对称放置。
 */
UCLASS()
class RM2019SIM_API AWLHealZone : public AActor
{
	GENERATED_BODY()

public:
	AWLHealZone();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

	/** 触发体积（半径由 SetPadSize 决定） */
	UPROPERTY(VisibleAnywhere, Category = "WL|PVP") UBoxComponent* Pad = nullptr;
	/** 视觉标记网格（薄板，地面上的发光圈用） */
	UPROPERTY(VisibleAnywhere, Category = "WL|PVP") UStaticMeshComponent* Marker = nullptr;

	UPROPERTY(EditAnywhere, Category = "WL|PVP") EWLTeam OwnerTeam = EWLTeam::None;   // None = 中立
	UPROPERTY(EditAnywhere, Category = "WL|PVP") FLinearColor PadColour = FLinearColor(0.1f, 0.9f, 0.6f, 1.0f);
	UPROPERTY(EditAnywhere, Category = "WL|PVP") float PadHalfSizeCm = 150.0f;

	void SetPadSize(float HalfSizeCm);

protected:
	UFUNCTION()
	void OnPadBegin(UPrimitiveComponent* OverlappedComp, AActor* Other, UPrimitiveComponent* OtherComp,
		int32 OtherBodyIndex, bool bFromSweep, const FHitResult& Sweep);
	UFUNCTION()
	void OnPadEnd(UPrimitiveComponent* OverlappedComp, AActor* Other, UPrimitiveComponent* OtherComp,
		int32 OtherBodyIndex);

private:
	AWLRefereeSystem* Referee = nullptr;
};

/**
 * 竞技场构建器。由 PVP GameMode 在服务器上生成一次：地面/围墙/掩体/中央地形跨越/回血点/出生点/灯光。
 * 所有几何都登记进 Spawned，生成完立刻做一次镜像对称自检。
 */
UCLASS()
class RM2019SIM_API AWLPVPArena : public AActor
{
	GENERATED_BODY()

public:
	AWLPVPArena();

	virtual void BeginPlay() override;

	// ---------------------------------------------------------------- 尺寸（cm）
	static constexpr float HalfLengthCm = 1300.0f;    // 26 m 长
	static constexpr float HalfWidthCm = 900.0f;      // 18 m 宽
	static constexpr float WallHeightCm = 300.0f;
	static constexpr float WallThickCm = 30.0f;
	/** 中央高台高度（地形跨越） */
	static constexpr float PlatformHeightCm = 55.0f;

	/** 生成整个竞技场（幂等；只在服务器上做）。返回生成的 actor 数。 */
	int32 Build();

	/** 出生点（供裁判系统复活用） */
	const TArray<FTransform>& GetTeamSpawns(EWLTeam Team) const;

	/** 镜像对称自检：每个 x>0 的块都必须在 (-x,-y) 找到尺寸相同的孪生体 */
	bool VerifySymmetry(int32& OutPairs, int32& OutBad) const;

	UPROPERTY() TArray<AStaticMeshActor*> Spawned;
	UPROPERTY() TArray<AWLHealZone*> Pads;

private:
	AStaticMeshActor* SpawnBox(const FVector& Centre, const FVector& SizeCm, const FLinearColor& Colour, const TCHAR* Tag);
	AWLHealZone* SpawnPad(const FVector2D& CentreXY, EWLTeam Team);
	void SpawnTeamStart(const FVector& Loc, EWLTeam Team, int32 Index);
	void EnsureLighting();

	bool bBuilt = false;
	TArray<FTransform> SpawnsRed;
	TArray<FTransform> SpawnsBlue;
};
