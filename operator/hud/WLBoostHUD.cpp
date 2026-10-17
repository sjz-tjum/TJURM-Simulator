// WLBoostHUD.cpp
#include "WLBoostHUD.h"
#include "WLVehiclePawn.h"
// 画"视觉端回传的检测结果"需要完整类型（FWLAimNet 在头文件里只是前向声明）
#include "WLAimNet.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/Font.h"

// ---- first-person reticle: ( + ) with the heat gauges OUTSIDE the brackets, same curvature.
//
// The FPV camera looks along the muzzle axis, so the screen centre IS the aim point.  The brackets
// are arcs of one circle centred there, and the heat gauges use the SAME centre at a larger radius,
// which is what makes them follow the bracket's curvature instead of sitting in a straight bar
// beside it.  Both gauges fill from the bottom end upward, like a thermometer.
static void DrawArc(AHUD* Hud, float Cx, float Cy, float R, float A0Deg, float A1Deg,
	FLinearColor Col, float Thick)
{
	const int32 Steps = FMath::Max(2, FMath::RoundToInt(FMath::Abs(A1Deg - A0Deg) / 4.0f));
	float Px = Cx + R * FMath::Cos(FMath::DegreesToRadians(A0Deg));
	float Py = Cy + R * FMath::Sin(FMath::DegreesToRadians(A0Deg));
	for (int32 i = 1; i <= Steps; ++i)
	{
		const float A = FMath::DegreesToRadians(FMath::Lerp(A0Deg, A1Deg, static_cast<float>(i) / Steps));
		const float X = Cx + R * FMath::Cos(A);
		const float Y = Cy + R * FMath::Sin(A);
		if (!FMath::IsNearlyEqual(Px, X) || !FMath::IsNearlyEqual(Py, Y))
		{
			Hud->DrawLine(Px, Py, X, Y, Col, Thick);
		}
		Px = X;
		Py = Y;
	}
}

static void DrawReticle(AHUD* Hud, float Cx, float Cy, float Pct, bool bOverheated, float TimeSec)
{
	const float RParen = 24.0f;             // bracket radius
	const float RHeat = 34.0f;              // gauge radius: OUTSIDE the bracket
	const float Span = 62.0f;               // degrees either side of the horizontal
	const FLinearColor Bracket(0.10f, 1.00f, 0.35f, 0.85f);

	// "+" in the middle
	const float Arm = 7.0f;
	Hud->DrawLine(Cx - Arm, Cy, Cx + Arm, Cy, Bracket, 1.6f);
	Hud->DrawLine(Cx, Cy - Arm, Cx, Cy + Arm, Bracket, 1.6f);

	// the brackets, opening toward the centre
	DrawArc(Hud, Cx, Cy, RParen, -Span, +Span, Bracket, 1.6f);                    // "(" on the left
	DrawArc(Hud, Cx, Cy, RParen, 180.0f - Span, 180.0f + Span, Bracket, 1.6f);    // ")" on the right

	// heat gauges: same centre and span, larger radius, filled from the bottom up
	FLinearColor HeatCol;
	if (bOverheated)
	{
		const float Blink = 0.45f + 0.55f * FMath::Abs(FMath::Sin(TimeSec * 7.0f));
		HeatCol = FLinearColor(1.0f, 0.15f, 0.10f, 0.45f + 0.55f * Blink);
	}
	else if (Pct < 0.5f) { HeatCol = FLinearColor(0.25f, 0.85f, 0.35f, 0.90f); }
	else if (Pct < 0.8f) { HeatCol = FLinearColor(1.00f, 0.80f, 0.15f, 0.90f); }
	else { HeatCol = FLinearColor(1.00f, 0.35f, 0.10f, 0.95f); }

	const float T = FMath::Clamp(Pct, 0.0f, 1.0f);
	// +Span is the BOTTOM end of the right-hand arc, so the gauge grows upward as heat rises; the
	// left gauge is its mirror about the vertical, i.e. [180-Span, 180-End].  Writing that end as
	// 180+(End-Span) instead put it 180-80.6 = 99.4 deg, which drew a 19 deg stub instead of the
	// 80.6 deg the right side had.
	const float End = Span - 2.0f * Span * T;
	DrawArc(Hud, Cx, Cy, RHeat, End, +Span, HeatCol, 3.0f);                       // right gauge
	DrawArc(Hud, Cx, Cy, RHeat, 180.0f - Span, 180.0f - End, HeatCol, 3.0f);      // left gauge

	if (GEngine && GEngine->GetMediumFont())
	{
		const FString Txt = bOverheated
			? FString::Printf(TEXT("OVERHEAT  %.0f%%"), Pct * 100.0f)
			: FString::Printf(TEXT("%.0f%%"), Pct * 100.0f);
		Hud->DrawText(Txt, bOverheated ? FLinearColor(1.0f, 0.3f, 0.25f) : FLinearColor(0.85f, 0.9f, 0.85f),
			Cx - 22.0f, Cy + RHeat + 5.0f, GEngine->GetMediumFont(), 1.0f, false);
	}
}

void AWLBoostHUD::DrawHUD()
{
	Super::DrawHUD();
	if (!Canvas)
	{
		return;                                   // headless (-nullrhi) runs have no canvas
	}
	const AWLVehiclePawn* V = Cast<AWLVehiclePawn>(GetOwningPawn());
	if (!V)
	{
		return;
	}

	const float Now = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;

	// ---- the target car's health, floating above it.
	//
	// For "can I hit another robot's armour plates" the hit has to be visible where it happens, not
	// only in the corner bar (which shows the PLAYER's HP).  Projected to screen, so it tracks the
	// car as the camera moves.
	// ---- 自瞄相机画面（WLAimView 1）：直接把模型看到的那张图铺在屏幕中间偏右。
	// 这是"怎么进自瞄"的答案之一：不用命令行开关，Play 进去输入 WLAimView 1 就能看自瞄视角。
	if (UTexture* AimTex = V->GetAimViewTexture())
	{
		const float VW = Canvas ? Canvas->SizeX : 1280.0f;
		const float VH = Canvas ? Canvas->SizeY : 720.0f;
		const float SW = VW * 0.42f;
		const float SH = SW * (1024.0f / 1280.0f);
		const float SX = VW - SW - 16.0f;
		DrawRect(FLinearColor(0.0f, 0.0f, 0.0f, 0.45f), SX - 6.0f, 16.0f - 6.0f, SW + 12.0f, SH + 12.0f);
		DrawTexture(AimTex, SX, 16.0f, SW, SH, 0.0f, 0.0f, 1.0f, 1.0f);
		DrawText(TEXT("AIM CAM 1280x1024 f=1250 (WLAimView 0 to hide)"), FLinearColor(0.6f, 0.95f, 1.0f), SX, 16.0f + SH + 4.0f, nullptr, 1.0f);

		// ---- 视觉端回传的检测（灯条配对出的四角点）画在自瞄画面上 ----
		// 用户 2026-10-11："即使灯条在我视野里，按右键也不显示灯条" ⇒ 必须看得见识别结果。
		// 检测坐标是自瞄相机像素（1280x1024），按 overlay 矩形等比映射。
		if (FWLAimNet* Net = V->GetAimNetForHUD())
		{
			// ★ 帧号对齐（用户要求：云台晃时框不能跟着晃 ✗）。
			// 屏幕上的图是"更早抓到的那一帧"，而检测是"最新收到的那一帧" ⇒ 云台一动就错位 ✓。
			// 只画与当前已发送帧号相差 ≤2 帧的检测 ⇒ 框钉在装甲板上 ✓；
			// 同时超时（LastDetRecvTime）保证"没有目标时框消失" ✓。
			// ★ 图 / 框 严格同帧（用户要求[改成实时的]）：
			// 预览显示的是 RT 环里滞后 AimDisplayDelay 帧的那张（GetAimViewFrameId），
			// 所以只画帧号与它一致（容差 1 帧）的检测 ⇒ 云台怎么晃，框都钉在装甲板上 ✓✓。
			// 若框相对画面略早/略晚，用 WLAimDelay <0..7> 现场滑显示窗口即可 ✓（不用重编译）。
			// [回退] 环未启用（ViewFid<0）时就退回"按时间新鲜度画"（改动前的行为，能看见但云台动时略飘）
			// ★ 2026-10-13：容差 1 -> 8（用户报"绿框一直频闪"）
			//   原因：仿真发帧率提高后，视觉端处理总是落后几帧 ⇒ |ViewFid − DetFrameId| > 1
			//   的帧占绝大多数 ⇒ HUD 判定"不新鲜"就不画 ⇒ 框一闪一闪 ✗
			//   容差放宽到 8 帧即可稳定显示（框的时延靠下面的运动补偿补，不影响观感）✓
			const int32 ViewFid = V->GetAimViewFrameId();
			const bool bFresh = (Net->LastDetRecvTime > 0.0) &&
				((FPlatformTime::Seconds() - Net->LastDetRecvTime) < 0.25) &&
				((ViewFid < 0) ? (Net->LastFrameId - Net->DetFrameId <= 12)
				               : (FMath::Abs(ViewFid - Net->DetFrameId) <= 8));
			// ★ 运动补偿（用户："云台一晃绿框就飘，停下来又回到板上" ✓ = 检测比画面晚几帧）。
			// 用"检测帧 -> 现在"的云台转角差把四个角点平移回去：du=-f*dyaw，dv=+f*dpitch ✓
			float ShiftU = 0.0f, ShiftV = 0.0f;
			{
				float Y0 = 0.0f, P0 = 0.0f, Y1 = 0.0f, P1 = 0.0f;
				const bool bHave0 = Net->GetFrameSample(Net->DetFrameId, Y0, P0);
				const bool bHave1 = Net->GetFrameSample(Net->LastFrameId, Y1, P1);
				if (bHave0 && bHave1)
				{
					const float F = 1250.0f;      // 自瞄相机 f_px（1280x1024, HFOV 54.22 度）
					// 系数由 WLAimComp 控制（默认 0 = 不补偿，回到"能看见但略飘"的已知状态 ✓）
					const float G = V->AimBoxCompGain;
					ShiftU = -F * (Y1 - Y0) * G;
					ShiftV = F * (P1 - P0) * G;
				}
			}
			const TArray<FVector2D>& D = Net->GetDetections();
			const int32 N = bFresh ? (D.Num() / 4) : 0;
			for (int32 i = 0; i < N; ++i)
			{
				FVector2D Pts[4];
				for (int32 k = 0; k < 4; ++k)
				{
					const FVector2D& q = D[i * 4 + k];
					Pts[k] = FVector2D(SX + ((q.X + ShiftU) / 1280.0f) * SW,
						16.0f + ((q.Y + ShiftV) / 1024.0f) * SH);
				}
				const FLinearColor Col(0.1f, 1.0f, 0.1f, 1.0f);
				for (int32 k = 0; k < 4; ++k)
				{
					const FVector2D& A = Pts[k];
					const FVector2D& B = Pts[(k + 1) % 4];
					DrawLine(A.X, A.Y, B.X, B.Y, Col, 2.0f);
				}
			}
			// "数据有多新"直接打出来：有角点却显示 no data ⇒ 是回传断了 → 一眼能分辨
			float DetAge = -1.0f;
			if (FWLAimNet* Net2 = V->GetAimNetForHUD())
			{
				if (Net2->LastDetRecvTime > 0.0)
				{
					DetAge = (float)(FPlatformTime::Seconds() - Net2->LastDetRecvTime);
				}
			}
			DrawText(FString::Printf(TEXT("LC bars detected: %d plate(s)  |det %.1fs ago|"),
				N, DetAge),
				(N > 0) ? FLinearColor(0.1f, 1.0f, 0.1f, 1.0f) : FLinearColor(1.0f, 0.5f, 0.2f, 1.0f),
				SX, 16.0f + SH + 18.0f, nullptr, 1.0f);
		}
	}

	if (AWLVehiclePawn* Dummy = V->TargetDummy.Get())
	{
		const FVector Above = Dummy->GetActorLocation() + FVector(0.0f, 0.0f, 95.0f);
		const FVector Screen = Project(Above);
		if (Screen.Z > 0.0f)                       // in front of the camera
		{
			const float DW = 90.0f, DH = 8.0f;
			const float DX = Screen.X - DW * 0.5f;
			const float DY = Screen.Y;
			const float DPct = Dummy->GetHealthPct();
			DrawRect(FLinearColor(0.02f, 0.03f, 0.04f, 0.75f), DX - 2.0f, DY - 2.0f, DW + 4.0f, DH + 4.0f);
			DrawRect(FLinearColor(0.10f, 0.11f, 0.13f, 0.85f), DX, DY, DW, DH);
			const FLinearColor DCol = (DPct > 0.60f) ? FLinearColor(0.20f, 0.85f, 0.30f, 0.95f)
				: (DPct > 0.30f) ? FLinearColor(1.00f, 0.80f, 0.15f, 0.95f)
				: FLinearColor(0.95f, 0.18f, 0.12f, 0.95f);
			DrawRect(DCol, DX, DY, DW * DPct, DH);
			if (UFont* F2 = GEngine ? GEngine->GetMediumFont() : nullptr)
			{
				// The target is a RUNTIME spawn, so it has no details panel and the console does not
				// open in this build: this line is the feedback that T (spin) and Y (regen) did
				// something.  Without it the keys look dead.
				DrawText(FString::Printf(TEXT("TARGET %.0f/%.0f  %s  %s%s"),
					Dummy->Health, Dummy->MaxHealth,
					Dummy->bTargetSpins ? TEXT("SPIN") : TEXT("stop"),
					Dummy->bRegenEnabled ? TEXT("regen") : TEXT("no-regen"),
					(Dummy->bRegenerating ? TEXT(" (healing)") : TEXT("")),
					(DPct <= 0.0f) ? TEXT("  DESTROYED") : TEXT("")),
					(DPct <= 0.0f) ? FLinearColor(1.0f, 0.4f, 0.4f) : FLinearColor::White,
					DX, DY - 18.0f, F2, 1.0f, false);
			}
		}
	}

	// ---- first person: the ( + ) reticle with its heat gauges; third person: no reticle
	const bool bFirstPerson = V->IsFirstPerson();
	if (bFirstPerson && V->bHeatEnabled)
	{
		DrawReticle(this, Canvas->SizeX * 0.5f, Canvas->SizeY * 0.5f,
			V->GetHeatPct(), V->IsOverheated(), Now);
	}

	const float Pct = V->CapacitorPct;
	const bool bOn = V->IsCapacitorActive();
	const bool bEmpty = (Pct <= 0.001f);

	// ---- bottom LEFT stack: HP on top, then heat (third person only), then the capacitor.
	// The capacitor used to sit bottom-centre; the user asked for it in the bottom-left corner next
	// to the new health bar, so all the readouts live in one column at the left edge.  Every label
	// sits to the RIGHT of its own bar - stacking labels above bars collided as soon as there were
	// three of them (measured: the capacitor's label landed inside the HP bar).
	const float BarW = FMath::Clamp(Canvas->SizeX * 0.24f, 160.0f, 400.0f);
	const float RowHP = Canvas->SizeY - 108.0f;
	const float RowHeat = Canvas->SizeY - 74.0f;
	const float RowCap = Canvas->SizeY - 40.0f;
	const float X = 22.0f;
	const float LabelX = X + BarW + 10.0f;
	UFont* Font = GEngine ? GEngine->GetMediumFont() : nullptr;
	auto Label = [&](const FString& Txt, FLinearColor Col, float Y)
	{
		if (Font)
		{
			DrawText(Txt, Col, LabelX, Y, Font, 1.0f, false);
		}
	};

	// ---- HEALTH
	{
		const float HPct = V->GetHealthPct();
		const float HBarH = 18.0f;
		DrawRect(FLinearColor(0.02f, 0.03f, 0.04f, 0.75f), X - 4.0f, RowHP - 4.0f, BarW + 8.0f, HBarH + 8.0f);
		DrawRect(FLinearColor(0.10f, 0.11f, 0.13f, 0.85f), X, RowHP, BarW, HBarH);
		const FLinearColor HCol = (HPct > 0.60f) ? FLinearColor(0.20f, 0.85f, 0.30f, 0.95f)
			: (HPct > 0.30f) ? FLinearColor(1.00f, 0.80f, 0.15f, 0.95f)
			: FLinearColor(0.95f, 0.18f, 0.12f, 0.95f);
		DrawRect(HCol, X, RowHP, BarW * HPct, HBarH);
		for (int32 i = 1; i < 4; ++i)
		{
			DrawRect(FLinearColor(0.0f, 0.0f, 0.0f, 0.35f), X + BarW * (0.25f * i), RowHP, 1.0f, HBarH);
		}
		// a healing bar reads as "safe": the HP fill gets a soft green edge and the label says REGEN
		if (V->bRegenerating)
		{
			const float Pulse = 0.45f + 0.55f * FMath::Abs(FMath::Sin(Now * 4.0f));
			DrawRect(FLinearColor(0.35f, 1.00f, 0.45f, 0.25f + 0.35f * Pulse),
				X + BarW * HPct, RowHP, 2.0f, HBarH);
		}
		const FString RegenTag = V->bRegenerating
			? FString::Printf(TEXT("   REGEN +%.0f/s"), V->RegenPerSec) : FString();
		Label(FString::Printf(TEXT("HP  %3.0f / %.0f%s%s"), V->Health, V->MaxHealth,
			(HPct <= 0.0f) ? TEXT("   DESTROYED") : TEXT(""), *RegenTag),
			(HPct <= 0.30f) ? FLinearColor(1.0f, 0.5f, 0.45f)
				: (V->bRegenerating ? FLinearColor(0.6f, 1.0f, 0.7f) : FLinearColor::White), RowHP + 1.0f);
	}

	// ---- HEAT (third person only: in first person it is around the reticle)
	if (!bFirstPerson && V->bHeatEnabled)
	{
		const float HBarH = 12.0f;
		DrawRect(FLinearColor(0.02f, 0.03f, 0.04f, 0.75f), X - 4.0f, RowHeat - 4.0f, BarW + 8.0f, HBarH + 8.0f);
		DrawRect(FLinearColor(0.10f, 0.11f, 0.13f, 0.85f), X, RowHeat, BarW, HBarH);
		const float HT = V->GetHeatPct();
		const FLinearColor HCol = V->IsOverheated()
			? FLinearColor(1.0f, 0.15f, 0.10f, 0.95f)
			: (HT < 0.5f ? FLinearColor(0.25f, 0.85f, 0.35f, 0.9f)
				: (HT < 0.8f ? FLinearColor(1.0f, 0.80f, 0.15f, 0.9f) : FLinearColor(1.0f, 0.35f, 0.10f, 0.95f)));
		DrawRect(HCol, X, RowHeat, BarW * FMath::Clamp(HT, 0.0f, 1.0f), HBarH);
		Label(FString::Printf(TEXT("HEAT %3.0f%%%s"), HT * 100.0f,
			V->IsOverheated() ? TEXT("  OVERHEAT") : TEXT("")),
			V->IsOverheated() ? FLinearColor(1.0f, 0.45f, 0.45f) : FLinearColor::White, RowHeat - 1.0f);
	}

	// frame + track (capacitor: bottom row of the left-hand stack)
	const float BarH = 16.0f;
	const float Y = RowCap;
	DrawRect(FLinearColor(0.02f, 0.03f, 0.04f, 0.75f), X - 4.0f, Y - 4.0f, BarW + 8.0f, BarH + 8.0f);
	DrawRect(FLinearColor(0.10f, 0.11f, 0.13f, 0.85f), X, Y, BarW, BarH);

	// fill: cyan when boosting, green when recovering, blinking red when empty
	FLinearColor Fill;
	if (bEmpty)
	{
		const float Blink = 0.45f + 0.55f * FMath::Abs(FMath::Sin(GetWorld() ? GetWorld()->GetTimeSeconds() * 6.0f : 0.0f));
		Fill = FLinearColor(0.95f, 0.15f, 0.15f, 0.55f + 0.45f * Blink);
	}
	else if (bOn)
	{
		Fill = FLinearColor(0.18f, 0.85f, 1.00f, 0.95f);
	}
	else
	{
		Fill = FLinearColor(0.25f, 0.80f, 0.35f, 0.90f);
	}
	DrawRect(Fill, X, Y, BarW * FMath::Clamp(Pct, 0.0f, 1.0f), BarH);

	// a tick every 25 % so the charge can be read at a glance
	for (int32 i = 1; i < 4; ++i)
	{
		DrawRect(FLinearColor(0.0f, 0.0f, 0.0f, 0.35f), X + BarW * (0.25f * i), Y, 1.0f, BarH);
	}

	Label(FString::Printf(TEXT("CAP  %3.0f%%  %s"),
		Pct * 100.0f,
		bEmpty ? TEXT("EMPTY - recharging") : (bOn ? TEXT("BOOST x1.7 (SHIFT)") : TEXT("ready (SHIFT)"))),
		bEmpty ? FLinearColor(1.0f, 0.45f, 0.45f) : FLinearColor::White, Y + 1.0f);
}
