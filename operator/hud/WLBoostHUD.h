// WLBoostHUD.h
//
// The capacitor ("超级电容" boost) endurance bar.
//
// The project had no HUD class at all - the game is played through the pawn's own camera with
// GEngine on-screen debug messages only.  A boost that drains and recharges needs a readable gauge,
// so this is a minimal AHUD that draws one bar, bottom centre: the fill is the capacitor charge,
// cyan while boosting, green while recharging, and it turns red and blinks when the capacitor is
// empty and the boost has cut out.
//
// It reads the state from the possessed AWLVehiclePawn every frame; it never writes anything.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "WLBoostHUD.generated.h"

UCLASS()
class RM2019SIM_API AWLBoostHUD : public AHUD
{
	GENERATED_BODY()

public:
	virtual void DrawHUD() override;
};
