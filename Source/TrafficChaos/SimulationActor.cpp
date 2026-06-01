// Fill out your copyright notice in the Description page of Project Settings.

#include "SimulationActor.h"

#include "Kismet/KismetMathLibrary.h"

// Sets default values
ASimulationActor::ASimulationActor()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	Simulator.AddEntity();
}

void ASimulationActor::BeginPlay()
{
	Super::BeginPlay();
	Simulator.Initialize(20, 2000);
	
	const auto AddEntityAtRandomLocation = [this](const float MinRange, const float MaxRange) -> void 
	{
		const float X = UKismetMathLibrary::RandomFloatInRange(MinRange, MaxRange);
		const float Y = UKismetMathLibrary::RandomFloatInRange(MinRange, MaxRange);
	
		Simulator.AddEntity({X, Y});
	};

	int NumEntities = 10;
	while (NumEntities > 0)
	{
		AddEntityAtRandomLocation(0, 2000);
		--NumEntities;
	}
}

void ASimulationActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	Simulator.Update(DeltaSeconds);
	Simulator.Debug_Draw(GetWorld(), DeltaSeconds);
}
