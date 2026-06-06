// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Simulation/Simulator.h"
#include "SimulationActor.generated.h"

UCLASS()
class TRAFFICCHAOS_API ASimulationActor : public AActor
{
	GENERATED_BODY()

public:

	// Sets default values for this actor's properties
	ASimulationActor();
	
	virtual void BeginPlay() override;
	
	virtual void Tick(float DeltaSeconds) override;
	void Debug_Draw(float DeltaSeconds);

private:
	
	TCSimulator Simulator;
	
	UPROPERTY(EditAnywhere, Category = "Simulation", meta = (ClampMin = 1, ClampMax = 100, UIMin = 1, UIMax = 100))
	int GridResolution = 1;
	
	UPROPERTY(EditAnywhere, Category = "Simulation", meta = (ClampMin = 1, UIMin = 1))
	float WorldSpan = 1;
	
	UPROPERTY(EditAnywhere, Category = "Simulation", meta = (ClampMin = 0, UIMin = 0))
	int EntityCount = 1;
	
	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bDrawDensityField = false;
	
	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bDrawPotentialField = false;
	
	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bDrawCellVelocityField = false;
	
	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bDrawDesiredVelocityField = false;
	
	UPROPERTY(EditAnywhere, Category = "Debug")
	bool bDrawEntities = true;
};
