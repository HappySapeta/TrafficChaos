// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Simulation/Simulator.h"
#include "SimulationActor.generated.h"

USTRUCT(BlueprintType)
struct FTCEntitySpawnConfiguration
{
	GENERATED_BODY()
	
	UPROPERTY(EditAnywhere)
	FVector2f InitialPosition;
	
	UPROPERTY(EditAnywhere)
	FVector2f InitialVelocity;
};

UCLASS()
class TRAFFICCHAOS_API ASimulationActor : public AActor
{
	GENERATED_BODY()

public:

	// Sets default values for this actor's properties
	ASimulationActor();
	
	virtual void Tick(const float DeltaSeconds) override;
	
protected:
	
	virtual void BeginPlay() override;
	
private:
	
	void DrawDebugGraphics(const float DeltaSeconds);

private:
	
	TCSimulator Simulator;
	
	UPROPERTY(EditAnywhere, Category = "Entities")
	TArray<FTCEntitySpawnConfiguration> SpawnConfigurations;
	
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
