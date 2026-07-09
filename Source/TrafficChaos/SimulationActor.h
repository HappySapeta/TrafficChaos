// Copyright Anupam Sahu. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Simulation/Simulator.h"
#include "SimulationActor.generated.h"

USTRUCT(BlueprintType)
struct FTCSpawnConfiguration
{
	GENERATED_BODY()
	
	UPROPERTY(EditAnywhere)
	FVector2f Origin = {0, 0};
	
	UPROPERTY(EditAnywhere, meta = (ClampMin = 0, UIMin = 0))
	float SpawnRange = 1.0f;
	
	UPROPERTY(EditAnywhere, meta = (ClampMin = 0, ClampMax = 1, UIMin = 0, UIMax = 1))
	float SpawnAreaWidth = 1.0f;
	
	UPROPERTY(EditAnywhere, meta = (ClampMin = 0, ClampMax = 6.28, UIMin = 0, UIMax = 6.28))
	float Rotation = 0.0f;
	
	UPROPERTY(EditAnywhere)
	FVector2f Goal = {0, 0};
	
	UPROPERTY(EditAnywhere)
	FColor Color;
	
	UPROPERTY(EditAnywhere, meta = (ClampMin = 0, ClampMax = 100, UIMin = 0, UIMax = 100))
	int Amount = 1;
	
	UPROPERTY(EditAnywhere)
	FVector2f OverrideVelocity;
	
	UPROPERTY(EditAnywhere)
	bool bUseOverrideVelocity = false;
};

UCLASS()
class TRAFFICCHAOS_API ASimulationActor : public AActor
{
	GENERATED_BODY()

public:
	
	// Sets default values for this actor's properties
	ASimulationActor();
	
	virtual void Tick(const float DeltaSeconds) override;
	
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
	
protected:
	
	virtual void BeginPlay() override;
	
private:
	
	void SpawnEntities();
	
	void DrawDebugGraphics(const float DeltaSeconds);
	
private:
	
	UPROPERTY(EditAnywhere, Category = "Simulation Settings", meta = (ClampMin = 1, ClampMax = 100, UIMin = 1, UIMax = 100))
	int GridResolution = 1;
	
	UPROPERTY(EditAnywhere, Category = "Simulation Settings", meta = (ClampMin = 1, UIMin = 1))
	float WorldSpan = 1;
	
	UPROPERTY(EditAnywhere, Category = "Debug Settings")
	bool bDrawDensityField = false;
	
	UPROPERTY(EditAnywhere, Category = "Debug Settings")
	bool bDrawPotentialField = false;
	
	UPROPERTY(EditAnywhere, Category = "Debug Settings")
	bool bDrawCellVelocityField = false;
	
	UPROPERTY(EditAnywhere, Category = "Debug Settings")
	bool bDrawDesiredVelocityField = false;
	
	UPROPERTY(EditAnywhere, Category = "Debug Settings")
	bool bDrawEntities = true;
	
	UPROPERTY(EditAnywhere, Category = "Debug Settings")
	bool bDrawTraces = false;
	
	UPROPERTY(EditAnywhere, Category = "Debug Settings", meta = (ClampMin = 0, UIMin = 0))
	int DebugGroupID = 0;
	
	UPROPERTY(EditAnywhere, Category = "Simulation Settings")
	FTCSimulationParameters SimParameters;
	
	UPROPERTY(EditAnywhere, Category = "Simulation Settings")
	FTCSocialForceParameters PedParameters;
	
	UPROPERTY(EditAnywhere, Category = "Simulation Settings")
	TArray<FTCSpawnConfiguration> SpawnConfigurations;
	
private:
	
	TCSimulator Simulator;
	TArray<FTCEntity> Entities;
	TArray<FColor> EntityColors;
};
