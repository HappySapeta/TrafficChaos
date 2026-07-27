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

USTRUCT()
struct FTCDebugSettings
{
	GENERATED_BODY()
	
	UPROPERTY(EditAnywhere)
	bool bDrawDensityField = false;
	
	UPROPERTY(EditAnywhere)
	bool bDrawPotentialField = false;
	
	UPROPERTY(EditAnywhere)
	bool bDrawCellVelocityField = false;
	
	UPROPERTY(EditAnywhere)
	bool bDrawDesiredVelocityField = false;
	
	UPROPERTY(EditAnywhere)
	bool bDrawEntities = true;
	
	UPROPERTY(EditAnywhere)
	bool bDrawTraces = false;

	UPROPERTY(EditAnywhere)
	bool bDrawWalls = false;

	UPROPERTY(EditAnywhere)
	bool bDrawDiscomfortZones = false;
	
	UPROPERTY(EditAnywhere)
	bool bDrawGrid = false;

	UPROPERTY(EditAnywhere, meta = (ClampMin = 0, UIMin = 0))
	int DebugGroupID = 0;
};

USTRUCT()
struct FTCDiscomfortZone
{
	GENERATED_BODY()
	
	UPROPERTY(EditAnywhere)
	FVector2f Coords;
	
	UPROPERTY(EditAnywhere)
	float Amount = 1.0f;
};

UCLASS()
class TRAFFICCHAOS_API ASimulationActor : public AActor
{
	GENERATED_BODY()

public:
	
	// Sets default values for this actor's properties
	ASimulationActor();
	
	virtual void Tick(const float DeltaSeconds) override;
	
	UFUNCTION(CallInEditor)
	void StartCollectingMetrics();
	
	UFUNCTION(CallInEditor)
	void StopAndSaveMetrics();

	UFUNCTION(BlueprintCallable)
	void SetUpdateEnabled(bool bValue);

	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
	
protected:
	
	virtual void BeginPlay() override;
	
private:
	
	void CollectMetrics();
	
	void SpawnEntities();
	
	void DrawDebugGraphics(const float DeltaSeconds);
	
private:
	
	UPROPERTY(EditAnywhere)
	int32 RandomSeed = 0;
	
	UPROPERTY(EditAnywhere, DisplayName = "Baseline Continuum Crowds")
	FTCBaselineSimulationParameters BaselineCrowdSimParams;
	
	UPROPERTY(EditAnywhere, DisplayName = "Enhanced Continuum Crowds")
	FTCSimulationParameters EnhancedCrowdSimParams;
	
	UPROPERTY(EditAnywhere, DisplayName = "Social Force Model")
	FTCSocialForceParameters SocialForceParams;
	
	UPROPERTY(EditAnywhere)
	TArray<FTCSpawnConfiguration> SpawnConfigurations;
	
	UPROPERTY(EditAnywhere)
	TArray<FVector2f> WallConfigurations;
	
	UPROPERTY(EditAnywhere)
	TArray<FTCDiscomfortZone> DiscomfortZones;
	
	UPROPERTY(EditAnywhere, DisplayName = "Debug Settings")
	FTCDebugSettings DebugSettings;
	
	UPROPERTY(EditAnywhere)
	FString TestName = TEXT("Default");
	
private:
	
	TCSimulator Simulator;
	TArray<FTCEntity> Entities;
	TArray<FColor> EntityColors;
	bool bIsUpdateEnabled = true;
	FRandomStream RandomStream;
	
	TArray<TArray<FVector2f>> Metric_Positions;
	TArray<TArray<float>> Metric_Distance;
	TArray<TArray<float>> Metric_InterPedDistance;
	bool bShouldCollectMetrics = false;
};
