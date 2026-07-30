// Copyright Anupam Sahu. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "StructUtils/InstancedStruct.h"
#include "TrafficChaos/BaselineContinuumCrowdSimulator/BaselineContinuumCrowdSimulator.h"
#include "TrafficChaos/FastContinuumCrowdSimulator/FastContinuumCrowdSimulator.h"
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

UENUM()
enum class ESimulatorType
{
	Baseline,
	Fast
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
	void DrawDebugBaseline(float DeltaSeconds);
	void DrawDebugFast(float DeltaSeconds);
	
protected:
	
	virtual void BeginPlay() override;
	
private:
	
	void CollectMetrics();
	void SpawnEntities();
	void InitializeSimulator(TSharedPtr<TCSimulatorBase> Simulator, TInstancedStruct<FTCSimulationParameters> Parameters);
	
private:
	
	UPROPERTY(EditAnywhere, Category = "Common")
	int32 RandomSeed = 0;
	
	UPROPERTY(EditAnywhere, Category = "Common")
	ESimulatorType SimulatorType = ESimulatorType::Fast; 
	
	
	UPROPERTY(EditAnywhere, Category = "Common")
	float WorldSpan = 10.0f;
	
	UPROPERTY(EditAnywhere, Category = "Common")
	int Resolution = 2;
	
	UPROPERTY(EditAnywhere, Category = "Common")
	TArray<FTCSpawnConfiguration> SpawnConfigurations;
	
	UPROPERTY(EditAnywhere, Category = "Common")
	TArray<FVector2f> WallConfigurations;
	
	UPROPERTY(EditAnywhere, Category = "Common") 
	TArray<FTCDiscomfortZone> DiscomfortZones;
	
	UPROPERTY(EditAnywhere, Category = "Debug")
	FTCDebugSettings DebugSettings;
	
	UPROPERTY(EditAnywhere, Category = "Simulation Settings")
	TInstancedStruct<FTCSimulationParameters> BaselineCrowdSimParams;
	
	UPROPERTY(EditAnywhere, Category = "Simulation Settings")
	TInstancedStruct<FTCSimulationParameters> FastCrowdSimParams;
	
	UPROPERTY(EditAnywhere, Category = "Simulation Settings")
	FTCSocialForceParameters SocialForceParams;
	
private: // Simulators
	
	TSharedPtr<TCBaselineContinuumCrowdSimulator> BaselineSimulator;
	TSharedPtr<TCFastContinuumCrowdSimulator> FastSimulator;
	TWeakPtr<TCSimulatorBase> CurrentSimulator;
	
private: // Entities
	
	TArray<FTCEntity> Entities;
	TArray<FColor> EntityColors;
	bool bIsUpdateEnabled = true;
	FRandomStream RandomStream;
	
private: // Metrics
	
	TArray<TArray<FVector2f>> Metric_Positions;
	TArray<TArray<float>> Metric_Distance;
	TArray<TArray<float>> Metric_InterPedDistance;
	bool bShouldCollectMetrics = false;
};
