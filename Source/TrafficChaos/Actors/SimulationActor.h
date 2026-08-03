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

	UFUNCTION(CallInEditor, Category = "Commands")
	void SimulateFast();
	
	UFUNCTION(CallInEditor, Category = "Commands")
	void SimulateBaseline();

	UFUNCTION(CallInEditor, Category = "Commands")
	void StartVisualisation();

	UFUNCTION(CallInEditor, Category = "Commands")
	void StopVisualisation();
	
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
	
private:
	
	void InitialiseSimulation();
	void StartSimulator();
	void Simulate(float DeltaSeconds);
	void PlayVisualisation();
	void CollectMetrics();
	void ResetMetrics();
	void InitialiseEntityStartLocations();
	void DrawDebugBaseline();
	void DrawDebugFast();
	
private:
	
	UPROPERTY(EditAnywhere, Category = "Simulation Settings")
	float SimulationTimeStep = 0.1f;
	
	UPROPERTY(EditAnywhere, Category = "Simulation Settings")
	float SimulationLength = 5.0f;
	
	UPROPERTY(EditAnywhere, Category = "Simulation Settings", meta = (UIMin = 1.0f, UIMax = 10.0f, ClampMin = 1.0f, ClampMax = 10.0f))
	float VisualisationPlaybackRate = 1.0f;
	
	UPROPERTY(EditAnywhere, Category = "Simulation Settings")
	int32 RandomSeed = 0;
	
	UPROPERTY(EditAnywhere, Category = "Simulation Settings")
	float WorldSpan = 10.0f;
	
	UPROPERTY(EditAnywhere, Category = "Simulation Settings")
	int Resolution = 2;
	
	UPROPERTY(EditAnywhere, Category = "Simulation Parameters")
	TInstancedStruct<FTCSimulationParameters> BaselineCrowdSimParams;
	
	UPROPERTY(EditAnywhere, Category = "Simulation Parameters")
	TInstancedStruct<FTCSimulationParameters> FastCrowdSimParams;
	
	UPROPERTY(EditAnywhere, Category = "Simulation Parameters")
	FTCSocialForceParameters SocialForceParams;
	
	UPROPERTY(EditAnywhere, Category = "World Configuration")
	TArray<FTCSpawnConfiguration> SpawnConfigurations;

	UPROPERTY(EditAnywhere, Category = "World Configuration")
	TArray<FVector2f> WallConfigurations;
	
	UPROPERTY(EditAnywhere, Category = "World Configuration") 
	TArray<FTCDiscomfortZone> DiscomfortZones;
	
	UPROPERTY(EditAnywhere, Category = "Metrics")
	bool bShouldCaptureMetrics = false;
	
	UPROPERTY(EditAnywhere, Category = "Metrics", meta = (EditCondition = "bShouldCaptureMetrics"))
	bool bShouldCaptureDistances = false;
	
	UPROPERTY(EditAnywhere, Category = "Metrics", meta = (EditCondition = "bShouldCaptureMetrics"))
	bool bShouldCapturePositions = false;
	
	UPROPERTY(EditAnywhere, Category = "Debug")
	FTCDebugSettings DebugSettings;
	
private: // Simulators
	
	TSharedPtr<TCBaselineContinuumCrowdSimulator> BaselineSimulator;
	TSharedPtr<TCFastContinuumCrowdSimulator> FastSimulator;
	TWeakPtr<TCSimulatorBase> CurrentSimulator;
	
private: // Simulation
	
	TArray<TArray<TPair<FVector2f, int>>> SimulationCache;
	FTimerHandle VizTimerHandle;
	float ElapsedSimTime = 0;
	int VisualisationFrameIndex = 0;
	
private: // Entities
	
	TArray<FTCEntity> Entities;
	TArray<FColor> EntityColors;
	bool bIsUpdateEnabled = true;
	FRandomStream RandomStream;
	
private: // Metrics
	
	TArray<TArray<FVector2f>> Metric_Positions;
	TArray<TArray<float>> Metric_Distance;
	TArray<TArray<float>> Metric_InterPedDistance;
	
private: // Simulated Annealing
	
	TInstancedStruct<FTCSimulationParameters> BaselineCrowdSimParamsCopy;
	TInstancedStruct<FTCSimulationParameters> FastCrowdSimParamsCopy;
};
