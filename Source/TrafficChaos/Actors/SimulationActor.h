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

UENUM()
enum class ESimulatorType
{
	Baseline,
	Fast
};

struct FTCPedDensityVelocityCell
{
	FVector2f AvgVelocity = FVector2f::ZeroVector;
	int Density = 0;
};

struct FTCFrameVorticityMetric
{
	float BaselineVorticity = 0.0f;
	float TestVorticity = 0.0f;
	float Difference = 0.0f;
	FTCFrameVorticityMetric& operator+=(const FTCFrameVorticityMetric& VorticityMetric)
	{
		BaselineVorticity += VorticityMetric.BaselineVorticity;
		TestVorticity += VorticityMetric.TestVorticity;
		Difference += VorticityMetric.Difference;
		
		return *this;
	}

	FString ToString() const
	{
		return FString::Printf(TEXT("Baseline Vorticity : %f, Test Vorticity : %f, Difference : %f"),
			BaselineVorticity, TestVorticity, Difference);
	}
};

struct FTCMetrics
{
	float TotalAbsoluteDifferenceMetric = 0.0f;
	float TotalPathLengthMetric = 0.0f;
	float TotalInterPedDistanceMetric = 0.0f;
	FTCFrameVorticityMetric TotalVorticityMetric = {};
};


UCLASS()
class TRAFFICCHAOS_API ASimulationActor : public AActor
{
	GENERATED_BODY()

public:
	
	// Sets default values for this actor's properties 
	ASimulationActor();

	void Tick(float DeltaSeconds) override;
	
	UFUNCTION(CallInEditor, Category = "Simulation Commands")
	void SimulateFast();
	
	UFUNCTION(CallInEditor, Category = "Simulation Commands")
	void SimulateBaseline();

	UFUNCTION(CallInEditor, Category = "Simulation Commands")
	void PlayVisualisation();
	
	UFUNCTION(CallInEditor, Category = "Simulation Commands")
	void StopVisualisation();
	
	UFUNCTION(CallInEditor, Category = "Evaluation Commands")
	void Evaluate();

	UFUNCTION(CallInEditor, Category = "Evaluation Commands")
	void PlayEvaluationVisualisation();
	
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
	void InitPedDensityVelocityField(const TArray<FTCEntity>& EntityArray);

protected:
	
	void BeginPlay() override;
	
private:
	
	void InitialiseSimulation();
	void StartSimulator();
	void Simulate(float DeltaSeconds);
	void InitialiseEntityStartLocations();
	void DrawDebugBaseline();
	void DrawDebugFast();
	void MetricCompare(const TArray<FTCEntity>& Baseline, const TArray<FTCEntity>& Test);
	float CalcFrameAbsoluteDifferenceMetric(const TArray<FTCEntity>& Baseline, const TArray<FTCEntity>& Test) const;
	float CalcFramePathLengthMetric(const TArray<FTCEntity>& Baseline, const TArray<FTCEntity>& Test);
	float CalcFrameInterPedestrianDistanceMetric(const TArray<FTCEntity>& Baseline, const TArray<FTCEntity>& Test) const;
	FTCFrameVorticityMetric CalcFrameVorticityMetric(const TArray<FTCEntity>& Baseline, const TArray<FTCEntity>& Test);
	void ResetPedDensityVelocityField();

private:
	
	UPROPERTY(EditAnywhere, Category = "PIE Settings")
	ESimulatorType SimulatorType = ESimulatorType::Fast;
	
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
	int NumEntitiesPerGroup = 1;
	
	UPROPERTY(EditAnywhere, Category = "World Configuration")
	TArray<FTCSpawnConfiguration> SpawnConfigurations;

	UPROPERTY(EditAnywhere, Category = "World Configuration")
	TArray<FVector2f> WallConfigurations;
	
	UPROPERTY(EditAnywhere, Category = "World Configuration") 
	TArray<FTCDiscomfortZone> DiscomfortZones;
	
	UPROPERTY(EditAnywhere, Category = "Metrics")
	bool bNormaliseMetrics = false;
	
	UPROPERTY(EditAnywhere, Category = "Debug")
	FTCDebugSettings DebugSettings;
	
private: // Simulators
	
	TSharedPtr<TCBaselineContinuumCrowdSimulator> BaselineSimulator;
	TSharedPtr<TCFastContinuumCrowdSimulator> FastSimulator;
	TWeakPtr<TCSimulatorBase> CurrentSimulator;
	
private: // Simulation
	
	TArray<TArray<TPair<FVector2f, int>>> BaselineSimCache;
	TArray<TArray<TPair<FVector2f, int>>> FastSimCache;
	TArray<TArray<TPair<FVector2f, int>>> PrimarySimulationCache;
	FTimerHandle VizTimerHandle;
	FTimerHandle EvaluationVizTimerHandle;
	float ElapsedSimTime = 0;
	int VisualisationFrameIndex = 0;
	
private: // Entities
	
	TArray<FTCEntity> Entities;
	TArray<FColor> EntityColors;
	bool bIsUpdateEnabled = true;
	FRandomStream RandomStream;
	
private: // Metrics
	
	FTCMetrics Metrics;
	FRpSpatialData<FTCPedDensityVelocityCell> PedVelocityField;
	FRpSpatialData<int> PedDensityField;
	TArray<FVector2f> BaselinePreviousPositions;
	TArray<FVector2f> TestPreviousPositions;
	TArray<int> BaselineCollisions;
	TArray<int> TestCollisions;
};
