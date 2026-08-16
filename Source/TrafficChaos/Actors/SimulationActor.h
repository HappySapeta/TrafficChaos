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

struct FTCPedVelocityCell
{
	FVector2f AvgVelocity = FVector2f::ZeroVector;
	int Density = 0;
};

struct FTCPedDensityCell
{
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
		return FString::Printf(TEXT("%.2f,%.2f,%.2f"),
			BaselineVorticity, TestVorticity, Difference);
	}
};

struct FTCCollisionsMetric
{
	float BaselineCollisions = 0;
	float TestCollisions = 0;
	
	FTCCollisionsMetric& operator+=(const FTCCollisionsMetric& CollisionsMetric)
	{
		BaselineCollisions += CollisionsMetric.BaselineCollisions;
		TestCollisions += CollisionsMetric.TestCollisions;
		
		return *this;
	}
	
	FString ToString() const
	{
		return FString::Printf(TEXT("%.2f,%.2f"),
			BaselineCollisions, TestCollisions);
	}
};

struct FTCDensityMetric
{
	float BaselineAvgDensity = 0;
	float TestAvgDensity = 0;
	
	FTCDensityMetric& operator+=(const FTCDensityMetric& DensityMetric)
	{
		BaselineAvgDensity += DensityMetric.BaselineAvgDensity;
		TestAvgDensity += DensityMetric.TestAvgDensity;
		
		return *this;
	}
	
	FString ToString() const
	{
		return FString::Printf(TEXT("%.2f,%.2f"), BaselineAvgDensity, TestAvgDensity);
	}
};

struct FTCSpeedMetric
{
	float BaselineAvgSpeed = 0.0f;
	float TestAvgSpeed = 0.0f;
	
	FTCSpeedMetric& operator+=(const FTCSpeedMetric& SpeedMetric)
	{
		BaselineAvgSpeed += SpeedMetric.BaselineAvgSpeed;
		TestAvgSpeed += SpeedMetric.TestAvgSpeed;
		
		return *this;
	}
	
	FString ToString() const
	{
		return FString::Printf(TEXT("%.2f,%.2f"), BaselineAvgSpeed, TestAvgSpeed);
	}
};

struct FTCPathLengthMetric
{
	float BaselinePathLength = 0.0f;
	float TestPathLength = 0.0f;
	float Difference = 0.0f;
	
	FTCPathLengthMetric& operator+=(const FTCPathLengthMetric& PathMetric)
	{
		BaselinePathLength += PathMetric.BaselinePathLength;
		TestPathLength += PathMetric.TestPathLength;
		Difference += PathMetric.Difference;
		
		return *this;
	}
	
	FString ToString() const
	{
		return FString::Printf
		(
			TEXT("%.2f,%.2f,%.2f"),
			BaselinePathLength, 
			TestPathLength,
			Difference
		);
	}
};

struct FTCInterPedDistanceMetric
{
	float BaselinePedDistance = 0.0f;
	float TestPedDistance = 0.0f;
	float Difference = 0.0f;
	
	FTCInterPedDistanceMetric& operator+=(const FTCInterPedDistanceMetric& InterPedMetric)
	{
		BaselinePedDistance += InterPedMetric.BaselinePedDistance;
		TestPedDistance += InterPedMetric.TestPedDistance;
		Difference += InterPedMetric.Difference;
		
		return *this;
	}
	
	FString ToString() const
	{
		return FString::Printf
		(
			TEXT("%.2f,%.2f,%.2f"), 
			BaselinePedDistance, 
			TestPedDistance,
			Difference
		);
	}
};

struct FTCMetrics
{
	float TotalAbsoluteDifferenceMetric = 0.0f;
	FTCInterPedDistanceMetric TotalPedDistanceMetric = {};
	FTCPathLengthMetric TotalPathLengthMetric = {};
	FTCFrameVorticityMetric TotalVorticityMetric = {};
	FTCCollisionsMetric TotalCollisionsMetric = {};
	FTCDensityMetric TotalDensityMetric = {};
	FTCSpeedMetric TotalSpeedMetric = {};
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

protected:
	
	void BeginPlay() override;
	
private:
	
	void InitialiseSimulation();
	void NormaliseMetrics(int NumFrames);
	void StartSimulator();
	void Simulate(float DeltaSeconds);
	void InitialiseEntityStartLocations();
	void DrawDebugBaseline();
	void DrawDebugFast();
	float CalcFrameAbsoluteDifferenceMetric(const TArray<FTCEntity>& Baseline, const TArray<FTCEntity>& Test) const;
	FTCInterPedDistanceMetric CalcFrameInterPedestrianDistanceMetric(const TArray<FTCEntity>& Baseline, const TArray<FTCEntity>& Test) const;
	FTCPathLengthMetric CalcFramePathLengthMetric(const TArray<FTCEntity>& Baseline, const TArray<FTCEntity>& Test);
	FTCDensityMetric CalcFrameAverageDensityMetric(const TArray<FTCEntity>& Baseline, const TArray<FTCEntity>& Test);
	FTCFrameVorticityMetric CalcFrameVorticityMetric(const TArray<FTCEntity>& Baseline, const TArray<FTCEntity>& Test);
	FTCCollisionsMetric CalcFrameCollisionsMetric(const TArray<FTCEntity>& Baseline, const TArray<FTCEntity>& Test) const;
	FTCSpeedMetric CalcFrameAverageSpeedMetric(const TArray<FTCEntity>& Baseline, const TArray<FTCEntity>& Test);
	void InitPedVelocityField(const TArray<FTCEntity>& EntityArray);
	void ResetPedDensityVelocityField();
	void InitPedDensityField(const TArray<FTCEntity>& EntityArray);
	void ResetPedDensityField();

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
	FRpSpatialData<FTCPedVelocityCell> PedVelocityField;
	FRpSpatialData<FTCPedDensityCell> PedDensityField;
	TArray<FVector2f> BaselinePreviousPositions;
	TArray<FVector2f> TestPreviousPositions;
	TArray<float> Speeds;
};
