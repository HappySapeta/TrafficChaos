// Copyright Anupam Sahu. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "SpatialData.h"
#include "Containers/Deque.h"
#include "SpatialAcceleration/RpImplicitGrid.h"
#include "TrafficChaos/SimulatorBase/SimulatorBase.h"
#include "TrafficChaos/SimulatorBase/SocialForceModel.h"
#include "FastContinuumCrowdSimulator.generated.h"

struct FTCFastCell
{
	FVector2f Coords;
	
	uint8 ByteDensity;
	EDirectionIndex Direction;
	float Discomfort;
	
	TArray<float> Potential;
	TArray<FVector2f> DesiredVelocity;
	TArray<TStaticArray<float, ANISOTROPY>> CostField;
	TArray<TStaticArray<float, ANISOTROPY>> PotentialGradient;
	bool bIsWall = false;
};

USTRUCT()
struct FTCFastSimulationParameters : public FTCSimulationParameters
{
	GENERATED_BODY()
	
	UPROPERTY(EditAnywhere, meta = (ClampMin = 0, ClampMax = 1, UIMin = 0, UIMax = 1))
	float PathCostConstant = 1;
	
	UPROPERTY(EditAnywhere, meta = (ClampMin = 0, ClampMax = 1, UIMin = 0, UIMax = 1))
	float TimeCostConstant = 1;
	
	UPROPERTY(EditAnywhere, meta = (ClampMin = 0, ClampMax = 1, UIMin = 0, UIMax = 1))
	float DiscomfortConstant = 1;
	
	UPROPERTY(EditAnywhere, meta = (ClampMin = 0, ClampMax = 1, UIMin = 0, UIMax = 1))
	float DensityConstant = 1;
	
	UPROPERTY(EditAnywhere, meta = (ClampMin = 1, UIMin = 1))
	int DensityLookahead = 1;
	
	UPROPERTY(EditAnywhere, meta = (ClampMin = 1, UIMin = 1))
	int VelocityLookahead = 1;
	
	UPROPERTY(EditAnywhere, meta = (ClampMin = 0, UIMin = 0))
	float DensityExponent = 1.0f;
};

class TRAFFICCHAOS_API TCFastContinuumCrowdSimulator : public TCSimulatorBase
{
public:
	
	const FRpSpatialData<FTCFastCell>& GetFieldData() const
	{
		return Field;
	}
	
	void SetSimulationParameters(const TInstancedStruct<FTCSimulationParameters> Parameters) override
	{
		SimParameters = Parameters.Get<FTCFastSimulationParameters>();
	}
	void SetAdvectionParameters(const FTCSocialForceParameters& Parameters) override
	{
		PedParameters = Parameters;
	}
	const FRpImplicitGrid& GetImplicitGrid() const
	{
		return ImplicitGrid;
	}
	
public:
	
	virtual void Initialize(const float NewWorldSpan, const int NewResolution, const int NewNumGroups, const TInstancedStruct<FTCSimulationParameters> Parameters, const FTCSocialForceParameters& SocialForceParameters) override;
	virtual void MoveEntites(TArray<FTCEntity>& Entities, const float DeltaTime) override;
	virtual void UpdateSimulation(const TArray<FTCEntity>& Entities, const float DeltaTime) override;
	virtual void RegisterGoal(const int GroupID, const FVector2f& WorldLocation) override;
	virtual void RegisterWall(const FVector2f& WorldLocation) override;
	virtual void RegisterDiscomfort(const FVector2f& WorldLocation, const float Amount) override;

private:
	
	void UpdateDensityAndVelocityField(const TArray<FTCEntity>& Entities);
	void UpdateCostField(const int GroupID);
	void UpdatePotentialField(int GroupID);
	void UpdatePotentialGradient(const int GroupID);
	void UpdateDesiredVelocityField(const int GroupID);
	float GetFiniteDifferenceApproximation(const FVector2f& Coords, const int GroupID);
	FTCCheapestNeighbor GetCheapestNeighbor(const FVector2f& Coords, EDirectionIndex First, EDirectionIndex Second, const int GroupID);
	TArray<FTCNeighbor<FTCFastCell>> GetNeighbors(const FVector2f& Coords);
	float GetSocialForceInfluence(const FVector2f& DesiredDirection, const FVector2f& Force);
	EDirectionIndex ConvertVectorToDirectionIndex(const FVector2f& Vector) const;

	int NumGroups = 0;
	FRpSpatialData<FTCFastCell> Field;
	TMap<int, FVector2f> Goals;
	
	TArray<FTCFastCell*> Knowns;
	TDeque<FTCFastCell*> Candidates;
	
	FTCFastSimulationParameters SimParameters;
	FTCSocialForceParameters PedParameters;
	FRpImplicitGrid ImplicitGrid;
	
}; 