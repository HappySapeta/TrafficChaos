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
	uint8 Discomfort;
	
	TArray<float> Potential;
	TArray<FVector2f> DesiredVelocity;
	TStaticArray<float, ANISOTROPY> CostField;
	bool bIsWall = false;
};

USTRUCT()
struct FTCFastSimulationParameters : public FTCSimulationParameters
{
	GENERATED_BODY()
	
	UPROPERTY(EditAnywhere)
	float PathCostConstant = 1;
	
	UPROPERTY(EditAnywhere)
	float TimeCostConstant = 1;
	
	UPROPERTY(EditAnywhere)
	float DiscomfortConstant = 1;
	
	UPROPERTY(EditAnywhere)
	float DensityConstant = 1;
	
	UPROPERTY(EditAnywhere)
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
	virtual FTCMemoryMetric GetMaxAllocatedSize() const override;
public:
	
	virtual void Initialize(const float NewWorldSpan, const int NewResolution, const int NewNumGroups, const TInstancedStruct<FTCSimulationParameters> Parameters, const FTCSocialForceParameters& SocialForceParameters) override;
	virtual void MoveEntites(TArray<FTCEntity>& Entities, const float DeltaTime) override;
	virtual void UpdateSimulation(const TArray<FTCEntity>& Entities) override;
	virtual void RegisterGoal(const int GroupID, const FVector2f& WorldLocation) override;
	virtual void RegisterWall(const FVector2f& WorldLocation) override;
	virtual void RegisterDiscomfort(const FVector2f& WorldLocation, const float Amount) override;

private:
	
	void UpdateDensityAndVelocityField(const TArray<FTCEntity>& Entities);
	void UpdateCostField();
	void UpdatePotentialField(int GroupID);
	void UpdateDesiredVelocityField(const int GroupID);
	float GetFiniteDifferenceApproximation(const FVector2f& Coords, const int GroupID);
	FTCCheapestNeighbor GetCheapestNeighbor(const FVector2f& Coords, EDirectionIndex First, EDirectionIndex Second, const int GroupID);
	TArray<FTCNeighbor<FTCFastCell>> GetNeighbors(const FVector2f& Coords);
	float GetSocialForceInfluence(const FVector2f& DesiredDirection, const FVector2f& Force);
	EDirectionIndex ConvertVectorToDirectionIndex(const FVector2f& Vector) const;
	FVector2f CalculatedDesiredVelocity(const FVector2f& GridLocation, const int GroupID);
	
	void Meta_UpdateCandidatesSize();
	void Meta_UpdateKnownsSize();

private:

	int NumGroups = 0;
	FRpSpatialData<FTCFastCell> Field;
	TMap<int, FVector2f> Goals;
	
	TSet<FTCFastCell*> Knowns;
	TDeque<FTCFastCell*> Candidates;
	
	FTCFastSimulationParameters SimParameters;
	FTCSocialForceParameters PedParameters;
	FRpImplicitGrid ImplicitGrid;
	
	TArray<FVector> EntityPositions;
	SIZE_T MaxCandidatesSize = 0;
	SIZE_T MaxKnownsSize = 0;
}; 