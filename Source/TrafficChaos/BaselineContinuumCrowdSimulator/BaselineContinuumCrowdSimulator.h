// Copyright Anupam Sahu. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "SpatialData.h"
#include "SpatialAcceleration/RpImplicitGrid.h"
#include "StructUtils/InstancedStruct.h"
#include "TrafficChaos/SimulatorBase/SimulatorBase.h"
#include "TrafficChaos/SimulatorBase/SocialForceModel.h"
#include "BaselineContinuumCrowdSimulator.generated.h"

struct FTCBaselineCell
{
	FVector2f Coords;
	
	float Density;
	FVector2f Velocity;
	float Discomfort;
	
	TArray<float> Potential;
	TArray<FVector2f> DesiredVelocity;
	TStaticArray<float, ANISOTROPY> SpeedField;
	TArray<TStaticArray<float, ANISOTROPY>> CostField;
	TArray<TStaticArray<float, ANISOTROPY>> PotentialGradient;
	bool bIsWall = false;
};

USTRUCT()
struct FTCBaselineSimParameters : public FTCSimulationParameters
{
	GENERATED_BODY()
	
	UPROPERTY(EditAnywhere)
	FFloatRange SpeedRange = FFloatRange(10.0f, 150.0f);
	
	UPROPERTY(EditAnywhere)
	FFloatRange DensityRange = FFloatRange(0.2f, 2.0f);
	
	UPROPERTY(EditAnywhere)
	double DensityExponent = 1.0f;
	
	UPROPERTY(EditAnywhere)
	int VelocityLookahead = 1;
	
	UPROPERTY(EditAnywhere)
	int DensityLookahead = 1;
	
	UPROPERTY(EditAnywhere)
	float PathCostConstant = 1;
	
	UPROPERTY(EditAnywhere)
	float TimeCostConstant = 1;
	
	UPROPERTY(EditAnywhere)
	float DiscomfortConstant = 1;
};

class TRAFFICCHAOS_API TCBaselineContinuumCrowdSimulator : public TCSimulatorBase
{
public:
	
	const FRpSpatialData<FTCBaselineCell>& GetFieldData() const
	{
		return Field;
	}

	void SetSimulationParameters(const TInstancedStruct<FTCSimulationParameters> Parameters) override
	{
		SimParameters = Parameters.Get<FTCBaselineSimParameters>();
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
	
	virtual void Initialize(const float NewWorldSpan, const int NewResolution, const int NewNumGroups) override;
	virtual void MoveEntites(TArray<FTCEntity>& Entities, const float DeltaTime) override;
	virtual void UpdateSimulation(const TArray<FTCEntity>& Entities, const float DeltaTime) override;
	virtual void RegisterGoal(const int GroupID, const FVector2f& WorldLocation) override;
	virtual void RegisterWall(const FVector2f& WorldLocation) override;
	virtual void RegisterDiscomfort(const FVector2f& WorldLocation, const float Amount) override;

private:
	
	void UpdateCostField(const int GroupID);
	void UpdatePotentialField_FM(const int GroupID);
	void UpdateSpeedField();
	void UpdateDensityAndVelocityField(const TArray<FTCEntity>& Entities);
	void UpdatePotentialGradient(const int GroupID);
	void UpdateDesiredVelocityField(const int GroupID);
	float GetFiniteDifferenceApproximation(const FVector2f& Coords, const int GroupID);
	FTCCheapestNeighbor GetCheapestNeighbor(const FVector2f& Coords, EDirectionIndex First, EDirectionIndex Second, const int GroupID);
	TArray<FTCNeighbor<FTCBaselineCell>> GetNeighbors(const FVector2f& Coords);
	float GetSocialForceInfluence(const FVector2f& DesiredDirection, const FVector2f& Force);

private:
	
	int NumGroups = 0;
	FRpSpatialData<FTCBaselineCell> Field;
	TMap<int, FVector2f> Goals;
	
	TSet<FTCBaselineCell*> Knowns;
	TArray<FTCBaselineCell*> Candidates;
	 
	FTCBaselineSimParameters SimParameters;
	FRpImplicitGrid ImplicitGrid;
	FTCSocialForceParameters PedParameters;
}; 