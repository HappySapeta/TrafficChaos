// Copyright Anupam Sahu. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "SpatialData.h"
#include "Containers/Deque.h"
#include "SocialForceModel.h"
#include "SimulationDataTypes.h"
#include "SpatialAcceleration/RpImplicitGrid.h"

class TRAFFICCHAOS_API TCSimulator
{
public:
 
	TCSimulator()
		:Field(1,0,{})
	{}
	
	const FRpSpatialData<FTCCell>& GetFieldData() const
	{
		return Field;
	}

#ifdef USE_BASELINE_MODEL
	void SetSimulationParameters(const FTCBaselineSimulationParameters& Parameters)
	{
		SimParameters = Parameters; 
	}
#else
	void SetSimulationParameters(const FTCSimulationParameters& Parameters)
	{
		SimParameters = Parameters; 
	}
#endif
	
	void SetAdvectionParameters(const FTCSocialForceParameters& Parameters)
	{
		PedParameters = Parameters;
	}
	
	const FRpImplicitGrid& GetImplicitGrid() const
	{
		return ImplicitGrid;
	}
	
	void Initialize(float Resolution, float WorldSize, int NumGroups);
	void RegisterGoal(const int GroupID, const FVector2f& Goal);
	void RegisterWall(const FVector2f& WallCoords);
	void RegisterDiscomfort(const FVector2f& WallCoords, const float Amount);
	void CrowdAdvection(TArray<FTCEntity>& Entities, float TimeStep);
	void Update(const TArray<FTCEntity>& Entities, float DeltaSeconds);

private:
	
	void UpdateCostField(const int GroupID);
#ifdef USE_BASELINE_MODEL
	void UpdatePotentialField_FM(const int GroupID);
	void UpdateSpeedField();
#else
	void UpdatePotentialField_BFS(int GroupID);
	EDirectionIndex ConvertVectorToDirectionIndex(FVector2f Vector) const;
#endif
	
	void UpdateDensityAndVelocityField(const TArray<FTCEntity>& Entities);
	void UpdatePotentialGradient(const int GroupID);
	void UpdateDesiredVelocityField(const int GroupID);
	float GetFiniteDifferenceApproximation(const FVector2f& Coords, const int GroupID);
	FTCCheapestNeighbor GetCheapestNeighbor(const FVector2f& Coords, EDirectionIndex First, EDirectionIndex Second, const int GroupID);
	TArray<FTCNeighbor> GetNeighbors(const FVector2f& Coords);
	float GetSocialForceInfluence(const FVector2f& DesiredDirection, const FVector2f& Force);

private:
	
	FRpSpatialData<FTCCell> Field;
	TMap<int, FVector2f> Goals;
	
#ifdef USE_BASELINE_MODEL
	TSet<FTCCell*> Knowns;
	TArray<FTCCell*> Candidates;
#else
	TArray<FTCCell*> Knowns;
	TArray<FTCCell*> Unknowns;
	TDeque<FTCCell*> Candidates;
#endif
	 
#ifdef USE_BASELINE_MODEL
	FTCBaselineSimulationParameters SimParameters;
#else
	FTCSimulationParameters SimParameters;
#endif
	
	FTCSocialForceParameters PedParameters;
	
	FRpImplicitGrid ImplicitGrid;
	
	int NumGroups = 0;
}; 