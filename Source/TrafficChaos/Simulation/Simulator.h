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

	void SetSimulationParameters(const FTCSimulationParameters& Parameters)
	{
		SimParameters = Parameters; 
	}
	
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
	void CrowdAdvection(TArray<FTCEntity>& Entities, float TimeStep);
	void Update(const TArray<FTCEntity>& Entities, float DeltaSeconds);

private:

#ifdef USE_SOLVER_OPTIMISATION
	void UpdatePotentialField_BFS(int GroupID);
#else
	void UpdatePotentialField_FM(const int GroupID);
#endif
	
	void UpdateDensityAndVelocityField(const TArray<FTCEntity>& Entities);
	void UpdateCostField(int GroupID);
	void UpdatePotentialGradient(const int GroupID);
	void UpdateDesiredVelocityField(const int GroupID);
	float GetFiniteDifferenceApproximation(const FVector2f& Coords, const int GroupID);
	FTCCheapestNeighbor GetCheapestNeighbor(const FVector2f& Coords, EDirectionIndex First, EDirectionIndex Second, const int GroupID);
	TArray<FTCNeighbor> GetNeighbors(const FVector2f& Coords);
	float GetSocialForceInfluence(const FVector2f& DesiredDirection, const FVector2f& Force);
	EDirectionIndex ConvertVectorToDirectionIndex(FVector2f Vector) const;

private:
	
	FRpSpatialData<FTCCell> Field;
	
	TMap<int, FVector2f> Goals;
	TArray<FTCCell*> Knowns;
	TArray<FTCCell*> Unknowns;
	TDeque<FTCCell*> Candidates;
	TArray<FTCCell*> CandidatesHeap;
	
	FTCSimulationParameters SimParameters;
	FTCSocialForceParameters PedParameters;
	
	FRpImplicitGrid ImplicitGrid;
	
	int NumGroups = 0;
}; 