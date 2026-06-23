#pragma once

#include "CoreMinimal.h"
#include "SpatialData.h"
#include "Containers/Deque.h"
#include "SocialForceModel.h"
#include "SimulationDataTypes.h"

class TRAFFICCHAOS_API TCSimulator
{
public:
 
	TCSimulator()
		:Field(1,0,{})
	{}
	
	void Initialize(float Resolution, float WorldSize, int NumGroups);
	void Update(const TArray<FTCEntity>& Entities, float DeltaSeconds);
	void PerformCrowdAdvection(TArray<FTCEntity>& Entities, float DeltaSeconds);
	
	const FRpSpatialData<FTCCell>& GetFieldData() const
	{
		return Field;
	}

	void SetSimulationParameters(const FTCSimulationParameters& Parameters)
	{
		SimParameters = Parameters; 
	}
	
	void SetPedParameters(const FTCSocialForceParameters& Parameters)
	{
		PedParameters = Parameters;
	}

	void CreateGoal(const int GroupID, const FVector2f& Goal);

private:
	
	void Solve(const int GroupID);
	
	void UpdateDensityAndVelocityField(const TArray<FTCEntity>& Entities);
	void UpdateSpeedField();
	void UpdateCostField();
	
	void UpdatePotentialGradient(const int GroupID);
	void UpdateDesiredVelocityField(const int GroupID);
	
	float GetFiniteDifferenceApproximation(const FVector2f& Coords, const int GroupID);
	float GaussianDistribution(const float Distance);
	
	TArray<FTCCell*> GetNeighbors(const FVector2f& Coords);

private:
	
	FRpSpatialData<FTCCell> Field;
	
	TMap<int, FVector2f> Goals;
	TArray<FTCCell*> Knowns;
	TDeque<FTCCell*> Candidates;
	
	FTCSimulationParameters SimParameters;
	FTCSocialForceParameters PedParameters;
	
	bool bSolved = false;
	int NumGroups = 0;
}; 