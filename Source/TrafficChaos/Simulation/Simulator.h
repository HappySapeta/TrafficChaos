#pragma once

#include "CoreMinimal.h"
#include "SpatialData.h"
#include "Containers/Deque.h"
#include "SocialForceModel.h"
#include "Simulator.generated.h"

const FVector2f D_NORTH			{ 0, -1};
const FVector2f D_NORTH_WEST	{-1, -1};
const FVector2f D_WEST			{-1,  0};
const FVector2f D_SOUTH_WEST	{-1, +1};
const FVector2f D_SOUTH			{ 0, +1};
const FVector2f D_SOUTH_EAST	{+1, +1};
const FVector2f D_EAST			{+1,  0};
const FVector2f D_NORTH_EAST	{+1, -1};

// North, West, South, EastDIR
const TStaticArray<FVector2f, 4> DIRECTION_OFFSETS
{
	D_NORTH, D_WEST, D_SOUTH, D_EAST,
};

enum EDirectionIndex : uint8
{
	NORTH,
	WEST,
	SOUTH,
	EAST,
};

const TArray<EDirectionIndex> CARDINAL_DIRECTIONS
{
	NORTH, WEST, SOUTH, EAST
};

struct FTCCell
{
	FTCCell()
		: 
	Density(0.0f), 
	Potential(0.0f), 
	Velocity(FVector2f::ZeroVector),
	Coords(FVector2f::ZeroVector),
	CostField({0,0,0,0})
	{}
	
	float Density;
	float Potential;
	FVector2f Velocity;
	FVector2f Coords;
	FVector2f DesiredVelocity;
	TStaticArray<float, 4> CostField;
	TStaticArray<float, 4> SpeedField;
	TStaticArray<float, 4> PotentialGradient;
};

struct FTCCheapestNeighbor
{
	float Potential;
	float CostToTravel;
	float Sum() const
	{
		return Potential + CostToTravel;
	}
};

USTRUCT()
struct FTCSimulationParameters
{
	GENERATED_BODY()
	
	UPROPERTY(EditAnywhere)
	float GaussianFallOff = 1.0;
	
	UPROPERTY(EditAnywhere)
	float GaussianScale = 1.0f;
	
	UPROPERTY(EditAnywhere)
	float MaxTopoSpeed = 30;
	
	UPROPERTY(EditAnywhere)
	float MinTopoSpeed = 10;
	
	UPROPERTY(EditAnywhere)
	float MinSlope = 0;
	
	UPROPERTY(EditAnywhere)
	float MaxSlope = 1;
	
	UPROPERTY(EditAnywhere)
	float MinDensity = 0;
	
	UPROPERTY(EditAnywhere)
	float MaxDensity = 5;
	
	UPROPERTY(EditAnywhere)
	float DensityExponent = 1;
	
	UPROPERTY(EditAnywhere)
	int VelocityLookupOffset = 3;
	
	UPROPERTY(EditAnywhere)
	int DensityLookupOffset = 2;
	
	UPROPERTY(EditAnywhere)
	float PathCostConstant = 1;
	
	UPROPERTY(EditAnywhere)
	float TimeCostConstant = 1;
};

class TRAFFICCHAOS_API TCSimulator
{
public:
 
	TCSimulator()
		:Field(1,0,{})
	{}
	
	void Initialize(const float Resolution, const float WorldSize);
	void Update(const TArray<FVector2f>& EntityPositions, const TArray<FVector2f>& EntityVelocities, const float DeltaSeconds);
	void PerformCrowdAdvection(TArray<FVector2f>& EntityPositions, TArray<FVector2f>& EntityVelocities, float DeltaSeconds);
	
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

private:
	
	void Solve(const FVector2f& GoalCoords);
	
	void UpdateDensityAndVelocityField(const TArray<FVector2f>& EntityPositions, const TArray<FVector2f>& EntityVelocities);
	void UpdateSpeedField();
	void UpdateCostField();
	
	void UpdatePotentialGradient();
	void UpdateDesiredVelocityField();
	
	float GetFiniteDifferenceApproximation(const FVector2f& Coords);
	float GaussianDistribution(const float Distance);
	
	TArray<FTCCell*> GetNeighbors(const FVector2f& Coords);

private:
	
	FRpSpatialData<FTCCell> Field;
	
	TArray<FTCCell*> Knowns;
	TDeque<FTCCell*> Candidates;
	
	FTCSimulationParameters SimParameters;
	FTCSocialForceParameters PedParameters;
	
	bool bSolved = false;
	
}; 