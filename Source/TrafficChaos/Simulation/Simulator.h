#pragma once

#include "CoreMinimal.h"
#include "SpatialData.h"

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

struct FTCMostOptimalNode
{
	bool operator()(const FTCCell& Left, const FTCCell& Right) const
	{
		return Left.Potential > Right.Potential;
	};
};

class TRAFFICCHAOS_API TCSimulator
{
public:
 
	TCSimulator()
		:Field(1,0,{})
	{}
	
	void Initialize(const float Resolution, const float WorldSize);
	void Update(const TArray<FVector2f>& EntityPositions, const TArray<FVector2f>& EntityVelocities, const float DeltaSeconds);
	
	const FRpSpatialData<FTCCell>& GetFieldData() const
	{
		return Field;
	}

private:
	
	void Solve(const FVector2f& GoalCoords);
	
	void UpdateDensityField(const TArray<FVector2f>& EntityPositions);
	void UpdateCellVelocityField(const TArray<FVector2f>& EntityPositions, const TArray<FVector2f>& EntityVelocities);
	void UpdateSpeedField();
	void UpdateCostField();
	
	void UpdatePotentialGradient();
	void UpdateDesiredVelocityField();
	
	float GetFiniteDifferenceApproximation(const FVector2f& Coords);
	float GaussianDistribution(float Distance);
	
	TArray<FTCCell*> GetNeighbors(const FVector2f& Coords);

private:
	
	FRpSpatialData<FTCCell> Field;
	
	TArray<FTCCell*> Knowns;
	TArray<FTCCell*> Unknowns;
	TArray<FTCCell*> Candidates;
	
	bool bSolved = false;
}; 