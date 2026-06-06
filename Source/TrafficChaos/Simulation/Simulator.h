#pragma once

#include "CoreMinimal.h"
#include "SpatialData.h"

constexpr float MAX_POTENTIAL = 1;

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

struct FTCEntityArray
{
	void Add(const FVector2f& InitialPosition, const FVector2f InitialVelocity, const float InitialHeading)
	{
		Positions.Push(InitialPosition);
		Velocities.Push(InitialVelocity);
		HeadingAngles.Push(InitialHeading);
	
		++Size;
	}
	
	int Num()
	{
		return Size;
	}
	
	TArray<FVector2f> Positions;
	TArray<FVector2f> Velocities;
	TArray<float> HeadingAngles;
	
private:
	
	int Size = 0;
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
	
	void AddEntity
	(
		const FVector2f& InitialPosition = FVector2f::ZeroVector,
		const FVector2f InitialVelocity = FVector2f::ZeroVector, 
		const float InitialHeading = 0.0f
	);
	
	void Update(const float DeltaSeconds);
	
	void Debug_Draw(const UWorld* World, float DeltaSeconds);

private:
	
	void UpdateDensityField();
	void UpdateVelocityField();
	void UpdateCostField();
	float GetFiniteDifferenceApproximation(const FVector2f& Coords);
	void Solve(const FVector2f& GoalCoords);
	void CalculatePotentialGradient();
	void CalculateDesiredVelocityField();
	float GaussianDistribution(float Distance);
	float GetSpeedField(const FVector2f& Velocity, EDirectionIndex Direction);
	TArray<FTCCell*> GetNeighbors(const FVector2f& Coords);

private:
	
	void Debug_MoveEntities(const float DeltaSeconds);

private:
	
	FRpSpatialData<FTCCell> Field;
	FTCEntityArray Entities;
	
	TArray<FTCCell*> Knowns;
	TArray<FTCCell*> Unknowns;
	TArray<FTCCell*> Candidates;
	
	bool bSolved = false;
}; 