#pragma once

#include "CoreMinimal.h"
#include "SimulationDataTypes.generated.h"

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

struct FTCEntity
{
	FVector2f Position;
	FVector2f Velocity;
	int GroupID;
};

struct FTCCell
{
	FTCCell()
		: 
	Density(0.0f), 
	Velocity(FVector2f::ZeroVector),
	Coords(FVector2f::ZeroVector),
	CostField({0,0,0,0})
	{}
	
	float Density;
	FVector2f Velocity;
	FVector2f Coords;
	TStaticArray<float, 4> CostField;
	TStaticArray<float, 4> SpeedField;
	
	TArray<float> Potential;
	TArray<FVector2f> DesiredVelocity;
	TArray<TStaticArray<float, 4>> PotentialGradient;
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