// Copyright Anupam Sahu. All Rights Reserved.
#pragma once

#define USE_DENSITY_OPTIMISATION
#define USE_VELOCITY_OPTIMISATION
#define USE_SOLVER_OPTIMISATION

#include "CoreMinimal.h"
#include "SimulationDataTypes.generated.h"

constexpr int NUM_DIRECTIONS = 9;

const FVector2f D_NORTH			{ 0, -1};
const FVector2f D_NORTH_WEST	{-1, -1};
const FVector2f D_WEST			{-1,  0};
const FVector2f D_SOUTH_WEST	{-1, +1};
const FVector2f D_SOUTH			{ 0, +1};
const FVector2f D_SOUTH_EAST	{+1, +1};
const FVector2f D_EAST			{+1,  0};
const FVector2f D_NORTH_EAST	{+1, -1};
const FVector2f D_ORIGIN		{ 0,  0};

// North, West, South, EastDIR
const TStaticArray<FVector2f, NUM_DIRECTIONS> DIRECTION_OFFSETS
{
	D_NORTH,
	D_WEST,
	D_SOUTH,
	D_EAST,
	D_NORTH_WEST,
	D_SOUTH_WEST,
	D_SOUTH_EAST,
	D_NORTH_EAST,
	D_ORIGIN
};

enum EDirectionIndex : uint8
{
	NORTH,
	WEST,
	SOUTH,
	EAST,
	NORTH_WEST,
	SOUTH_WEST,
	SOUTH_EAST,
	NORTH_EAST,
	NONE,
};

struct FTCEntity
{
	FVector2f Position = FVector2f::ZeroVector;
	FVector2f Velocity = FVector2f::ZeroVector;
	FVector2f OverrideVelocity = FVector2f::ZeroVector;
	bool bUseOverrideVelocity = false;
	int GroupID = 0;
};

struct FTCCell
{
	FVector2f Coords;
	
#ifdef USE_DENSITY_OPTIMISATION
	uint8 ByteDensity;
#else
	float Density;
#endif
	
#ifdef USE_VELOCITY_OPTIMISATION
	EDirectionIndex Direction;
#else
	FVector2f Velocity;
#endif
	float Discomfort;
	
	TArray<float> Potential;
	TArray<FVector2f> DesiredVelocity;
	TArray<TStaticArray<float, NUM_DIRECTIONS>> CostField;
	TArray<TStaticArray<float, NUM_DIRECTIONS>> PotentialGradient;
};

struct FTCNeighbor
{
	FTCCell* Cell;
	EDirectionIndex DirectionIndex;
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

UENUM()
enum ETCAnisotropy : uint8
{
	FOUR_WAY = 4,
	EIGHT_WAY = 8
};

USTRUCT()
struct FTCSimulationParameters
{
	GENERATED_BODY()
	
	UPROPERTY(EditAnywhere)
	float PathCostConstant = 1;
	
	UPROPERTY(EditAnywhere)
	float TimeCostConstant = 1;
	
	UPROPERTY(EditAnywhere)
	float DiscomfortConstant = 1;
	
	UPROPERTY(EditAnywhere, Category = "Test")
	float DensityConstant = 1;
	
	UPROPERTY(EditAnywhere, Category = "Test")
	TEnumAsByte<ETCAnisotropy> Anisotropy = ETCAnisotropy::FOUR_WAY;
};