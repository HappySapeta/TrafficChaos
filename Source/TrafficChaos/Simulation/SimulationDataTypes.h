// Copyright Anupam Sahu. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "SimulationDataTypes.generated.h"

constexpr int NUM_DIRECTIONS = 8;

const FVector2f D_NORTH			{ 0, -1};
const FVector2f D_NORTH_WEST	{-1, -1};
const FVector2f D_WEST			{-1,  0};
const FVector2f D_SOUTH_WEST	{-1, +1};
const FVector2f D_SOUTH			{ 0, +1};
const FVector2f D_SOUTH_EAST	{+1, +1};
const FVector2f D_EAST			{+1,  0};
const FVector2f D_NORTH_EAST	{+1, -1};

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
	D_NORTH_EAST
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
	NORTH_EAST
};

const TArray<EDirectionIndex> CARDINAL_DIRECTIONS
{
	NORTH,
	WEST,
	SOUTH,
	EAST,
	NORTH_WEST,
	SOUTH_WEST,
	SOUTH_EAST,
	NORTH_EAST
};

struct FTCEntity
{
	FVector2f Position;
	FVector2f Velocity;
	int GroupID;
};

struct FTCCell
{
	float Density;
	float Discomfort;
	FVector2f Velocity;
	FVector2f Coords;
	TStaticArray<float, NUM_DIRECTIONS> CostField;
	TStaticArray<float, NUM_DIRECTIONS> SpeedField;
	
	TArray<float> Potential;
	TArray<FVector2f> DesiredVelocity;
	TArray<TStaticArray<float, NUM_DIRECTIONS>> PotentialGradient;
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
	float MaxTopoSpeed = 30;
	
	UPROPERTY(EditAnywhere)
	float MinTopoSpeed = 10;
	
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
	
	UPROPERTY(EditAnywhere)
	float DiscomfortConstant = 1;
	
	UPROPERTY(EditAnywhere)
	float DensityConstant = 1;
	
	UPROPERTY(EditAnywhere)
	TEnumAsByte<ETCAnisotropy> Anisotropy = ETCAnisotropy::FOUR_WAY; 
};