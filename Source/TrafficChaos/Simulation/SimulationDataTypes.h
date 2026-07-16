// Copyright Anupam Sahu. All Rights Reserved.
#pragma once

//#define USE_BASELINE_MODEL
#define ENABLE_VELOCITY_OVERRIDING

#include "CoreMinimal.h"
#include "SimulationDataTypes.generated.h"

#ifdef USE_BASELINE_MODEL
constexpr int NUM_DIRECTIONS = 4;
#else
constexpr int NUM_DIRECTIONS = 9;
#endif

const FVector2f D_NORTH			{ 0, -1};
const FVector2f D_NORTH_WEST	{-1, -1};
const FVector2f D_WEST			{-1,  0};
const FVector2f D_SOUTH_WEST	{-1, +1};
const FVector2f D_SOUTH			{ 0, +1};
const FVector2f D_SOUTH_EAST	{+1, +1};
const FVector2f D_EAST			{+1,  0};
const FVector2f D_NORTH_EAST	{+1, -1};
const FVector2f D_ORIGIN		{ 0,  0};

#ifdef USE_BASELINE_MODEL
// North, West, South, EastDIR
const TStaticArray<FVector2f, NUM_DIRECTIONS> DIRECTION_OFFSETS
{
	D_NORTH,
	D_WEST,
	D_SOUTH,
	D_EAST
};
#else
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
#endif

#ifdef USE_BASELINE_MODEL
enum EDirectionIndex : uint8
{
	NORTH,
	WEST,
	SOUTH,
	EAST
};
#else
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
#endif

struct FTCEntity
{
	FVector2f Position = FVector2f::ZeroVector;
	FVector2f Velocity = FVector2f::ZeroVector;
	int GroupID = 0;
	
#ifdef ENABLE_VELOCITY_OVERRIDING
	FVector2f OverrideVelocity = FVector2f::ZeroVector;
	bool bUseOverrideVelocity = false;
#endif
};

#ifdef USE_BASELINE_MODEL
struct FTCCell
{
	FVector2f Coords;
	
	float Density;
	FVector2f Velocity;
	float Discomfort;
	
	TArray<float> Potential;
	TArray<FVector2f> DesiredVelocity;
	TStaticArray<float, NUM_DIRECTIONS> SpeedField;
	TArray<TStaticArray<float, NUM_DIRECTIONS>> CostField;
	TArray<TStaticArray<float, NUM_DIRECTIONS>> PotentialGradient;
};
#else
struct FTCCell
{
	FVector2f Coords;
	
	uint8 ByteDensity;
	EDirectionIndex Direction;
	float Discomfort;
	
	TArray<float> Potential;
	TArray<FVector2f> DesiredVelocity;
	TArray<TStaticArray<float, NUM_DIRECTIONS>> CostField;
	TArray<TStaticArray<float, NUM_DIRECTIONS>> PotentialGradient;
};
#endif

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
struct FTCBaselineSimulationParameters
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