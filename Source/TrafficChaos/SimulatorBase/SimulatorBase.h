// Copyright Anupam Sahu. All Rights Reserved.
#pragma once

#define ENABLE_VELOCITY_OVERRIDING

#include "CoreMinimal.h"
#include "SocialForceModel.h"
#include "StructUtils/InstancedStruct.h"
#include "SimulatorBase.generated.h"

const FVector2f D_NORTH			{ 0, -1};
const FVector2f D_NORTH_WEST	{-1, -1};
const FVector2f D_WEST			{-1,  0};
const FVector2f D_SOUTH_WEST	{-1, +1};
const FVector2f D_SOUTH			{ 0, +1};
const FVector2f D_SOUTH_EAST	{+1, +1};
const FVector2f D_EAST			{+1,  0};
const FVector2f D_NORTH_EAST	{+1, -1};
const FVector2f D_ORIGIN		{ 0,  0};

constexpr int ANISOTROPY = 4;
constexpr int NUM_OFFSETS = 5;

enum EDirectionIndex : uint8
{
	NORTH,
	WEST,
	SOUTH,
	EAST,
	NONE,
};

const TStaticArray<FVector2f, NUM_OFFSETS> DIRECTION_OFFSETS
{
	D_NORTH,
	D_WEST,
	D_SOUTH,
	D_EAST,
	D_ORIGIN
};

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

template<typename CellType>
struct FTCNeighbor
{
	CellType* Cell;
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

USTRUCT()
struct FTCSimulationParameters
{
	GENERATED_BODY();
};

class TCSimulatorBase
{
public:
	
	virtual ~TCSimulatorBase() = default;
	
	virtual void Initialize(const float NewWorldSpan, const int NewResolution, const int NewNumGroups, const TInstancedStruct<FTCSimulationParameters> Parameters, const FTCSocialForceParameters& SocialForceParameters) = 0;
	virtual void MoveEntites(TArray<FTCEntity>& Entities, const float DeltaTime) = 0;
	virtual void UpdateSimulation(const TArray<FTCEntity>& Entities, const float DeltaTime) = 0;
	virtual void RegisterGoal(const int GroupID, const FVector2f& WorldLocation) = 0;
	virtual void RegisterWall(const FVector2f& WorldLocation) = 0;
	virtual void RegisterDiscomfort(const FVector2f& WorldLocation, const float Amount) = 0;
	virtual void SetSimulationParameters(const TInstancedStruct<FTCSimulationParameters> NewSimParamters) = 0;
	virtual void SetAdvectionParameters(const FTCSocialForceParameters& SocialForceParameters) = 0;
};

