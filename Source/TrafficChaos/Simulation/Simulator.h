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

// North, West, South, East
const TStaticArray<FVector2f, 4> BASIC_OFFSETS
{
	D_NORTH, D_WEST, D_SOUTH, D_EAST,
};

enum EDirectionIndex : uint8_t
{
	NORTH = 0,
	WEST,
	SOUTH,
	EAST,
	COUNT
};

constexpr TStaticArray<EDirectionIndex, 4> CARDINAL_DIRECTIONS
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
	float Density;
	FVector2f Velocity;
	float Potential;
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
	float GaussianDistribution(float Distance);
	
private:
	
	void Debug_MoveEntities(const float DeltaSeconds);

private:
	
	FRpSpatialData<FTCCell> Field;
	FTCEntityArray Entities;
}; 