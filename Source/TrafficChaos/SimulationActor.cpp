// Fill out your copyright notice in the Description page of Project Settings.

#include "SimulationActor.h"
#include "Kismet/KismetMathLibrary.h"

// Sets default values
ASimulationActor::ASimulationActor()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	Simulator.AddEntity();
}

void ASimulationActor::BeginPlay()
{
	Super::BeginPlay();
	Simulator.Initialize(GridResolution, WorldSpan);
	
	const auto AddEntityAtRandomLocation = [this](const float MinRange, const float MaxRange) -> void 
	{
		const float X = UKismetMathLibrary::RandomFloatInRange(MinRange, MaxRange);
		const float Y = UKismetMathLibrary::RandomFloatInRange(MinRange, MaxRange);
	
		Simulator.AddEntity({X, Y});
	};

	int NumEntities = EntityCount;
	while (NumEntities > 0)
	{
		AddEntityAtRandomLocation(0, WorldSpan);
		--NumEntities;
	}
}

void ASimulationActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	Simulator.Update(DeltaSeconds);
	Debug_Draw(DeltaSeconds);
}

void ASimulationActor::Debug_Draw(const float DeltaSeconds)
{
	const UWorld* World = GetWorld();
	const FRpSpatialData<FTCCell>& Field = Simulator.GetFieldData();
	
	// Draw entities.
	if (bDrawEntities)
	{
		const FTCEntityArray& EntityPositions = Simulator.GetEntityPositions();
		for (const FVector2f& Position : EntityPositions.Positions)
		{
			DrawDebugSphere(World, {Position.X, Position.Y, 0.0f}, 10.0f, 10, FColor::Green);
		}
	}
	
	// Debug DensityField.
	if (bDrawDensityField)
	{
		const auto DrawDensities = [this, World, Field](const FTCCell* Cell, const FVector2f& Coords)
		{
			const float DebugBoxExtent = Field.GetCellSize();
			const FVector2f WorldCoords = Field.GridToWorld(Coords);
			const FLinearColor DebugColor = FLinearColor::LerpUsingHSV(FLinearColor{1.0f, 1.0f, 1.0f, 0.1f},
																	   FLinearColor{1.0f, 0.0f, 0.0f, 0.5f}, 
																	   Cell->Density);
		
			const FVector BoxMin = {WorldCoords.X, WorldCoords.Y, 0};
			const FVector BoxMax = {WorldCoords.X + DebugBoxExtent, WorldCoords.Y + DebugBoxExtent, DebugBoxExtent};
			DrawDebugSolidBox(World, FBox(BoxMin, BoxMax), DebugColor.ToFColor(false));
		};
	
		Field.ForEachCellPerform(DrawDensities);
	}
	
	// Debug potential field.
	if (bDrawPotentialField)
	{
		const auto DrawPotential = [this, World, Field, DeltaSeconds](const FTCCell* Cell, const FVector2f& Coords)
		{
			const float DebugBoxExtent = Field.GetCellSize();
			const FVector2f WorldCoords = Field.GridToWorld(Coords);
			const FVector BoxMin = {WorldCoords.X, WorldCoords.Y, 0};
			const FVector BoxMax = {WorldCoords.X + DebugBoxExtent, WorldCoords.Y + DebugBoxExtent, 100};
			DrawDebugSolidBox(World, FBox(BoxMin, BoxMax), FColor(255, 255, 255, 10));
			
			const FString String = FString::Printf(TEXT("%.0f"), Cell->Potential);
			const FVector StringLocation = {WorldCoords.X + DebugBoxExtent / 2, WorldCoords.Y + DebugBoxExtent / 2, 0.0f}; 
			DrawDebugString(World, StringLocation , String, this, FColor::Red, DeltaSeconds);
		};
	
		Field.ForEachCellPerform(DrawPotential);
	}
	
	// Debug VelocityField.
	if (bDrawCellVelocityField)
	{
		const auto DrawVelocties = [this, World, Field](const FTCCell* Cell, const FVector2f& Coords) -> void
		{
			const float CellSize = Field.GetCellSize();
			const FVector2f WorldLocation = Field.GridToWorld(Coords);
			const FVector2f Direction = Cell->Velocity.IsNearlyZero() ? FVector2f{1.0f, 0.0f} : Cell->Velocity.GetSafeNormal();
			const FColor DebugColor = FColor::Yellow;
			DrawDebugCone
			(
				World, 
				{WorldLocation.X + (CellSize / 2), WorldLocation.Y + (CellSize / 2), 0}, 
				{-Direction.X, -Direction.Y, 0.0f}, 
				50.0f, PI/6, PI/6, 12, DebugColor
			);
		};
		Field.ForEachCellPerform(DrawVelocties);
	}
	
	// Debug DesiredVelocityField.
	if (bDrawDesiredVelocityField)
	{
		const auto DrawVelocties = [this, World, Field](const FTCCell* Cell, const FVector2f& Coords) -> void
		{
			const float CellSize = Field.GetCellSize();
			const FVector2f WorldLocation = Field.GridToWorld(Coords);
			const FVector2f Direction = Cell->DesiredVelocity.IsNearlyZero() ? FVector2f{1.0f, 0.0f} : Cell->DesiredVelocity.GetSafeNormal();
			const FColor DebugColor = FColor::Red;
			DrawDebugCone
			(
				World, 
				{WorldLocation.X + (CellSize / 2), WorldLocation.Y + (CellSize / 2), 0}, 
				{-Direction.X, -Direction.Y, 0.0f}, 
				50.0f, PI/6, PI/6, 12, DebugColor
			);
		};
		Field.ForEachCellPerform(DrawVelocties);
	}
}