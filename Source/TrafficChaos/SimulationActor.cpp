// Fill out your copyright notice in the Description page of Project Settings.

#include "SimulationActor.h"
constexpr float ENTITY_MOVEMENT_RADIUS = 5.0f;
constexpr float ENTITY_MOVEMENT_SPEED = 0.0f;

// Sets default values
ASimulationActor::ASimulationActor()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

void ASimulationActor::BeginPlay()
{
	Super::BeginPlay();
	Simulator.Initialize(GridResolution, WorldSpan);
	EntityPositions.Push({WorldSpan * 0.0f, WorldSpan * 0.5f});
	EntityPositions.Push({WorldSpan * 0.3f, WorldSpan * 0.5f});
	EntityPositions.Push({WorldSpan * 0.6f, WorldSpan * 0.5f});
	EntityVelocities.Push({0, 1});
	EntityVelocities.Push({0, 1});
	EntityVelocities.Push({0, 1});
}

void ASimulationActor::UpdateEntityPositionsAndVelocities(float DeltaSeconds)
{
	const float Time = GetWorld()->GetTimeSeconds();	
	const float PosX = 0; //ENTITY_MOVEMENT_RADIUS * FMath::Cos(Time * ENTITY_MOVEMENT_SPEED);
	const float PosY = ENTITY_MOVEMENT_RADIUS * FMath::Sin(Time * ENTITY_MOVEMENT_SPEED);
	
	for (int EntityIndex = 0; EntityIndex < EntityPositions.Num(); ++EntityIndex)
	{
		EntityPositions[EntityIndex] += {PosX, PosY};
		EntityVelocities[EntityIndex] = {PosX / DeltaSeconds, PosY / DeltaSeconds};
	}
}

void ASimulationActor::Tick(const float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	//UpdateEntityPositionsAndVelocities(DeltaSeconds);
	Simulator.Update(EntityPositions, EntityVelocities, DeltaSeconds);
	DrawDebugGraphics(DeltaSeconds);
}

void ASimulationActor::DrawDebugGraphics(const float DeltaSeconds)
{
	const UWorld* World = GetWorld();
	const FRpSpatialData<FTCCell>& Field = Simulator.GetFieldData();
	
	// Draw entities.
	if (bDrawEntities)
	{
		for (const FVector2f& Position : EntityPositions)
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
			
			const FString String = FString::Printf(TEXT("%.2f"), Cell->Potential);
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
			if (Cell->Velocity.IsNearlyZero())
			{
				return;
			}
			
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
			if (Cell->DesiredVelocity.IsNearlyZero())
			{
				return;
			}
			
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