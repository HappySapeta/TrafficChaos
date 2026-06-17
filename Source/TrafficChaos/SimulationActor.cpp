// Fill out your copyright notice in the Description page of Project Settings.

#include "SimulationActor.h"

#include "Kismet/KismetMathLibrary.h"
constexpr float ENTITY_MOVEMENT_RADIUS = 5.0f;
constexpr float ENTITY_MOVEMENT_SPEED = 0.0f;

void ASimulationActor::PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent)
{
	FName PropertyName = (PropertyChangedEvent.MemberProperty != nullptr) ? PropertyChangedEvent.MemberProperty->GetFName() : NAME_None;
	if (PropertyName == GET_MEMBER_NAME_CHECKED(ASimulationActor, Parameters))
	{
		Simulator.SetSimulationParameters(Parameters);
	}
	
	Super::PostEditChangeProperty(PropertyChangedEvent);
}

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
	SpawnEntities();
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

void ASimulationActor::SpawnEntities()
{
	for (const FTCSpawnConfiguration& Configuration : SpawnConfigurations)
	{
		const float& SpawnRange = Configuration.SpawnRange;
		const float& H = Configuration.Origin.X;
		const float& K = Configuration.Origin.Y;
		const float& A = Configuration.SpawnAreaWidth;
		const float& R = Configuration.Rotation;
		int NumSpawned = 0;
		while (NumSpawned < Configuration.Amount)
		{
			const float S = UKismetMathLibrary::RandomFloatInRange(0, SpawnRange);
			const float T = UKismetMathLibrary::RandomFloatInRange(0, 2 * PI);
			const float X = FMath::Clamp(S * (A * FMath::Cos(T) * FMath::Cos(R) - FMath::Sin(T) * FMath::Sin(R)) + H, 0, SpawnRange);
			const float Y = FMath::Clamp(S * (A * FMath::Cos(T) * FMath::Sin(R) + FMath::Sin(T) * FMath::Cos(R)) + K, 0, SpawnRange);
			
			EntityPositions.Push({X, Y});
			EntityVelocities.Push({Configuration.Velocity});
			
			++NumSpawned;
		}
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
			const FVector LineStart = {WorldLocation.X, WorldLocation.Y, 0};
			const FVector LineEnd = {WorldLocation.X + Direction.X * CellSize / 2, WorldLocation.Y + Direction.Y * CellSize / 2, 0};
			DrawDebugLine(World, LineStart, LineStart, FColor::Purple, false, -1, 0, 7.0f);
			DrawDebugLine(World, LineStart, LineEnd, FColor::Purple, false, -1, 0, 2.0f);
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
			const FVector LineStart = {WorldLocation.X, WorldLocation.Y, 0};
			const FVector LineEnd = {WorldLocation.X + Direction.X * CellSize / 2, WorldLocation.Y + Direction.Y * CellSize / 2, 0};
			DrawDebugLine(World, LineStart, LineStart, FColor::Cyan, false, -1, 0, 7.0f);
			DrawDebugLine(World, LineStart, LineEnd, FColor::Cyan, false, -1, 0, 2.0f);
		};
		Field.ForEachCellPerform(DrawVelocties);
	}
}