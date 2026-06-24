// Copyright Anupam Sahu. All Rights Reserved.

 #include "SimulationActor.h"

#include "Kismet/KismetMathLibrary.h"
constexpr float ENTITY_MOVEMENT_RADIUS = 5.0f;
constexpr float ENTITY_MOVEMENT_SPEED = 0.0f;

void ASimulationActor::PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent)
{
	FName PropertyName = (PropertyChangedEvent.MemberProperty != nullptr) ? PropertyChangedEvent.MemberProperty->GetFName() : NAME_None;
	if (PropertyName == GET_MEMBER_NAME_CHECKED(ASimulationActor, SimParameters))
	{
		GEngine->AddOnScreenDebugMessage(-1, 2, FColor::White, FString::Printf(TEXT("Sim Parameters changed : %s"), *PropertyChangedEvent.GetPropertyName().ToString()));
		Simulator.SetSimulationParameters(SimParameters);
	}
	
	if (PropertyName == GET_MEMBER_NAME_CHECKED(ASimulationActor, PedParameters))
	{
		GEngine->AddOnScreenDebugMessage(-1, 2, FColor::White, FString::Printf(TEXT("Ped Parameters changed : %s"), *PropertyChangedEvent.GetPropertyName().ToString()));
		Simulator.SetAdvectionParameters(PedParameters);
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
	Simulator.Initialize(GridResolution, WorldSpan, SpawnConfigurations.Num());
	Simulator.SetSimulationParameters(SimParameters);
	Simulator.SetAdvectionParameters(PedParameters);
	SpawnEntities();
}

void ASimulationActor::SpawnEntities()
{
	int GroupID = 0;
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
			const float X = S * (A * FMath::Cos(T) * FMath::Cos(R) - FMath::Sin(T) * FMath::Sin(R)) + H;
			const float Y = S * (A * FMath::Cos(T) * FMath::Sin(R) + FMath::Sin(T) * FMath::Cos(R)) + K;
			
			const FVector2f NewPosition = {X, Y};
			if (!Simulator.GetFieldData().IsValidWorldPosition(NewPosition))
			{
				continue;
			}
			
			Entities.Push({FVector2f{X, Y}, FVector2f{FVector2f::ZeroVector}, GroupID});
			++NumSpawned;
		}
		EntityColors.Push(Configuration.Color);
		
		const float GoalX = FMath::Clamp(Configuration.Goal.X, 0, WorldSpan);
		const float GoalY = FMath::Clamp(Configuration.Goal.Y, 0, WorldSpan);
		Simulator.RegisterGoal(GroupID, {GoalX, GoalY});
		++GroupID;
	}
}

void ASimulationActor::Tick(const float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	Simulator.Update(Entities, DeltaSeconds);
	Simulator.CrowdAdvection(Entities, DeltaSeconds);
	DrawDebugGraphics(DeltaSeconds);
}

void ASimulationActor::DrawDebugGraphics(const float DeltaSeconds)
{
	const UWorld* World = GetWorld();
	const FRpSpatialData<FTCCell>& Field = Simulator.GetFieldData();
	
	// Draw entities.
	if (bDrawEntities)
	{
		for (const FTCEntity& Entity : Entities)
		{
			DrawDebugSphere(World, {Entity.Position.X, Entity.Position.Y, 0.0f}, 25.0f, 10,  EntityColors[Entity.GroupID]);
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
			
			const FString String = FString::Printf(TEXT("%.2f"), Cell->Potential[DebugGroupID]);
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
			if (Cell->DesiredVelocity[DebugGroupID].IsNearlyZero())
			{
				return;
			}
			
			const float CellSize = Field.GetCellSize();
			const FVector2f WorldLocation = Field.GridToWorld(Coords);
			const FVector2f Direction = Cell->DesiredVelocity[DebugGroupID].IsNearlyZero() ? FVector2f{1.0f, 0.0f} : Cell->DesiredVelocity[DebugGroupID].GetSafeNormal();
			const FVector LineStart = {WorldLocation.X, WorldLocation.Y, 0};
			const FVector LineEnd = {WorldLocation.X + Direction.X * CellSize / 2, WorldLocation.Y + Direction.Y * CellSize / 2, 0};
			DrawDebugLine(World, LineStart, LineStart, FColor::Cyan, false, -1, 0, 7.0f);
			DrawDebugLine(World, LineStart, LineEnd, FColor::Cyan, false, -1, 0, 2.0f);
		};
		Field.ForEachCellPerform(DrawVelocties);
	}
}