// Copyright Anupam Sahu. All Rights Reserved.

#include "SimulationActor.h"

#include "Kismet/KismetMathLibrary.h"
constexpr float ENTITY_MOVEMENT_RADIUS = 5.0f;
constexpr float ENTITY_MOVEMENT_SPEED = 0.0f;

void ASimulationActor::PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent)
{
	FName PropertyName = (PropertyChangedEvent.MemberProperty != nullptr) ? PropertyChangedEvent.MemberProperty->GetFName() : NAME_None;
	
#ifdef USE_BASELINE_MODEL
	if (PropertyName == GET_MEMBER_NAME_CHECKED(ASimulationActor, BaselineSimParameters))
	{
		GEngine->AddOnScreenDebugMessage(-1, 2, FColor::White, FString::Printf(TEXT("Sim Parameters changed : %s"), *PropertyChangedEvent.GetPropertyName().ToString()));
		Simulator.SetSimulationParameters(BaselineSimParameters);
	}
#else
	if (PropertyName == GET_MEMBER_NAME_CHECKED(ASimulationActor, SimParameters))
	{
		GEngine->AddOnScreenDebugMessage(-1, 2, FColor::White, FString::Printf(TEXT("Sim Parameters changed : %s"), *PropertyChangedEvent.GetPropertyName().ToString()));
		Simulator.SetSimulationParameters(SimParameters);
	}
#endif
	
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
#ifdef USE_BASELINE_MODEL
	Simulator.SetSimulationParameters(BaselineSimParameters);
#else
	Simulator.SetSimulationParameters(SimParameters);
#endif
	
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

			Entities.Push
			({
				FVector2f{X, Y}, FVector2f{FVector2f::ZeroVector},
				GroupID,
#ifdef ENABLE_VELOCITY_OVERRIDING
				Configuration.OverrideVelocity,
				Configuration.bUseOverrideVelocity
#endif
			});
			
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
	if (bIsUpdateEnabled)
	{
		Simulator.Update(Entities, DeltaSeconds);
		Simulator.CrowdAdvection(Entities, DeltaSeconds);
	}
	DrawDebugGraphics(DeltaSeconds);
}

void ASimulationActor::SetUpdateEnabled(const bool bValue)
{
	bIsUpdateEnabled = bValue;
}

void ASimulationActor::DrawDebugGraphics(const float DeltaSeconds)
{
	const UWorld* World = GetWorld();
	const FRpSpatialData<FTCCell>& Field = Simulator.GetFieldData();
	
	const FRpImplicitGrid& ImplicitGrid = Simulator.GetImplicitGrid();
	ImplicitGrid.DrawDebug(World, DeltaSeconds);
	
	// Draw entities.
	if (bDrawEntities)
	{
		for (const FTCEntity& Entity : Entities)
		{
			if (EntityColors[Entity.GroupID].A == 0)
			{
				continue;
			}
			
			if (!Simulator.GetFieldData().IsValidWorldPosition(Entity.Position))
			{
				continue;
			}
			
			const FVector Position = {Entity.Position.X, Entity.Position.Y, 0.0f};
			DrawDebugSphere(World, Position, 25.0f, 10, EntityColors[Entity.GroupID]);
			if (bDrawTraces)
			{
				DrawDebugPoint(World, Position, 2.0f, EntityColors[Entity.GroupID], false, 20.0f);
			}
		}
	}
	
	// Debug DensityField.
	if (bDrawDensityField)
	{
		float MaxDensity = TNumericLimits<float>::Min();
		const auto GetMaxDensity = [&MaxDensity, this](const FTCCell* Cell, const FVector2f& Coords)
		{
#ifndef USE_BASELINE_MODEL 
			const float& Density = Cell->ByteDensity;
#else
			const float& Density = Cell->Density;
#endif
			if (Density > MaxDensity)
			{
				MaxDensity = Density;
			}
		};
		Field.ForEachCellPerform(GetMaxDensity);
		
		const auto DrawDensities = [this, World, Field, DeltaSeconds, MaxDensity](const FTCCell* Cell, const FVector2f& Coords)
		{
#ifndef USE_BASELINE_MODEL
			const float& Density = Cell->ByteDensity;
#else
			const float& Density = Cell->Density;
#endif
			if (Density == 0)
			{
				return;
			}
			
			const float NormDensity = Density / MaxDensity; 
			const float DebugBoxExtent = Field.GetCellSize();
			const FVector2f WorldCoords = Field.GridToWorld(Coords);
			const FLinearColor DebugColor = FLinearColor::LerpUsingHSV(FLinearColor{1.0f, 1.0f, 1.0f, 0.1f}, FLinearColor{1.0f, 0.0f, 0.0f, 0.5f}, NormDensity);
		
			const FVector BoxMin = {WorldCoords.X, WorldCoords.Y, 0};
			const FVector BoxMax = {WorldCoords.X + DebugBoxExtent, WorldCoords.Y + DebugBoxExtent, DebugBoxExtent};
			DrawDebugSolidBox(World, FBox(BoxMin, BoxMax), DebugColor.ToFColor(false));
			
			const FString String = FString::Printf(TEXT("%.2f"), Density);
			const FVector StringLocation = {WorldCoords.X + DebugBoxExtent / 2, WorldCoords.Y + DebugBoxExtent / 2, 0.0f}; 
			DrawDebugString(World, StringLocation , String, this, FColor::White, DeltaSeconds);
		};
	
		Field.ForEachCellPerform(DrawDensities);
	}
	
	// Debug potential field.
	if (bDrawPotentialField)
	{
		float MaxPotential = TNumericLimits<float>::Min();
		const auto GetMaxPotential = [&MaxPotential, this](const FTCCell* Cell, const FVector2f& Coords)
		{
			const float& Potential = Cell->Potential[DebugGroupID];
			if (Potential > MaxPotential)
			{
				MaxPotential = Potential;
			}
		};
		
		Field.ForEachCellPerform(GetMaxPotential);
		const auto DrawPotential = [this, World, Field, MaxPotential](const FTCCell* Cell, const FVector2f& Coords)
		{
			const float NormPotential = Cell->Potential[DebugGroupID] / MaxPotential;
			
			const float DebugBoxExtent = Field.GetCellSize();
			const FVector2f WorldCoords = Field.GridToWorld(Coords);
			const FVector BoxMin = {WorldCoords.X, WorldCoords.Y, 0};
			const FVector BoxMax = {WorldCoords.X + DebugBoxExtent, WorldCoords.Y + DebugBoxExtent, 100};
			const FColor BoxColor = FLinearColor(FMath::Square(NormPotential), 0, 0, 1).ToFColor(false);
			DrawDebugSolidBox(World, FBox(BoxMin, BoxMax), BoxColor);
		};
	
		Field.ForEachCellPerform(DrawPotential);
	}
	
	// Debug VelocityField.
	if (bDrawCellVelocityField)
	{
		const auto DrawVelocties = [this, World, Field](const FTCCell* Cell, const FVector2f& Coords) -> void
		{
#ifndef USE_BASELINE_MODEL
			if (Cell->Direction == EDirectionIndex::NONE)
			{
				return;
			}
			
			const float CellSize = Field.GetCellSize(); 
			const FVector2f WorldLocation = Field.GridToWorld(Coords);
			const FVector2f Direction = DIRECTION_OFFSETS[Cell->Direction].GetSafeNormal();
			const FVector LineStart = {WorldLocation.X, WorldLocation.Y, 0};
			const FVector LineEnd = {WorldLocation.X + Direction.X * CellSize / 2, WorldLocation.Y + Direction.Y * CellSize / 2, 0};
			DrawDebugLine(World, LineStart, LineStart, FColor::Purple, false, -1, 0, 7.0f);
			DrawDebugLine(World, LineStart, LineEnd, FColor::Purple, false, -1, 0, 2.0f);
#else
			if (Cell->Velocity.IsNearlyZero())
			{
				return;
			}
			
			const float CellSize = Field.GetCellSize(); 
			const FVector2f WorldLocation = Field.GridToWorld(Coords);
			const FVector2f Direction = Cell->Velocity.GetSafeNormal();
			const FVector LineStart = {WorldLocation.X, WorldLocation.Y, 0};
			const FVector LineEnd = {WorldLocation.X + Direction.X * CellSize / 2, WorldLocation.Y + Direction.Y * CellSize / 2, 0};
			DrawDebugLine(World, LineStart, LineStart, FColor::Purple, false, -1, 0, 7.0f);
			DrawDebugLine(World, LineStart, LineEnd, FColor::Purple, false, -1, 0, 2.0f);
#endif
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