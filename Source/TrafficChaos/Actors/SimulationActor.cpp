// Copyright Anupam Sahu. All Rights Reserved.

#include "SimulationActor.h"
#include "Kismet/KismetMathLibrary.h"

// Sets default values
ASimulationActor::ASimulationActor()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;
	
	BaselineCrowdSimParams = TInstancedStruct<FTCBaselineSimParameters>::Make();
	FastCrowdSimParams = TInstancedStruct<FTCFastSimulationParameters>::Make();
	BaselineSimulator = MakeShared<TCBaselineContinuumCrowdSimulator>();
	FastSimulator = MakeShared<TCFastContinuumCrowdSimulator>();
	CurrentSimulator = FastSimulator;
}

void ASimulationActor::InitialiseSimulation()
{
	PrimarySimulationCache.Reset();
	ElapsedSimTime = 0;
	UKismetMathLibrary::SetRandomStreamSeed(RandomStream, RandomSeed);
	
	BaselineSimulator->Initialize(WorldSpan, Resolution, SpawnConfigurations.Num(), BaselineCrowdSimParams, SocialForceParams);
	FastSimulator->Initialize(WorldSpan, Resolution, SpawnConfigurations.Num(), FastCrowdSimParams, SocialForceParams);
	
	for (const FVector2f& WallCoords : WallConfigurations)
	{
		BaselineSimulator->RegisterWall(WallCoords);
		FastSimulator->RegisterWall(WallCoords);
	}
	
	for (const FTCDiscomfortZone& Zone : DiscomfortZones)
	{
		BaselineSimulator->RegisterDiscomfort(Zone.Coords, Zone.Amount);
		FastSimulator->RegisterDiscomfort(Zone.Coords, Zone.Amount);
	}
	
	InitialiseEntityStartLocations();
}

void ASimulationActor::Evaluate()
{
	StopVisualisation();
	InitialiseSimulation();
	
	AvgAbsoluteDifferenceMetric = 0.0f;
	AvgPathLengthMetric = 0.0f;
	AvgInterPedDistanceMetric = 0.0f;
	
	ReferencePreviousPositions.Init(FVector2f::ZeroVector,Entities.Num());
	TestPreviousPositions.Init(FVector2f::ZeroVector, Entities.Num());
	for (int Index = 0; Index < Entities.Num(); ++Index)
	{
		const FVector2f& Position = Entities[Index].Position;
		ReferencePreviousPositions[Index] = Position;
		TestPreviousPositions[Index] = Position;
	}
	
	BaselineSimCache.Reset();
	FastSimCache.Reset();
	
	TArray<FTCEntity> BaselineEntities = Entities;
	TArray<FTCEntity> FastSimEntities = Entities;
	
	int NumFrames = 0;
	while (ElapsedSimTime < SimulationLength)
	{
		BaselineSimulator->UpdateSimulation(BaselineEntities, SimulationTimeStep);
		BaselineSimulator->MoveEntites(BaselineEntities, SimulationTimeStep);
		
		FastSimulator->UpdateSimulation(FastSimEntities, SimulationTimeStep);
		FastSimulator->MoveEntites(FastSimEntities, SimulationTimeStep);
		
		BaselineSimCache.Push({});
		FastSimCache.Push({});
		for (int Index = 0; Index < Entities.Num(); ++Index)
		{
			BaselineSimCache.Last().Push({BaselineEntities[Index].Position, BaselineEntities[Index].GroupID});
			FastSimCache.Last().Push({FastSimEntities[Index].Position, FastSimEntities[Index].GroupID});
		}
		
		MetricCompare(BaselineEntities, FastSimEntities);
		
		ElapsedSimTime += SimulationTimeStep;
		++NumFrames;
	}
	
	AvgAbsoluteDifferenceMetric /= NumFrames;
	AvgInterPedDistanceMetric /= NumFrames;
	AvgInterPedDistanceMetric /= NumFrames;
	
	if (bNormaliseMetrics)
	{
		AvgAbsoluteDifferenceMetric /= WorldSpan;
		AvgInterPedDistanceMetric /= WorldSpan;
		AvgPathLengthMetric /= WorldSpan;
	}
	
	UE_LOG(LogTemp, Warning, TEXT("Abs Diff = %f, Path Length = %f, InterPed Dist = %f"), AvgAbsoluteDifferenceMetric, AvgPathLengthMetric, AvgInterPedDistanceMetric);
}

void ASimulationActor::MetricCompare(const TArray<FTCEntity>& Reference, const TArray<FTCEntity>& Test)
{
	const int NumEntities = Entities.Num();
	
	float AbsoluteDifferenceMetric = 0.0f;
	float PathLengthMetric = 0.0f;
	float InterPedDistanceMetric = 0.0f;
	
	for (int Index = 0; Index < NumEntities; ++Index)
	{
		const FVector2f& ReferencePosition = Reference[Index].Position;
		const FVector2f& TestPosition = Test[Index].Position;
		
		// Absolute Difference Metric
		AbsoluteDifferenceMetric += FVector2f::Distance(ReferencePosition, TestPosition);
		
		// Path Length Metric
		{
			const FVector2f& ReferencePreviousPosition = ReferencePreviousPositions[Index];
			const FVector2f& TestPreviousPosition = TestPreviousPositions[Index];
			PathLengthMetric += FVector2f::Distance(ReferencePreviousPosition, ReferencePosition) - FVector2f::Distance(TestPreviousPosition, TestPosition);
			ReferencePreviousPositions[Index] = ReferencePosition;
			TestPreviousPositions[Index] = TestPosition;
		}
		
		// Inter-pedestrian Distance Metric
		{
			float ReferenceInterPedDistance = 0.0f;
			float TestInterPedDistance = 0.0f;
		
			for (int OtherIndex = Index + 1; OtherIndex < NumEntities; ++OtherIndex)
			{
				const FVector2f& BaselineOtherPosition = Reference[OtherIndex].Position;
				const FVector2f& TestOtherPosition = Test[OtherIndex].Position;
				ReferenceInterPedDistance += FVector2f::Distance(BaselineOtherPosition, ReferencePosition);
				TestInterPedDistance += FVector2f::Distance(TestOtherPosition, TestPosition);
			}
		
			ReferenceInterPedDistance /= (NumEntities - 1);
			TestInterPedDistance /= (NumEntities - 1);
		
			InterPedDistanceMetric += FMath::Abs(ReferenceInterPedDistance - TestInterPedDistance);
		}
	}
	
	AvgAbsoluteDifferenceMetric += AbsoluteDifferenceMetric / NumEntities;
	AvgInterPedDistanceMetric += InterPedDistanceMetric / NumEntities;
	AvgPathLengthMetric += PathLengthMetric / NumEntities;
}

void ASimulationActor::SimulateFast()
{
	CurrentSimulator = FastSimulator;
	StartSimulator();
}

void ASimulationActor::SimulateBaseline()
{
	CurrentSimulator = BaselineSimulator;
	StartSimulator();
}

void ASimulationActor::StartSimulator()
{
	StopVisualisation();
	InitialiseSimulation();
	
	// Simulation
	double AverageTimeSpent = 0;
	int NumFrames = 0;
	while (ElapsedSimTime < SimulationLength)
	{
		const uint64 StartCycles = FPlatformTime::Cycles64();
		Simulate(SimulationTimeStep);
		const uint64 EndCycles = FPlatformTime::Cycles64();
		
		ElapsedSimTime += SimulationTimeStep;
		++NumFrames;
		AverageTimeSpent += FPlatformTime::ToMilliseconds64(EndCycles - StartCycles);
	}
		
	UE_LOG(LogTemp, Warning, TEXT("Time spent per frame = %lf") ,AverageTimeSpent / NumFrames);
}

void ASimulationActor::StopVisualisation()
{
	if (const UWorld* World = GetWorld())
	{
		VisualisationFrameIndex = 0;
		World->GetTimerManager().ClearTimer(EvaluationVizTimerHandle);
		World->GetTimerManager().ClearTimer(VizTimerHandle);
	}
}

void ASimulationActor::PlayEvaluationVisualisation()
{
	const auto Play = [this]()
	{
		if (VisualisationFrameIndex >= BaselineSimCache.Num())
		{
			StopVisualisation();
			return;
		}
	
		const TArray<TPair<FVector2f, int>>& FirstFrame = BaselineSimCache[VisualisationFrameIndex];
		for (const auto& Entity : FirstFrame)
		{
			const FVector2f& Position = Entity.Get<0>();
			DrawDebugSphere(GetWorld(), {Position.X, Position.Y, 0.0f}, 25.0f, 10, FColor::Red, false, SimulationTimeStep / VisualisationPlaybackRate);
		}
	
		const TArray<TPair<FVector2f, int>>& SecondFrame = FastSimCache[VisualisationFrameIndex];
		for (const auto& Entity : SecondFrame)
		{
			const FVector2f& Position = Entity.Get<0>();
			DrawDebugSphere(GetWorld(), {Position.X, Position.Y, 0.0f}, 25.0f, 10, FColor::Blue, false, SimulationTimeStep / VisualisationPlaybackRate);
		}
		++VisualisationFrameIndex;
	};
	
	StopVisualisation();
	if (const UWorld* World = GetWorld())
	{
		FTimerDelegate TimerDelegate;
		TimerDelegate.BindLambda(Play);
		World->GetTimerManager().SetTimer(EvaluationVizTimerHandle, TimerDelegate, SimulationTimeStep, true);
	}
}

void ASimulationActor::PlayVisualisation()
{
	const auto Play = [this]()
	{
		if (VisualisationFrameIndex >= PrimarySimulationCache.Num())
		{
			StopVisualisation();
			return;
		}
	
		const TArray<TPair<FVector2f, int>>& Frame = PrimarySimulationCache[VisualisationFrameIndex];
		for (const auto& Entity : Frame)
		{
			const FVector2f& Position = Entity.Get<0>();
			const FColor Color = EntityColors[Entity.Get<1>()];
			DrawDebugSphere(GetWorld(), {Position.X, Position.Y, 0.0f}, 25.0f, 10, Color, false, SimulationTimeStep / VisualisationPlaybackRate);
		}
		++VisualisationFrameIndex;
	};
	
	StopVisualisation();
	if (const UWorld* World = GetWorld())
	{
		FTimerDelegate TimerDelegate;
		TimerDelegate.BindLambda(Play);
		World->GetTimerManager().SetTimer(VizTimerHandle, TimerDelegate, SimulationTimeStep / VisualisationPlaybackRate, true);
	}
}

void ASimulationActor::Simulate(const float DeltaSeconds)
{
	check(CurrentSimulator.IsValid());
	PrimarySimulationCache.Push({});
	
	CurrentSimulator.Pin()->UpdateSimulation(Entities, SimulationTimeStep);
	CurrentSimulator.Pin()->MoveEntites(Entities, SimulationTimeStep);
	
	for (const FTCEntity& Entity : Entities)
	{
		PrimarySimulationCache.Last().Push({Entity.Position, Entity.GroupID});
	}
}

void ASimulationActor::DrawDebugBaseline()
{
	const UWorld* World = GetWorld();
	const FRpSpatialData<FTCBaselineCell>& Field = StaticCastSharedPtr<TCBaselineContinuumCrowdSimulator>(BaselineSimulator)->GetFieldData();
	
	// Draw entities.
	if (DebugSettings.bDrawEntities)
	{
		for (const FTCEntity& Entity : Entities)
		{
			if (EntityColors[Entity.GroupID].A == 0)
			{
				continue;
			}
			
			if (!Field.IsValidWorldPosition(Entity.Position))
			{
				continue;
			}
			
			const FVector Position = {Entity.Position.X, Entity.Position.Y, 0.0f};
			DrawDebugSphere(World, Position, 25.0f, 10, EntityColors[Entity.GroupID]);
			if (DebugSettings.bDrawTraces)
			{
				DrawDebugPoint(World, Position, 2.0f, EntityColors[Entity.GroupID], false, 20.0f);
			}
		}
	}
	
	// Debug DensityField.
	if (DebugSettings.bDrawDensityField)
	{
		float MaxDensity = TNumericLimits<float>::Min();
		const auto GetMaxDensity = [&MaxDensity, this](const FTCBaselineCell* Cell, const FVector2f& Coords)
		{
			if (Cell->bIsWall)
			{
				return;
			}
			const float& Density = Cell->Density;
			if (Density > MaxDensity)
			{
				MaxDensity = Density;
			}
		};
		Field.ForEachCellPerform(GetMaxDensity);
		
		const auto DrawDensities = [this, World, Field, MaxDensity](const FTCBaselineCell* Cell, const FVector2f& Coords)
		{
			const float& Density = Cell->Density;
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
			DrawDebugString(World, StringLocation , String, this, FColor::White, SimulationTimeStep);
		};
	
		Field.ForEachCellPerform(DrawDensities);
	}
	
	// Debug potential field.
	if (DebugSettings.bDrawPotentialField)
	{
		float MaxPotential = TNumericLimits<float>::Min();
		const auto GetMaxPotential = [&MaxPotential, this](const FTCBaselineCell* Cell, const FVector2f& Coords)
		{
			if (Cell->bIsWall)
			{
				return;
			}
			
			const float& Potential = Cell->Potential[DebugSettings.DebugGroupID];
			if (Potential > MaxPotential)
			{
				MaxPotential = Potential;
			}
		};
		
		Field.ForEachCellPerform(GetMaxPotential);
		const auto DrawPotential = [this, World, Field, MaxPotential](const FTCBaselineCell* Cell, const FVector2f& Coords)
		{
			const float NormPotential = Cell->Potential[DebugSettings.DebugGroupID] / MaxPotential;
			
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
	if (DebugSettings.bDrawCellVelocityField)
	{
		const auto DrawVelocties = [this, World, Field](const FTCBaselineCell* Cell, const FVector2f& Coords) -> void
		{
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
		};
		Field.ForEachCellPerform(DrawVelocties);
	}
	
	// Debug DesiredVelocityField.
	if (DebugSettings.bDrawDesiredVelocityField)
	{
		const auto DrawVelocties = [this, World, Field](const FTCBaselineCell* Cell, const FVector2f& Coords) -> void
		{
			if (Cell->DesiredVelocity[DebugSettings.DebugGroupID].IsNearlyZero())
			{
				return;
			}
			
			const float CellSize = Field.GetCellSize();
			const FVector2f WorldLocation = Field.GridToWorld(Coords);
			const FVector2f Direction = Cell->DesiredVelocity[DebugSettings.DebugGroupID].GetSafeNormal();
			const FVector LineStart = {WorldLocation.X + CellSize / 2, WorldLocation.Y + CellSize / 2, 0};
			const FVector LineEnd = LineStart + FVector{Direction.X, Direction.Y, 0.0f} * CellSize * 0.5f;
			const FColor Color = SpawnConfigurations[DebugSettings.DebugGroupID].Color;
			DrawDebugLine(World, LineStart, LineStart, Color, false, -1, 0, 7.0f);
			DrawDebugLine(World, LineStart, LineEnd, Color, false, -1, 0, 2.0f);
		};
		Field.ForEachCellPerform(DrawVelocties);
	}
	
	if (DebugSettings.bDrawWalls)
	{
		const auto DrawWall = [World, Field](const FTCBaselineCell* Cell, const FVector2f& Coords)
		{
			if (Cell->bIsWall)
			{
				const float DebugBoxExtent = Field.GetCellSize();
				const FVector2f WorldCoords = Field.GridToWorld(Coords);
				const FVector BoxMin = {WorldCoords.X, WorldCoords.Y, 0};
				const FVector BoxMax = {WorldCoords.X + DebugBoxExtent, WorldCoords.Y + DebugBoxExtent, 100};
				const FColor BoxColor{0,0,255,128};
				DrawDebugSolidBox(World, FBox(BoxMin, BoxMax), BoxColor);
			}
		};
		Field.ForEachCellPerform(DrawWall);
	}
	
	if (DebugSettings.bDrawGrid)
	{
		const auto DrawBox = [World, Field](const FTCBaselineCell* Cell, const FVector2f& Coords)
		{
			const float DebugBoxExtent = Field.GetCellSize();
			const FVector2f WorldCoords = Field.GridToWorld(Coords);
			const FVector BoxMin = {WorldCoords.X, WorldCoords.Y, 0};
			const FVector BoxMax = {WorldCoords.X + DebugBoxExtent, WorldCoords.Y + DebugBoxExtent, 100};
			const FLinearColor BoxColor(1, 1, 1, 0.1);
			DrawDebugSolidBox(World, FBox(BoxMin, BoxMax), BoxColor.ToFColor(false));
		};
		Field.ForEachCellPerform(DrawBox);
	}
	
	if (DebugSettings.bDrawDiscomfortZones)
	{
		const auto DrawWall = [World, Field](const FTCBaselineCell* Cell, const FVector2f& Coords)
		{
			if (Cell->Discomfort != 0)
			{
				const float DebugBoxExtent = Field.GetCellSize();
				const FVector2f WorldCoords = Field.GridToWorld(Coords);
				const FVector BoxMin = {WorldCoords.X, WorldCoords.Y, 0};
				const FVector BoxMax = {WorldCoords.X + DebugBoxExtent, WorldCoords.Y + DebugBoxExtent, 100};
				const FColor BoxColor = FLinearColor::LerpUsingHSV(FLinearColor::Transparent, FLinearColor::Green, Cell->Discomfort).ToFColor(false);
				DrawDebugSolidBox(World, FBox(BoxMin, BoxMax), BoxColor);
			}
		};
		Field.ForEachCellPerform(DrawWall);
	}
}

void ASimulationActor::DrawDebugFast()
{
	const UWorld* World = GetWorld();
	const FRpSpatialData<FTCFastCell>& Field = StaticCastSharedPtr<TCFastContinuumCrowdSimulator>(FastSimulator)->GetFieldData();
	
	// Draw entities.
	if (DebugSettings.bDrawEntities)
	{
		for (const FTCEntity& Entity : Entities)
		{
			if (EntityColors[Entity.GroupID].A == 0)
			{
				continue;
			}
			
			if (!Field.IsValidWorldPosition(Entity.Position))
			{
				continue;
			}
			
			const FVector Position = {Entity.Position.X, Entity.Position.Y, 0.0f};
			DrawDebugSphere(World, Position, 25.0f, 10, EntityColors[Entity.GroupID]);
			if (DebugSettings.bDrawTraces)
			{
				DrawDebugPoint(World, Position, 2.0f, EntityColors[Entity.GroupID], false, 20.0f);
			}
		}
	}
	
	// Debug DensityField.
	if (DebugSettings.bDrawDensityField)
	{
		float MaxDensity = TNumericLimits<float>::Min();
		const auto GetMaxDensity = [&MaxDensity, this](const FTCFastCell* Cell, const FVector2f& Coords)
		{
			if (Cell->bIsWall)
			{
				return;
			}
			const float& Density = Cell->ByteDensity;
			if (Density > MaxDensity)
			{
				MaxDensity = Density;
			}
		};
		Field.ForEachCellPerform(GetMaxDensity);
		
		const auto DrawDensities = [this, World, Field, MaxDensity](const FTCFastCell* Cell, const FVector2f& Coords)
		{
			const float& Density = Cell->ByteDensity;
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
			DrawDebugString(World, StringLocation , String, this, FColor::White, SimulationTimeStep);
		};
	
		Field.ForEachCellPerform(DrawDensities);
	}
	
	// Debug potential field.
	if (DebugSettings.bDrawPotentialField)
	{
		float MaxPotential = TNumericLimits<float>::Min();
		const auto GetMaxPotential = [&MaxPotential, this](const FTCFastCell* Cell, const FVector2f& Coords)
		{
			if (Cell->bIsWall)
			{
				return;
			}
			
			const float& Potential = Cell->Potential[DebugSettings.DebugGroupID];
			if (Potential > MaxPotential)
			{
				MaxPotential = Potential;
			}
		};
		
		Field.ForEachCellPerform(GetMaxPotential);
		const auto DrawPotential = [this, World, Field, MaxPotential](const FTCFastCell* Cell, const FVector2f& Coords)
		{
			const float NormPotential = Cell->Potential[DebugSettings.DebugGroupID] / MaxPotential;
			
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
	if (DebugSettings.bDrawCellVelocityField)
	{
		const auto DrawVelocties = [this, World, Field](const FTCFastCell* Cell, const FVector2f& Coords) -> void
		{
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
		};
		Field.ForEachCellPerform(DrawVelocties);
	}
	
	// Debug DesiredVelocityField.
	if (DebugSettings.bDrawDesiredVelocityField)
	{
		const auto DrawVelocties = [this, World, Field](const FTCFastCell* Cell, const FVector2f& Coords) -> void
		{
			if (Cell->DesiredVelocity[DebugSettings.DebugGroupID].IsNearlyZero())
			{
				return;
			}
			
			const float CellSize = Field.GetCellSize();
			const FVector2f WorldLocation = Field.GridToWorld(Coords);
			const FVector2f Direction = Cell->DesiredVelocity[DebugSettings.DebugGroupID].GetSafeNormal();
			const FVector LineStart = {WorldLocation.X + CellSize / 2, WorldLocation.Y + CellSize / 2, 0};
			const FVector LineEnd = LineStart + FVector{Direction.X, Direction.Y, 0.0f} * CellSize * 0.5f;
			const FColor Color = SpawnConfigurations[DebugSettings.DebugGroupID].Color;
			DrawDebugLine(World, LineStart, LineStart, Color, false, -1, 0, 7.0f);
			DrawDebugLine(World, LineStart, LineEnd, Color, false, -1, 0, 2.0f);
		};
		Field.ForEachCellPerform(DrawVelocties);
	}
	
	if (DebugSettings.bDrawWalls)
	{
		const auto DrawWall = [World, Field](const FTCFastCell* Cell, const FVector2f& Coords)
		{
			if (Cell->bIsWall)
			{
				const float DebugBoxExtent = Field.GetCellSize();
				const FVector2f WorldCoords = Field.GridToWorld(Coords);
				const FVector BoxMin = {WorldCoords.X, WorldCoords.Y, 0};
				const FVector BoxMax = {WorldCoords.X + DebugBoxExtent, WorldCoords.Y + DebugBoxExtent, 100};
				const FColor BoxColor{0,0,255,128};
				DrawDebugSolidBox(World, FBox(BoxMin, BoxMax), BoxColor);
			}
		};
		Field.ForEachCellPerform(DrawWall);
	}
	
	if (DebugSettings.bDrawGrid)
	{
		const auto DrawBox = [World, Field](const FTCFastCell* Cell, const FVector2f& Coords)
		{
			const float DebugBoxExtent = Field.GetCellSize();
			const FVector2f WorldCoords = Field.GridToWorld(Coords);
			const FVector BoxMin = {WorldCoords.X, WorldCoords.Y, 0};
			const FVector BoxMax = {WorldCoords.X + DebugBoxExtent, WorldCoords.Y + DebugBoxExtent, 100};
			const FLinearColor BoxColor(1, 1, 1, 0.1);
			DrawDebugSolidBox(World, FBox(BoxMin, BoxMax), BoxColor.ToFColor(false));
		};
		Field.ForEachCellPerform(DrawBox);
	}
	
	if (DebugSettings.bDrawDiscomfortZones)
	{
		const auto DrawWall = [World, Field](const FTCFastCell* Cell, const FVector2f& Coords)
		{
			if (Cell->Discomfort != 0)
			{
				const float DebugBoxExtent = Field.GetCellSize();
				const FVector2f WorldCoords = Field.GridToWorld(Coords);
				const FVector BoxMin = {WorldCoords.X, WorldCoords.Y, 0};
				const FVector BoxMax = {WorldCoords.X + DebugBoxExtent, WorldCoords.Y + DebugBoxExtent, 100};
				const FColor BoxColor = FLinearColor::LerpUsingHSV(FLinearColor::Transparent, FLinearColor::Green, Cell->Discomfort).ToFColor(false);
				DrawDebugSolidBox(World, FBox(BoxMin, BoxMax), BoxColor);
			}
		};
		Field.ForEachCellPerform(DrawWall);
	}
}

void ASimulationActor::InitialiseEntityStartLocations()
{
	Entities.Reset();
	EntityColors.Reset();
	
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
			const float X = FMath::Clamp(S * (A * FMath::Cos(T) * FMath::Cos(R) - FMath::Sin(T) * FMath::Sin(R)) + H, 0, WorldSpan);
			const float Y = FMath::Clamp(S * (A * FMath::Cos(T) * FMath::Sin(R) + FMath::Sin(T) * FMath::Cos(R)) + K, 0, WorldSpan);
			
			const FVector2f NewPosition{X, Y};
			{
				const FRpSpatialData<FTCFastCell>& Field = StaticCastSharedPtr<TCFastContinuumCrowdSimulator>(FastSimulator)->GetFieldData();
				if (Field.GetDataAt(Field.WorldToGrid(NewPosition))->bIsWall)
				{
					continue;
				}
			}
			{
				const FRpSpatialData<FTCBaselineCell>& Field = StaticCastSharedPtr<TCBaselineContinuumCrowdSimulator>(BaselineSimulator)->GetFieldData();
				if (Field.GetDataAt(Field.WorldToGrid(NewPosition))->bIsWall)
				{
					continue;
				}
			}

			Entities.Push({NewPosition, FVector2f{FVector2f::ZeroVector}, GroupID});
			++NumSpawned;
		}
		EntityColors.Push(Configuration.Color);
		
		const float GoalX = FMath::Clamp(Configuration.Goal.X, 0, WorldSpan);
		const float GoalY = FMath::Clamp(Configuration.Goal.Y, 0, WorldSpan);
		BaselineSimulator->RegisterGoal(GroupID, {GoalX, GoalY});
		FastSimulator->RegisterGoal(GroupID, {GoalX, GoalY});
		
		++GroupID;
	}
}

void ASimulationActor::PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent)
{
	FName PropertyName = (PropertyChangedEvent.MemberProperty != nullptr) ? PropertyChangedEvent.MemberProperty->GetFName() : NAME_None;
	
	if (PropertyName == GET_MEMBER_NAME_CHECKED(ASimulationActor, BaselineCrowdSimParams))
	{
		GEngine->AddOnScreenDebugMessage(-1, 2, FColor::White, FString::Printf(TEXT("Sim Parameters changed : %s"), *PropertyChangedEvent.GetPropertyName().ToString()));
		BaselineSimulator->SetSimulationParameters(BaselineCrowdSimParams);
	}

	if (PropertyName == GET_MEMBER_NAME_CHECKED(ASimulationActor, FastCrowdSimParams))
	{
		GEngine->AddOnScreenDebugMessage(-1, 2, FColor::White, FString::Printf(TEXT("Sim Parameters changed : %s"), *PropertyChangedEvent.GetPropertyName().ToString()));
		FastSimulator->SetSimulationParameters(FastCrowdSimParams);
	}
	
	if (PropertyName == GET_MEMBER_NAME_CHECKED(ASimulationActor, SocialForceParams))
	{
		GEngine->AddOnScreenDebugMessage(-1, 2, FColor::White, FString::Printf(TEXT("Ped Parameters changed : %s"), *PropertyChangedEvent.GetPropertyName().ToString()));
		FastSimulator->SetAdvectionParameters(SocialForceParams);
		BaselineSimulator->SetAdvectionParameters(SocialForceParams);
	}
	
	Super::PostEditChangeProperty(PropertyChangedEvent);
}