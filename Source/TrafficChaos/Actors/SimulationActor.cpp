// Copyright Anupam Sahu. All Rights Reserved.

#include "SimulationActor.h"
#include "Kismet/KismetMathLibrary.h"

constexpr float COLLISION_MARGIN = 1.01f;

// Sets default values
ASimulationActor::ASimulationActor()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	
	BaselineCrowdSimParams = TInstancedStruct<FTCBaselineSimParameters>::Make();
	FastCrowdSimParams = TInstancedStruct<FTCFastSimulationParameters>::Make();
	BaselineSimulator = MakeShared<TCBaselineContinuumCrowdSimulator>();
	FastSimulator = MakeShared<TCFastContinuumCrowdSimulator>();
}

void ASimulationActor::BeginPlay()
{
	Super::BeginPlay();
	StopVisualisation();
	InitialiseSimulation();
	switch (SimulatorType)
	{
		case ESimulatorType::Baseline:
			CurrentSimulator = BaselineSimulator;
			break;
		case ESimulatorType::Fast:
			CurrentSimulator = FastSimulator;
			break;
	}
}

void ASimulationActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	CurrentSimulator.Pin()->UpdateSimulation(Entities);
	CurrentSimulator.Pin()->MoveEntites(Entities, DeltaSeconds);

	switch (SimulatorType)
	{
		case ESimulatorType::Baseline:
			DrawDebugBaseline();
			break;
		case ESimulatorType::Fast:
			DrawDebugFast();
			break;
	}
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

void ASimulationActor::NormaliseMetrics(const int NumFrames)
{
	const int NumEntities = Entities.Num();
	const int NumPairs = (NumEntities * (NumEntities - 1)) / 2;
	{
		Metrics.TotalAbsoluteDifferenceMetric = FMath::Abs(Metrics.TotalAbsoluteDifferenceMetric) / (NumFrames * NumEntities);
	}
	{
		Metrics.TotalPathLengthMetric.BaselinePathLength /= NumEntities;
		Metrics.TotalPathLengthMetric.TestPathLength /= NumEntities;
		Metrics.TotalPathLengthMetric.Difference /= NumEntities;
	}
	{
		Metrics.TotalPedDistanceMetric.BaselinePedDistance = FMath::Abs(Metrics.TotalPedDistanceMetric.BaselinePedDistance) / (NumFrames * NumPairs);
		Metrics.TotalPedDistanceMetric.TestPedDistance = FMath::Abs(Metrics.TotalPedDistanceMetric.TestPedDistance) / (NumFrames * NumPairs);
		Metrics.TotalPedDistanceMetric.Difference = FMath::Abs(Metrics.TotalPedDistanceMetric.Difference) / (NumFrames * NumPairs);
	}
	{
		Metrics.TotalVorticityMetric.Difference = FMath::Abs(Metrics.TotalVorticityMetric.Difference) / NumFrames;
		Metrics.TotalVorticityMetric.BaselineVorticity = Metrics.TotalVorticityMetric.BaselineVorticity / NumFrames;
		Metrics.TotalVorticityMetric.TestVorticity = Metrics.TotalVorticityMetric.TestVorticity / NumFrames;
	}
	{
		Metrics.TotalCollisionsMetric.BaselineCollisions /= NumFrames;
		Metrics.TotalCollisionsMetric.TestCollisions /= NumFrames;
	}
	{
		Metrics.TotalDensityMetric.BaselineAvgDensity /= NumFrames;
		Metrics.TotalDensityMetric.TestAvgDensity /= NumFrames;
	}
	{
		Metrics.TotalSpeedMetric.BaselineAvgSpeed /= NumFrames;
		Metrics.TotalSpeedMetric.TestAvgSpeed /= NumFrames;
	}
}

void ASimulationActor::Evaluate()
{
	StopVisualisation();
	InitialiseSimulation();
	
	BaselinePreviousPositions.Init(FVector2f::ZeroVector, Entities.Num());
	TestPreviousPositions.Init(FVector2f::ZeroVector, Entities.Num());
	PedVelocityField.Initialize(Resolution, WorldSpan, {});
	const int DensityFieldTargetResolution = FMath::RoundToInt(WorldSpan / 100.0f);
	PedDensityField.Initialize(DensityFieldTargetResolution, WorldSpan, {});
	Speeds.Init(0, Entities.Num());
	
	for (int Index = 0; Index < Entities.Num(); ++Index)
	{
		const FVector2f& Position = Entities[Index].Position;
		BaselinePreviousPositions[Index] = Position;
		TestPreviousPositions[Index] = Position;
	}
	
	TArray<FTCEntity> BaselineEntities = Entities;
	TArray<FTCEntity> FastSimEntities = Entities;
	
	Metrics = {};
	
	const int NumFrames = SimulationLength / SimulationTimeStep;
	for (int Frame = 0; Frame < NumFrames; ++Frame)
	{
		BaselineSimulator->UpdateSimulation(BaselineEntities);
		BaselineSimulator->MoveEntites(BaselineEntities, SimulationTimeStep);
		
		FastSimulator->UpdateSimulation(FastSimEntities);
		FastSimulator->MoveEntites(FastSimEntities, SimulationTimeStep);
		
		Metrics.TotalAbsoluteDifferenceMetric += CalcFrameAbsoluteDifferenceMetric(BaselineEntities, FastSimEntities);
		Metrics.TotalPathLengthMetric += CalcFramePathLengthMetric(BaselineEntities, FastSimEntities);
		Metrics.TotalPedDistanceMetric += CalcFrameInterPedestrianDistanceMetric(BaselineEntities, FastSimEntities);
		Metrics.TotalVorticityMetric += CalcFrameVorticityMetric(BaselineEntities, FastSimEntities);
		Metrics.TotalCollisionsMetric += CalcFrameCollisionsMetric(BaselineEntities, FastSimEntities);
		Metrics.TotalDensityMetric += CalcFrameAverageDensityMetric(BaselineEntities, FastSimEntities);
		Metrics.TotalSpeedMetric += CalcFrameAverageSpeedMetric(BaselineEntities, FastSimEntities);
	}
	
	NormaliseMetrics(NumFrames);
	
	UE_LOG
	(
		LogTemp, Warning, TEXT("%s,%d,%.2f, %s, %s, %s, %s, %s, %s"),
		*GetWorld()->GetMapName(),
		RandomSeed,
		Metrics.TotalAbsoluteDifferenceMetric, 
		*Metrics.TotalPedDistanceMetric.ToString(),
		*Metrics.TotalPathLengthMetric.ToString(),
		*Metrics.TotalVorticityMetric.ToString(),
		*Metrics.TotalCollisionsMetric.ToString(),
		*Metrics.TotalDensityMetric.ToString(),
		*Metrics.TotalSpeedMetric.ToString()
	)

	//UE_LOG
	//(
	//	LogTemp, Warning, TEXT("Baseline Memory. Field : %llu, Solver : %llu"), 
	//	BaselineSimulator->GetMaxAllocatedSize().FieldSize,
	//	BaselineSimulator->GetMaxAllocatedSize().SolverDataSize
	//);
	
	//UE_LOG
	//(
	//	LogTemp, Warning, TEXT("Test Memory. Field : %llu, Solver : %llu"), 
	//	FastSimulator->GetMaxAllocatedSize().FieldSize,
	//	FastSimulator->GetMaxAllocatedSize().SolverDataSize
	//);
}

float ASimulationActor::CalcFrameAbsoluteDifferenceMetric(const TArray<FTCEntity>& Baseline, const TArray<FTCEntity>& Test) const
{
	float FrameAbsoluteDifferenceMetric = 0.0f;
	const int NumEntities = Entities.Num();
	for (int Index = 0; Index < NumEntities; ++Index)
	{
		FrameAbsoluteDifferenceMetric += FVector2f::Distance(Baseline[Index].Position, Test[Index].Position);
	}
	
	return FrameAbsoluteDifferenceMetric;
}

FTCPathLengthMetric ASimulationActor::CalcFramePathLengthMetric(const TArray<FTCEntity>& Baseline, const TArray<FTCEntity>& Test)
{
	float Difference = 0.0f;
	float FrameBaselinePathLength = 0.0f;
	float FrameTestPathLength = 0.0f;
	const int NumEntities = Entities.Num();
	for (int Index = 0; Index < NumEntities; ++Index)
	{
		const FVector2f& BaselineCurrentPosition = Baseline[Index].Position;
		const FVector2f& TestCurrentPosition = Test[Index].Position;
		const FVector2f& BaselinePreviousPosition = BaselinePreviousPositions[Index];
		const FVector2f& TestPreviousPosition = TestPreviousPositions[Index];
		
		const float BaselinePathLength = FVector2f::Distance(BaselinePreviousPosition, BaselineCurrentPosition);
		const float TestPathLength = FVector2f::Distance(TestPreviousPosition, TestCurrentPosition);
		BaselinePreviousPositions[Index] = BaselineCurrentPosition;
		TestPreviousPositions[Index] = TestCurrentPosition;
		
		FrameBaselinePathLength += BaselinePathLength;
		FrameTestPathLength += TestPathLength;
		Difference += BaselinePathLength - TestPathLength;
	}
	
	return 
	{
		.BaselinePathLength = FrameBaselinePathLength, 
		.TestPathLength = FrameTestPathLength, 
		.Difference = Difference
	};
}

FTCInterPedDistanceMetric ASimulationActor::CalcFrameInterPedestrianDistanceMetric(const TArray<FTCEntity>& Baseline, const TArray<FTCEntity>& Test) const
{
	const int NumEntities = Entities.Num();
	const auto CalculateInterPedDistanceSum = [NumEntities](const TArray<FTCEntity>& TargetEntities) -> float
	{
		float InterPedDistance = 0.0f;
		for (int Index = 0; Index < NumEntities; ++Index)
		{
			for (int OtherIndex = Index + 1; OtherIndex < NumEntities; ++OtherIndex)
			{
				InterPedDistance += FVector2f::Distance(TargetEntities[OtherIndex].Position, TargetEntities[Index].Position);
			}
		}
		
		return InterPedDistance;
	};
	
	const float BaselinePedDistance = CalculateInterPedDistanceSum(Baseline);
	const float TestPedDistance = CalculateInterPedDistanceSum(Test);
	const float Difference = BaselinePedDistance - TestPedDistance;
	return {BaselinePedDistance, TestPedDistance, Difference};
}

FTCFrameVorticityMetric ASimulationActor::CalcFrameVorticityMetric(const TArray<FTCEntity>& Baseline, const TArray<FTCEntity>& Test)
{
	const float CellSize = WorldSpan / static_cast<float>(Resolution);
	
	float Vorticity = 0.0f;
	const auto CalculateVorticity = [this, CellSize, &Vorticity](const FTCPedVelocityCell* Cell, const FVector2f& Coords) -> void
	{
		if (Cell->Density == 0)
		{
			return;
		}
			
		float DeltaVy;
		if (const FTCPedVelocityCell* EastCell = PedVelocityField.GetDataAt(Coords, D_EAST))
		{
			if (EastCell->Density == 0)
			{
				return;
			}
			DeltaVy = EastCell->AvgVelocity.Y - Cell->AvgVelocity.Y;
		}
		else
		{
			return;
		}
			
		float DeltaVx;
		if (const FTCPedVelocityCell* NorthCell = PedVelocityField.GetDataAt(Coords, D_SOUTH))
		{
			if (NorthCell->Density == 0)
			{
				return;
			}
			DeltaVx = NorthCell->AvgVelocity.X - Cell->AvgVelocity.X;
		}
		else
		{
			return;
		}
		
		Vorticity += DeltaVy / CellSize - DeltaVx / CellSize;
	};
		
	InitPedVelocityField(Baseline);
	PedVelocityField.ForEachCellPerform(CalculateVorticity);
	const float BaselineVorticity = Vorticity;
	
	Vorticity = 0.0f;
	InitPedVelocityField(Test);
	PedVelocityField.ForEachCellPerform(CalculateVorticity);
	const float TestVorticity = Vorticity;
	
	const FTCFrameVorticityMetric FrameVorticityMetric {BaselineVorticity, TestVorticity, BaselineVorticity - TestVorticity};
	return FrameVorticityMetric;
}

FTCCollisionsMetric ASimulationActor::CalcFrameCollisionsMetric(const TArray<FTCEntity>& Baseline, const TArray<FTCEntity>& Test) const
{
	const int NumEntities = Entities.Num();
	const auto CountCollisions = [this, NumEntities](const TArray<FTCEntity>& EntityArray) -> int
	{
		int NumCollidedPairs = 0;
		for (int Index = 0; Index < NumEntities; ++Index)
		{
			const FVector2f& Position = EntityArray[Index].Position;
			for (int OtherIndex = Index + 1; OtherIndex < NumEntities; ++OtherIndex)
			{
				const FVector2f& OtherPosition = EntityArray[OtherIndex].Position;
				if (FVector2f::Distance(Position, OtherPosition) < (2 * SocialForceParams.PedestrianHalfSize * COLLISION_MARGIN))
				{
					++NumCollidedPairs;
				}
			}
		}
		
		return NumCollidedPairs;
	};
	
	const int NumBaselineCollisionPairs = CountCollisions(Baseline);
	const int NumTestCollisionPairs = CountCollisions(Test);
	
	return {static_cast<float>(NumBaselineCollisionPairs), static_cast<float>(NumTestCollisionPairs)};
}

FTCDensityMetric ASimulationActor::CalcFrameAverageDensityMetric(const TArray<FTCEntity>& Baseline, const TArray<FTCEntity>& Test)
{
	int NumOccupiedCells = 0;
	const auto UpdateOccupancy = [&NumOccupiedCells](const FTCPedDensityCell* CellDensity, const FVector2f& Coords) -> void
	{
		if (CellDensity->Density == 0)
		{
			return;
		}
			
		++NumOccupiedCells;
	};
		
	InitPedDensityField(Baseline);
	PedDensityField.ForEachCellPerform(UpdateOccupancy);
	const float BaselineAvgDensity = (Entities.Num() / static_cast<float>(NumOccupiedCells));
		
	NumOccupiedCells = 0;
	InitPedDensityField(Test);
	PedDensityField.ForEachCellPerform(UpdateOccupancy);
	const float TestAvgDensity = (Entities.Num() / static_cast<float>(NumOccupiedCells));
	
	return {BaselineAvgDensity, TestAvgDensity};
}

FTCSpeedMetric ASimulationActor::CalcFrameAverageSpeedMetric(const TArray<FTCEntity>& Baseline, const TArray<FTCEntity>& Test)
{
	const auto CalcAverageSpeed = [this](const TArray<FTCEntity>& EntityArray) -> float
	{
		for (int Index = 0; Index < EntityArray.Num(); ++Index)
		{
			const FTCEntity& Entity = EntityArray[Index];
			Speeds[Index] = Entity.Velocity.Length(); 
		}
		
		return Algo::Accumulate(Speeds, 0) / EntityArray.Num();
	};
	
	const float BaselineAvgSpeed = CalcAverageSpeed(Baseline);
	const float TestAvgSpeed = CalcAverageSpeed(Test);
	
	return {BaselineAvgSpeed, TestAvgSpeed};
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
		
	UE_LOG(LogTemp, Warning, TEXT("Time spent per frame in ms = %lf"), AverageTimeSpent / NumFrames);
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
	
	CurrentSimulator.Pin()->UpdateSimulation(Entities);
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
			DrawDebugSphere(World, Position, SocialForceParams.PedestrianHalfSize, 10, EntityColors[Entity.GroupID]);
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
				constexpr FColor BoxColor{0,0,255,128};
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
			constexpr FLinearColor BoxColor(1, 1, 1, 0.1);
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
			
			const FString String = FString::Printf(TEXT("%.2f"), Cell->Potential[DebugSettings.DebugGroupID]);
			const FVector StringLocation = {WorldCoords.X + DebugBoxExtent / 2, WorldCoords.Y + DebugBoxExtent / 2, 0.0f}; 
			DrawDebugString(World, StringLocation , String, this, FColor::White, SimulationTimeStep);
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
				constexpr FColor BoxColor{0,0,255,128};
				DrawDebugSolidBox(World, FBox(BoxMin, BoxMax), BoxColor);
			}
		};
		Field.ForEachCellPerform(DrawWall);
	}
	
	if (DebugSettings.bDrawGrid)
	{
		const auto DrawBox = [this, World, Field](const FTCFastCell* Cell, const FVector2f& Coords)
		{
			const float DebugBoxExtent = Field.GetCellSize();
			const FVector2f WorldCoords = Field.GridToWorld(Coords);
			const FVector BoxMin = {WorldCoords.X, WorldCoords.Y, 0};
			const FVector BoxMax = {WorldCoords.X + DebugBoxExtent, WorldCoords.Y + DebugBoxExtent, 100};
			constexpr FLinearColor BoxColor(1, 1, 1, 0.1);
			DrawDebugSolidBox(World, FBox(BoxMin, BoxMax), BoxColor.ToFColor(false));
			
			const FString String = FString::Printf(TEXT("%.0f, %.0f"), Coords.X, Coords.Y);
			const FVector StringLocation = {WorldCoords.X + DebugBoxExtent / 2, WorldCoords.Y + DebugBoxExtent / 2, 0.0f}; 
			DrawDebugString(World, StringLocation , String, this, FColor::White, SimulationTimeStep);
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
		
		while (NumSpawned < NumEntitiesPerGroup)
		{
			const float S = UKismetMathLibrary::RandomFloatInRangeFromStream(RandomStream, 0, SpawnRange);
			const float T = UKismetMathLibrary::RandomFloatInRangeFromStream(RandomStream, 0, 2 * PI);
			const float X = FMath::Clamp(S * (A * FMath::Cos(T) * FMath::Cos(R) - FMath::Sin(T) * FMath::Sin(R)) + H, 0, WorldSpan);
			const float Y = FMath::Clamp(S * (A * FMath::Cos(T) * FMath::Sin(R) + FMath::Sin(T) * FMath::Cos(R)) + K, 0, WorldSpan);
			
			const FVector2f NewPosition{X, Y};
			{
				const FRpSpatialData<FTCFastCell>& Field = StaticCastSharedPtr<TCFastContinuumCrowdSimulator>(FastSimulator)->GetFieldData();
				if (Field.GetDataAt(Field.WorldToGridIndices(NewPosition))->bIsWall)
				{
					continue;
				}
			}
			{
				const FRpSpatialData<FTCBaselineCell>& Field = StaticCastSharedPtr<TCBaselineContinuumCrowdSimulator>(BaselineSimulator)->GetFieldData();
				if (Field.GetDataAt(Field.WorldToGridIndices(NewPosition))->bIsWall)
				{
					continue;
				}
			}
			
			Entities.Push(
			{
				.Position = NewPosition, 
				.Velocity = FVector2f{FVector2f::ZeroVector}, 
				.GroupID = GroupID,
#ifdef ENABLE_VELOCITY_OVERRIDING
				.OverrideVelocity = Configuration.OverrideVelocity,
				.bUseOverrideVelocity = Configuration.bUseOverrideVelocity
#endif
			});
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

void ASimulationActor::InitPedVelocityField(const TArray<FTCEntity>& EntityArray)
{
	ResetPedDensityVelocityField();
	for (int Index = 0; Index < EntityArray.Num(); ++Index)
	{
		const FVector2f& EntityVelocity = EntityArray[Index].Velocity;
		const FVector2f& GridIndices = PedVelocityField.WorldToGridIndices(EntityArray[Index].Position);
		if (FTCPedVelocityCell* Cell = PedVelocityField.GetDataAt(GridIndices))
		{
			const FVector2f& TotalVelocity = Cell->AvgVelocity * Cell->Density;
			const FVector2f& NewTotalVelocity = TotalVelocity + EntityVelocity;
			Cell->Density += 1;
			Cell->AvgVelocity = NewTotalVelocity / Cell->Density;
		}
	}
}

void ASimulationActor::InitPedDensityField(const TArray<FTCEntity>& EntityArray)
{
	ResetPedDensityField();
	for (int Index = 0; Index < EntityArray.Num(); ++Index)
	{
		const FVector2f& GridIndices = PedDensityField.WorldToGridIndices(EntityArray[Index].Position);
		if (FTCPedDensityCell* Cell = PedDensityField.GetDataAt(GridIndices))
		{
			Cell->Density += 1;
		}
	}
}

void ASimulationActor::ResetPedDensityField()
{
	const auto Reset = [](FTCPedDensityCell* Cell, const FVector2f& Coords)
	{
		Cell->Density = 0;
	};
	PedDensityField.ForEachCellPerform(Reset);
}

void ASimulationActor::ResetPedDensityVelocityField()
{
	const auto Reset = [](FTCPedVelocityCell* Cell, const FVector2f& Coords)
	{
		Cell->AvgVelocity = FVector2f::ZeroVector;
		Cell->Density = 0;
	};
	PedVelocityField.ForEachCellPerform(Reset);
}
