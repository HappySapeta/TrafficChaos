#include "Simulator.h"

constexpr float MAX_COST = TNumericLimits<float>::Max();

void TCSimulator::Initialize(const float Resolution, const float WorldSize)
{
	Field.Initialize(Resolution, WorldSize, {});
	const auto InitializeCells = [](FTCCell* Cell, const FVector2f& Coords)
	{
		Cell->Coords = Coords;
	};
	
	Field.ForEachCellPerform(InitializeCells);
}

void TCSimulator::Update(const TArray<FVector2f>& EntityPositions, const TArray<FVector2f>& EntityVelocities, const float DeltaSeconds)
{
	UpdateDensityAndVelocityField(EntityPositions, EntityVelocities);
	UpdateSpeedField();
	UpdateCostField();
	
	Solve({static_cast<float>(Field.GetResolution() / 2), 0});
	
	UpdatePotentialGradient();
	UpdateDesiredVelocityField();
}

void TCSimulator::PerformCrowdAdvection
(
	const TArray<FVector2f>& EntityPositions, 
	const TArray<FVector2f>& EntityVelocities,
	TArray<FVector2f>& OutNewVelocities,
	const float DeltaSeconds
)
{
	const int NumEntities = EntityPositions.Num();
	check(NumEntities == EntityVelocities.Num());
	OutNewVelocities.Reserve(NumEntities);
	
	for (int EntityIndex = 0; EntityIndex < NumEntities; ++EntityIndex)
	{
		const FVector2f& EntityPosition = EntityPositions[EntityIndex];
		if (!Field.IsValidWorldPosition(EntityPosition))
		{
			continue;
		}
		
		const FVector2f GridLocation = Field.WorldToGrid(EntityPosition);
		const FVector2f& DesiredVelocity = Field.GetDataAt(GridLocation)->DesiredVelocity;
		const FVector2f& CurrentVelocity = EntityVelocities[EntityIndex];
		const FVector2f Direction = (DesiredVelocity.GetSafeNormal() * PedParameters.LookaheadDistance).GetSafeNormal();
		const FVector2f Acceleration = 1/PedParameters.RelaxationTime * (PedParameters.MaxSpeed * Direction - CurrentVelocity);
		
		OutNewVelocities[EntityIndex] = EntityVelocities[EntityIndex] + Acceleration * DeltaSeconds;
	}
}

void TCSimulator::Solve(const FVector2f& GoalCoords)
{
	checkf(Field.IsValidGridCoordinate(GoalCoords), TEXT("Invalid coordinates for goal."));
	
	// 1. Clear all lists. 
	Knowns.Empty();
	Candidates.Empty();
	
	// 1. Get the goal cell.
	FTCCell* GoalCell = Field.GetDataAt(GoalCoords);
	GoalCell->Potential = 0;
	Candidates.PushFirst(GoalCell);
	
	// 2. Initialize potentials
	const auto InitializePotential = [GoalCoords](FTCCell* Cell, const FVector2f& Coords)
	{
		if (Coords != GoalCoords)
		{
			Cell->Potential = MAX_COST;
		}
	};
	Field.ForEachCellPerform(InitializePotential);
	
	// 4. BFS
	while (!Candidates.IsEmpty())
	{
		FTCCell* Current = Candidates.First();
		Candidates.PopFirst();
		
		const TArray<FTCCell*> Neighbors = GetNeighbors(Current->Coords);
		for (FTCCell* Neighbor : Neighbors)
		{
			if (Knowns.Contains(Neighbor))
			{
				continue;
			}
			
			float& CurrentPotential = Neighbor->Potential;
			const float NewPotential = GetFiniteDifferenceApproximation(Neighbor->Coords);
			if(NewPotential < CurrentPotential)
			{
				CurrentPotential = NewPotential;
				Candidates.PushLast(Neighbor);
			}
		}
		
		Knowns.Add(Current);
	}
}

void TCSimulator::UpdateDensityAndVelocityField(const TArray<FVector2f>& EntityPositions, const TArray<FVector2f>& EntityVelocities)
{
	const auto ResetCellDensityAndVelocties = [](FTCCell* Cell, const FVector2f& Coords) -> void
	{
		Cell->Density = 0;
		Cell->Velocity = {0, 0};
	};
	
	Field.ForEachCellPerform(ResetCellDensityAndVelocties);
	
	for(int EntityIndex = 0; EntityIndex < EntityPositions.Num(); ++EntityIndex)
	{
		const FVector2f& EntityPosition = EntityPositions[EntityIndex];
		const FVector2f& EntityVelocity = EntityVelocities[EntityIndex];
		if(!Field.IsValidWorldPosition(EntityPosition))
		{
			continue;
		}
		
		const FVector2f EntityPreciseCoords = Field.WorldToGridCentered(EntityPosition);
		const FVector2f ClosestCellCenterCoords = {FMath::RoundToInt(EntityPreciseCoords.X) - 0.5f, FMath::RoundToInt(EntityPreciseCoords.Y) - 0.5f};
		
		const FVector2f Delta = EntityPreciseCoords - ClosestCellCenterCoords;
		
		if(FTCCell* SouthEastCell = Field.GetDataAt(ClosestCellCenterCoords, D_SOUTH_EAST)) // C
		{
			const float DensityContribution = FMath::Pow(std::min(Delta.X, Delta.Y), SimParameters.DensityExponent);
			SouthEastCell->Density += std::max(DensityContribution, SimParameters.MinDensity);
			SouthEastCell->Velocity += DensityContribution * EntityVelocity;
		}

		if(FTCCell* EastCell = Field.GetDataAt(ClosestCellCenterCoords, D_EAST)) // D
		{
			const float DensityContribution = FMath::Pow(std::min(Delta.X, 1 - Delta.Y), SimParameters.DensityExponent);
			EastCell->Density += std::min(DensityContribution, SimParameters.MinDensity);
			EastCell->Velocity += DensityContribution * EntityVelocity;
		}

		if(FTCCell* ClosestCell = Field.GetDataAt(ClosestCellCenterCoords)) // A
		{
			const float DensityContribution = FMath::Pow(std::min(1 - Delta.X, 1 - Delta.Y), SimParameters.DensityExponent);
			ClosestCell->Density += std::min(DensityContribution, SimParameters.MinDensity);
			ClosestCell->Velocity += DensityContribution * EntityVelocity;
		}

		if(FTCCell* SouthCell = Field.GetDataAt(ClosestCellCenterCoords, D_SOUTH)) // B
		{
			const float DensityContribution = FMath::Pow(std::min(1 - Delta.X, Delta.Y), SimParameters.DensityExponent);
			SouthCell->Density += std::min(DensityContribution, SimParameters.MinDensity);
			SouthCell->Velocity += DensityContribution * EntityVelocity;
		}
	}

	const auto Operation = [](FTCCell* Cell, const FVector2f& Coords) -> void
	{
		if(Cell->Density != 0.0f)
		{
			Cell->Velocity /= Cell->Density;
		}
	};
	
	Field.ForEachCellPerform(Operation);
}

void TCSimulator::UpdateSpeedField()
{
	const auto CalculateSpeedField = [this](FTCCell* Cell, const FVector2f& Coords)
	{
		for(EDirectionIndex DirectionIndex : CARDINAL_DIRECTIONS)
		{
			const FVector2f& Direction = DIRECTION_OFFSETS[DirectionIndex];
			
			float TempFlowSpeed = 0.1f;
			if(const FTCCell* NeighborCell = Field.GetDataAt(Coords, Direction * static_cast<float>(SimParameters.VelocityLookupOffset)))
			{
				TempFlowSpeed = FMath::Max(FVector2f::DotProduct(NeighborCell->Velocity, Direction.GetSafeNormal()), 0.1f);
			}
			const float FlowSpeed = TempFlowSpeed;

			float TempNeighborDensity = SimParameters.MinDensity;
			if(FTCCell* NeighborCell = Field.GetDataAt(Coords, Direction * static_cast<float>(SimParameters.DensityLookupOffset)))
			{
				TempNeighborDensity = NeighborCell->Density;
			}
			const float NeighborDensity = TempNeighborDensity;
			
			if(NeighborDensity <= SimParameters.MinDensity)
			{
				Cell->SpeedField[DirectionIndex] = SimParameters.MaxTopoSpeed;
			}
			else if(NeighborDensity >= SimParameters.MaxDensity)
			{
				Cell->SpeedField[DirectionIndex] = FlowSpeed;
			}
			else
			{
				Cell->SpeedField[DirectionIndex] = SimParameters.MaxTopoSpeed + ((NeighborDensity - SimParameters.MinDensity)/(SimParameters.MaxDensity - SimParameters.MinDensity)) * (FlowSpeed - SimParameters.MaxTopoSpeed);	
			}
		}
	};

	Field.ForEachCellPerform(CalculateSpeedField);
}

void TCSimulator::UpdateCostField()
{
	const auto CalculateCost = [this](FTCCell* Cell, const FVector2f& Coords)
	{
		for(const EDirectionIndex DirectionIndex : CARDINAL_DIRECTIONS)
		{
			const FTCCell* NeighborCell = Field.GetDataAt(Coords, DIRECTION_OFFSETS[DirectionIndex]);
			if(!NeighborCell)
			{
				Cell->CostField[DirectionIndex] = MAX_COST;
				continue;
			}
			
			const float SpeedField = Cell->SpeedField[DirectionIndex];
			if(SpeedField != 0)
			{
				Cell->CostField[DirectionIndex] = (SimParameters.PathCostConstant * SpeedField + SimParameters.TimeCostConstant) / SpeedField;
			}
			else
			{
				Cell->CostField[DirectionIndex] = MAX_COST;
			}
		}
	};

	Field.ForEachCellPerform(CalculateCost);
}

void TCSimulator::UpdatePotentialGradient()
{
	const auto Operation = [this](FTCCell* Cell, const FVector2f& Coords) -> void
	{
		for(const EDirectionIndex DirectionIndex : CARDINAL_DIRECTIONS)
		{
			if(const FTCCell* Neighbor = Field.GetDataAt(Coords, DIRECTION_OFFSETS[DirectionIndex]))
			{
				const float Gradient = Neighbor->Potential - Cell->Potential;
				Cell->PotentialGradient[DirectionIndex] = Gradient;
			}
		}
	};
	
	Field.ForEachCellPerform(Operation);
}

void TCSimulator::UpdateDesiredVelocityField()
{
	const auto CalculateDesiredVelocity = [this](FTCCell* Cell, const FVector2f& Coords) -> void
	{
		FVector4 NormPotential = FVector4
		{
			Cell->PotentialGradient[NORTH],
			Cell->PotentialGradient[WEST],
			Cell->PotentialGradient[SOUTH],
			Cell->PotentialGradient[EAST]
		}.GetSafeNormal();
		
		Cell->PotentialGradient[NORTH] = NormPotential.X;
		Cell->PotentialGradient[WEST] = NormPotential.Y;
		Cell->PotentialGradient[SOUTH] = NormPotential.Z;
		Cell->PotentialGradient[EAST] = NormPotential.W;

		Cell->DesiredVelocity = {0, 0};
		Cell->DesiredVelocity += -Cell->SpeedField[NORTH] * NormPotential.X * DIRECTION_OFFSETS[NORTH];
		Cell->DesiredVelocity += -Cell->SpeedField[WEST] * NormPotential.Y * DIRECTION_OFFSETS[WEST];
		Cell->DesiredVelocity += -Cell->SpeedField[SOUTH] * NormPotential.Z * DIRECTION_OFFSETS[SOUTH];
		Cell->DesiredVelocity += -Cell->SpeedField[EAST] * NormPotential.W * DIRECTION_OFFSETS[EAST];	
	};

	Field.ForEachCellPerform(CalculateDesiredVelocity);
	
	int X = 0;
}

float TCSimulator::GetFiniteDifferenceApproximation(const FVector2f& Coords)
{
	auto GetCheapestAdjCellOnAxis = [this](const FVector2f& Coordinates, const EDirectionIndex FirstDirection, const EDirectionIndex SecondDirection) -> FTCCheapestNeighbor
	{
		FTCCell* CurrentCell = Field.GetDataAt(Coordinates);
		FTCCell* FirstNeighbor = Field.GetDataAt(Coordinates, DIRECTION_OFFSETS[FirstDirection]);
		FTCCell* SecondNeighbor = Field.GetDataAt(Coordinates, DIRECTION_OFFSETS[SecondDirection]);

		if(FirstNeighbor && !SecondNeighbor)
		{
			return {FirstNeighbor->Potential, CurrentCell->CostField[FirstDirection]};
		}
		else if(!FirstNeighbor && SecondNeighbor)
		{
			return {SecondNeighbor->Potential, CurrentCell->CostField[SecondDirection]};
		}
		else if(!FirstNeighbor && !SecondNeighbor)
		{
			return {MAX_COST, MAX_COST};
		}

		const FTCCheapestNeighbor ResultFirst = {FirstNeighbor->Potential, CurrentCell->CostField[FirstDirection]};
		const FTCCheapestNeighbor ResultSecond = {SecondNeighbor->Potential, CurrentCell->CostField[SecondDirection]};
		
		return ResultFirst.Sum() < ResultSecond.Sum() ? ResultFirst : ResultSecond;
	};
	
	auto Square = [](const float V){ return V * V; };
	const auto [PhiX, Cx] = GetCheapestAdjCellOnAxis(Coords, EAST, WEST);
	const auto [PhiY, Cy] = GetCheapestAdjCellOnAxis(Coords, NORTH, SOUTH);
	
	if(PhiX == MAX_COST)
	{
		return PhiY + Cy;
	}

	if(PhiY == MAX_COST)
	{
		return PhiX + Cx;
	}
	
	const float QuadraticCoeffA = Square(Cy) + Square(Cx);
	const float QuadraticCoeffB = -2 * ((PhiX * Square(Cy)) + (PhiY * Square(Cx)));
	const float QuadraticCoeffC = (Square(PhiX) * Square(Cy)) + (Square(PhiY) * Square(Cx)) - (Square(Cx) * Square(Cy));

	const float TermUnderSqrt = Square(QuadraticCoeffB) - (4 * QuadraticCoeffA * QuadraticCoeffC);
	if(TermUnderSqrt >= 0.0f)
	{
		const float FirstSolution = (-QuadraticCoeffB + FMath::Sqrt(TermUnderSqrt)) / (2 * QuadraticCoeffA);
		const float SecondSolution = (-QuadraticCoeffB - FMath::Sqrt(TermUnderSqrt)) / (2 * QuadraticCoeffA);

		const float ResultPotential = FMath::Max(FirstSolution, SecondSolution);

		if(ResultPotential > PhiX && ResultPotential > PhiY)
		{
			return ResultPotential;
		}
	}
	
	return FMath::Min(PhiX + Cx, PhiY + Cy);
}

float TCSimulator::GaussianDistribution(const float Distance)
{
	return FMath::Exp(-1 * FMath::Square(Distance) / SimParameters.GaussianFallOff) * SimParameters.GaussianScale;
}

TArray<FTCCell*> TCSimulator::GetNeighbors(const FVector2f& Coords)
{
	static TArray<FTCCell*> Neighbors;
	Neighbors.Empty();
	
	for (const FVector2f& Offset : DIRECTION_OFFSETS)
	{
		if (FTCCell* Node = Field.GetDataAt(Coords, Offset))
		{
			Neighbors.Push(Node);
		}
	}
	
	return Neighbors;
}