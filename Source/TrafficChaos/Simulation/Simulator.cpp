#include "Simulator.h"

constexpr float GAUSSIAN_FALLOFF = 1.0;
constexpr float GAUSSIAN_SCALE = 1.0f;
constexpr float MAX_TOPO_SPEED = 1;
constexpr float MIN_TOPO_SPEED = 0.1;
constexpr float MIN_SLOPE = 0;
constexpr float MAX_SLOPE = 1;
constexpr float MIN_DENSITY = 0;
constexpr float MAX_DENSITY = 1;
constexpr int VELOCITY_LOOKUP_OFFSET = 1;
constexpr int DENSITY_LOOKUP_OFFSET = 1;
constexpr float PATH_COST_CONSTANT = 1;
constexpr float TIME_COST_CONSTANT = 1;
constexpr float DISCOMFORT_COST_CONSTANT = 1;

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

void TCSimulator::RegisterEntity(const FVector2f& InitialPosition, const FVector2f& InitialVelocity)
{
	EntityPositions.Add(InitialPosition);
	EntityVelocities.Add(InitialVelocity);
}

void TCSimulator::Update(const float DeltaSeconds)
{
	UpdateEntityPositions(DeltaSeconds);
	
	UpdateDensityField();
	UpdateCellVelocityField();
	UpdateSpeedField();
	UpdateCostField();
	
	Solve({Field.GetResolution() - 1, Field.GetResolution() - 1});
	
	UpdatePotentialGradient();
	UpdateDesiredVelocityField();
}

void TCSimulator::Solve(const FVector2f& GoalCoords)
{
	FTCCell* GoalCell = Field.GetDataAt(GoalCoords);
	checkf(GoalCell, TEXT("Invalid coordinates for goal."));
	
	Knowns.Empty();
	Unknowns.Empty();
	Candidates.Empty();

	const auto InitiallizePotentials = [this, GoalCell](FTCCell* Cell, const FVector2f& Coords)->void
	{
		if(Cell != GoalCell)
		{
			Cell->Potential = MAX_COST;
			Unknowns.Push(Cell);
		}
		else
		{
			Cell->Potential = 0.0f;
			Knowns.Push(Cell);
		}
	};
	Field.ForEachCellPerform(InitiallizePotentials);
	
	for(int Index = 0; Index < Unknowns.Num(); ++Index)
	{
		FTCCell* Cell = Unknowns[Index];
		
		float& CurrentPotential = Cell->Potential;
		const float NewPotential = GetFiniteDifferenceApproximation(Cell->Coords);
		if(NewPotential < CurrentPotential)
		{
			CurrentPotential = NewPotential;
			Unknowns.RemoveSwap(Cell);
			Candidates.HeapPush(Cell, FTCMostOptimalNode());
		}
	}

	while(!Candidates.IsEmpty())
	{
		FTCCell* Cell;
		Candidates.HeapPop(Cell, FTCMostOptimalNode());

		Knowns.Push(Cell);

		for(FTCCell* Neighbor : GetNeighbors(Cell->Coords))
		{
			if(Knowns.Contains(Neighbor))
			{
				continue;
			}

			float& CurrentPotential = Neighbor->Potential;
			const float NewPotential = GetFiniteDifferenceApproximation(Neighbor->Coords);
			if(NewPotential < CurrentPotential)
			{
				CurrentPotential = NewPotential;
				
				Unknowns.RemoveSwap(Neighbor);
				Candidates.HeapPush(Neighbor, FTCMostOptimalNode());
			}
		}
	}
}

void TCSimulator::UpdateEntityPositions(const float DeltaSeconds)
{
	for (int EntityIndex = 0; EntityIndex < EntityPositions.Num(); ++EntityIndex)
	{
		EntityPositions[EntityIndex] = EntityPositions[EntityIndex] + EntityVelocities[EntityIndex] * DeltaSeconds;
	}
}

void TCSimulator::UpdateDensityField()
{
	const auto ResetDensities = [](FTCCell* Cell, const FVector2f& Coords) -> void
	{
		Cell->Density = 0;
	};
	Field.ForEachCellPerform(ResetDensities);
	
	for (const FVector2f& EntityPosition : EntityPositions)
	{
		const FVector2f GridCoords = Field.WorldToGridSnapped(EntityPosition);
		
		const auto ApplyDensity = [this, GridCoords](const FVector2f Offset, const float Distance)
		{
			if (FTCCell* Cell = Field.GetDataAt(GridCoords, Offset))
			{
				Cell->Density = FMath::Min(1.0f, Cell->Density + GaussianDistribution(Distance));
			}
		};

		ApplyDensity({0, 0}, 0.0f);
		ApplyDensity(D_NORTH, 1.0f);
		ApplyDensity(D_NORTH_WEST, 1.414f);
		ApplyDensity(D_WEST, 1.0f);
		ApplyDensity(D_SOUTH_WEST, 1.414f);
		ApplyDensity(D_SOUTH, 1.0f);
		ApplyDensity(D_SOUTH_EAST, 1.414f);
		ApplyDensity(D_EAST, 1.0f);
		ApplyDensity(D_NORTH_EAST, 1.414f);
	}
}

void TCSimulator::UpdateCellVelocityField()
{
	// Reset velocities to zero.
	const auto ResetVelocities = [](FTCCell* Cell, const FVector2f& Coords) -> void
	{
		Cell->Velocity = FVector2f::ZeroVector;
	};
	Field.ForEachCellPerform(ResetVelocities);
	
	for (int EntityIndex = 0; EntityIndex < EntityPositions.Num(); ++EntityIndex)
	{
		const FVector2f& EntityVelocity = EntityVelocities[EntityIndex];
		const FVector2f& GridLocation = Field.WorldToGridSnapped(EntityPositions[EntityIndex]);
		
		if (FTCCell* Cell = Field.GetDataAt(GridLocation))
		{
			Cell->Velocity += EntityVelocity;
		}
	}
	
	const auto AverageVelocities = [this](FTCCell* Cell, const FVector2f& Coords)
	{
		checkf(Field.IsValidGridCoordinate(Coords), TEXT("Invalid grid coordinates."));
		
		const float Density = Field.GetDataAt(Coords)->Density;
		Field.GetDataAt(Coords)->Velocity /= FMath::Max(Density, 1.0f);
	};
	Field.ForEachCellPerform(AverageVelocities);
}

void TCSimulator::UpdateSpeedField()
{
	const auto CalculateSpeedField = [this](FTCCell* Cell, const FVector2f& Coords)
	{
		for(EDirectionIndex DirectionIndex : CARDINAL_DIRECTIONS)
		{
			const FVector2f& Direction = DIRECTION_OFFSETS[DirectionIndex];
			
			float TempFlowSpeed = 0.1f;
			if(const FTCCell* NeighborCell = Field.GetDataAt(Coords, Direction * static_cast<float>(VELOCITY_LOOKUP_OFFSET)))
			{
				TempFlowSpeed = FMath::Max(FVector2f::DotProduct(NeighborCell->Velocity, Direction.GetSafeNormal()), 0.1f);
			}
			const float FlowSpeed = TempFlowSpeed;

			float TempNeighborDensity = MIN_DENSITY;
			if(FTCCell* NeighborCell = Field.GetDataAt(Coords, Direction * static_cast<float>(DENSITY_LOOKUP_OFFSET)))
			{
				TempNeighborDensity = NeighborCell->Density;
			}
			const float NeighborDensity = TempNeighborDensity;
			
			if(NeighborDensity <= MIN_DENSITY)
			{
				Cell->SpeedField[DirectionIndex] = MAX_TOPO_SPEED;
			}
			else if(NeighborDensity >= MAX_DENSITY)
			{
				Cell->SpeedField[DirectionIndex] = FlowSpeed;
			}
			else
			{
				Cell->SpeedField[DirectionIndex] = MAX_TOPO_SPEED + ((NeighborDensity - MIN_DENSITY)/(MAX_DENSITY - MIN_DENSITY)) * (FlowSpeed - MAX_TOPO_SPEED);	
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
			const float Discomfort = 0; //NeighborCell->Discomfort;
			if(SpeedField != 0)
			{
				Cell->CostField[DirectionIndex] = (PATH_COST_CONSTANT * SpeedField + TIME_COST_CONSTANT + DISCOMFORT_COST_CONSTANT * Discomfort) / SpeedField;
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
	return FMath::Exp(-1 * FMath::Square(Distance) / GAUSSIAN_FALLOFF) * GAUSSIAN_SCALE;
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