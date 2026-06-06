#include "Simulator.h"

constexpr float GAUSSIAN_FALLOFF = 1.0;
constexpr float GAUSSIAN_SCALE = 1.0f;

constexpr float MAX_POTENTIAL = 100;

void TCSimulator::Initialize(const float Resolution, const float WorldSize)
{
	Field.Initialize(Resolution, WorldSize, {});
}

void TCSimulator::AddEntity(const FVector2f& InitialPosition, const FVector2f InitialVelocity, const float InitialHeading)
{
	Entities.Add(InitialPosition, InitialVelocity, InitialHeading);
}

void TCSimulator::UpdateDensityField()
{
	const auto ResetDensities = [](FTCCell* Cell, const FVector2f& Coords) -> void
	{
		Cell->Density = 0;
	};
	Field.ForEachCellPerform(ResetDensities);
	
	for (int EntityIndex = 0; EntityIndex < Entities.Num(); ++EntityIndex)
	{
		const FVector2f GridCoords = Field.WorldToGridSnapped(Entities.Positions[EntityIndex]);
		
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

void TCSimulator::UpdateVelocityField()
{
	// Reset velocities to zero.
	const auto ResetVelocities = [](FTCCell* Cell, const FVector2f& Coords) -> void
	{
		Cell->Velocity = FVector2f::ZeroVector;
	};
	Field.ForEachCellPerform(ResetVelocities);
	
	for (int EntityIndex = 0; EntityIndex < Entities.Num(); ++EntityIndex)
	{
		const FVector2f& EntityVelocity = Entities.Velocities[EntityIndex];
		const FVector2f& GridLocation = Field.WorldToGridSnapped(Entities.Positions[EntityIndex]);
		
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

void TCSimulator::UpdateCostField()
{
	const auto UpdateCosts = [this](FTCCell* Cell, const FVector2f& Coords) -> void
	{
		for(const EDirectionIndex DirectionIndex : CARDINAL_DIRECTIONS)
		{
			const FTCCell* NeighborCell = Field.GetDataAt(Coords, DIRECTION_OFFSETS[DirectionIndex]);
			if(!NeighborCell)
			{
				Cell->CostField[DirectionIndex] = MAX_POTENTIAL;
				continue;
			}
			
			const float SpeedField = GetSpeedField(Cell->Velocity, DirectionIndex);
			Cell->CostField[DirectionIndex] = Cell->Density + SpeedField;
		}
	};
	
	Field.ForEachCellPerform(UpdateCosts);
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
			return {MAX_POTENTIAL, MAX_POTENTIAL};
		}

		const FTCCheapestNeighbor ResultFirst = {FirstNeighbor->Potential, CurrentCell->CostField[FirstDirection]};
		const FTCCheapestNeighbor ResultSecond = {SecondNeighbor->Potential, CurrentCell->CostField[SecondDirection]};
		
		return ResultFirst.Sum() < ResultSecond.Sum() ? ResultFirst : ResultSecond;
	};
	
	auto Square = [](const float V){ return V * V; };
	const auto [PhiX, Cx] = GetCheapestAdjCellOnAxis(Coords, EAST, WEST);
	const auto [PhiY, Cy] = GetCheapestAdjCellOnAxis(Coords, NORTH, SOUTH);
	
	if(PhiX == MAX_POTENTIAL)
	{
		return PhiY + Cy;
	}

	if(PhiY == MAX_POTENTIAL)
	{
		return PhiX + Cx;
	}
	
	const float QuadraticCoeffA = Square(Cy) + Square(Cx);
	const float QuadraticCoeffB = -2 * ((PhiX * Square(Cy)) + (PhiY * Square(Cx)));
	const float QuadraticCoeffC = (Square(PhiX) * Square(Cy)) + (Square(PhiY) * Square(Cx)) - (Square(Cx) * Square(Cy));

	const float TermUnderSqrt = Square(QuadraticCoeffB) - (4 * QuadraticCoeffA * QuadraticCoeffC);
	if(TermUnderSqrt >= 0.0f)
	{
		const float FirstSolution = (-QuadraticCoeffB + sqrt(TermUnderSqrt)) / (2 * QuadraticCoeffA);
		const float SecondSolution = (-QuadraticCoeffB - sqrt(TermUnderSqrt)) / (2 * QuadraticCoeffA);

		const float ResultPotential = std::max(FirstSolution, SecondSolution);

		if(ResultPotential > PhiX && ResultPotential > PhiY)
		{
			return ResultPotential;
		}
	}
	
	return std::min(PhiX + Cx, PhiY + Cy);
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
			Cell->Potential = MAX_POTENTIAL;
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

void TCSimulator::CalculatePotentialGradient()
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

void TCSimulator::CalculateDesiredVelocityField()
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
		Cell->DesiredVelocity += -GetSpeedField(Cell->Velocity, NORTH) * NormPotential.X * DIRECTION_OFFSETS[NORTH];
		Cell->DesiredVelocity += -GetSpeedField(Cell->Velocity, WEST) * NormPotential.Y * DIRECTION_OFFSETS[WEST];
		Cell->DesiredVelocity += -GetSpeedField(Cell->Velocity, SOUTH) * NormPotential.Z * DIRECTION_OFFSETS[SOUTH];
		Cell->DesiredVelocity += -GetSpeedField(Cell->Velocity, EAST) * NormPotential.W * DIRECTION_OFFSETS[EAST];	
	};

	Field.ForEachCellPerform(CalculateDesiredVelocity);
}

float TCSimulator::GaussianDistribution(const float Distance)
{
	return FMath::Exp(-1 * FMath::Square(Distance) / GAUSSIAN_FALLOFF) * GAUSSIAN_SCALE;
}

float TCSimulator::GetSpeedField(const FVector2f& Velocity, const EDirectionIndex Direction)
{
	switch (Direction)
	{
		case NORTH:
			return Velocity.Y;
		case WEST:
			return -Velocity.X;
		case SOUTH:
			return -Velocity.Y;
		case EAST:
			return Velocity.X;
	}
	
	return 0;
};

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

void TCSimulator::Update(const float DeltaSeconds)
{
	Debug_MoveEntities(DeltaSeconds);
	UpdateDensityField();
	UpdateVelocityField();
	UpdateCostField();
	
	if (!bSolved)
	{
		Solve({Field.GetResolution() - 1, Field.GetResolution() - 1});
		//bSolved = true;
		
		CalculatePotentialGradient();
		CalculateDesiredVelocityField();
	}
}

void TCSimulator::Debug_MoveEntities(const float DeltaSeconds)
{
	constexpr float Radius = 1.0f;
	static float Theta = 0.0f;
	if (Theta >= 2 * PI)
	{
		Theta = 0;
	}
	
	Theta += DeltaSeconds;
	
	for (int EntityIndex = 0; EntityIndex < Entities.Num(); ++EntityIndex)
	{
		const FVector2f Translation = FVector2f(Radius * FMath::Cos(Theta), Radius * FMath::Sin(Theta));
		Entities.Positions[EntityIndex] += Translation;
		Entities.Velocities[EntityIndex] = Translation / DeltaSeconds; 
	}
}
