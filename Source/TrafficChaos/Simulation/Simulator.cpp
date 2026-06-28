// Copyright Anupam Sahu. All Rights Reserved.

#include "Simulator.h"

constexpr float MAX_COST = TNumericLimits<float>::Max();
constexpr float HALF_FOV = FMath::DegreesToRadians(200.0f) * 0.5f;
constexpr float WEAK_INF = 0.5f;

void TCSimulator::Initialize(const float Resolution, const float WorldSize, const int NewNumGroups)
{
	ImplicitGrid.Initialize(FFloatRange(0, WorldSize), Resolution);
	Field.Initialize(Resolution, WorldSize, {});
	
	const auto InitializeCell = [NewNumGroups](FTCCell* Cell, const FVector2f& Coords)
	{
		Cell->Coords = Coords;
		Cell->Density = 0;
		Cell->Discomfort = 0;
		Cell->DesiredVelocity.Init({}, NewNumGroups);
		Cell->Potential.Init({}, NewNumGroups);
		Cell->PotentialGradient.Init({}, NewNumGroups);
	};
	Field.ForEachCellPerform(InitializeCell);
	
	NumGroups = NewNumGroups;
}

void TCSimulator::RegisterGoal(const int GroupID, const FVector2f& Goal)
{
	Goals.Add({GroupID, Goal});
}

void TCSimulator::CrowdAdvection(TArray<FTCEntity>& Entities, const float TimeStep)
{
	TArray<FVector> Positions;
	for (const FTCEntity& Entity : Entities)
	{
		Positions.Push({Entity.Position.X, Entity.Position.Y, 0.0f});
	}
	ImplicitGrid.Update(Positions);
	
	for (int EntityIndex = 0; EntityIndex < Entities.Num(); ++EntityIndex)
	{
		FVector2f Force = FVector2f::ZeroVector;
		
		const FVector2f& CurrentVelocity = Entities[EntityIndex].Velocity;
		const FVector2f& CurrentPosition = Entities[EntityIndex].Position;
		
		const FVector2f GridLocation = Field.WorldToGrid(CurrentPosition);
		const FVector2f& DesiredVelocity = Field.GetDataAt(GridLocation)->DesiredVelocity[Entities[EntityIndex].GroupID];
		const FVector2f DesiredDirection = DesiredVelocity.GetSafeNormal();
		const FVector2f DrivingForce = FTCSocialForces::GetDrivingForce(CurrentVelocity, DesiredDirection, PedParameters); 
		Force += GetSocialForceInfluence(DesiredDirection, DrivingForce) * DrivingForce;
		 
		FRpSearchResults Results;
		ImplicitGrid.RadialSearch({CurrentPosition.X, CurrentPosition.Y, 0}, PedParameters.AvoidanceRadius, Results);
		
		for (int OtherEntityIndex : Results)
		{
			if (EntityIndex == OtherEntityIndex)
			{
				continue;
			}
			
			const FVector2f& OtherPosition = Entities[OtherEntityIndex].Position;
			const FVector2f& OtherVelocity = Entities[OtherEntityIndex].Velocity;
			const FVector2f AvoidanceForce = FTCSocialForces::GetAvoidanceForce(CurrentPosition, OtherPosition, OtherVelocity, TimeStep, PedParameters); 
			Force += GetSocialForceInfluence(DesiredDirection, -AvoidanceForce) * AvoidanceForce;
		}
		
		const auto LimitSpeed = [this](const FVector2f& Velocity) -> FVector2f
		{
			return FMath::Min(PedParameters.DesiredSpeed, Velocity.Length()) * Velocity.GetSafeNormal();
		};
		
		Entities[EntityIndex].Velocity = LimitSpeed(Entities[EntityIndex].Velocity + Force * TimeStep);
		Entities[EntityIndex].Position += Entities[EntityIndex].Velocity * TimeStep;
	}	
}

void TCSimulator::Update(const TArray<FTCEntity>& Entities, const float DeltaSeconds)
{
	UpdateDensityAndVelocityField(Entities);
	UpdateSpeedField();
	
	for (int GroupID = 0; GroupID < NumGroups; ++GroupID)
	{
		UpdateCostField();
		Solve(GroupID);
		UpdatePotentialGradient(GroupID);
		UpdateDesiredVelocityField(GroupID);
	}
}

void TCSimulator::Solve(const int GroupID)
{
	check(Goals.Contains(GroupID));
	
	const FVector2f GoalCoords = Field.WorldToGrid(Goals[GroupID]);
	checkf(Field.IsValidGridCoordinate(GoalCoords), TEXT("Invalid coordinates for goal."));
	
	// 1. Clear all lists. 
	Knowns.Empty();
	Candidates.Empty();
	
	// 1. Get the goal cell.
	FTCCell* GoalCell = Field.GetDataAt(GoalCoords);
	GoalCell->Potential[GroupID] = 0;
	Candidates.PushFirst(GoalCell);
	
	// 2. Initialize potentials
	const auto InitializePotential = [GoalCoords, GroupID](FTCCell* Cell, const FVector2f& Coords)
	{
		if (Coords != GoalCoords)
		{
			Cell->Potential[GroupID] = MAX_COST;
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
			
			const float NewPotential = GetFiniteDifferenceApproximation(Neighbor->Coords, GroupID);
			if(NewPotential < Neighbor->Potential[GroupID])
			{
				Neighbor->Potential[GroupID] = NewPotential;
				Candidates.PushLast(Neighbor);
			}
		}
		
		Knowns.Add(Current);
	}
}

void TCSimulator::UpdateDensityAndVelocityField(const TArray<FTCEntity>& Entities)
{
	const auto ResetCellDensityAndVelocties = [](FTCCell* Cell, const FVector2f& Coords) -> void
	{
		Cell->Density = 0;
		Cell->Velocity = {0, 0};
	};
	Field.ForEachCellPerform(ResetCellDensityAndVelocties);
	
	for(const FTCEntity& Entity : Entities)
	{
		const FVector2f& EntityPosition = Entity.Position;
		const FVector2f& EntityVelocity = Entity.Velocity;
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

	const auto CalcAverageVelocity = [](FTCCell* Cell, const FVector2f& Coords) -> void
	{
		if(Cell->Density != 0.0f)
		{
			Cell->Velocity /= Cell->Density;
		}
	};
	Field.ForEachCellPerform(CalcAverageVelocity);
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
			const float Discomfort = NeighborCell->Discomfort;
			if(SpeedField != 0)
			{
				Cell->CostField[DirectionIndex] = (SimParameters.PathCostConstant * SpeedField + SimParameters.TimeCostConstant + SimParameters.DiscomfortConstant * Discomfort) / SpeedField;
			}
			else
			{
				Cell->CostField[DirectionIndex] = MAX_COST;
			}
		}
	};
	Field.ForEachCellPerform(CalculateCost);
}

void TCSimulator::UpdatePotentialGradient(const int GroupID)
{
	const auto Operation = [this, GroupID](FTCCell* Cell, const FVector2f& Coords) -> void
	{
		for(const EDirectionIndex DirectionIndex : CARDINAL_DIRECTIONS)
		{
			if(const FTCCell* Neighbor = Field.GetDataAt(Coords, DIRECTION_OFFSETS[DirectionIndex]))
			{
				const float Gradient = Neighbor->Potential[GroupID] - Cell->Potential[GroupID];
				Cell->PotentialGradient[GroupID][DirectionIndex] = Gradient;
			}
		}
	};
	
	Field.ForEachCellPerform(Operation);
}

void TCSimulator::UpdateDesiredVelocityField(const int GroupID)
{
	const auto CalculateDesiredVelocity = [this, GroupID](FTCCell* Cell, const FVector2f& Coords) -> void
	{
		float RootSquareSum = 0; 
		for (int Direction = 0; Direction < NUM_DIRECTIONS; ++Direction)
		{
			RootSquareSum += FMath::Square(Cell->PotentialGradient[GroupID][Direction]);
		}
		RootSquareSum = FMath::Sqrt(RootSquareSum);
		
		Cell->DesiredVelocity[GroupID] = {0, 0};
		for (int Direction = 0; Direction < NUM_DIRECTIONS; ++Direction)
		{
			const float NormPotential = Cell->PotentialGradient[GroupID][Direction] / RootSquareSum;
			Cell->PotentialGradient[GroupID][Direction] = NormPotential;
			Cell->DesiredVelocity[GroupID] += -Cell->SpeedField[Direction] * NormPotential * DIRECTION_OFFSETS[Direction];
		}
	};
	Field.ForEachCellPerform(CalculateDesiredVelocity);
}

FTCCheapestNeighbor TCSimulator::GetCheapestNeighbor(const FVector2f& Coords, const EDirectionIndex First, const EDirectionIndex Second, int GroupID)
{
	FTCCell* CurrentCell = Field.GetDataAt(Coords);
	FTCCell* FirstNeighbor = Field.GetDataAt(Coords, DIRECTION_OFFSETS[First]);
	FTCCell* SecondNeighbor = Field.GetDataAt(Coords, DIRECTION_OFFSETS[Second]);

	if(FirstNeighbor && !SecondNeighbor)
	{
		return {FirstNeighbor->Potential[GroupID], CurrentCell->CostField[First]};
	}
	else if(!FirstNeighbor && SecondNeighbor)
	{
		return {SecondNeighbor->Potential[GroupID], CurrentCell->CostField[Second]};
	}
	else if(!FirstNeighbor && !SecondNeighbor)
	{
		return {MAX_COST, MAX_COST};
	}

	const FTCCheapestNeighbor ResultFirst = {FirstNeighbor->Potential[GroupID], CurrentCell->CostField[First]};
	const FTCCheapestNeighbor ResultSecond = {SecondNeighbor->Potential[GroupID], CurrentCell->CostField[Second]};
		
	return ResultFirst.Sum() < ResultSecond.Sum() ? ResultFirst : ResultSecond;
}

float TCSimulator::GetFiniteDifferenceApproximation(const FVector2f& Coords, const int GroupID)
{
	const auto [PhiX, Cx] = GetCheapestNeighbor(Coords, EAST, WEST, GroupID);
	const auto [PhiY, Cy] = GetCheapestNeighbor(Coords, NORTH, SOUTH, GroupID);
	
	if(PhiX == MAX_COST && PhiY < MAX_COST)
	{
		return Cy + PhiY;
	}

	if(PhiY == MAX_COST && PhiX < MAX_COST)
	{
		return Cx + PhiX;
	}
	
	if (PhiX == MAX_COST && PhiY == MAX_COST)
	{
		return MAX_COST;
	}
	
	const float QuadraticCoeffA = Cy + Cx;
	const float QuadraticCoeffB = -2 * ((PhiX * Cy) + (PhiY * Cx));
	const float QuadraticCoeffC = (FMath::Square(PhiX) * Cy) + (FMath::Square(PhiY) * Cx) - (Cx * Cy);

	const float TermUnderSqrt = FMath::Square(QuadraticCoeffB) - (4 * QuadraticCoeffA * QuadraticCoeffC);
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

float TCSimulator::GetSocialForceInfluence(const FVector2f& DesiredDirection, const FVector2f& Force)
{
	if (FVector2f::DotProduct(DesiredDirection, Force) >= Force.Length() * FMath::Cos(HALF_FOV))
	{
		return 1.0f;
	}
	
	return WEAK_INF;
}
