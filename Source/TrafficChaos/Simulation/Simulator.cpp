// Copyright Anupam Sahu. All Rights Reserved.

#include "Simulator.h"
#include "Math.h"
#include "Kismet/KismetMathLibrary.h"

constexpr float MAX_COST = TNumericLimits<float>::Max();

void TCSimulator::RegisterGoal(const int GroupID, const FVector2f& Goal)
{
	Goals.Add({GroupID, Goal});
}

void TCSimulator::RegisterWall(const FVector2f& WallCoords)
{
	if (FTCCell* Cell = Field.GetDataAt(Field.WorldToGrid(WallCoords)))
	{
		for (int GroupID = 0; GroupID < NumGroups; ++GroupID)
		{
			Cell->Potential[GroupID] = MAX_COST;
			Cell->bIsWall = true;
		}
	}
}

void TCSimulator::RegisterDiscomfort(const FVector2f& WallCoords, const float Amount)
{
	if (FTCCell* Cell = Field.GetDataAt(Field.WorldToGrid(WallCoords)))
	{
		for (int GroupID = 0; GroupID < NumGroups; ++GroupID)
		{
			Cell->Potential[GroupID] = MAX_COST;
			Cell->Discomfort = Amount;
		}
	}
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
		if (Entities[EntityIndex].bUseOverrideVelocity)
		{
			Entities[EntityIndex].Position += Entities[EntityIndex].OverrideVelocity * TimeStep;
			continue;
		}

		FVector2f Force = FVector2f::ZeroVector;

		const FVector2f& CurrentVelocity = Entities[EntityIndex].Velocity;
		const FVector2f& CurrentPosition = Entities[EntityIndex].Position;

		const FVector2f GridLocation = Field.WorldToGrid(CurrentPosition);
		const FVector2f DesiredVelocity = Field.GetDataAt(GridLocation)->DesiredVelocity[Entities[EntityIndex].GroupID];
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
			const FVector2f AvoidanceForce = FTCSocialForces::GetAvoidanceForce(CurrentPosition, OtherPosition, OtherVelocity, PedParameters.AvoidanceTimestep, PedParameters);
			Force += GetSocialForceInfluence(DesiredDirection, -AvoidanceForce) * AvoidanceForce;
		}

		FVector2f NewVelocity = Entities[EntityIndex].Velocity + Force * TimeStep;

		// Limit Speed
		NewVelocity = NewVelocity.GetSafeNormal() * FMath::Min(PedParameters.DesiredSpeed, NewVelocity.Length());

		if (PedParameters.bEnableTurningLimit)
		{
			// Limit Angle
			const float Angle = FMath::ClampAngle
			(
				FRpMath::GetSignedAngleDegrees(DesiredVelocity, NewVelocity), -PedParameters.MaxTurnAngle, PedParameters.MaxTurnAngle
			);
			const FVector2f NewDirection = DesiredDirection.GetRotated(Angle);
			NewVelocity = NewDirection * NewVelocity.Length();
		}

		Entities[EntityIndex].Velocity = NewVelocity;
		Entities[EntityIndex].Position += NewVelocity * TimeStep;
	}
}

float TCSimulator::GetFiniteDifferenceApproximation(const FVector2f& Coords, const int GroupID)
{
	const auto [PhiX, Cx] = GetCheapestNeighbor(Coords, EAST, WEST, GroupID);
	const auto [PhiY, Cy] = GetCheapestNeighbor(Coords, NORTH, SOUTH, GroupID);

	if (PhiX == MAX_COST && PhiY < MAX_COST)
	{
		return Cy + PhiY;
	}

	if (PhiY == MAX_COST && PhiX < MAX_COST)
	{
		return Cx + PhiX;
	}

	if (PhiY == MAX_COST && PhiX == MAX_COST)
	{
		return MAX_COST;
	}

	const float QuadraticCoeffA = FMath::Square(Cy) + FMath::Square(Cx);
	const float QuadraticCoeffB = -2 * ((PhiX * FMath::Square(Cy)) + (PhiY * FMath::Square(Cx)));
	const float QuadraticCoeffC =
		(FMath::Square(PhiX) * FMath::Square(Cy)) +
		(FMath::Square(PhiY) * FMath::Square(Cx)) -
		(FMath::Square(Cx) * FMath::Square(Cy));

	const float TermUnderSqrt = FMath::Square(QuadraticCoeffB) - (4 * QuadraticCoeffA * QuadraticCoeffC);
	if (TermUnderSqrt >= 0.0f)
	{
		const float FirstSolution = (-QuadraticCoeffB + FMath::Sqrt(TermUnderSqrt)) / (2 * QuadraticCoeffA);
		const float SecondSolution = (-QuadraticCoeffB - FMath::Sqrt(TermUnderSqrt)) / (2 * QuadraticCoeffA);

		const float ResultPotential = FMath::Max(FirstSolution, SecondSolution);

		if (ResultPotential > PhiX && ResultPotential > PhiY)
		{
			return ResultPotential;
		}
	}

	return FMath::Min(PhiX + Cx, PhiY + Cy);
}

FTCCheapestNeighbor TCSimulator::GetCheapestNeighbor
(
	const FVector2f& Coords,
	const EDirectionIndex First,
	const EDirectionIndex Second,
	const int GroupID
)
{
	FTCCell* CurrentCell = Field.GetDataAt(Coords);
	FTCCell* FirstNeighbor = Field.GetDataAt(Coords, DIRECTION_OFFSETS[First]);
	FTCCell* SecondNeighbor = Field.GetDataAt(Coords, DIRECTION_OFFSETS[Second]);

	if (FirstNeighbor && !SecondNeighbor)
	{
		return {FirstNeighbor->Potential[GroupID], CurrentCell->CostField[GroupID][First]};
	}
	else if (!FirstNeighbor && SecondNeighbor)
	{
		return {SecondNeighbor->Potential[GroupID], CurrentCell->CostField[GroupID][Second]};
	}
	else if (!FirstNeighbor && !SecondNeighbor)
	{
		return {MAX_COST, MAX_COST};
	}

	const FTCCheapestNeighbor ResultFirst = {FirstNeighbor->Potential[GroupID], CurrentCell->CostField[GroupID][First]};
	const FTCCheapestNeighbor ResultSecond = {SecondNeighbor->Potential[GroupID], CurrentCell->CostField[GroupID][Second]};

	return ResultFirst.Sum() < ResultSecond.Sum() ? ResultFirst : ResultSecond;
}

TArray<FTCNeighbor> TCSimulator::GetNeighbors(const FVector2f& Coords)
{
	TArray<FTCNeighbor> Neighbors;

	for (int DirectionIndex = 0; DirectionIndex < ANISOTROPY; ++DirectionIndex)
	{
		if (FTCCell* Node = Field.GetDataAt(Coords, DIRECTION_OFFSETS[DirectionIndex]))
		{
			Neighbors.Push({Node, static_cast<EDirectionIndex>(DirectionIndex)});
		}
	}

	return Neighbors;
}

float TCSimulator::GetSocialForceInfluence(const FVector2f& DesiredDirection, const FVector2f& Force)
{
	if (FVector2f::DotProduct(DesiredDirection, Force) >= Force.Length() * FMath::Cos(FMath::DegreesToRadians(PedParameters.HalfFOV)))
	{
		return 1.0f;
	}

	return PedParameters.WeakInfluence;
}

#ifndef USE_BASELINE_MODEL
void TCSimulator::Initialize(const float Resolution, const float WorldSize, const int NewNumGroups)
{
	ImplicitGrid.Initialize(FFloatRange(0, WorldSize), Resolution);
	Field.Initialize(Resolution, WorldSize, {});

	const auto InitializeCell = [NewNumGroups](FTCCell* Cell, const FVector2f& Coords)
	{
		Cell->Coords = Coords;
		Cell->ByteDensity = 0;
		Cell->Direction = EDirectionIndex::NONE;
		Cell->Discomfort = 0;
		Cell->Potential.Init(0, NewNumGroups);
		Cell->DesiredVelocity.Init({0, 0}, NewNumGroups);
		Cell->CostField.Init({}, NewNumGroups);
		Cell->PotentialGradient.Init({}, NewNumGroups);
	};
	Field.ForEachCellPerform(InitializeCell);

	NumGroups = NewNumGroups;
}

void TCSimulator::Update(const TArray<FTCEntity>& Entities, const float DeltaSeconds)
{
	UpdateDensityAndVelocityField(Entities);
	for (int GroupID = 0; GroupID < NumGroups; ++GroupID)
	{
		UpdateCostField(GroupID);
		UpdatePotentialField_BFS(GroupID);
		UpdatePotentialGradient(GroupID);
		UpdateDesiredVelocityField(GroupID);
	}
}

void TCSimulator::UpdatePotentialField_BFS(const int GroupID)
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
	GoalCell->bIsWall = false;
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

		if (Knowns.Contains(Current))
		{
			continue;
		}

		const TArray<FTCNeighbor> Neighbors = GetNeighbors(Current->Coords);
		for (const auto& [Neighbor, NeighborDirection] : Neighbors)
		{
			if (Neighbor->bIsWall)
			{
				continue;
			}
			
			const float NewPotential = GetFiniteDifferenceApproximation(Neighbor->Coords, GroupID);
			if (NewPotential < Neighbor->Potential[GroupID])
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
		Cell->ByteDensity = 0;
		Cell->Direction = EDirectionIndex::NONE;
	};
	Field.ForEachCellPerform(ResetCellDensityAndVelocties);

	for (const FTCEntity& Entity : Entities)
	{
		const FVector2f& EntityPosition = Entity.Position;
		const FVector2f& EntityVelocity = Entity.Velocity;
		if (!Field.IsValidWorldPosition(EntityPosition))
		{
			continue;
		}

		if (FTCCell* Cell = Field.GetDataAt(Field.WorldToGrid(EntityPosition)))
		{
			Cell->ByteDensity = Cell->ByteDensity + 1 == 0 ? TNumericLimits<uint8>::Max() : Cell->ByteDensity + 1;

			Cell->Direction = ConvertVectorToDirectionIndex
			(
				DIRECTION_OFFSETS[Cell->Direction].GetSafeNormal() + EntityVelocity.GetSafeNormal()
			);
		}
	}
	
	const auto CalcAverageVelocity = [this](FTCCell* Cell, const FVector2f& Coords) -> void
	{
		if (Cell->ByteDensity != 0)
		{
			const FVector2f& AvgDirection = DIRECTION_OFFSETS[Cell->Direction] / Cell->ByteDensity;
			Cell->Direction = ConvertVectorToDirectionIndex(AvgDirection);
		}
	};
	Field.ForEachCellPerform(CalcAverageVelocity);
}

void TCSimulator::UpdateCostField(const int GroupID)
{
	const auto CalculateCost = [this, GroupID](FTCCell* CurrentCell, const FVector2f& Coords)
	{
		for (int DirectionIndex = 0; DirectionIndex < NUM_DIRECTIONS; ++DirectionIndex)
		{
			float TotalCost = 0;
			const FTCCell* NeighborCell = Field.GetDataAt(CurrentCell->Coords, DIRECTION_OFFSETS[DirectionIndex]);
			if (!NeighborCell)
			{
				continue;
			}
			
			// Density Cost
			{
				const float MaxDensity = FMath::Square(Field.GetCellSize()) / (PI * FMath::Square(PedParameters.AvoidanceRadius * 0.5f));
				const float NormDensity = FMath::Pow(FMath::Min(NeighborCell->ByteDensity / MaxDensity, 1), SimParameters.DensityExponent);
				const float DensityCost = NormDensity * SimParameters.DensityConstant;

				TotalCost += DensityCost;
			}
			
			// Velocity Cost
			{
				const FVector2f NeighborVelocity = DIRECTION_OFFSETS[NeighborCell->Direction];
				const float DotProduct = -FVector2f::DotProduct(DIRECTION_OFFSETS[DirectionIndex].GetSafeNormal(), NeighborVelocity.GetSafeNormal());
				const float ClampedDotProduct = FMath::Max(DotProduct, 0.1f);
				const float NormDotProduct = UKismetMathLibrary::NormalizeToRange(ClampedDotProduct, 0, 1);
				const float VelocityCost = NormDotProduct * SimParameters.TimeCostConstant;
				
				TotalCost += VelocityCost;
			}
			
			// Distance Cost
			{
				const float NormDistance = DIRECTION_OFFSETS[DirectionIndex].Length() / FMath::Sqrt(2.0f);
				const float DistanceCost = NormDistance * SimParameters.PathCostConstant;
				
				TotalCost += DistanceCost;
			}
			
			// Discomfort Cost
			{
				TotalCost += NeighborCell->Discomfort * SimParameters.DiscomfortConstant;
			}
			
			TotalCost /= 4;
			CurrentCell->CostField[GroupID][DirectionIndex] = TotalCost;
		}
	}; 
	Field.ForEachCellPerform(CalculateCost);
}

void TCSimulator::UpdatePotentialGradient(const int GroupID)
{
	const auto Operation = [this, GroupID](FTCCell* Cell, const FVector2f& Coords) -> void
	{
		for (int DirectionIndex = 0; DirectionIndex < ANISOTROPY; ++DirectionIndex)
		{
			if (const FTCCell* Neighbor = Field.GetDataAt(Coords, DIRECTION_OFFSETS[DirectionIndex]))
			{
				const float Gradient = Cell->Potential[GroupID] - Neighbor->Potential[GroupID];
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
		float MaxPotential = TNumericLimits<float>::Min();
		float MinPotential = TNumericLimits<float>::Max();
		for (int DirectionIndex = 0; DirectionIndex < NUM_DIRECTIONS; ++DirectionIndex)
		{
			const float PotentialGradient = Cell->PotentialGradient[GroupID][DirectionIndex];
			if (PotentialGradient > MaxPotential)
			{
				MaxPotential = PotentialGradient;
			}
			if (PotentialGradient < MinPotential)
			{
				MinPotential = PotentialGradient;
			}
		}

		Cell->DesiredVelocity[GroupID] = {0, 0};
		FVector2f DirectionVector = FVector2f::ZeroVector;
		for (int DirectionIndex = 0; DirectionIndex < NUM_DIRECTIONS; ++DirectionIndex)
		{
			const float PotentialGradient = Cell->PotentialGradient[GroupID][DirectionIndex];
			const float NormPotential = UKismetMathLibrary::NormalizeToRange(PotentialGradient, MinPotential, MaxPotential);

			DirectionVector += NormPotential * DIRECTION_OFFSETS[DirectionIndex];
		}

		Cell->DesiredVelocity[GroupID] = (DirectionVector / NUM_DIRECTIONS).GetSafeNormal() * PedParameters.DesiredSpeed;
	};
	Field.ForEachCellPerform(CalculateDesiredVelocity);
}

EDirectionIndex TCSimulator::ConvertVectorToDirectionIndex(FVector2f Vector) const
{
	uint8 Result = 0;
	Vector = -Vector.GetSafeNormal();
	float MinDotProduct = 1.0f;

	for (int DirectionIndex = 0; DirectionIndex < ANISOTROPY; ++DirectionIndex)
	{
		const float DotProduct = FVector2f::DotProduct(DIRECTION_OFFSETS[DirectionIndex].GetSafeNormal(), Vector);
		if (DotProduct < MinDotProduct)
		{
			MinDotProduct = DotProduct;
			Result = DirectionIndex;
		}
	}

	return static_cast<EDirectionIndex>(Result);
}
#endif
#ifdef USE_BASELINE_MODEL
void TCSimulator::Initialize(const float Resolution, const float WorldSize, const int NewNumGroups)
{
	ImplicitGrid.Initialize(FFloatRange(0, WorldSize), Resolution);
	Field.Initialize(Resolution, WorldSize, {});

	const auto InitializeCell = [NewNumGroups](FTCCell* Cell, const FVector2f& Coords)
	{
		Cell->Coords = Coords;
		Cell->Density = 0;
		Cell->Velocity = {0, 0};
		Cell->Discomfort = 0;
		Cell->Potential.Init(0, NewNumGroups);
		Cell->DesiredVelocity.Init({0, 0}, NewNumGroups);
		Cell->CostField.Init({}, NewNumGroups);
		Cell->PotentialGradient.Init({}, NewNumGroups);
	};
	Field.ForEachCellPerform(InitializeCell);

	NumGroups = NewNumGroups;
}

void TCSimulator::Update(const TArray<FTCEntity>& Entities, const float DeltaSeconds)
{
	UpdateDensityAndVelocityField(Entities);
	UpdateSpeedField();
	
	for (int GroupID = 0; GroupID < NumGroups; ++GroupID)
	{
		UpdateCostField(GroupID);
		UpdatePotentialField_FM(GroupID);
		UpdatePotentialGradient(GroupID);
		UpdateDesiredVelocityField(GroupID);
	}
}

void TCSimulator::UpdatePotentialField_FM(const int GroupID)
{
	const auto LowestPotentialOnTop = [GroupID](const FTCCell& A, const FTCCell& B) -> bool
	{
		return A.Potential[GroupID] < B.Potential[GroupID];
	};
	
	Knowns.Empty();
	Candidates.Empty();
	
	FTCCell* GoalCell = Field.GetDataAt(Field.WorldToGrid(Goals[GroupID]));
	GoalCell->Potential[GroupID] = 0;
	Knowns.Add(GoalCell);
	
	const auto InitializeCell = [GoalCell, GroupID, this](FTCCell* Cell, const FVector2f& Coords)
	{
		if (Cell != GoalCell)
		{
			Cell->Potential[GroupID] = MAX_COST;
		}
	};
	Field.ForEachCellPerform(InitializeCell);
	
	for (auto& [Neighbor, Direction] : GetNeighbors(GoalCell->Coords))
	{
		Neighbor->Potential[GroupID] = GetFiniteDifferenceApproximation(Neighbor->Coords, GroupID);
		Candidates.HeapPush(Neighbor, LowestPotentialOnTop);
	}
	
	while (!Candidates.IsEmpty())
	{
		FTCCell* Cell;
		Candidates.HeapPop(Cell, LowestPotentialOnTop);
		Knowns.Add(Cell);
		
		for (auto& [Neighbor, Direction] : GetNeighbors(Cell->Coords))
		{
			if (Knowns.Contains(Neighbor) || Neighbor->bIsWall)
			{
				continue;
			}
			
			const float NewPotential = GetFiniteDifferenceApproximation(Neighbor->Coords, GroupID);
			if (NewPotential < Neighbor->Potential[GroupID])
			{
				Neighbor->Potential[GroupID] = NewPotential;
				Candidates.HeapPush(Neighbor, LowestPotentialOnTop);
			}
		}
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

	for (const FTCEntity& Entity : Entities)
	{
		const FVector2f& EntityPosition = Entity.Position;
		const FVector2f& EntityVelocity = Entity.Velocity;
		if (!Field.IsValidWorldPosition(EntityPosition))
		{
			continue;
		}
		
		const FVector2f EntityPreciseCoords = Field.WorldToGridCentered(EntityPosition);
		const FVector2f ClosestCellCenterCoords = {FMath::RoundToInt(EntityPreciseCoords.X) - 0.5f, FMath::RoundToInt(EntityPreciseCoords.Y) - 0.5f};

		const FVector2f Delta = EntityPreciseCoords - ClosestCellCenterCoords;

		if (FTCCell* SouthEastCell = Field.GetDataAt(ClosestCellCenterCoords, D_SOUTH_EAST)) // C
		{
			const float DensityContribution = FMath::Pow(std::min(Delta.X, Delta.Y), SimParameters.DensityExponent);
			SouthEastCell->Density += std::max(DensityContribution, static_cast<float>(SimParameters.DensityRange.GetLowerBoundValue()));
			SouthEastCell->Velocity += DensityContribution * EntityVelocity;
		}

		if (FTCCell* EastCell = Field.GetDataAt(ClosestCellCenterCoords, D_EAST)) // D
		{
			const float DensityContribution = FMath::Pow(std::min(Delta.X, 1 - Delta.Y), SimParameters.DensityExponent);
			EastCell->Density += std::min(DensityContribution, static_cast<float>(SimParameters.DensityRange.GetLowerBoundValue()));
			EastCell->Velocity += DensityContribution * EntityVelocity;
		}

		if (FTCCell* ClosestCell = Field.GetDataAt(ClosestCellCenterCoords)) // A
		{
			const float DensityContribution = FMath::Pow(std::min(1 - Delta.X, 1 - Delta.Y), SimParameters.DensityExponent);
			ClosestCell->Density += std::min(DensityContribution, static_cast<float>(SimParameters.DensityRange.GetLowerBoundValue()));
			ClosestCell->Velocity += DensityContribution * EntityVelocity;
		}

		if (FTCCell* SouthCell = Field.GetDataAt(ClosestCellCenterCoords, D_SOUTH)) // B
		{
			const float DensityContribution = FMath::Pow(std::min(1 - Delta.X, Delta.Y), SimParameters.DensityExponent);
			SouthCell->Density += std::min(DensityContribution, static_cast<float>(SimParameters.DensityRange.GetLowerBoundValue()));
			SouthCell->Velocity += DensityContribution * EntityVelocity;
		}
	}
	
	const auto CalcAverageVelocity = [this](FTCCell* Cell, const FVector2f& Coords) -> void
	{
		if (Cell->Density != 0)
		{
			Cell->Velocity /= Cell->Density;
		}
	};
	Field.ForEachCellPerform(CalcAverageVelocity);
}

void TCSimulator::UpdateCostField(const int GroupID)
{
	const auto CalculateCost = [this, GroupID](FTCCell* Cell, const FVector2f& Coords)
	{
		for(int DirectionIndex = 0; DirectionIndex < NUM_DIRECTIONS; ++DirectionIndex)
		{
			const FTCCell* NeighborCell = Field.GetDataAt(Coords, DIRECTION_OFFSETS[DirectionIndex]);
			if(!NeighborCell)
			{
				Cell->CostField[GroupID][DirectionIndex] = MAX_COST;
				continue;
			}
			
			const float SpeedField = Cell->SpeedField[DirectionIndex];
			const float Discomfort = NeighborCell->Discomfort;
			if(SpeedField != 0)
			{
				Cell->CostField[GroupID][DirectionIndex] = 
					((DIRECTION_OFFSETS[DirectionIndex].Length() * SimParameters.PathCostConstant * SpeedField) + 
					(SimParameters.TimeCostConstant) + 
					(SimParameters.DiscomfortConstant * Discomfort)) / SpeedField;
			}
			else
			{
				Cell->CostField[GroupID][DirectionIndex] = MAX_COST;
			}
		}
	};
	Field.ForEachCellPerform(CalculateCost);
}

void TCSimulator::UpdateSpeedField()
{
	const auto CalculateSpeedField = [this](FTCCell* Cell, const FVector2f& Coords)
	{
		for(int DirectionIndex = 0; DirectionIndex < NUM_DIRECTIONS; ++DirectionIndex)
		{
			const FVector2f& Direction = DIRECTION_OFFSETS[DirectionIndex];
			
			float TempFlowSpeed = 0.1f;
			if(const FTCCell* NeighborCell = Field.GetDataAt(Coords, Direction * static_cast<float>(SimParameters.VelocityLookahead)))
			{
				TempFlowSpeed = FMath::Max(FVector2f::DotProduct(NeighborCell->Velocity, Direction.GetSafeNormal()), 0.1f);
			}
			const float FlowSpeed = TempFlowSpeed;

			uint8 TempNeighborDensity = SimParameters.DensityRange.GetLowerBoundValue();
			if(FTCCell* NeighborCell = Field.GetDataAt(Coords, Direction * static_cast<float>(SimParameters.DensityLookahead)))
			{
				TempNeighborDensity = NeighborCell->Density;
			}
			const uint8 NeighborDensity = TempNeighborDensity;
			
			if(NeighborDensity <= SimParameters.DensityRange.GetLowerBoundValue())
			{
				Cell->SpeedField[DirectionIndex] = PedParameters.DesiredSpeed;
			}
			else if(NeighborDensity >= SimParameters.DensityRange.GetUpperBoundValue())
			{
				Cell->SpeedField[DirectionIndex] = FlowSpeed;
			}
			else
			{
				Cell->SpeedField[DirectionIndex] = PedParameters.DesiredSpeed + ((NeighborDensity - SimParameters.DensityRange.GetLowerBoundValue())/(SimParameters.DensityRange.GetUpperBoundValue() - SimParameters.DensityRange.GetLowerBoundValue())) * (FlowSpeed - PedParameters.DesiredSpeed);	
			}
		}
	};
	Field.ForEachCellPerform(CalculateSpeedField);
}

void TCSimulator::UpdatePotentialGradient(const int GroupID)
{
	const auto Operation = [this, GroupID](FTCCell* Cell, const FVector2f& Coords) -> void
	{
		for (int DirectionIndex = 0; DirectionIndex < NUM_DIRECTIONS; ++DirectionIndex)
		{
			if (const FTCCell* Neighbor = Field.GetDataAt(Coords, DIRECTION_OFFSETS[DirectionIndex]))
			{
				const float Gradient = Cell->Potential[GroupID] - Neighbor->Potential[GroupID];
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
		float MaxPotential = TNumericLimits<float>::Min();
		float MinPotential = TNumericLimits<float>::Max();
		for (int DirectionIndex = 0; DirectionIndex < NUM_DIRECTIONS; ++DirectionIndex)
		{
			const float PotentialGradient = Cell->PotentialGradient[GroupID][DirectionIndex];
			if (PotentialGradient > MaxPotential)
			{
				MaxPotential = PotentialGradient;
			}
			if (PotentialGradient < MinPotential)
			{
				MinPotential = PotentialGradient;
			}
		}

		Cell->DesiredVelocity[GroupID] = {0, 0};
		FVector2f DirectionVector = FVector2f::ZeroVector;
		for (int DirectionIndex = 0; DirectionIndex < NUM_DIRECTIONS; ++DirectionIndex)
		{
			const float PotentialGradient = Cell->PotentialGradient[GroupID][DirectionIndex];
			const float NormPotential = UKismetMathLibrary::NormalizeToRange(PotentialGradient, MinPotential, MaxPotential);

			DirectionVector += NormPotential * DIRECTION_OFFSETS[DirectionIndex];
		}

		Cell->DesiredVelocity[GroupID] = (DirectionVector / NUM_DIRECTIONS).GetSafeNormal() * PedParameters.DesiredSpeed;
	};
	Field.ForEachCellPerform(CalculateDesiredVelocity);
}
#endif