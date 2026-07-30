// Copyright Anupam Sahu. All Rights Reserved.

#include "FastContinuumCrowdSimulator.h"
#include "Math.h"
#include "Kismet/KismetMathLibrary.h"

constexpr float MAX_COST = TNumericLimits<float>::Max();

void TCFastContinuumCrowdSimulator::Initialize
(
	const float NewWorldSpan, const int NewResolution, const int NewNumGroups, 
	const TInstancedStruct<FTCSimulationParameters>
	Parameters, const FTCSocialForceParameters& SocialForceParameters
)
{
	ImplicitGrid.Initialize(FFloatRange(0, NewWorldSpan), NewResolution);
	Field.Initialize(NewResolution, NewWorldSpan, {});

	const auto InitializeCell = [NewNumGroups](FTCFastCell* Cell, const FVector2f& Coords)
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
	SetSimulationParameters(Parameters);
	SetAdvectionParameters(SocialForceParameters);
}

void TCFastContinuumCrowdSimulator::RegisterGoal(const int GroupID, const FVector2f& WorldLocation)
{
	Goals.Add({GroupID, Field.WorldToGrid(WorldLocation)});
}

void TCFastContinuumCrowdSimulator::RegisterWall(const FVector2f& WallCoords)
{
	if (FTCFastCell* Cell = Field.GetDataAt(Field.WorldToGrid(WallCoords)))
	{
		for (int GroupID = 0; GroupID < NumGroups; ++GroupID)
		{
			Cell->Potential[GroupID] = MAX_COST;
			Cell->bIsWall = true;
		}
	}
}

void TCFastContinuumCrowdSimulator::RegisterDiscomfort(const FVector2f& WallCoords, const float Amount)
{
	if (FTCFastCell* Cell = Field.GetDataAt(Field.WorldToGrid(WallCoords)))
	{
		for (int GroupID = 0; GroupID < NumGroups; ++GroupID)
		{
			Cell->Potential[GroupID] = MAX_COST;
			Cell->Discomfort = Amount;
		}
	}
}

void TCFastContinuumCrowdSimulator::MoveEntites(TArray<FTCEntity>& Entities, const float TimeStep)
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

void TCFastContinuumCrowdSimulator::UpdateSimulation(const TArray<FTCEntity>& Entities, const float DeltaSeconds)
{
	UpdateDensityAndVelocityField(Entities);
	for (int GroupID = 0; GroupID < NumGroups; ++GroupID)
	{
		UpdateCostField(GroupID);
		UpdatePotentialField(GroupID);
		UpdatePotentialGradient(GroupID);
		UpdateDesiredVelocityField(GroupID);
	}
}

void TCFastContinuumCrowdSimulator::UpdateDensityAndVelocityField(const TArray<FTCEntity>& Entities)
{
	const auto ResetCellDensityAndVelocties = [](FTCFastCell* Cell, const FVector2f& Coords) -> void
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

		if (FTCFastCell* Cell = Field.GetDataAt(Field.WorldToGrid(EntityPosition)))
		{
			Cell->ByteDensity = Cell->ByteDensity + 1 == 0 ? TNumericLimits<uint8>::Max() : Cell->ByteDensity + 1;

			Cell->Direction = ConvertVectorToDirectionIndex
			(
				DIRECTION_OFFSETS[Cell->Direction].GetSafeNormal() + EntityVelocity.GetSafeNormal()
			);
		}
	}
	
	const auto CalcAverageVelocity = [this](FTCFastCell* Cell, const FVector2f& Coords) -> void
	{
		if (Cell->ByteDensity != 0)
		{
			const FVector2f& AvgDirection = DIRECTION_OFFSETS[Cell->Direction] / Cell->ByteDensity;
			Cell->Direction = ConvertVectorToDirectionIndex(AvgDirection);
		}
	};
	Field.ForEachCellPerform(CalcAverageVelocity);
}

void TCFastContinuumCrowdSimulator::UpdateCostField(const int GroupID)
{
	const auto CalculateCost = [this, GroupID](FTCFastCell* CurrentCell, const FVector2f& Coords)
	{
		for (int DirectionIndex = 0; DirectionIndex < ANISOTROPY; ++DirectionIndex)
		{
			float TotalCost = 0;
			const FTCFastCell* NeighborCell = Field.GetDataAt(CurrentCell->Coords, DIRECTION_OFFSETS[DirectionIndex]);
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

void TCFastContinuumCrowdSimulator::UpdatePotentialField(const int GroupID)
{
	check(Goals.Contains(GroupID));

	const FVector2f GoalCoords = Goals[GroupID];
	checkf(Field.IsValidGridCoordinate(GoalCoords), TEXT("Invalid coordinates for goal."));

	// 1. Clear all lists. 
	Knowns.Empty();
	Candidates.Empty();

	// 1. Get the goal cell.
	FTCFastCell* GoalCell = Field.GetDataAt(GoalCoords);
	GoalCell->Potential[GroupID] = 0;
	GoalCell->bIsWall = false;
	Candidates.PushFirst(GoalCell);

	// 2. Initialize potentials
	const auto InitializePotential = [GoalCoords, GroupID](FTCFastCell* Cell, const FVector2f& Coords)
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
		FTCFastCell* Current = Candidates.First();
		Candidates.PopFirst();

		if (Knowns.Contains(Current))
		{
			continue;
		}

		const TArray<FTCNeighbor<FTCFastCell>> Neighbors = GetNeighbors(Current->Coords);
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

void TCFastContinuumCrowdSimulator::UpdatePotentialGradient(const int GroupID)
{
	const auto Operation = [this, GroupID](FTCFastCell* Cell, const FVector2f& Coords) -> void
	{
		for (int DirectionIndex = 0; DirectionIndex < ANISOTROPY; ++DirectionIndex)
		{
			if (const FTCFastCell* Neighbor = Field.GetDataAt(Coords, DIRECTION_OFFSETS[DirectionIndex]))
			{
				const float Gradient = Cell->Potential[GroupID] - Neighbor->Potential[GroupID];
				Cell->PotentialGradient[GroupID][DirectionIndex] = Gradient;
			}
		}
	};

	Field.ForEachCellPerform(Operation);
}

void TCFastContinuumCrowdSimulator::UpdateDesiredVelocityField(const int GroupID)
{
	const auto CalculateDesiredVelocity = [this, GroupID](FTCFastCell* Cell, const FVector2f& Coords) -> void
	{
		float MaxPotential = TNumericLimits<float>::Min();
		float MinPotential = TNumericLimits<float>::Max();
		for (int DirectionIndex = 0; DirectionIndex < ANISOTROPY; ++DirectionIndex)
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
		for (int DirectionIndex = 0; DirectionIndex < ANISOTROPY; ++DirectionIndex)
		{
			const float PotentialGradient = Cell->PotentialGradient[GroupID][DirectionIndex];
			const float NormPotential = UKismetMathLibrary::NormalizeToRange(PotentialGradient, MinPotential, MaxPotential);

			DirectionVector += NormPotential * DIRECTION_OFFSETS[DirectionIndex];
		}

		Cell->DesiredVelocity[GroupID] = (DirectionVector / ANISOTROPY).GetSafeNormal() * PedParameters.DesiredSpeed;
	};
	Field.ForEachCellPerform(CalculateDesiredVelocity);
}

float TCFastContinuumCrowdSimulator::GetFiniteDifferenceApproximation(const FVector2f& Coords, const int GroupID)
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

FTCCheapestNeighbor TCFastContinuumCrowdSimulator::GetCheapestNeighbor(const FVector2f& Coords, const EDirectionIndex First, const EDirectionIndex Second, const int GroupID)
{
	FTCFastCell* CurrentCell = Field.GetDataAt(Coords);
	FTCFastCell* FirstNeighbor = Field.GetDataAt(Coords, DIRECTION_OFFSETS[First]);
	FTCFastCell* SecondNeighbor = Field.GetDataAt(Coords, DIRECTION_OFFSETS[Second]);

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

TArray<FTCNeighbor<FTCFastCell>> TCFastContinuumCrowdSimulator::GetNeighbors(const FVector2f& Coords)
{
	TArray<FTCNeighbor<FTCFastCell>> Neighbors;

	for (int DirectionIndex = 0; DirectionIndex < ANISOTROPY; ++DirectionIndex)
	{
		if (FTCFastCell* Node = Field.GetDataAt(Coords, DIRECTION_OFFSETS[DirectionIndex]))
		{
			Neighbors.Push({Node, static_cast<EDirectionIndex>(DirectionIndex)});
		}
	}

	return Neighbors;
}

float TCFastContinuumCrowdSimulator::GetSocialForceInfluence(const FVector2f& DesiredDirection, const FVector2f& Force)
{
	if (FVector2f::DotProduct(DesiredDirection, Force) >= Force.Length() * FMath::Cos(FMath::DegreesToRadians(PedParameters.HalfFOV)))
	{
		return 1.0f;
	}

	return PedParameters.WeakInfluence;
}

EDirectionIndex TCFastContinuumCrowdSimulator::ConvertVectorToDirectionIndex(const FVector2f& Vector) const
{
	uint8 Result = 0;
	float MaxDotProduct = -1.0f;
	const FVector2f& InputDirection = Vector.GetSafeNormal();

	for (int DirectionIndex = 0; DirectionIndex < NUM_OFFSETS; ++DirectionIndex)
	{
		const float DotProduct = FVector2f::DotProduct(DIRECTION_OFFSETS[DirectionIndex].GetSafeNormal(), InputDirection);
		if (DotProduct > MaxDotProduct)
		{
			MaxDotProduct = DotProduct;
			Result = DirectionIndex;
		}
	}

	return static_cast<EDirectionIndex>(Result);
}
