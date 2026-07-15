// Copyright Anupam Sahu. All Rights Reserved.

#include "Simulator.h"
#include "Math.h"
#include "Kismet/KismetMathLibrary.h"

constexpr float MAX_COST = TNumericLimits<float>::Max();
constexpr float HALF_FOV = 100.0f;
constexpr float WEAK_INF = 0.5f;
constexpr float AVOIDANCE_TIMESTEP = 2.0f;
constexpr float GAUSSIAN_DENSITY_MIN = 0.2f; 

void TCSimulator::Initialize(const float Resolution, const float WorldSize, const int NewNumGroups)
{
	ImplicitGrid.Initialize(FFloatRange(0, WorldSize), Resolution);
	Field.Initialize(Resolution, WorldSize, {});

	const auto InitializeCell = [NewNumGroups](FTCCell* Cell, const FVector2f& Coords)
	{
		Cell->Coords = Coords;

#ifdef USE_DENSITY_OPTIMISATION
		Cell->ByteDensity = 0;
#else
		Cell->Density = 0;
#endif

#ifdef USE_VELOCITY_OPTIMISATION
		Cell->Direction = EDirectionIndex::NONE;
#else
		Cell->Velocity = {0, 0};
#endif

		Cell->Discomfort = 0;

		Cell->Potential.Init(0, NewNumGroups);
		Cell->DesiredVelocity.Init({0, 0}, NewNumGroups);
		Cell->CostField.Init({}, NewNumGroups);
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
		if (Entities[EntityIndex].bUseOverrideVelocity)
		{
			Entities[EntityIndex].Position += Entities[EntityIndex].OverrideVelocity * TimeStep;
			continue;
		}

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
			const FVector2f AvoidanceForce = FTCSocialForces::GetAvoidanceForce(CurrentPosition, OtherPosition, OtherVelocity, AVOIDANCE_TIMESTEP, PedParameters);
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

void TCSimulator::Update(const TArray<FTCEntity>& Entities, const float DeltaSeconds)
{
	UpdateDensityAndVelocityField(Entities);

	for (int GroupID = 0; GroupID < NumGroups; ++GroupID)
	{
		UpdateCostField(GroupID);
#ifdef USE_SOLVER_OPTIMISATION
		UpdatePotentialField_BFS(GroupID);
#else
		UpdatePotentialField_FM(GroupID);
#endif
		UpdatePotentialGradient(GroupID);
		UpdateDesiredVelocityField(GroupID);
	}
}

#ifdef USE_SOLVER_OPTIMISATION
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
#endif

#ifndef USE_SOLVER_OPTIMISATION
void TCSimulator::UpdatePotentialField_FM(const int GroupID)
{
	check(Goals.Contains(GroupID));

	const FVector2f GoalCoords = Field.WorldToGrid(Goals[GroupID]);
	checkf(Field.IsValidGridCoordinate(GoalCoords), TEXT("Invalid coordinates for goal."));

	FTCCell* GoalCell = Field.GetDataAt(GoalCoords);

	const auto LowestPotentialOnTop = [GroupID](const FTCCell& Left, const FTCCell& Right) -> bool
	{
		return Left.Potential[GroupID] < Right.Potential[GroupID];
	};

	Knowns.Empty();
	Unknowns.Empty();
	Candidates.Empty();

	const auto InitiallizePotentials = [this, GoalCell, GroupID](FTCCell* Cell, const FVector2f& Coords)-> void
	{
		if (Cell != GoalCell)
		{
			Cell->Potential[GroupID] = MAX_COST;
			Unknowns.Push(Cell);
		}
		else
		{
			Cell->Potential[GroupID] = 0.0f;
			Knowns.Push(Cell);
		}
	};
	Field.ForEachCellPerform(InitiallizePotentials);

	for (int Index = 0; Index < Unknowns.Num(); ++Index)
	{
		FTCCell* Cell = Unknowns[Index];

		const float CurrentPotential = Cell->Potential[GroupID];
		const float NewPotential = GetFiniteDifferenceApproximation(Cell->Coords, GroupID);
		if (NewPotential < CurrentPotential)
		{
			Cell->Potential[GroupID] = NewPotential;
			CandidatesHeap.HeapPush(Cell, LowestPotentialOnTop);
			Unknowns.RemoveSwap(Cell);
		}
	}

	while (!Candidates.IsEmpty())
	{
		FTCCell* Cell;
		CandidatesHeap.HeapPop(Cell, LowestPotentialOnTop);

		Knowns.Push(Cell);

		for (auto& [Neighbor, NeighborDirection] : GetNeighbors(Cell->Coords))
		{
			if (Knowns.Contains(Neighbor))
			{
				continue;
			}

			float& CurrentPotential = Neighbor->Potential[GroupID];
			const float NewPotential = GetFiniteDifferenceApproximation(Neighbor->Coords, GroupID);
			if (NewPotential < CurrentPotential)
			{
				CurrentPotential = NewPotential;
				CandidatesHeap.HeapPush(Neighbor, LowestPotentialOnTop);
			}
		}
	}
}
#endif

void TCSimulator::UpdateDensityAndVelocityField(const TArray<FTCEntity>& Entities)
{
	const auto ResetCellDensityAndVelocties = [](FTCCell* Cell, const FVector2f& Coords) -> void
	{
#ifdef USE_DENSITY_OPTIMISATION
		Cell->ByteDensity = 0;
#else
		Cell->Density = 0;
#endif
#ifdef USE_VELOCITY_OPTIMISATION
		Cell->Direction = EDirectionIndex::NONE;
#else
		Cell->Velocity = {0, 0};
#endif
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

#ifdef USE_DENSITY_OPTIMISATION
		{
			if (FTCCell* Cell = Field.GetDataAt(Field.WorldToGrid(EntityPosition)))
			{
				Cell->ByteDensity = Cell->ByteDensity + 1 == 0 ? TNumericLimits<uint8>::Max() : Cell->ByteDensity + 1;

#ifdef USE_VELOCITY_OPTIMISATION
				Cell->Direction = ConvertVectorToDirectionIndex
				(
					DIRECTION_OFFSETS[Cell->Direction].GetSafeNormal() + EntityVelocity.GetSafeNormal()
				);
#else
				Cell->Velocity += EntityVelocity;
#endif
			}
		}
#else
		const FVector2f EntityPreciseCoords = Field.WorldToGridCentered(EntityPosition);
		const FVector2f ClosestCellCenterCoords = {
			FMath::RoundToInt(EntityPreciseCoords.X) - 0.5f, FMath::RoundToInt(EntityPreciseCoords.Y) - 0.5f
		};

		const FVector2f Delta = EntityPreciseCoords - ClosestCellCenterCoords;

		if (FTCCell* SouthEastCell = Field.GetDataAt(ClosestCellCenterCoords, D_SOUTH_EAST)) // C
		{
			const float DensityContribution = FMath::Pow(std::min(Delta.X, Delta.Y), 1);
			SouthEastCell->Density += std::max(DensityContribution, static_cast<float>(GAUSSIAN_DENSITY_MIN));
			SouthEastCell->Velocity += DensityContribution * EntityVelocity;
		}

		if (FTCCell* EastCell = Field.GetDataAt(ClosestCellCenterCoords, D_EAST)) // D
		{
			const float DensityContribution = FMath::Pow(std::min(Delta.X, 1 - Delta.Y), 1);
			EastCell->Density += std::min(DensityContribution, static_cast<float>(GAUSSIAN_DENSITY_MIN));
			EastCell->Velocity += DensityContribution * EntityVelocity;
		}

		if (FTCCell* ClosestCell = Field.GetDataAt(ClosestCellCenterCoords)) // A
		{
			const float DensityContribution = FMath::Pow(std::min(1 - Delta.X, 1 - Delta.Y), 1);
			ClosestCell->Density += std::min(DensityContribution, static_cast<float>(GAUSSIAN_DENSITY_MIN));
			ClosestCell->Velocity += DensityContribution * EntityVelocity;
		}

		if (FTCCell* SouthCell = Field.GetDataAt(ClosestCellCenterCoords, D_SOUTH)) // B
		{
			const float DensityContribution = FMath::Pow(std::min(1 - Delta.X, Delta.Y), 1);
			SouthCell->Density += std::min(DensityContribution, static_cast<float>(GAUSSIAN_DENSITY_MIN));
			SouthCell->Velocity += DensityContribution * EntityVelocity;
		}
#endif

		const auto CalcAverageVelocity = [this](FTCCell* Cell, const FVector2f& Coords) -> void
		{
#ifdef USE_DENSITY_OPTIMISATION
			if (Cell->ByteDensity != 0)
			{

#ifdef USE_VELOCITY_OPTIMISATION
				const FVector2f& AvgDirection = DIRECTION_OFFSETS[Cell->Direction] / Cell->ByteDensity;
				Cell->Direction = ConvertVectorToDirectionIndex(AvgDirection);
#else
				Cell->Velocity /= Cell->ByteDensity;
#endif
			}
#else
			if (Cell->Density != 0)
			{
#ifdef USE_VELOCITY_OPTIMISATION
				Cell->Direction = ConvertVectorToDirectionIndex(DIRECTION_OFFSETS[Cell->Direction].GetSafeNormal() / Cell->Density);
#else
				Cell->Velocity /= Cell->Density;
			}
#endif
#endif
		};
		Field.ForEachCellPerform(CalcAverageVelocity);
	}
}

void TCSimulator::UpdateCostField(const int GroupID)
{
	const auto CalculateCost = [this, GroupID](FTCCell* CurrentCell, const FVector2f& Coords)
	{
		for (int DirectionIndex = 0; DirectionIndex < SimParameters.Anisotropy; ++DirectionIndex)
		{
			CurrentCell->CostField[GroupID][DirectionIndex] = 0;
			if (const FTCCell* NeighborCell = Field.GetDataAt(CurrentCell->Coords, DIRECTION_OFFSETS[DirectionIndex]))
			{
				FVector2f NeighborVelocity = FVector2f::ZeroVector;
#ifdef USE_VELOCITY_OPTIMISATION
				NeighborVelocity = DIRECTION_OFFSETS[NeighborCell->Direction];
#else
				NeighborVelocity = NeighborCell->Velocity;
#endif

#ifdef USE_DENSITY_OPTIMISATION
				const float NeighborDensity = NeighborCell->ByteDensity;
#else
				const float NeighborDensity = NeighborCell->Density;
#endif

				const float DensityCost = NeighborDensity * SimParameters.DensityConstant;
				const float VelocityCost = FMath::Max(
					-FVector2f::DotProduct(DIRECTION_OFFSETS[DirectionIndex], NeighborVelocity.GetSafeNormal()),
					0.1f) * SimParameters.TimeCostConstant;
				const float DistanceCost = DIRECTION_OFFSETS[DirectionIndex].Length() * SimParameters.PathCostConstant;

				CurrentCell->CostField[GroupID][DirectionIndex] = DensityCost + VelocityCost + DistanceCost;
			}
		}
	};
	Field.ForEachCellPerform(CalculateCost);
}

void TCSimulator::UpdatePotentialGradient(const int GroupID)
{
	const auto Operation = [this, GroupID](FTCCell* Cell, const FVector2f& Coords) -> void
	{
		for (int DirectionIndex = 0; DirectionIndex < SimParameters.Anisotropy; ++DirectionIndex)
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
		for (int DirectionIndex = 0; DirectionIndex < SimParameters.Anisotropy; ++DirectionIndex)
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
		for (int DirectionIndex = 0; DirectionIndex < SimParameters.Anisotropy; ++DirectionIndex)
		{
			const float PotentialGradient = Cell->PotentialGradient[GroupID][DirectionIndex];
			const float NormPotential = UKismetMathLibrary::NormalizeToRange(PotentialGradient, MinPotential, MaxPotential);

			DirectionVector += NormPotential * DIRECTION_OFFSETS[DirectionIndex];
		}

		Cell->DesiredVelocity[GroupID] = (DirectionVector / SimParameters.Anisotropy).GetSafeNormal() * PedParameters.
			DesiredSpeed;
	};
	Field.ForEachCellPerform(CalculateDesiredVelocity);
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

TArray<FTCNeighbor> TCSimulator::GetNeighbors(const FVector2f& Coords)
{
	TArray<FTCNeighbor> Neighbors;

	for (int DirectionIndex = 0; DirectionIndex < SimParameters.Anisotropy; ++DirectionIndex)
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
	if (FVector2f::DotProduct(DesiredDirection, Force) >= Force.Length() * FMath::Cos(FMath::DegreesToRadians(HALF_FOV)))
	{
		return 1.0f;
	}

	return WEAK_INF;
}

EDirectionIndex TCSimulator::ConvertVectorToDirectionIndex(FVector2f Vector) const
{
	uint8 Result = 0;
	Vector = -Vector.GetSafeNormal();
	float MinDotProduct = 1.0f;

	for (int DirectionIndex = 0; DirectionIndex < SimParameters.Anisotropy; ++DirectionIndex)
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
