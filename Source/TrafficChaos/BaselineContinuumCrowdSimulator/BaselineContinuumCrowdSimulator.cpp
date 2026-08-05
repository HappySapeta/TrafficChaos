// Copyright Anupam Sahu. All Rights Reserved.

#include "BaselineContinuumCrowdSimulator.h"
#include "Math.h"
#include "Kismet/KismetMathLibrary.h"

constexpr float MAX_COST = TNumericLimits<float>::Max();

void TCBaselineContinuumCrowdSimulator::RegisterGoal(const int GroupID, const FVector2f& Goal)
{
	Goals.Add({GroupID, Goal});
}

void TCBaselineContinuumCrowdSimulator::RegisterWall(const FVector2f& WallCoords)
{
	if (FTCBaselineCell* Cell = Field.GetDataAt(Field.WorldToGrid(WallCoords)))
	{
		for (int GroupID = 0; GroupID < NumGroups; ++GroupID)
		{
			Cell->Potential[GroupID] = MAX_COST;
			Cell->bIsWall = true;
		}
	}
}

void TCBaselineContinuumCrowdSimulator::RegisterDiscomfort(const FVector2f& WallCoords, const float Amount)
{
	if (FTCBaselineCell* Cell = Field.GetDataAt(Field.WorldToGrid(WallCoords)))
	{
		for (int GroupID = 0; GroupID < NumGroups; ++GroupID)
		{
			Cell->Potential[GroupID] = MAX_COST;
			Cell->Discomfort = Amount;
		}
	}
}

void TCBaselineContinuumCrowdSimulator::MoveEntites(TArray<FTCEntity>& Entities, const float TimeStep)
{
	EntityPositions.Reset(Entities.Num());
	for (const FTCEntity& Entity : Entities)
	{
		EntityPositions.Push({Entity.Position.X, Entity.Position.Y, 0.0f});
	}
	ImplicitGrid.Update(EntityPositions);

	for (int EntityIndex = 0; EntityIndex < Entities.Num(); ++EntityIndex)
	{
#ifdef ENABLE_VELOCITY_OVERRIDING
		if (Entities[EntityIndex].bUseOverrideVelocity)
		{
			Entities[EntityIndex].Position += Entities[EntityIndex].OverrideVelocity * TimeStep;
			continue;
		}
#endif

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

float TCBaselineContinuumCrowdSimulator::GetFiniteDifferenceApproximation(const FVector2f& Coords, const int GroupID)
{
	const auto [PhiX, Cx] = GetCheapestNeighbor(Coords, EAST, WEST, GroupID);
	const auto [PhiY, Cy] = GetCheapestNeighbor(Coords, NORTH, SOUTH, GroupID);

	if (PhiY == MAX_COST && PhiX == MAX_COST)
	{
		return MAX_COST;
	}
	
	if (PhiX == MAX_COST && PhiY < MAX_COST)
	{
		return FMath::Max(FMath::Sqrt(Cy) + PhiY, -FMath::Sqrt(Cy) + PhiY);
	}

	if (PhiY == MAX_COST && PhiX < MAX_COST)
	{
		return FMath::Max(FMath::Sqrt(Cx) + PhiX, -FMath::Sqrt(Cx) + PhiX);
	}

	const float QuadraticCoeffA = Cy + Cx;
	const float QuadraticCoeffB = -2 * ((PhiX * Cy) + (PhiY * Cx));
	const float QuadraticCoeffC = (FMath::Square(PhiX) * Cy) + (FMath::Square(PhiY) * Cx) - (Cx * Cy);

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

	return ((PhiX + Cx) + (PhiY + Cy)) / 2.0f;
}

FTCCheapestNeighbor TCBaselineContinuumCrowdSimulator::GetCheapestNeighbor
(
	const FVector2f& Coords,
	const EDirectionIndex First,
	const EDirectionIndex Second,
	const int GroupID
)
{
	FTCBaselineCell* CurrentCell = Field.GetDataAt(Coords);
	FTCBaselineCell* FirstNeighbor = Field.GetDataAt(Coords, DIRECTION_OFFSETS[First]);
	FTCBaselineCell* SecondNeighbor = Field.GetDataAt(Coords, DIRECTION_OFFSETS[Second]);

	check(CurrentCell)
	
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

TArray<FTCNeighbor<FTCBaselineCell>> TCBaselineContinuumCrowdSimulator::GetNeighbors(const FVector2f& Coords)
{
	TArray<FTCNeighbor<FTCBaselineCell>> Neighbors;

	for (int DirectionIndex = 0; DirectionIndex < ANISOTROPY; ++DirectionIndex)
	{
		if (FTCBaselineCell* Node = Field.GetDataAt(Coords, DIRECTION_OFFSETS[DirectionIndex]))
		{
			Neighbors.Push({Node, static_cast<EDirectionIndex>(DirectionIndex)});
		}
	}

	return Neighbors;
}

float TCBaselineContinuumCrowdSimulator::GetSocialForceInfluence(const FVector2f& DesiredDirection, const FVector2f& Force)
{
	if (FVector2f::DotProduct(DesiredDirection, Force) >= Force.Length() * FMath::Cos(FMath::DegreesToRadians(PedParameters.HalfFOV)))
	{
		return 1.0f;
	}

	return PedParameters.WeakInfluence;
}

void TCBaselineContinuumCrowdSimulator::Initialize
(
	const float NewWorldSpan, 
	const int NewResolution, 
	const int NewNumGroups, 
	const TInstancedStruct<FTCSimulationParameters> Parameters, 
	const FTCSocialForceParameters& SocialForceParameters
)
{
	ImplicitGrid.Initialize(FFloatRange(0, NewWorldSpan), NewResolution);
	Field.Initialize(NewResolution, NewWorldSpan, {});

	const auto InitializeCell = [NewNumGroups](FTCBaselineCell* Cell, const FVector2f& Coords)
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
	
	Knowns.Reserve(Field.GetNum());
	Candidates.Reserve(Field.GetNum());
	
	NumGroups = NewNumGroups;
	SetSimulationParameters(Parameters);
	SetAdvectionParameters(SocialForceParameters);
}

void TCBaselineContinuumCrowdSimulator::UpdateSimulation(const TArray<FTCEntity>& Entities, const float DeltaSeconds)
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

void TCBaselineContinuumCrowdSimulator::UpdatePotentialField_FM(const int GroupID)
{
	const auto LowestPotentialOnTop = [GroupID](const FTCBaselineCell& A, const FTCBaselineCell& B) -> bool
	{
		return A.Potential[GroupID] < B.Potential[GroupID];
	};
	
	Knowns.Reset();
	Candidates.Reset();
	
	FTCBaselineCell* GoalCell = Field.GetDataAt(Field.WorldToGrid(Goals[GroupID]));
	GoalCell->Potential[GroupID] = 0;
	Knowns.Add(GoalCell);
	
	const auto InitializeCell = [GoalCell, GroupID, this](FTCBaselineCell* Cell, const FVector2f& Coords)
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
		FTCBaselineCell* Cell;
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

void TCBaselineContinuumCrowdSimulator::UpdateDensityAndVelocityField(const TArray<FTCEntity>& Entities)
{
	const auto ResetCellDensityAndVelocties = [](FTCBaselineCell* Cell, const FVector2f& Coords) -> void
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
		if (FTCBaselineCell* SouthEastCell = Field.GetDataAt(ClosestCellCenterCoords, D_SOUTH_EAST)) // C
		{
			const float DensityContribution = FMath::Pow(std::min(Delta.X, Delta.Y), SimParameters.DensityExponent);
			const float MaxDensityContribution = 1/FMath::Pow(2, SimParameters.DensityExponent); 
			SouthEastCell->Density += std::min(DensityContribution, MaxDensityContribution);
			SouthEastCell->Velocity += DensityContribution * EntityVelocity;
		}

		if (FTCBaselineCell* EastCell = Field.GetDataAt(ClosestCellCenterCoords, D_EAST)) // D
		{
			const float DensityContribution = FMath::Pow(std::min(Delta.X, 1 - Delta.Y), SimParameters.DensityExponent);
			const float MaxDensityContribution = 1/FMath::Pow(2, SimParameters.DensityExponent);
			EastCell->Density += std::min(DensityContribution, MaxDensityContribution);
			EastCell->Velocity += DensityContribution * EntityVelocity;
		}

		if (FTCBaselineCell* CurrentCell = Field.GetDataAt(ClosestCellCenterCoords)) // A
		{
			const float DensityContribution = FMath::Pow(std::min(1 - Delta.X, 1 - Delta.Y), SimParameters.DensityExponent);
			const float MinDensityContribution = 1/FMath::Pow(2, SimParameters.DensityExponent);
			CurrentCell->Density += std::max(DensityContribution, MinDensityContribution);
			CurrentCell->Velocity += DensityContribution * EntityVelocity;
		}

		if (FTCBaselineCell* SouthCell = Field.GetDataAt(ClosestCellCenterCoords, D_SOUTH)) // B
		{
			const float DensityContribution = FMath::Pow(std::min(1 - Delta.X, Delta.Y), SimParameters.DensityExponent);
			const float MaxDensityContribution = 1/FMath::Pow(2, SimParameters.DensityExponent);
			SouthCell->Density += std::min(DensityContribution, MaxDensityContribution);
			SouthCell->Velocity += DensityContribution * EntityVelocity;
		}
	}
	
	const auto CalcAverageVelocity = [this](FTCBaselineCell* Cell, const FVector2f& Coords) -> void
	{
		if (Cell->Density != 0)
		{
			Cell->Velocity /= Cell->Density;
		}
	};
	Field.ForEachCellPerform(CalcAverageVelocity);
}

void TCBaselineContinuumCrowdSimulator::UpdateCostField(const int GroupID)
{
	const auto CalculateCost = [this, GroupID](FTCBaselineCell* Cell, const FVector2f& Coords)
	{
		for(int DirectionIndex = 0; DirectionIndex < ANISOTROPY; ++DirectionIndex)
		{
			const FTCBaselineCell* NeighborCell = Field.GetDataAt(Coords, DIRECTION_OFFSETS[DirectionIndex]);
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

void TCBaselineContinuumCrowdSimulator::UpdateSpeedField()
{
	const auto CalculateSpeedField = [this](FTCBaselineCell* Cell, const FVector2f& Coords)
	{
		for(int DirectionIndex = 0; DirectionIndex < ANISOTROPY; ++DirectionIndex)
		{
			const FVector2f& Direction = DIRECTION_OFFSETS[DirectionIndex];
			
			float FlowSpeed = FVector2f::DotProduct(Cell->Velocity, Direction.GetSafeNormal());
			if (SimParameters.VelocityLookahead >= 1)
			{
				if(const FTCBaselineCell* NeighborCell = Field.GetDataAt(Coords, Direction * static_cast<float>(SimParameters.VelocityLookahead)))
				{
					FlowSpeed = FVector2f::DotProduct(NeighborCell->Velocity, Direction.GetSafeNormal());
				}
			}
			FlowSpeed = FMath::Max(FlowSpeed, 0.1);

			float Density = Cell->Density;
			if (SimParameters.DensityLookahead >= 1)
			{
				if(FTCBaselineCell* NeighborCell = Field.GetDataAt(Coords, Direction * static_cast<float>(SimParameters.DensityLookahead)))
				{
					Density = NeighborCell->Density;
				}	
			}
			
			if(Density <= SimParameters.DensityRange.GetLowerBoundValue())
			{
				Cell->SpeedField[DirectionIndex] = SimParameters.SpeedRange.GetUpperBoundValue();
			}
			else if(Density >= SimParameters.DensityRange.GetUpperBoundValue())
			{
				Cell->SpeedField[DirectionIndex] = FlowSpeed;
			}
			else
			{
				Cell->SpeedField[DirectionIndex] = PedParameters.DesiredSpeed + ((Density - SimParameters.DensityRange.GetLowerBoundValue())/(SimParameters.DensityRange.GetUpperBoundValue() - SimParameters.DensityRange.GetLowerBoundValue())) * (FlowSpeed - PedParameters.DesiredSpeed);	
			}
		}
	};
	Field.ForEachCellPerform(CalculateSpeedField);
}

void TCBaselineContinuumCrowdSimulator::UpdatePotentialGradient(const int GroupID)
{
	const auto Operation = [this, GroupID](FTCBaselineCell* Cell, const FVector2f& Coords) -> void
	{
		for (int DirectionIndex = 0; DirectionIndex < ANISOTROPY; ++DirectionIndex)
		{
			if (const FTCBaselineCell* Neighbor = Field.GetDataAt(Coords, DIRECTION_OFFSETS[DirectionIndex]))
			{
				const float Gradient = Cell->Potential[GroupID] - Neighbor->Potential[GroupID];
				Cell->PotentialGradient[GroupID][DirectionIndex] = Gradient;
			}
		}
	};

	Field.ForEachCellPerform(Operation);
}

void TCBaselineContinuumCrowdSimulator::UpdateDesiredVelocityField(const int GroupID)
{
	const auto CalculateDesiredVelocity = [this, GroupID](FTCBaselineCell* Cell, const FVector2f& Coords) -> void
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