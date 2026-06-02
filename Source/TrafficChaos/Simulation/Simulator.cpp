#include "Simulator.h"

constexpr float GAUSSIAN_FALLOFF = 1.0;
constexpr float GAUSSIAN_SCALE = 1.0f;

void TCSimulator::Initialize(const float Resolution, const float WorldSize)
{
	DensityField.Initialize(Resolution, WorldSize, 0);
	VelocityField.Initialize(Resolution, WorldSize, FVector2f::ZeroVector);
}

void TCSimulator::AddEntity(const FVector2f& InitialPosition, const FVector2f InitialVelocity, const float InitialHeading)
{
	Entities.Add(InitialPosition, InitialVelocity, InitialHeading);
}

void TCSimulator::UpdateDensityField()
{
	const auto ResetDensities = [](float* Cell, const FVector2f& Coords) -> void
	{
		*Cell = 0;
	};
	DensityField.ForEachCellPerform(ResetDensities);
	
	for (int EntityIndex = 0; EntityIndex < Entities.Num(); ++EntityIndex)
	{
		const FVector2f GridCoords = DensityField.WorldToGridSnapped(Entities.Positions[EntityIndex]);
		
		const auto ApplyDensity = [this, GridCoords](const FVector2f Offset, const float Distance)
		{
			float* CellDensity = DensityField.GetDataAt(GridCoords, Offset);
			if (CellDensity)
			{
				*CellDensity = FMath::Min(1.0f, *CellDensity + GaussianDistribution(Distance));
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
	const auto ResetVelocities = [](FVector2f* Velocity, const FVector2f& Coords) -> void
	{
		*Velocity = FVector2f::ZeroVector;
	};
	
	VelocityField.ForEachCellPerform(ResetVelocities);
	
	for (int EntityIndex = 0; EntityIndex < Entities.Num(); ++EntityIndex)
	{
		const FVector2f& Velocity = Entities.Velocities[EntityIndex];
		const FVector2f& GridLocation = VelocityField.WorldToGridSnapped(Entities.Positions[EntityIndex]);
		
		if (FVector2f* CellVelocity = VelocityField.GetDataAt(GridLocation))
		{
			*CellVelocity += Velocity;
		}
	}
	
	const auto AverageVelocities = [this](FVector2f* Cell, const FVector2f& Coords)
	{
		checkf(DensityField.IsValidGridCoordinate(Coords), TEXT("Invalid grid coordinates."));
		
		const float Density = *DensityField.GetDataAt(Coords);
		*VelocityField.GetDataAt(Coords) /= FMath::Max(Density, 1.0f);
	};
	VelocityField.ForEachCellPerform(AverageVelocities);
}

float TCSimulator::GaussianDistribution(const float Distance)
{
	return FMath::Exp(-1 * FMath::Square(Distance) / GAUSSIAN_FALLOFF) * GAUSSIAN_SCALE;
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

const FRpSpatialData<float>& TCSimulator::GetDensityField() const
{
	return DensityField;
}

void TCSimulator::Update(const float DeltaSeconds)
{
	Debug_MoveEntities(DeltaSeconds);
	UpdateDensityField();
	UpdateVelocityField();
}

void TCSimulator::Debug_Draw(const UWorld* World, const float DeltaSeconds)
{
	// Debug DensityField.
	{
		for (int EntityIndex = 0; EntityIndex < Entities.Num(); ++EntityIndex)
		{
			const FVector Center = {Entities.Positions[EntityIndex].X, Entities.Positions[EntityIndex].Y, 0.0f};
			DrawDebugSphere(World, Center, 10.0f, 8, FColor::Green);
		}
	
		const float DebugBoxExtent = DensityField.GetCellSize();
		const auto DrawDensities = [this, World, DebugBoxExtent](float* Density, const FVector2f& Coords)
		{
			const FVector2f WorldCoords = DensityField.GridToWorld(Coords);
			const FLinearColor DebugColor = FLinearColor::LerpUsingHSV(FLinearColor{1.0f, 1.0f, 1.0f, 0.1f},
																	   FLinearColor{1.0f, 0.0f, 0.0f, 0.5f}, 
																	   *Density);
		
			const FVector BoxMin = {WorldCoords.X, WorldCoords.Y, 0};
			const FVector BoxMax = {WorldCoords.X + DebugBoxExtent, WorldCoords.Y + DebugBoxExtent, DebugBoxExtent};
			DrawDebugSolidBox(World, FBox(BoxMin, BoxMax), DebugColor.ToFColor(false));
		};
	
		DensityField.ForEachCellPerform(DrawDensities);
	}
	
	// Debug VelocityField.
	{
		const float CellSize = VelocityField.GetCellSize();
		const auto DrawVelocties = [this, World, CellSize](FVector2f* Velocity, const FVector2f& Coords) -> void
		{
			const FVector2f WorldLocation = VelocityField.GridToWorld(Coords);
			const FVector2f Direction = Velocity->IsNearlyZero() ? FVector2f{1.0f, 0.0f} : Velocity->GetSafeNormal();
			const FColor DebugColor = FColor::Yellow;
			DrawDebugCone
			(
				World, 
				{WorldLocation.X + (CellSize / 2), WorldLocation.Y + (CellSize / 2), 0}, 
				{-Direction.X, -Direction.Y, 0.0f}, 
				50.0f, PI/6, PI/6, 12, DebugColor
			);
		};
		VelocityField.ForEachCellPerform(DrawVelocties);
	}
}
