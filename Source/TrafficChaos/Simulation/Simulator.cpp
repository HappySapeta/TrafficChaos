#include "Simulator.h"

constexpr float GAUSSIAN_FALLOFF = 1.0;
constexpr float GAUSSIAN_SCALE = 1.0f;

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

float TCSimulator::GaussianDistribution(const float Distance)
{
	return FMath::Exp(-1 * FMath::Square(Distance) / GAUSSIAN_FALLOFF) * GAUSSIAN_SCALE;
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
	
		const float DebugBoxExtent = Field.GetCellSize();
		const auto DrawDensities = [this, World, DebugBoxExtent](FTCCell* Cell, const FVector2f& Coords)
		{
			const FVector2f WorldCoords = Field.GridToWorld(Coords);
			const FLinearColor DebugColor = FLinearColor::LerpUsingHSV(FLinearColor{1.0f, 1.0f, 1.0f, 0.1f},
																	   FLinearColor{1.0f, 0.0f, 0.0f, 0.5f}, 
																	   Cell->Density);
		
			const FVector BoxMin = {WorldCoords.X, WorldCoords.Y, 0};
			const FVector BoxMax = {WorldCoords.X + DebugBoxExtent, WorldCoords.Y + DebugBoxExtent, DebugBoxExtent};
			DrawDebugSolidBox(World, FBox(BoxMin, BoxMax), DebugColor.ToFColor(false));
		};
	
		Field.ForEachCellPerform(DrawDensities);
	}
	
	// Debug VelocityField.
	{
		const float CellSize = Field.GetCellSize();
		const auto DrawVelocties = [this, World, CellSize](FTCCell* Cell, const FVector2f& Coords) -> void
		{
			const FVector2f WorldLocation = Field.GridToWorld(Coords);
			const FVector2f Direction = Cell->Velocity.IsNearlyZero() ? FVector2f{1.0f, 0.0f} : Cell->Velocity.GetSafeNormal();
			const FColor DebugColor = FColor::Yellow;
			DrawDebugCone
			(
				World, 
				{WorldLocation.X + (CellSize / 2), WorldLocation.Y + (CellSize / 2), 0}, 
				{-Direction.X, -Direction.Y, 0.0f}, 
				50.0f, PI/6, PI/6, 12, DebugColor
			);
		};
		Field.ForEachCellPerform(DrawVelocties);
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
