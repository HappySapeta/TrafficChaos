#include "Simulator.h"

constexpr float GAUSSIAN_FALLOFF = 1.0;
constexpr float GAUSSIAN_SCALE = 1.0f;

void TCSimulator::Initialize(const float Resolution, const float WorldSize)
{
	CostField.Initialize(Resolution, WorldSize, 0);
}

void TCSimulator::AddEntity(const FVector2f& InitialPosition, const FVector2f InitialVelocity, const float InitialHeading)
{
	Entities.Add(InitialPosition, InitialVelocity, InitialHeading);
}

void TCSimulator::UpdateCostField()
{
	const auto ResetCost = [](float* Cell, const FVector2f& Coords) -> void
	{
		*Cell = 0;
	};
	CostField.ForEachCellPerform(ResetCost);
	
	for (int EntityIndex = 0; EntityIndex < Entities.Num(); ++EntityIndex)
	{
		const FVector2f GridCoords = CostField.WorldToGridSnapped(Entities.Positions[EntityIndex]);
		
		const auto ApplyDensity = [this, GridCoords](const FVector2f Offset, const float Distance)
		{
			float* CellCost = CostField.GetDataAt(GridCoords, Offset);
			if (CellCost)
			{
				*CellCost = FMath::Min(1.0f, *CellCost + GaussianDistribution(Distance));
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
		Entities.Positions[EntityIndex] += FVector2f(Radius * FMath::Cos(Theta), Radius * FMath::Sin(Theta)); 
	}
}

const FRpSpatialData<float>& TCSimulator::GetCostField() const
{
	return CostField;
}

void TCSimulator::Update(const float DeltaSeconds)
{
	UpdateCostField();
	Debug_MoveEntities(DeltaSeconds);
}

void TCSimulator::Debug_Draw(const UWorld* World, const float DeltaSeconds)
{
	for (int EntityIndex = 0; EntityIndex < Entities.Num(); ++EntityIndex)
	{
		const FVector Center = {Entities.Positions[EntityIndex].X, Entities.Positions[EntityIndex].Y, 0.0f};
		DrawDebugSphere(World, Center, 10.0f, 8, FColor::Green);
	}
	
	const float DebugBoxExtent = CostField.GetCellSize();
	const auto DrawCost = [this, World, DebugBoxExtent](float* Cost, const FVector2f& Coords)
	{
		const FVector2f WorldCoords = CostField.GridToWorld(Coords);
		const FLinearColor DebugColor = FLinearColor::LerpUsingHSV(FLinearColor{1.0f,1.0f,1.0f,0.1f}, FLinearColor{1.0f,0.0f,0.0f,0.5f}, *Cost);
		
		const FVector BoxMin = {WorldCoords.X, WorldCoords.Y, 0};
		const FVector BoxMax = {WorldCoords.X + DebugBoxExtent, WorldCoords.Y + DebugBoxExtent, DebugBoxExtent};
		DrawDebugSolidBox(World, FBox(BoxMin, BoxMax), DebugColor.ToFColor(false));
	};
	
	CostField.ForEachCellPerform(DrawCost);
}
