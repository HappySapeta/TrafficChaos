// Copyright Anupam Sahu. All Rights Reserved.

#pragma once
#include <CoreMinimal.h>
#include <SocialForceModel.generated.h>

USTRUCT()
struct FTCSocialForceParameters
{
	GENERATED_BODY()
	
	UPROPERTY(EditAnywhere)
	float DesiredSpeed = 1.0f;
	 
	UPROPERTY(EditAnywhere)
	float RelaxationTime = 0.1f;
	
	UPROPERTY(EditAnywhere)
	float WeakInfluence = 0.5f;
	
	UPROPERTY(EditAnywhere)
	float AvoidanceRadius = 1.0f;
	
	UPROPERTY(EditAnywhere)
	float AvoidanceStrength = 1.0f;
	
	UPROPERTY(EditAnywhere)
	float AvoidanceTimestep = 2.0f;
	
	UPROPERTY(EditAnywhere)
	bool bEnableTurningLimit = false;
	
	UPROPERTY(EditAnywhere, meta = (EditCondition = "bEnableTurningLimit", EditConditionHides = true))
	float MaxTurnAngle = 90.0f;
	
	UPROPERTY(EditAnywhere)
	float HalfFOV = 100.0f;
};

class FTCSocialForces
{
public:

	static FVector2f GetDrivingForce
	(
		const FVector2f& CurrentVelocity, 
		const FVector2f& DesiredDirection,
		const FTCSocialForceParameters& Parameters = {}
	)
	{
		return (Parameters.DesiredSpeed * DesiredDirection - CurrentVelocity) / Parameters.RelaxationTime; 
	}

	static FVector2f GetAvoidanceForce
	(
		const FVector2f& CurrentPosition, 
		const FVector2f& OtherPosition,
		const FVector2f& OtherVelocity, 
		const float DeltaTime,
		const FTCSocialForceParameters& Parameters = {}
	)
	{
		if (FVector2f::Distance(CurrentPosition, OtherPosition) > Parameters.AvoidanceRadius)
		{
			return FVector2f::ZeroVector;
		}
		
		const auto PotentialFunction = [Parameters](const float X) -> float
		{
			return FMath::Exp(-X / Parameters.AvoidanceRadius);
		};
		
		const auto GetSemiMinorAxis = [DeltaTime, OtherVelocity](const FVector2f& VecToOther) -> float
		{
			const float OrderTerm = OtherVelocity.Length() * DeltaTime;
			const float SquareRootTerm = VecToOther.Length() + (VecToOther - OrderTerm * OtherVelocity.GetSafeNormal()).Length();
			return 0.5f * FMath::Sqrt(FMath::Max(FMath::Square(SquareRootTerm) - FMath::Square(OrderTerm)));
		};
		
		const FVector2f VecToOther = OtherPosition - CurrentPosition;
		
		const float SemiMinorAxis = GetSemiMinorAxis(VecToOther);
		const float Potential = PotentialFunction(SemiMinorAxis);
		return -VecToOther.GetSafeNormal() * Potential * Parameters.AvoidanceStrength;
	}
};