// Copyright Anupam Sahu. All Rights Reserved.

#pragma once
#include <CoreMinimal.h>
#include <SocialForceModel.generated.h>

USTRUCT()
struct FTCSocialForceParameters
{
	GENERATED_BODY()
	
	UPROPERTY(EditAnywhere)
	float PedestrianSize = 25.0f;
	
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
	float PedestrianHalfSize = 25.0f;
	
	UPROPERTY(EditAnywhere)
	float HalfFOV = 100.0f;
};

class FTCSocialForces
{
public:

	static FVector2f GetDrivingForce(const FVector2f& CurrentVelocity, const FVector2f& DesiredDirection, const FTCSocialForceParameters& Parameters = {});
	static FVector2f GetDrivingForceForVelocity(const FVector2f& CurrentVelocity, const FVector2f& DesiredVelocity, const FTCSocialForceParameters& Parameters = {});
	static FVector2f GetAvoidanceForce(const FVector2f& CurrentPosition, const FVector2f& OtherPosition, const FVector2f& OtherVelocity, const float DeltaTime, const FTCSocialForceParameters& Parameters = {});
	static float PotentialFunction(float X, const float AvoidanceRadius);
	static float GetSemiMinorAxis(const FVector2f& Vec, const FVector2f& OtherVelocity, float DeltaTime);
};