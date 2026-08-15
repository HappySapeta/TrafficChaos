#include "SocialForceModel.h"

FVector2f FTCSocialForces::GetDrivingForce(const FVector2f& CurrentVelocity, const FVector2f& DesiredDirection, const FTCSocialForceParameters& Parameters)
{
	return (Parameters.DesiredSpeed * DesiredDirection - CurrentVelocity) / Parameters.RelaxationTime;
}

FVector2f FTCSocialForces::GetDrivingForceForVelocity(const FVector2f& CurrentVelocity, const FVector2f& DesiredVelocity, const FTCSocialForceParameters& Parameters)
{
	return (DesiredVelocity - CurrentVelocity) / Parameters.RelaxationTime;
}

FVector2f FTCSocialForces::GetAvoidanceForce(const FVector2f& CurrentPosition, const FVector2f& OtherPosition, const FVector2f& OtherVelocity, const float DeltaTime, const FTCSocialForceParameters& Parameters)
{
	if (FVector2f::Distance(CurrentPosition, OtherPosition) > Parameters.AvoidanceRadius)
	{
		return FVector2f::ZeroVector;
	}

	const FVector2f VecToOther = OtherPosition - CurrentPosition;

	const float SemiMinorAxis = GetSemiMinorAxis(VecToOther, OtherVelocity, DeltaTime);
	const float Potential = PotentialFunction(SemiMinorAxis, Parameters.AvoidanceRadius);
	return -VecToOther.GetSafeNormal() * Potential * Parameters.AvoidanceStrength;
}

float FTCSocialForces::PotentialFunction(const float X, const float AvoidanceRadius)
{
	return FMath::Exp(-X / AvoidanceRadius);
}

float FTCSocialForces::GetSemiMinorAxis(const FVector2f& Vec, const FVector2f& OtherVelocity, const float DeltaTime)
{
	const float OrderTerm = OtherVelocity.Length() * DeltaTime;
	const float SquareRootTerm = Vec.Length() + (Vec - OrderTerm * OtherVelocity.GetSafeNormal()).Length();
	const float Value = 0.5f * FMath::Sqrt(FMath::Max(FMath::Square(SquareRootTerm) - FMath::Square(OrderTerm), 0));
	return Value;
};