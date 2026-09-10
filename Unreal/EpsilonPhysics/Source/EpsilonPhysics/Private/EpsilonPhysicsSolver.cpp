#include "EpsilonPhysicsSolver.h"

DEFINE_LOG_CATEGORY(LogEpsilonPhysics);

namespace
{
constexpr double ContactNormalTolerance = 1.0e-12;
constexpr double ContactNormalToleranceSquared = ContactNormalTolerance * ContactNormalTolerance;
constexpr double ContactPenetrationTolerance = 1.0e-12;

bool IsFiniteScalar(const double Value)
{
	return FMath::IsFinite(Value);
}

bool IsFiniteVector(const FVector3d& Value)
{
	return IsFiniteScalar(Value.X) && IsFiniteScalar(Value.Y) && IsFiniteScalar(Value.Z);
}

FVector3d GetDeterministicAxis(const int32 ParticleAIndex, const int32 ParticleBIndex)
{
	const FVector3d Axes[3] =
	{
		FVector3d(1.0, 0.0, 0.0),
		FVector3d(0.0, 1.0, 0.0),
		FVector3d(0.0, 0.0, 1.0)
	};

	const FVector3d Axis = Axes[(ParticleAIndex + ParticleBIndex) % UE_ARRAY_COUNT(Axes)];
	return ((ParticleAIndex ^ ParticleBIndex) & 1) == 0 ? Axis : -Axis;
}

FVector3d GetStableCollisionNormal(
	const FVector3d& RelativePosition,
	const FVector3d& RelativeVelocity,
	const int32 ParticleAIndex,
	const int32 ParticleBIndex,
	double& OutDistance)
{
	const double DistanceSquared = RelativePosition.SizeSquared();
	if (IsFiniteScalar(DistanceSquared) && DistanceSquared > ContactNormalToleranceSquared)
	{
		OutDistance = FMath::Sqrt(DistanceSquared);
		if (IsFiniteScalar(OutDistance) && OutDistance > ContactNormalTolerance)
		{
			return RelativePosition / OutDistance;
		}
	}

	OutDistance = 0.0;

	const double RelativeSpeedSquared = RelativeVelocity.SizeSquared();
	if (IsFiniteScalar(RelativeSpeedSquared) && RelativeSpeedSquared > ContactNormalToleranceSquared)
	{
		return -RelativeVelocity / FMath::Sqrt(RelativeSpeedSquared);
	}

	return GetDeterministicAxis(ParticleAIndex, ParticleBIndex);
}
}

bool FEpsilonParticle::HasFiniteKinematicState() const
{
	return IsFiniteVector(Position)
		&& IsFiniteVector(Velocity)
		&& IsFiniteScalar(Mass)
		&& IsFiniteScalar(Radius)
		&& Radius >= 0.0;
}

bool FEpsilonParticle::HasFiniteState() const
{
	return HasFiniteKinematicState() && IsFiniteVector(Acceleration);
}

bool UEpsilonPhysicsSubsystem::InitializeSimulation(
	const TArray<FEpsilonParticle>& InParticles,
	const double InAlphaEff,
	const double InBeta,
	const double InSofteningLength)
{
	ClearError();

	if (!ValidateParameters(InAlphaEff, InBeta, InSofteningLength))
	{
		return false;
	}

	const double ProposedSofteningLengthSquared = InSofteningLength * InSofteningLength;
	if (!IsFiniteScalar(ProposedSofteningLengthSquared) || ProposedSofteningLengthSquared <= 0.0)
	{
		return RecordError(TEXT("SofteningLength^2 must remain finite and strictly positive during initialization."));
	}

	if (!ValidateParticles(InParticles, false))
	{
		return false;
	}

	TArray<FVector3d> InitialAccelerations;
	if (!EvaluateAccelerations(InParticles, InAlphaEff, InBeta, ProposedSofteningLengthSquared, InitialAccelerations))
	{
		return false;
	}

	AlphaEff = InAlphaEff;
	Beta = InBeta;
	SofteningLength = InSofteningLength;
	SofteningLengthSquared = ProposedSofteningLengthSquared;
	bIsConfigured = true;
	Particles = InParticles;
	if (!ApplyAccelerationsToParticles(Particles, InitialAccelerations))
	{
		return false;
	}

	return true;
}

bool UEpsilonPhysicsSubsystem::ConfigureSimulation(
	const double InAlphaEff,
	const double InBeta,
	const double InSofteningLength)
{
	ClearError();

	if (!ValidateParameters(InAlphaEff, InBeta, InSofteningLength))
	{
		return false;
	}

	const double ProposedSofteningLengthSquared = InSofteningLength * InSofteningLength;
	if (!IsFiniteScalar(ProposedSofteningLengthSquared) || ProposedSofteningLengthSquared <= 0.0)
	{
		return RecordError(TEXT("SofteningLength^2 must remain finite and strictly positive during configuration."));
	}

	AlphaEff = InAlphaEff;
	Beta = InBeta;
	SofteningLength = InSofteningLength;
	SofteningLengthSquared = ProposedSofteningLengthSquared;
	bIsConfigured = true;
	LastError.Reset();
	return true;
}

bool UEpsilonPhysicsSubsystem::StepSimulation(TArray<FEpsilonParticle>& InOutParticles, const double DeltaSeconds)
{
	ClearError();

	if (!IsFiniteScalar(DeltaSeconds))
	{
		return RecordError(TEXT("StepSimulation requires a finite DeltaSeconds value."));
	}

	if (DeltaSeconds <= 0.0)
	{
		return true;
	}

	if (!ValidateConfiguredState())
	{
		return RecordError(TEXT("ConfigureSimulation or InitializeSimulation must set finite AlphaEff, beta >= 0, and SofteningLength > 0 before stepping."));
	}

	if (!ValidateParticles(InOutParticles, false))
	{
		return false;
	}

	const double HalfDeltaSeconds = 0.5 * DeltaSeconds;
	if (!IsFiniteScalar(HalfDeltaSeconds))
	{
		return RecordError(TEXT("Velocity-Verlet half timestep became non-finite."));
	}

	TArray<FVector3d> CurrentAccelerations;
	if (!EvaluateAccelerations(InOutParticles, AlphaEff, Beta, SofteningLengthSquared, CurrentAccelerations))
	{
		return false;
	}

	TArray<FEpsilonParticle> UpdatedParticles = InOutParticles;
	for (int32 Index = 0; Index < UpdatedParticles.Num(); ++Index)
	{
		FEpsilonParticle& Particle = UpdatedParticles[Index];
		const FVector3d& CurrentAcceleration = CurrentAccelerations[Index];
		if (!IsFiniteVector(CurrentAcceleration))
		{
			return RecordError(FString::Printf(TEXT("Computed acceleration for particle %d is non-finite."), Index));
		}

		Particle.Acceleration = CurrentAcceleration;
		Particle.Velocity += CurrentAcceleration * HalfDeltaSeconds;
		Particle.Position += Particle.Velocity * DeltaSeconds;

		if (!Particle.HasFiniteKinematicState() || Particle.Mass <= 0.0)
		{
			return RecordError(FString::Printf(TEXT("Velocity-Verlet drift produced an invalid state for particle %d."), Index));
		}
	}

	if (!ResolveCollisions(UpdatedParticles))
	{
		return false;
	}

	TArray<FVector3d> NewAccelerations;
	if (!EvaluateAccelerations(UpdatedParticles, AlphaEff, Beta, SofteningLengthSquared, NewAccelerations))
	{
		return false;
	}

	if (!ApplyAccelerationsToParticles(UpdatedParticles, NewAccelerations))
	{
		return false;
	}

	for (int32 Index = 0; Index < UpdatedParticles.Num(); ++Index)
	{
		FEpsilonParticle& Particle = UpdatedParticles[Index];
		const FVector3d& NewAcceleration = NewAccelerations[Index];

		Particle.Velocity += NewAcceleration * HalfDeltaSeconds;
		if (!Particle.HasFiniteState() || Particle.Mass <= 0.0)
		{
			return RecordError(FString::Printf(TEXT("Velocity-Verlet finalization produced an invalid state for particle %d."), Index));
		}
	}

	InOutParticles = UpdatedParticles;
	Particles = InOutParticles;
	return true;
}

bool UEpsilonPhysicsSubsystem::StepSimulation(const double DeltaSeconds)
{
	return StepSimulation(Particles, DeltaSeconds);
}

void UEpsilonPhysicsSubsystem::ResetSimulation()
{
	Particles.Reset();
	AlphaEff = 1.0;
	Beta = 0.0;
	SofteningLength = 0.0;
	SofteningLengthSquared = 0.0;
	bIsConfigured = false;
	ClearError();
}

bool UEpsilonPhysicsSubsystem::ValidateParameters(const double InAlphaEff, const double InBeta, const double InSofteningLength)
{
	if (!IsFiniteScalar(InAlphaEff))
	{
		return RecordError(TEXT("AlphaEff must be finite."));
	}

	if (!IsFiniteScalar(InBeta))
	{
		return RecordError(TEXT("Beta must be finite."));
	}

	if (InBeta < 0.0)
	{
		return RecordError(TEXT("Beta must be non-negative."));
	}

	if (!IsFiniteScalar(InSofteningLength))
	{
		return RecordError(TEXT("SofteningLength must be finite."));
	}

	if (InSofteningLength <= 0.0)
	{
		return RecordError(TEXT("SofteningLength must be strictly positive."));
	}

	return true;
}

bool UEpsilonPhysicsSubsystem::ValidateConfiguredState() const
{
	return bIsConfigured
		&& FMath::IsFinite(AlphaEff)
		&& FMath::IsFinite(Beta)
		&& Beta >= 0.0
		&& FMath::IsFinite(SofteningLength)
		&& SofteningLength > 0.0
		&& FMath::IsFinite(SofteningLengthSquared)
		&& SofteningLengthSquared > 0.0;
}

bool UEpsilonPhysicsSubsystem::ValidateParticles(const TArray<FEpsilonParticle>& InParticles, const bool bRequireFiniteAcceleration)
{
	for (int32 Index = 0; Index < InParticles.Num(); ++Index)
	{
		const FEpsilonParticle& Particle = InParticles[Index];
		if (!Particle.HasFiniteKinematicState())
		{
			return RecordError(FString::Printf(
				TEXT("Particle %d contains a non-finite position, velocity, mass, or radius."),
				Index));
		}

		if (Particle.Mass <= 0.0)
		{
			return RecordError(FString::Printf(TEXT("Particle %d must have a strictly positive mass."), Index));
		}

		if (!IsFiniteScalar(Particle.Radius) || Particle.Radius < 0.0)
		{
			return RecordError(FString::Printf(TEXT("Particle %d must have a finite, non-negative radius."), Index));
		}

		if (bRequireFiniteAcceleration && !IsFiniteVector(Particle.Acceleration))
		{
			return RecordError(FString::Printf(TEXT("Particle %d contains a non-finite acceleration."), Index));
		}
	}

	return true;
}

bool UEpsilonPhysicsSubsystem::ResolveCollisions(TArray<FEpsilonParticle>& InOutParticles)
{
	if (InOutParticles.Num() < 2)
	{
		return true;
	}

	const int32 MaxIterations = FMath::Max(1, InOutParticles.Num());
	for (int32 Iteration = 0; Iteration < MaxIterations; ++Iteration)
	{
		bool bResolvedPenetrationThisIteration = false;

		for (int32 ParticleAIndex = 0; ParticleAIndex < InOutParticles.Num(); ++ParticleAIndex)
		{
			FEpsilonParticle& ParticleA = InOutParticles[ParticleAIndex];
			for (int32 ParticleBIndex = ParticleAIndex + 1; ParticleBIndex < InOutParticles.Num(); ++ParticleBIndex)
			{
				FEpsilonParticle& ParticleB = InOutParticles[ParticleBIndex];

				const double RadiiSum = ParticleA.Radius + ParticleB.Radius;
				if (!IsFiniteScalar(RadiiSum))
				{
					return RecordError(FString::Printf(TEXT("Combined radius for particles %d and %d is non-finite."), ParticleAIndex, ParticleBIndex));
				}

				if (RadiiSum <= 0.0)
				{
					continue;
				}

				const FVector3d RelativePosition = ParticleA.Position - ParticleB.Position;
				if (!IsFiniteVector(RelativePosition))
				{
					return RecordError(FString::Printf(TEXT("Collision displacement for particles %d and %d is non-finite."), ParticleAIndex, ParticleBIndex));
				}

				const FVector3d RelativeVelocity = ParticleA.Velocity - ParticleB.Velocity;
				if (!IsFiniteVector(RelativeVelocity))
				{
					return RecordError(FString::Printf(TEXT("Collision relative velocity for particles %d and %d is non-finite."), ParticleAIndex, ParticleBIndex));
				}

				double Distance = 0.0;
				const FVector3d Normal = GetStableCollisionNormal(
					RelativePosition,
					RelativeVelocity,
					ParticleAIndex,
					ParticleBIndex,
					Distance);
				if (!IsFiniteVector(Normal))
				{
					return RecordError(FString::Printf(TEXT("Collision normal for particles %d and %d is non-finite."), ParticleAIndex, ParticleBIndex));
				}

				const double PenetrationDepth = RadiiSum - Distance;
				if (!IsFiniteScalar(PenetrationDepth))
				{
					return RecordError(FString::Printf(TEXT("Collision penetration for particles %d and %d is non-finite."), ParticleAIndex, ParticleBIndex));
				}

				if (PenetrationDepth < -ContactPenetrationTolerance)
				{
					continue;
				}

				const double InvMassA = 1.0 / ParticleA.Mass;
				const double InvMassB = 1.0 / ParticleB.Mass;
				const double InvMassSum = InvMassA + InvMassB;
				if (!IsFiniteScalar(InvMassA) || !IsFiniteScalar(InvMassB) || !IsFiniteScalar(InvMassSum) || InvMassSum <= 0.0)
				{
					return RecordError(FString::Printf(TEXT("Collision inverse mass state for particles %d and %d is invalid."), ParticleAIndex, ParticleBIndex));
				}

				if (PenetrationDepth > 0.0)
				{
					const double CorrectionMagnitude = PenetrationDepth / InvMassSum;
					if (!IsFiniteScalar(CorrectionMagnitude))
					{
						return RecordError(FString::Printf(TEXT("Collision projection magnitude for particles %d and %d is non-finite."), ParticleAIndex, ParticleBIndex));
					}

					const FVector3d Correction = Normal * CorrectionMagnitude;
					ParticleA.Position += Correction * InvMassA;
					ParticleB.Position -= Correction * InvMassB;
					bResolvedPenetrationThisIteration = true;

					if (!ParticleA.HasFiniteKinematicState() || !ParticleB.HasFiniteKinematicState())
					{
						return RecordError(FString::Printf(TEXT("Collision projection produced an invalid kinematic state for particles %d and %d."), ParticleAIndex, ParticleBIndex));
					}
				}

				const double PreNormalRelativeVelocity = FVector3d::DotProduct(RelativeVelocity, Normal);
				if (!IsFiniteScalar(PreNormalRelativeVelocity))
				{
					return RecordError(FString::Printf(TEXT("Collision normal velocity for particles %d and %d is non-finite."), ParticleAIndex, ParticleBIndex));
				}

				if (PreNormalRelativeVelocity < 0.0)
				{
					const double NormalImpulseMagnitude = -(2.0 * PreNormalRelativeVelocity) / InvMassSum;
					if (!IsFiniteScalar(NormalImpulseMagnitude) || NormalImpulseMagnitude < 0.0)
					{
						return RecordError(FString::Printf(TEXT("Collision normal impulse for particles %d and %d is invalid."), ParticleAIndex, ParticleBIndex));
					}

					const FVector3d NormalImpulse = Normal * NormalImpulseMagnitude;
					ParticleA.Velocity += NormalImpulse * InvMassA;
					ParticleB.Velocity -= NormalImpulse * InvMassB;

					if (!ParticleA.HasFiniteKinematicState() || !ParticleB.HasFiniteKinematicState())
					{
						return RecordError(FString::Printf(TEXT("Collision impulse produced an invalid kinematic state for particles %d and %d."), ParticleAIndex, ParticleBIndex));
					}
				}
			}
		}

		if (!bResolvedPenetrationThisIteration)
		{
			break;
		}
	}

	return true;
}

bool UEpsilonPhysicsSubsystem::EvaluateAccelerations(
	const TArray<FEpsilonParticle>& InParticles,
	const double InAlphaEff,
	const double InBeta,
	const double InSofteningLengthSquared,
	TArray<FVector3d>& OutAccelerations)
{
	OutAccelerations.Reset();
	OutAccelerations.AddZeroed(InParticles.Num());

	for (int32 I = 0; I < InParticles.Num(); ++I)
	{
		const FEpsilonParticle& ParticleI = InParticles[I];
		for (int32 J = I + 1; J < InParticles.Num(); ++J)
		{
			const FEpsilonParticle& ParticleJ = InParticles[J];
			const FVector3d Rij = ParticleJ.Position - ParticleI.Position;
			if (!IsFiniteVector(Rij))
			{
				return RecordError(FString::Printf(TEXT("Pair displacement for particles %d and %d is non-finite."), I, J));
			}

			const double RadiusSquared = Rij.SizeSquared();
			if (!IsFiniteScalar(RadiusSquared))
			{
				return RecordError(FString::Printf(TEXT("RadiusSquared for particles %d and %d is non-finite."), I, J));
			}

			const double DistanceTerm = RadiusSquared + InSofteningLengthSquared;
			if (!IsFiniteScalar(DistanceTerm) || DistanceTerm <= 0.0)
			{
				return RecordError(FString::Printf(
					TEXT("Particles %d and %d produced a non-positive distance term; increase softening or separate overlapping particles."),
					I,
					J));
			}

			const double DistanceSqrt = FMath::Sqrt(DistanceTerm);
			const double DistancePowerThreeHalves = DistanceTerm * DistanceSqrt;
			const double DistanceCubed = DistanceTerm * DistanceTerm * DistanceTerm;
			if (!IsFiniteScalar(DistancePowerThreeHalves) || !IsFiniteScalar(DistanceCubed) || DistancePowerThreeHalves <= 0.0 || DistanceCubed <= 0.0)
			{
				return RecordError(FString::Printf(TEXT("Distance powers for particles %d and %d are invalid."), I, J));
			}

			const double AttractiveScale = (InAlphaEff * ParticleI.Mass * ParticleJ.Mass) / DistancePowerThreeHalves;
			const double RepulsiveScale = (4.0 * InBeta) / DistanceCubed;
			if (!IsFiniteScalar(AttractiveScale) || !IsFiniteScalar(RepulsiveScale))
			{
				return RecordError(FString::Printf(TEXT("Force scales for particles %d and %d are non-finite."), I, J));
			}

			const double ForceScale = AttractiveScale - RepulsiveScale;
			if (!IsFiniteScalar(ForceScale))
			{
				return RecordError(FString::Printf(TEXT("Net force scale for particles %d and %d is non-finite."), I, J));
			}

			const FVector3d Force = Rij * ForceScale;
			if (!IsFiniteVector(Force))
			{
				return RecordError(FString::Printf(TEXT("Net force for particles %d and %d is non-finite."), I, J));
			}

			OutAccelerations[I] += Force / ParticleI.Mass;
			OutAccelerations[J] -= Force / ParticleJ.Mass;

			if (!IsFiniteVector(OutAccelerations[I]) || !IsFiniteVector(OutAccelerations[J]))
			{
				return RecordError(FString::Printf(TEXT("Acceleration accumulation overflowed for particle pair %d and %d."), I, J));
			}
		}
	}

	return true;
}

bool UEpsilonPhysicsSubsystem::RecordError(const FString& Message)
{
	LastError = Message;
	UE_LOG(LogEpsilonPhysics, Error, TEXT("%s"), *Message);
	return false;
}

void UEpsilonPhysicsSubsystem::ClearError()
{
	LastError.Reset();
}

bool UEpsilonPhysicsSubsystem::ApplyAccelerationsToParticles(
	TArray<FEpsilonParticle>& InOutParticles,
	const TArray<FVector3d>& InAccelerations)
{
	if (InOutParticles.Num() != InAccelerations.Num())
	{
		return RecordError(TEXT("Acceleration count does not match particle count."));
	}

	for (int32 Index = 0; Index < InOutParticles.Num(); ++Index)
	{
		if (!IsFiniteVector(InAccelerations[Index]))
		{
			return RecordError(FString::Printf(TEXT("Acceleration for particle %d is non-finite."), Index));
		}

		InOutParticles[Index].Acceleration = InAccelerations[Index];
		if (!InOutParticles[Index].HasFiniteState())
		{
			return RecordError(FString::Printf(TEXT("Applying acceleration produced an invalid state for particle %d."), Index));
		}
	}

	return true;
}
