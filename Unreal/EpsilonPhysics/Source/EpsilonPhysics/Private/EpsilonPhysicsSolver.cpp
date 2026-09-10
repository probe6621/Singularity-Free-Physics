#include "EpsilonPhysicsSolver.h"

DEFINE_LOG_CATEGORY(LogEpsilonPhysics);

namespace
{
bool IsFiniteScalar(const double Value)
{
	return FMath::IsFinite(Value);
}

bool IsFiniteVector(const FVector3d& Value)
{
	return IsFiniteScalar(Value.X) && IsFiniteScalar(Value.Y) && IsFiniteScalar(Value.Z);
}
}

bool FEpsilonParticle::HasFiniteKinematicState() const
{
	return IsFiniteVector(Position)
		&& IsFiniteVector(Velocity)
		&& IsFiniteScalar(Mass);
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
			return RecordError(FString::Printf(TEXT("Particle %d contains a non-finite position, velocity, or mass."), Index));
		}

		if (Particle.Mass <= 0.0)
		{
			return RecordError(FString::Printf(TEXT("Particle %d must have a strictly positive mass."), Index));
		}

		if (bRequireFiniteAcceleration && !IsFiniteVector(Particle.Acceleration))
		{
			return RecordError(FString::Printf(TEXT("Particle %d contains a non-finite acceleration."), Index));
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
