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
	double& OutDistance,
	bool& bOutUsedFallbackNormal)
{
	bOutUsedFallbackNormal = false;

	const double DistanceSquared = RelativePosition.SizeSquared();
	if (IsFiniteScalar(DistanceSquared) && DistanceSquared > ContactNormalToleranceSquared)
	{
		OutDistance = FMath::Sqrt(DistanceSquared);
		if (IsFiniteScalar(OutDistance) && OutDistance > ContactNormalTolerance)
		{
			return RelativePosition / OutDistance;
		}
	}

	bOutUsedFallbackNormal = true;
	OutDistance = 0.0;

	const double RelativeSpeedSquared = RelativeVelocity.SizeSquared();
	if (IsFiniteScalar(RelativeSpeedSquared) && RelativeSpeedSquared > ContactNormalToleranceSquared)
	{
		return -RelativeVelocity / FMath::Sqrt(RelativeSpeedSquared);
	}

	return GetDeterministicAxis(ParticleAIndex, ParticleBIndex);
}

uint64 MakeContactPairKey(const int32 ParticleAIndex, const int32 ParticleBIndex)
{
	return (static_cast<uint64>(static_cast<uint32>(ParticleAIndex)) << 32)
		| static_cast<uint32>(ParticleBIndex);
}

void AccumulateContactInfo(
	TArray<FEpsilonContactInfo>& InOutContacts,
	TMap<uint64, int32>& InOutContactIndices,
	const FEpsilonContactInfo& Contact)
{
	const uint64 PairKey = MakeContactPairKey(Contact.ParticleAIndex, Contact.ParticleBIndex);
	if (const int32* ExistingIndex = InOutContactIndices.Find(PairKey))
	{
		FEpsilonContactInfo& Existing = InOutContacts[*ExistingIndex];
		Existing.Normal = Contact.Normal;
		Existing.ContactPoint = Contact.ContactPoint;
		Existing.PenetrationDepth = FMath::Max(Existing.PenetrationDepth, Contact.PenetrationDepth);
		Existing.NormalImpulse += Contact.NormalImpulse;
		Existing.TangentImpulse += Contact.TangentImpulse;
		Existing.CombinedRestitution = Contact.CombinedRestitution;
		Existing.CombinedFriction = Contact.CombinedFriction;
		Existing.PreNormalRelativeSpeed = FMath::Max(Existing.PreNormalRelativeSpeed, Contact.PreNormalRelativeSpeed);
		Existing.PostNormalRelativeSpeed = Contact.PostNormalRelativeSpeed;
		Existing.NormalEnergyDissipated += Contact.NormalEnergyDissipated;
		Existing.bAppliedImpulse = Existing.bAppliedImpulse || Contact.bAppliedImpulse;
		Existing.bUsedFallbackNormal = Existing.bUsedFallbackNormal || Contact.bUsedFallbackNormal;
		return;
	}

	InOutContactIndices.Add(PairKey, InOutContacts.Add(Contact));
}

FVector3d ComputeContactPoint(const FEpsilonParticle& ParticleA, const FEpsilonParticle& ParticleB, const FVector3d& Normal)
{
	const FVector3d SurfacePointA = ParticleA.Position - (Normal * ParticleA.Radius);
	const FVector3d SurfacePointB = ParticleB.Position + (Normal * ParticleB.Radius);
	return 0.5 * (SurfacePointA + SurfacePointB);
}
}

bool FEpsilonParticle::HasFiniteKinematicState() const
{
	return IsFiniteVector(Position)
		&& IsFiniteVector(Velocity)
		&& IsFiniteScalar(Mass)
		&& IsFiniteScalar(Radius)
		&& IsFiniteScalar(Restitution)
		&& IsFiniteScalar(Friction);
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

	LastContacts.Reset();
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

bool UEpsilonPhysicsSubsystem::ConfigureCollision(
	const bool bInCollisionEnabled,
	const double InProjectionPercent,
	const double InProjectionSlop)
{
	ClearError();

	if (!ValidateCollisionSettings(bInCollisionEnabled, InProjectionPercent, InProjectionSlop))
	{
		return false;
	}

	bCollisionEnabled = bInCollisionEnabled;
	CollisionProjectionPercent = InProjectionPercent;
	CollisionProjectionSlop = InProjectionSlop;
	LastError.Reset();
	return true;
}

bool UEpsilonPhysicsSubsystem::StepSimulation(TArray<FEpsilonParticle>& InOutParticles, const double DeltaSeconds)
{
	ClearError();
	LastContacts.Reset();

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

	if (!ValidateCollisionState())
	{
		return RecordError(TEXT("ConfigureCollision must set finite collision settings, projection percent in (0, 1] when enabled, and slop >= 0 before stepping."));
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

	TArray<FEpsilonContactInfo> ResolvedContacts;
	if (!ResolveCollisions(UpdatedParticles, ResolvedContacts))
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
	LastContacts = MoveTemp(ResolvedContacts);
	for (const FEpsilonContactInfo& Contact : LastContacts)
	{
		OnCollisionResolved.Broadcast(Contact);
	}
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
	bCollisionEnabled = true;
	CollisionProjectionPercent = 1.0;
	CollisionProjectionSlop = 0.0;
	LastContacts.Reset();
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

bool UEpsilonPhysicsSubsystem::ValidateCollisionSettings(
	const bool bInCollisionEnabled,
	const double InProjectionPercent,
	const double InProjectionSlop)
{
	if (!IsFiniteScalar(InProjectionPercent))
	{
		return RecordError(TEXT("Collision projection percent must be finite."));
	}

	if (InProjectionPercent < 0.0 || InProjectionPercent > 1.0)
	{
		return RecordError(TEXT("Collision projection percent must lie within [0, 1]."));
	}

	if (bInCollisionEnabled && InProjectionPercent <= 0.0)
	{
		return RecordError(TEXT("Collision projection percent must be strictly positive when collisions are enabled."));
	}

	if (!IsFiniteScalar(InProjectionSlop))
	{
		return RecordError(TEXT("Collision projection slop must be finite."));
	}

	if (InProjectionSlop < 0.0)
	{
		return RecordError(TEXT("Collision projection slop must be non-negative."));
	}

	return true;
}

bool UEpsilonPhysicsSubsystem::ValidateCollisionState() const
{
	return FMath::IsFinite(CollisionProjectionPercent)
		&& CollisionProjectionPercent >= 0.0
		&& CollisionProjectionPercent <= 1.0
		&& (!bCollisionEnabled || CollisionProjectionPercent > 0.0)
		&& FMath::IsFinite(CollisionProjectionSlop)
		&& CollisionProjectionSlop >= 0.0;
}

bool UEpsilonPhysicsSubsystem::ValidateParticles(const TArray<FEpsilonParticle>& InParticles, const bool bRequireFiniteAcceleration)
{
	for (int32 Index = 0; Index < InParticles.Num(); ++Index)
	{
		const FEpsilonParticle& Particle = InParticles[Index];
		if (!Particle.HasFiniteKinematicState())
		{
			return RecordError(FString::Printf(
				TEXT("Particle %d contains a non-finite position, velocity, mass, radius, restitution, or friction."),
				Index));
		}

		if (Particle.Mass <= 0.0)
		{
			return RecordError(FString::Printf(TEXT("Particle %d must have a strictly positive mass."), Index));
		}

		if (Particle.Radius < 0.0)
		{
			return RecordError(FString::Printf(TEXT("Particle %d must have a non-negative radius."), Index));
		}

		if (Particle.Restitution < 0.0 || Particle.Restitution > 1.0)
		{
			return RecordError(FString::Printf(TEXT("Particle %d must have restitution within [0, 1]."), Index));
		}

		if (Particle.Friction < 0.0)
		{
			return RecordError(FString::Printf(TEXT("Particle %d must have non-negative friction."), Index));
		}

		if (bRequireFiniteAcceleration && !IsFiniteVector(Particle.Acceleration))
		{
			return RecordError(FString::Printf(TEXT("Particle %d contains a non-finite acceleration."), Index));
		}
	}

	return true;
}

bool UEpsilonPhysicsSubsystem::ResolveCollisions(TArray<FEpsilonParticle>& InOutParticles, TArray<FEpsilonContactInfo>& OutContacts)
{
	OutContacts.Reset();
	if (!bCollisionEnabled || InOutParticles.Num() < 2)
	{
		return true;
	}

	TMap<uint64, int32> ContactIndices;
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

				const FVector3d RelativeVelocityBefore = ParticleA.Velocity - ParticleB.Velocity;
				if (!IsFiniteVector(RelativeVelocityBefore))
				{
					return RecordError(FString::Printf(TEXT("Collision relative velocity for particles %d and %d is non-finite."), ParticleAIndex, ParticleBIndex));
				}

				double Distance = 0.0;
				bool bUsedFallbackNormal = false;
				const FVector3d Normal = GetStableCollisionNormal(
					RelativePosition,
					RelativeVelocityBefore,
					ParticleAIndex,
					ParticleBIndex,
					Distance,
					bUsedFallbackNormal);
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

				const double CombinedRestitution = FMath::Sqrt(ParticleA.Restitution * ParticleB.Restitution);
				const double CombinedFriction = FMath::Sqrt(ParticleA.Friction * ParticleB.Friction);
				if (!IsFiniteScalar(CombinedRestitution) || !IsFiniteScalar(CombinedFriction))
				{
					return RecordError(FString::Printf(TEXT("Collision material combine for particles %d and %d is non-finite."), ParticleAIndex, ParticleBIndex));
				}

				const double PreNormalRelativeVelocity = FVector3d::DotProduct(RelativeVelocityBefore, Normal);
				if (!IsFiniteScalar(PreNormalRelativeVelocity))
				{
					return RecordError(FString::Printf(TEXT("Collision normal velocity for particles %d and %d is non-finite."), ParticleAIndex, ParticleBIndex));
				}

				double NormalImpulseMagnitude = 0.0;
				double TangentImpulseMagnitude = 0.0;
				bool bAppliedImpulse = false;
				if (PreNormalRelativeVelocity < 0.0)
				{
					NormalImpulseMagnitude = -((1.0 + CombinedRestitution) * PreNormalRelativeVelocity) / InvMassSum;
					if (!IsFiniteScalar(NormalImpulseMagnitude) || NormalImpulseMagnitude < 0.0)
					{
						return RecordError(FString::Printf(TEXT("Collision normal impulse for particles %d and %d is invalid."), ParticleAIndex, ParticleBIndex));
					}

					const FVector3d NormalImpulse = Normal * NormalImpulseMagnitude;
					ParticleA.Velocity += NormalImpulse * InvMassA;
					ParticleB.Velocity -= NormalImpulse * InvMassB;
					bAppliedImpulse = true;

					if (!ParticleA.HasFiniteKinematicState() || !ParticleB.HasFiniteKinematicState())
					{
						return RecordError(FString::Printf(TEXT("Collision normal impulse produced an invalid kinematic state for particles %d and %d."), ParticleAIndex, ParticleBIndex));
					}

					const FVector3d RelativeVelocityAfterNormalImpulse = ParticleA.Velocity - ParticleB.Velocity;
					if (!IsFiniteVector(RelativeVelocityAfterNormalImpulse))
					{
						return RecordError(FString::Printf(TEXT("Post-normal collision velocity for particles %d and %d is non-finite."), ParticleAIndex, ParticleBIndex));
					}

					const double PostNormalRelativeVelocity = FVector3d::DotProduct(RelativeVelocityAfterNormalImpulse, Normal);
					if (!IsFiniteScalar(PostNormalRelativeVelocity))
					{
						return RecordError(FString::Printf(TEXT("Post-normal collision speed for particles %d and %d is non-finite."), ParticleAIndex, ParticleBIndex));
					}

					const FVector3d TangentialVelocity = RelativeVelocityAfterNormalImpulse - (PostNormalRelativeVelocity * Normal);
					if (!IsFiniteVector(TangentialVelocity))
					{
						return RecordError(FString::Printf(TEXT("Collision tangential velocity for particles %d and %d is non-finite."), ParticleAIndex, ParticleBIndex));
					}

					const double TangentialSpeedSquared = TangentialVelocity.SizeSquared();
					if (!IsFiniteScalar(TangentialSpeedSquared))
					{
						return RecordError(FString::Printf(TEXT("Collision tangential speed for particles %d and %d is non-finite."), ParticleAIndex, ParticleBIndex));
					}

					if (CombinedFriction > 0.0 && TangentialSpeedSquared > ContactNormalToleranceSquared)
					{
						const double TangentialSpeed = FMath::Sqrt(TangentialSpeedSquared);
						const FVector3d Tangent = TangentialVelocity / TangentialSpeed;
						const double TangentImpulseUnclamped = -FVector3d::DotProduct(RelativeVelocityAfterNormalImpulse, Tangent) / InvMassSum;
						const double MaxFrictionImpulse = CombinedFriction * NormalImpulseMagnitude;
						const double TangentImpulseScalar = FMath::Clamp(TangentImpulseUnclamped, -MaxFrictionImpulse, MaxFrictionImpulse);
						if (!IsFiniteScalar(TangentImpulseScalar))
						{
							return RecordError(FString::Printf(TEXT("Collision tangent impulse for particles %d and %d is non-finite."), ParticleAIndex, ParticleBIndex));
						}

						const FVector3d TangentImpulse = Tangent * TangentImpulseScalar;
						ParticleA.Velocity += TangentImpulse * InvMassA;
						ParticleB.Velocity -= TangentImpulse * InvMassB;
						TangentImpulseMagnitude = FMath::Abs(TangentImpulseScalar);

						if (!ParticleA.HasFiniteKinematicState() || !ParticleB.HasFiniteKinematicState())
						{
							return RecordError(FString::Printf(TEXT("Collision tangent impulse produced an invalid kinematic state for particles %d and %d."), ParticleAIndex, ParticleBIndex));
						}
					}
				}

				const double ClampedPenetrationDepth = FMath::Max(0.0, PenetrationDepth);
				if (ClampedPenetrationDepth > 0.0)
				{
					const double CorrectablePenetration = FMath::Max(0.0, ClampedPenetrationDepth - CollisionProjectionSlop);
					if (CorrectablePenetration > 0.0)
					{
						const double CorrectionMagnitude = (CorrectablePenetration * CollisionProjectionPercent) / InvMassSum;
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
				}

				if (ClampedPenetrationDepth > 0.0 || bAppliedImpulse)
				{
					const FVector3d RelativeVelocityAfter = ParticleA.Velocity - ParticleB.Velocity;
					if (!IsFiniteVector(RelativeVelocityAfter))
					{
						return RecordError(FString::Printf(TEXT("Resolved collision velocity for particles %d and %d is non-finite."), ParticleAIndex, ParticleBIndex));
					}

					const double PostNormalRelativeVelocity = FVector3d::DotProduct(RelativeVelocityAfter, Normal);
					if (!IsFiniteScalar(PostNormalRelativeVelocity))
					{
						return RecordError(FString::Printf(TEXT("Resolved collision normal speed for particles %d and %d is non-finite."), ParticleAIndex, ParticleBIndex));
					}

					const double ReducedMass = 1.0 / InvMassSum;
					const double PreNormalRelativeSpeed = FMath::Abs(PreNormalRelativeVelocity);
					const double PostNormalRelativeSpeed = FMath::Abs(PostNormalRelativeVelocity);
					const double NormalEnergyDissipated = 0.5 * ReducedMass * FMath::Max(
						0.0,
						(PreNormalRelativeSpeed * PreNormalRelativeSpeed) - (PostNormalRelativeSpeed * PostNormalRelativeSpeed));
					if (!IsFiniteScalar(ReducedMass) || !IsFiniteScalar(NormalEnergyDissipated))
					{
						return RecordError(FString::Printf(TEXT("Collision energy bookkeeping for particles %d and %d is non-finite."), ParticleAIndex, ParticleBIndex));
					}

					FEpsilonContactInfo Contact;
					Contact.ParticleAIndex = ParticleAIndex;
					Contact.ParticleBIndex = ParticleBIndex;
					Contact.Normal = Normal;
					Contact.ContactPoint = ComputeContactPoint(ParticleA, ParticleB, Normal);
					Contact.PenetrationDepth = ClampedPenetrationDepth;
					Contact.NormalImpulse = NormalImpulseMagnitude;
					Contact.TangentImpulse = TangentImpulseMagnitude;
					Contact.CombinedRestitution = CombinedRestitution;
					Contact.CombinedFriction = CombinedFriction;
					Contact.PreNormalRelativeSpeed = PreNormalRelativeSpeed;
					Contact.PostNormalRelativeSpeed = PostNormalRelativeSpeed;
					Contact.NormalEnergyDissipated = NormalEnergyDissipated;
					Contact.bAppliedImpulse = bAppliedImpulse;
					Contact.bUsedFallbackNormal = bUsedFallbackNormal;
					AccumulateContactInfo(OutContacts, ContactIndices, Contact);
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
