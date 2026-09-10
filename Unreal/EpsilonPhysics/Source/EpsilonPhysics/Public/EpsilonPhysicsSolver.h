#pragma once

#include "CoreMinimal.h"
#include "EpsilonContactTypes.h"
#include "Subsystems/EngineSubsystem.h"

#include "EpsilonPhysicsSolver.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogEpsilonPhysics, Log, All);

USTRUCT(BlueprintType)
struct EPSILONPHYSICS_API FEpsilonParticle
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Epsilon Physics")
	FVector3d Position = FVector3d::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Epsilon Physics")
	FVector3d Velocity = FVector3d::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Epsilon Physics")
	FVector3d Acceleration = FVector3d::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Epsilon Physics", meta = (ClampMin = "0.0"))
	double Mass = 1.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Epsilon Physics|Collision", meta = (ClampMin = "0.0"))
	double Radius = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Epsilon Physics|Collision", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	double Restitution = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Epsilon Physics|Collision", meta = (ClampMin = "0.0"))
	double Friction = 0.0;

	bool HasFiniteKinematicState() const;
	bool HasFiniteState() const;
};

UCLASS(BlueprintType)
class EPSILONPHYSICS_API UEpsilonPhysicsSubsystem final : public UEngineSubsystem
{
	GENERATED_BODY()

public:
	UPROPERTY(BlueprintAssignable, Category = "Epsilon Physics|Collision")
	FEpsilonContactResolvedSignature OnCollisionResolved;

	UFUNCTION(BlueprintCallable, Category = "Epsilon Physics")
	bool InitializeSimulation(const TArray<FEpsilonParticle>& InParticles, double InAlphaEff, double InBeta, double InSofteningLength);

	UFUNCTION(BlueprintCallable, Category = "Epsilon Physics")
	bool ConfigureSimulation(double InAlphaEff, double InBeta, double InSofteningLength);

	UFUNCTION(BlueprintCallable, Category = "Epsilon Physics|Collision")
	bool ConfigureCollision(bool bInCollisionEnabled, double InProjectionPercent = 1.0, double InProjectionSlop = 0.0);

	UFUNCTION(BlueprintCallable, Category = "Epsilon Physics")
	bool StepSimulation(UPARAM(ref) TArray<FEpsilonParticle>& InOutParticles, double DeltaSeconds);

	bool StepSimulation(double DeltaSeconds);

	UFUNCTION(BlueprintCallable, Category = "Epsilon Physics")
	void ResetSimulation();

	UFUNCTION(BlueprintPure, Category = "Epsilon Physics")
	bool IsConfigured() const
	{
		return bIsConfigured;
	}

	UFUNCTION(BlueprintPure, Category = "Epsilon Physics|Collision")
	bool IsCollisionEnabled() const
	{
		return bCollisionEnabled;
	}

	UFUNCTION(BlueprintPure, Category = "Epsilon Physics")
	TArray<FEpsilonParticle> GetParticles() const
	{
		return Particles;
	}

	const TArray<FEpsilonParticle>& GetParticlesView() const
	{
		return Particles;
	}

	UFUNCTION(BlueprintPure, Category = "Epsilon Physics|Collision")
	TArray<FEpsilonContactInfo> GetLastContacts() const
	{
		return LastContacts;
	}

	const TArray<FEpsilonContactInfo>& GetLastContactsView() const
	{
		return LastContacts;
	}

	UFUNCTION(BlueprintPure, Category = "Epsilon Physics")
	FString GetLastError() const
	{
		return LastError;
	}

	UFUNCTION(BlueprintPure, Category = "Epsilon Physics")
	double GetAlphaEff() const
	{
		return AlphaEff;
	}

	UFUNCTION(BlueprintPure, Category = "Epsilon Physics")
	double GetBeta() const
	{
		return Beta;
	}

	UFUNCTION(BlueprintPure, Category = "Epsilon Physics")
	double GetSofteningLength() const
	{
		return SofteningLength;
	}

private:
	bool ValidateParameters(double InAlphaEff, double InBeta, double InSofteningLength);
	bool ValidateConfiguredState() const;
	bool ValidateCollisionSettings(bool bInCollisionEnabled, double InProjectionPercent, double InProjectionSlop);
	bool ValidateCollisionState() const;
	bool ValidateParticles(const TArray<FEpsilonParticle>& InParticles, bool bRequireFiniteAcceleration);
	bool EvaluateAccelerations(
		const TArray<FEpsilonParticle>& InParticles,
		double InAlphaEff,
		double InBeta,
		double InSofteningLengthSquared,
		TArray<FVector3d>& OutAccelerations);
	bool ResolveCollisions(TArray<FEpsilonParticle>& InOutParticles, TArray<FEpsilonContactInfo>& OutContacts);

	bool RecordError(const FString& Message);
	void ClearError();
	bool ApplyAccelerationsToParticles(TArray<FEpsilonParticle>& InOutParticles, const TArray<FVector3d>& InAccelerations);

	UPROPERTY(VisibleAnywhere, Category = "Epsilon Physics")
	TArray<FEpsilonParticle> Particles;

	UPROPERTY(VisibleAnywhere, Category = "Epsilon Physics")
	double AlphaEff = 1.0;

	UPROPERTY(VisibleAnywhere, Category = "Epsilon Physics")
	double Beta = 0.0;

	UPROPERTY(VisibleAnywhere, Category = "Epsilon Physics")
	double SofteningLength = 0.0;

	UPROPERTY(VisibleAnywhere, Category = "Epsilon Physics")
	double SofteningLengthSquared = 0.0;

	UPROPERTY(VisibleAnywhere, Category = "Epsilon Physics")
	bool bIsConfigured = false;

	UPROPERTY(VisibleAnywhere, Category = "Epsilon Physics|Collision")
	bool bCollisionEnabled = true;

	UPROPERTY(VisibleAnywhere, Category = "Epsilon Physics|Collision")
	double CollisionProjectionPercent = 1.0;

	UPROPERTY(VisibleAnywhere, Category = "Epsilon Physics|Collision")
	double CollisionProjectionSlop = 0.0;

	UPROPERTY(VisibleAnywhere, Category = "Epsilon Physics|Collision")
	TArray<FEpsilonContactInfo> LastContacts;

	UPROPERTY(VisibleAnywhere, Category = "Epsilon Physics")
	FString LastError;
};
