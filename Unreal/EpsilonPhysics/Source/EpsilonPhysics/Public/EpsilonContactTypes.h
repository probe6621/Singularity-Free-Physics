#pragma once

#include "CoreMinimal.h"

#include "EpsilonContactTypes.generated.h"

USTRUCT(BlueprintType)
struct EPSILONPHYSICS_API FEpsilonContactInfo
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Epsilon Physics|Collision")
	int32 ParticleAIndex = INDEX_NONE;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Epsilon Physics|Collision")
	int32 ParticleBIndex = INDEX_NONE;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Epsilon Physics|Collision")
	FVector3d Normal = FVector3d(1.0, 0.0, 0.0);

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Epsilon Physics|Collision")
	FVector3d ContactPoint = FVector3d::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Epsilon Physics|Collision", meta = (ClampMin = "0.0"))
	double PenetrationDepth = 0.0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Epsilon Physics|Collision", meta = (ClampMin = "0.0"))
	double NormalImpulse = 0.0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Epsilon Physics|Collision", meta = (ClampMin = "0.0"))
	double TangentImpulse = 0.0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Epsilon Physics|Collision", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	double CombinedRestitution = 0.0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Epsilon Physics|Collision", meta = (ClampMin = "0.0"))
	double CombinedFriction = 0.0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Epsilon Physics|Collision", meta = (ClampMin = "0.0"))
	double PreNormalRelativeSpeed = 0.0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Epsilon Physics|Collision", meta = (ClampMin = "0.0"))
	double PostNormalRelativeSpeed = 0.0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Epsilon Physics|Collision", meta = (ClampMin = "0.0"))
	double NormalEnergyDissipated = 0.0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Epsilon Physics|Collision")
	bool bAppliedImpulse = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Epsilon Physics|Collision")
	bool bUsedFallbackNormal = false;
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FEpsilonContactResolvedSignature, const FEpsilonContactInfo&, ContactInfo);
