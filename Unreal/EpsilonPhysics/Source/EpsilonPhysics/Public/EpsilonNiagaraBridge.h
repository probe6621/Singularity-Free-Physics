#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "EpsilonNiagaraBridge.generated.h"

class UNiagaraComponent;

UCLASS(ClassGroup = (EpsilonPhysics), meta = (BlueprintSpawnableComponent))
class EPSILONPHYSICS_API UEpsilonNiagaraBridge final : public UActorComponent
{
	GENERATED_BODY()

public:
	UEpsilonNiagaraBridge();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Epsilon|Target")
	TObjectPtr<UNiagaraComponent> TargetNiagaraSystem;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Epsilon|Parameters")
	float AlphaEff = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Epsilon|Parameters", meta = (ClampMin = "0.000001"))
	float Epsilon = 15.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Epsilon|Parameters", meta = (ClampMin = "0.0"))
	float Beta = 100.0f;

	UFUNCTION(BlueprintCallable, Category = "Epsilon|Control")
	void SetAttractors(const TArray<FVector4f>& InAttractors);

	UFUNCTION(BlueprintPure, Category = "Epsilon|Control")
	FString GetLastError() const { return LastError; }

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	static constexpr int32 MaximumAttractors = 64;

	bool ValidateInputs();
	void ReportError(const FString& Error);

	TArray<FVector4f> CachedAttractors;
	FString LastError;
	bool bReportedInvalidInputs = false;
};
