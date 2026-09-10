#include "EpsilonNiagaraBridge.h"

#include "EpsilonPhysicsSolver.h"
#include "NiagaraComponent.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"

UEpsilonNiagaraBridge::UEpsilonNiagaraBridge()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UEpsilonNiagaraBridge::SetAttractors(const TArray<FVector4f>& InAttractors)
{
	if (InAttractors.Num() > MaximumAttractors)
	{
		ReportError(FString::Printf(TEXT("Niagara supports at most %d Epsilon attractors per bridge."), MaximumAttractors));
		return;
	}

	for (int32 Index = 0; Index < InAttractors.Num(); ++Index)
	{
		const FVector4f& Attractor = InAttractors[Index];
		if (!FMath::IsFinite(Attractor.X) || !FMath::IsFinite(Attractor.Y) ||
			!FMath::IsFinite(Attractor.Z) || !FMath::IsFinite(Attractor.W) || Attractor.W < 0.0f)
		{
			ReportError(FString::Printf(TEXT("Attractor %d must have finite position and non-negative mass."), Index));
			return;
		}
	}

	CachedAttractors = InAttractors;
	LastError.Reset();
	bReportedInvalidInputs = false;
}

void UEpsilonNiagaraBridge::TickComponent(
	const float DeltaTime,
	const ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (TargetNiagaraSystem == nullptr || !ValidateInputs())
	{
		return;
	}

	TargetNiagaraSystem->SetVariableFloat(TEXT("User.AlphaEff"), AlphaEff);
	TargetNiagaraSystem->SetVariableFloat(TEXT("User.Epsilon"), Epsilon);
	TargetNiagaraSystem->SetVariableFloat(TEXT("User.Beta"), Beta);

	TArray<FVector4> AttractorsForNiagara;
	AttractorsForNiagara.Reserve(CachedAttractors.Num());
	for (const FVector4f& Attractor : CachedAttractors)
	{
		AttractorsForNiagara.Add(FVector4(Attractor.X, Attractor.Y, Attractor.Z, Attractor.W));
	}
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector4(
		TargetNiagaraSystem,
		TEXT("User.AttractorArray"),
		AttractorsForNiagara);
}

bool UEpsilonNiagaraBridge::ValidateInputs()
{
	const bool bParametersAreValid = FMath::IsFinite(AlphaEff) && FMath::IsFinite(Epsilon) &&
		Epsilon > 0.0f && FMath::IsFinite(Beta) && Beta >= 0.0f;
	if (!bParametersAreValid)
	{
		if (!bReportedInvalidInputs)
		{
			ReportError(TEXT("Niagara Epsilon parameters require finite AlphaEff, Epsilon > 0, and Beta >= 0."));
			bReportedInvalidInputs = true;
		}
		return false;
	}

	bReportedInvalidInputs = false;
	return true;
}

void UEpsilonNiagaraBridge::ReportError(const FString& Error)
{
	LastError = Error;
	UE_LOG(LogEpsilonPhysics, Error, TEXT("%s"), *Error);
}
