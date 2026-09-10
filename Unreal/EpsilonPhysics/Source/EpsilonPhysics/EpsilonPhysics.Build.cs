using UnrealBuildTool;

public class EpsilonPhysics : ModuleRules
{
	public EpsilonPhysics(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(
			new[]
			{
				"Core",
				"CoreUObject",
				"Engine"
			});
	}
}
