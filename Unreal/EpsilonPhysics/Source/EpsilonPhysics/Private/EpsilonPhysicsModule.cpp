#include "Modules/ModuleManager.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "ShaderCore.h"

class FEpsilonPhysicsModule final : public IModuleInterface
{
public:
	virtual void StartupModule() override
	{
		const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("EpsilonPhysics"));
		if (Plugin.IsValid())
		{
			AddShaderSourceDirectoryMapping(
				TEXT("/Plugin/EpsilonPhysics"),
				FPaths::Combine(Plugin->GetBaseDir(), TEXT("Shaders")));
		}
	}
};

IMPLEMENT_MODULE(FEpsilonPhysicsModule, EpsilonPhysics);
