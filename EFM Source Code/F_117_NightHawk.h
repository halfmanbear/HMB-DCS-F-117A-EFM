#pragma once

// Use ED's declarations directly so the exported ABI follows the bundled SDK.
#define EXPORT_ED_FM_PHYSICS_IMP extern "C" __declspec(dllexport)
#include "include/FM/wHumanCustomPhysicsAPI_ImplementationDeclare.h"
