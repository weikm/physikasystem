#pragma once
#ifdef PHYSIKASYSTEM_EXPORTS
#define PKA_PHYSICS_CLASS __declspec(dllexport)
#define PKA_PHYSICS_API extern "C" __declspec(dllexport)
#else
#define PKA_PHYSICS_CLASS __declspec(dllimport)
#define PKA_PHYSICS_API extern "C" __declspec(dllimport)
#endif