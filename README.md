# HMB F-117A external flight model

The F-117A EFM for DCS World: aircraft aerodynamics, propulsion, flight controls, automatic recovery, actuator motion, damage response, fuel accounting and native cockpit integration.

[Build instructions](EFM%20Source%20Code/README.md) describe the Visual Studio 2022 x64 DLL build.

The companion aircraft mod supplies the Lua cockpit, input profiles, models and flight performance bench. Its `docs/model-efm-compatibility.md` records which exterior models and damage mappings belong together. Push the EFM repository before a mod change that updates damage cells, because the mod's package workflow builds the latest EFM commit on the matching branch.

The repository's history includes the earlier CustomFM/F-16Demo-based implementation. Those revisions keep the licence files they were published with. The current licensing status is described in [EFM Source Code/LICENSE](EFM%20Source%20Code/LICENSE). SDK declarations remain ED material.
