# F-117A external flight model

This DLL implements the DCS external flight model callbacks declared by the bundled ED SDK. It contains the aircraft's component aerodynamic model, installed engine model, flight controls, actuators, fuel accounting and cockpit integration.

Build in Developer PowerShell for Visual Studio 2022, with its C++ desktop workload and a Windows 10 SDK:

```powershell
msbuild F117.sln /p:Configuration=Release /p:Platform=x64
```

The DLL is written to `bin/F_117_NightHawk.dll`. Intermediate files go to the repository's ignored `build/obj/` directory. Debug x64 is also supported. The project is x64 only, as DCS is.

`OutDir` and `IntDir` accept explicit MSBuild overrides. Release x64 can instead target a mod directory with `DcsSavedGamesModBinDir`, or derive that directory from an explicitly supplied `DcsSavedGamesRoot`. Normal builds use the local `bin` directory. The mod's package workflow can continue supplying its isolated output paths.

The project imports ED's callback declarations directly through `F_117_NightHawk.h`. SDK files under `include/` retain their original interfaces. `Inputs.h` defines stable integer command IDs: stock DCS bindings plus the custom protocol in the aircraft mod's `Cockpit/Scripts/command_defs.lua`.

From the repository root, run the production adapter regression:

```powershell
python tests/efm_adapters_regression.py build/tests
python tests/simulation_state_regression.py build/tests
```

The flight performance bench lives in the companion aircraft mod repository at `tools/fm_bench`. Pass `--dll` to its `scorecard.py` to check a particular build.

See `LICENSE` for the current licensing status and the retained historical notice.
