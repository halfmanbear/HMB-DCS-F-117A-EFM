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

## Source layout

| Folder | Contents |
|---|---|
| `F_117_NightHawk.cpp`, `dllmain.cpp` | The `ed_fm_*` exports (ED callback glue) and the per-frame sequence. |
| `FlightModel/` | `State.h` (shared flight-model state), `Forces.h`, `AirData` (airspeed, Mach, dynamic pressure) and `ControlLaws` (control-law drive, mixer, actuators, autopilot holds). |
| `Aero/`, `Aerodynamics/`, `Atmosphere/` | Coefficient assembly, the component aero model and air data. |
| `Engine/`, `Systems/` | Engine model and start logic; fuel, engine indications, propulsion effectiveness and gear rules. |
| `FlightControls/` | Pitch (`Pitch.cpp`), roll (`Roll.cpp`) and yaw (`Yaw.cpp`) laws, tuning constants in `Config.h`, shared state in `Internal.h`, pitch debug CSV in `PitchLog`. |
| `Actuators/` | Surface, gear and bay servos. |
| `Commands/` | Pilot command handlers and the autopilot, A/T and PAARS updates. |
| `Weapons/` | CCRP release model and the bay, pickle and laser sequence. |
| `Damage/` | Component damage, fire events and the damage debug log. |
| `Effects/` | Leading-edge vortex visual effect. |
| `Cockpit/` | Named cockpit parameters. |
| `Avionics/` | Autothrottle, PAARS, JDAM envelope and the weapon interop with the DCS cockpit. |
| `Avionics/Interop/` | IRADS sensor injection: `Loader` (native exports), `SensorInjection` (start-up), `SensorFrame` (per-frame update), `InputHandler` (pilot input), `LockAndTerrain` (designation), `CameraBridge` (render hook), `Session`, `Log`; shared state in `IradsInternal.h`. |
| `Diagnostics/` | Saved Games paths, `LogFile` (the only C stdio user) and the engine debug CSV. |
| `include/` | ED SDK headers, unchanged. |

New source files are registered with `python tools/add_project_file.py "<filter>" <path>...`.

## Code style

The code follows the JSF AV C++ rules in `docs/JSF-AV-rules.pdf`; `docs/JSF-deviations.md` lists the rules it does not follow.

- Format with `.clang-format` (120 columns, no tabs, Allman braces). The SDK headers under `include/` are not formatted.
- Names follow AV 45-52: functions, variables, constants and enumerators are `lower_snake_case`, types and namespaces are `Capitalized_snake_case`. Exports (`ed_fm_*`, `DllMain`) and the SDK keep their own names. Private data members end with `_`.

## Checks

```powershell
python tests/run_all.py build/tests      # all regression tests; set F117_MOD_DIR for the four mod-dependent ones
python tools/check_jsf.py --files        # greppable JSF AV rules (casts, stdio, tabs, line length, ...)
pwsh tools/check_exports.ps1             # ed_fm_* exports against build/refactor-baseline/exports.txt
python tools/make_compile_commands.py    # then: clang-tidy -p build --header-filter="EFM Source Code" <file>  (naming check)
```

See `LICENSE` for the current licensing status and the retained historical notice.
