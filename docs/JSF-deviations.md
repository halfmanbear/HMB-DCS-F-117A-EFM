# JSF AV rule deviations

The EFM follows the JSF AV C++ Coding Standards (`docs/JSF-AV-rules.pdf`) except where the rule
cannot be met in a DLL that implements a C SDK, builds with MSVC and runs inside DCS. Each deviation
below names the rule, where it applies and why. `tools/check_jsf.py` reports the rules that can be
checked mechanically; run `python tools/check_jsf.py --files` to see the current counts.

| Rule | Deviation | Scope | Reason |
|---|---|---|---|
| AV 26, 27, 31 | `#pragma once` instead of `#ifndef` guards; `#pragma comment(lib)` | all headers, `FlightControls.cpp` | MSVC convention; the guard macro adds nothing. |
| AV 175 | `nullptr` instead of `0` | all code | `nullptr` is the C++11 replacement for the rule's intent. |
| AV 209 | `double`, `int`, `float` used directly | all code | The ED SDK is declared in these types; wrapping them adds conversions without safety. |
| AV 163 | Unsigned arithmetic allowed | container sizes and indices | `size_t` is what the standard library and SDK use. |
| AV 96, 97 | C arrays in SDK-facing interfaces | `ed_fm_*` callbacks, draw args | The SDK passes `double*`, `float*` and fixed arrays. |
| AV 108 | Variadic functions | `Log`, `vsnprintf` wrapper | The log helpers forward to `vsnprintf`. |
| AV 113 | Early returns | guard clauses | Single exit would nest the body and hide the guard. |
| AV 130-132 | Comment density | all code | Comments state units, sources and reasons, not every line. |
| AV 16 | Certified libraries | all code | Not applicable to a game mod. |
| AV 208 | SEH `__try/__except` | `Weapons_Interop.cpp` | Guards native calls into DCS; no C++ exceptions are used. |
| AV 206 | Allocation during start-up | cockpit interop | Camera bridge and sensor storage are allocated once and never freed. |
| AV 49 against 51 | Lowercase acronyms | all code | The two rules conflict; AV 51 wins. |
| AV 98 | Exported `ed_fm_*` and `DllMain` stay global | `F_117_NightHawk.cpp`, `dllmain.cpp` | DCS resolves them by name. |
| AV 22, 23 | C stdio (`fopen_s`, `vfprintf`, `vsnprintf`) | `Diagnostics/LogFile.cpp` only | One small `LogFile` class wraps it; no other file includes `<cstdio>`. `tools/check_jsf.py` allows this file. |
| AV 24 | Win32 `GetEnvironmentVariableA` | `Diagnostics/Paths.cpp` | The `F117_LOG_ROOT` override and the profile folder. `getenv` is not used. |
| AV 45-52 | Names set by DCS or the SDK keep their spelling | `ed_fm_*`, `DllMain`, `include/` | They are resolved by name or declared by the SDK. |
| AV 110 | More than 7 parameters | `ed_fm_set_atmosphere`, `ed_fm_set_current_state`, `ed_fm_set_current_state_body_axis` | The DCS ABI fixes these signatures. |
| AV 45 | Private data members end with `_` | `Avionics/Autothrottle.h`, `Engine/EngineStart.h`, `FlightControls/TustinFilter.h` | Separates a member from its accessor of the same name. `.clang-tidy` ignores the suffix. |
| AV 50 | `F117` namespace keeps its designation | all code | A digit-bearing designation, not a word. |
| AV 206 | `new void*[]` copy of the indicator vtable | `Avionics/Interop/CameraBridge.cpp` | Allocated once when the camera hook is installed and never freed; the indicator keeps pointing at it. |

Status: the structural split, the rule check script, the formatting config and the AV 45-52 renames
are done. `python tools/check_jsf.py` reports no violations of the adopted rules, `clang-tidy`
(`EFM Source Code/.clang-tidy`) reports no naming violations, and no function exceeds 100 lines. The
only functions over 7 parameters are `ed_fm_set_atmosphere`, `ed_fm_set_current_state` and
`ed_fm_set_current_state_body_axis`, whose signatures the DCS ABI fixes (AV 110).

Not done: AV 135-142 ask for each global to have a single owner behind accessors. The shared
flight-model state is kept as `inline` variables in `FlightModel/State.h`, grouped by subsystem,
and the per-unit state is file-local. Replacing the shared variables with accessor objects would touch
nearly every call site and was left out of this refactor.
