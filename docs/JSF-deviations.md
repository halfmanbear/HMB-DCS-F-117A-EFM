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

Status: the structural refactor, the rule check script and this table are in place. Remaining
violations are tracked by `tools/check_jsf.py`; the rename to AV 45-52 naming and the cast, tab,
line-length and one-declaration-per-line cleanup are still to do.
