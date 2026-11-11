"""Report violations of the greppable JSF AV C++ rules in the project sources.

Usage: python tools/check_jsf.py [--strict]

Scans 'EFM Source Code' (the SDK headers under include/ are excluded) and prints one line per rule
with the violation count. Rules listed in docs/JSF-deviations.md as accepted deviations are reported
separately. With --strict the exit code is 1 when any adopted rule has violations.
Patterns are heuristics for triage; review each hit before changing code.
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / 'EFM Source Code'

# (rule, description, regex applied per line, adopted)
RULES = [
    ('AV 22', 'no stdio / printf family', re.compile(r'\b(printf|fprintf|sprintf|sprintf_s|snprintf|fopen|fputs|puts)\s*\(|<cstdio>|<stdio\.h>'), True),
    ('AV 24', 'no abort/exit/getenv/system', re.compile(r'\b(abort|exit|getenv|system)\s*\('), True),
    ('AV 29', 'no function-like macros', re.compile(r'^\s*#\s*define\s+\w+\('), True),
    ('AV 30', 'no #define constants', re.compile(r'^\s*#\s*define\s+\w+\s+[-0-9("]'), True),
    ('AV 41', 'lines at most 120 characters', re.compile(r'^.{121,}$'), True),
    ('AV 43', 'no tab characters', re.compile(r'\t'), True),
    ('AV 98', 'no using-directives', re.compile(r'^\s*using\s+namespace\b'), True),
    ('AV 126', 'only // comments', re.compile(r'/\*'), True),
    ('AV 152', 'one declaration per line', re.compile(r'^\s*(?:inline\s+|static\s+|constexpr\s+)*(?:double|float|int|bool|unsigned)\s+\w+(?:\{[^}]*\}|\s*=\s*[^,;(]+)?\s*,\s*\w+[^()]*;\s*$'), True),
    ('AV 153', 'no unions', re.compile(r'\bunion\b'), True),
    ('AV 185', 'no C-style casts', re.compile(r'(?<!sizeof)\((?:int|unsigned|float|double|char|size_t|void\s*\*|long)\)\s*[\w(&*]'), True),
    ('AV 189', 'no goto', re.compile(r'\bgoto\b'), True),
    ('AV 190', 'no continue', re.compile(r'\bcontinue\s*;'), True),
    ('AV 208', 'no C++ exceptions', re.compile(r'\b(throw|try|catch)\b(?!\s*\()'), True),
]

# Files that are accepted deviations for a rule (see docs/JSF-deviations.md).
ALLOWED = {
    'AV 22': ('Diagnostics/LogFile.cpp',),
}


def sources():
    for path in sorted(ROOT.rglob('*')):
        if path.suffix in ('.cpp', '.h') and 'include' not in path.relative_to(ROOT).parts:
            yield path


def main(argv):
    counts = {rule: 0 for rule, *_ in RULES}
    per_file = {}
    for path in sources():
        text = path.read_text(encoding='utf-8', errors='replace')
        relative = path.relative_to(ROOT).as_posix()
        for number, line in enumerate(text.splitlines(), 1):
            code = line.split('//', 1)[0] if 'AV 126' not in () else line
            for rule, _, pattern, _ in RULES:
                subject = line if rule in ('AV 41', 'AV 43', 'AV 126') else code
                if pattern.search(subject) and not relative.endswith(ALLOWED.get(rule, ())):
                    counts[rule] += 1
                    per_file.setdefault(rule, {}).setdefault(path.relative_to(ROOT).as_posix(), 0)
                    per_file[rule][path.relative_to(ROOT).as_posix()] += 1
    total = 0
    for rule, description, _, adopted in RULES:
        total += counts[rule] if adopted else 0
        print(f'{rule:7} {counts[rule]:5}  {description}')
        if counts[rule] and '--files' in argv:
            for name, n in sorted(per_file[rule].items(), key=lambda kv: -kv[1])[:5]:
                print(f'          {n:5}  {name}')
    print(f'\n{total} violations of adopted rules')
    return 1 if ('--strict' in argv and total) else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
