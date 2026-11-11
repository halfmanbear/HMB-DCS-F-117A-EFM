"""Write build/compile_commands.json for clang-tidy from the project's source list and build settings.

Usage: python tools/make_compile_commands.py

The MSBuild project is the source of truth; this mirrors its language standard, include paths and
preprocessor definitions for clang-cl so tools such as clang-tidy can parse each translation unit.
"""
import json
import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
ROOT = REPO / 'EFM Source Code'


def main():
    project = (ROOT / 'F_117_NightHawk.vcxproj').read_text(encoding='utf-8-sig')
    sources = sorted(set(re.findall(r'<ClCompile Include="([^"]+)"', project)))
    arguments = ['clang-cl.exe', '--driver-mode=cl', '/std:c++17', '/EHsc',
                 '/DWIN32', '/D_WINDOWS', '/D_USRDLL', '/DED_FM_TEMPLATE_EXPORTS',
                 '/DNOMINMAX', '/DNDEBUG', '/D_CRT_USE_BUILTIN_OFFSETOF',
                 f'/I{ROOT}', f'/I{ROOT / "include"}', f'/I{ROOT / "Avionics"}', '-Wno-everything']
    database = []
    for name in sources:
        path = (ROOT / name.replace('\\', '/')).resolve()
        database.append({'directory': str(ROOT), 'file': str(path), 'arguments': arguments + ['/c', str(path)]})
    out = REPO / 'build' / 'compile_commands.json'
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(database, indent=1), encoding='utf-8')
    print(f'{len(database)} translation units -> {out}')


if __name__ == '__main__':
    main()
