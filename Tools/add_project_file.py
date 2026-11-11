"""Register source files in F_117_NightHawk.vcxproj and its .filters file.

Usage: python tools/add_project_file.py "<filter>" <relative path> [<relative path> ...]
Paths are relative to 'EFM Source Code'. .cpp files become ClCompile, .h files ClInclude.
Already-listed files are skipped; a missing filter is created with a fresh GUID.
"""
import sys
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / 'EFM Source Code'
PROJ = ROOT / 'F_117_NightHawk.vcxproj'
FILT = ROOT / 'F_117_NightHawk.vcxproj.filters'


def read(path):
    raw = path.read_bytes()
    bom = raw.startswith(b'\xef\xbb\xbf')
    text = raw[3:].decode('utf-8') if bom else raw.decode('utf-8')
    return text, bom, '\r\n' if '\r\n' in text else '\n'


def write(path, text, bom):
    path.write_bytes((b'\xef\xbb\xbf' if bom else b'') + text.encode('utf-8'))


def main(argv):
    filt_name, files = argv[0], argv[1:]
    proj, pbom, pnl = read(PROJ)
    filt, fbom, fnl = read(FILT)
    for rel in files:
        tag = 'ClCompile' if rel.endswith('.cpp') else 'ClInclude'
        if f'<{tag} Include="{rel}"' in proj or f'<{tag} Include="{rel.replace("/", chr(92))}"' in proj:
            continue
        marker = f'    <{tag} Include='
        last = proj.rindex(marker)
        eol = proj.index('\n', last) + 1
        proj = proj[:eol] + f'    <{tag} Include="{rel}" />{pnl}' + proj[eol:]
        last = filt.rindex(marker)
        eol = filt.index('\n', last) + 1
        filt = filt[:eol] + f'    <{tag} Include="{rel}"><Filter>{filt_name}</Filter></{tag}>{fnl}' + filt[eol:]
    if f'<Filter Include="{filt_name}">' not in filt:
        last = filt.rindex('    <Filter Include=')
        eol = filt.index('\n', last) + 1
        guid = '{' + str(uuid.uuid4()).upper() + '}'
        filt = filt[:eol] + f'    <Filter Include="{filt_name}"><UniqueIdentifier>{guid}</UniqueIdentifier></Filter>{fnl}' + filt[eol:]
    write(PROJ, proj, pbom)
    write(FILT, filt, fbom)


if __name__ == '__main__':
    main(sys.argv[1:])
