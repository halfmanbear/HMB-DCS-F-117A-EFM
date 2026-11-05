"""Report inherited significant lines, separating SDK/schema facts from source.

Usage: python tests/standalone_provenance.py <DCS-API-directory> <output.json>
Uses the plan's fork point and git blame -w -M -C. This measures text origins;
it cannot certify independent authorship or detect a retyped implementation.
"""
from pathlib import Path
import json
import re
import subprocess
import sys

repo = Path(__file__).resolve().parents[1]
api = Path(sys.argv[1])
output = Path(sys.argv[2])
if not api.is_dir():
    raise SystemExit(f'ED API directory does not exist: {api}')


def git(*args):
    return subprocess.check_output(['git', '-C', str(repo), *args]).decode('utf-8', 'replace')


def norm(line):
    return re.sub(r'\s+', ' ', line.lstrip('\ufeff').strip())


trivial = re.compile(r'^(end|else|\}|\{|\};|\)|\),|\);|\},|//|#endif|#else|return;|break;|public:|private:|nil)$')


def significant(line):
    return len(line) >= 6 and not trivial.fullmatch(line)


sdk = set()
sdk_callbacks = set()
for p in api.rglob('*'):
    if not p.is_file() or p.suffix.lower() not in {'.h', '.cpp', '.vcxproj', '.filters', '.sln', '.txt'}:
        continue
    text = p.read_text(encoding='utf-8-sig', errors='replace')
    sdk.update(norm(line) for line in text.splitlines() if significant(norm(line)))
    sdk_callbacks.update(re.findall(r'\b(ed_fm_\w+)\s*\(', text))

ancestors = set(git('rev-list', '92f3bc9').split())
records = []
for name in git('ls-files').splitlines():
    p = repo / name
    if not p.is_file() or (p.suffix.lower() not in {'.h', '.cpp', '.py', '.lua', '.md', '.vcxproj', '.filters', '.sln', '.user', '.txt'} and p.name != 'LICENSE'):
        continue
    sha = None
    number = 0
    for row in git('blame', '-w', '-M', '-C', '--line-porcelain', '--', name).splitlines():
        match = re.match(r'^([0-9a-f]{40}) \d+ (\d+)', row)
        if match:
            sha, number = match[1], int(match[2])
        if not row.startswith('\t') or sha not in ancestors:
            continue
        line = norm(row[1:])
        if not significant(line) or line in sdk:
            continue
        category = 'source requiring review'
        callback = re.fullmatch(r'(?:void|double|bool|size_t)\s+(ed_fm_\w+)\s*\([^{};]*\)', line)
        if callback and callback[1] in sdk_callbacks:
            category = 'ED callback signature'
        elif p.suffix in {'.vcxproj', '.filters', '.sln', '.user'}:
            # Only inert schema/file-list/configuration tokens are excepted.
            # Any custom build commands or other inherited XML still require review.
            if (line.startswith(('<?xml ', '<ClInclude Include=', '<WindowsTargetPlatformVersion>',
                                 'Microsoft Visual Studio Solution File,', 'MinimumVisualStudioVersion ='))
                    or line == '<Project ToolsVersion="4.0" xmlns="http://schemas.microsoft.com/developer/msbuild/2003">'
                    or re.fullmatch(r'(Debug|Release)\|x86 = (Debug|Release)\|x86', line)):
                category = 'MSBuild/solution metadata'
        if name.startswith('docs/history/') or name == 'EFM Source Code/LICENSE':
            category = 'retained historical notice'
        records.append({'path': name, 'line': number, 'category': category, 'text': line})

counts = {category: sum(r['category'] == category for r in records)
          for category in sorted({r['category'] for r in records})}
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps({'fork': '92f3bc9', 'counts': counts, 'inherited_non_sdk_lines': records}, indent=2) + '\n')
print(json.dumps(counts, indent=2))
assert not any(r['category'] == 'source requiring review' for r in records), output
