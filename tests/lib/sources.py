"""Shared source loading for the regression tests.

Tests extract function bodies and constants from the production sources by
text. Loading every project source as one blob means a test keeps working when
code moves between translation units.
"""
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SRC = REPO / 'EFM Source Code'
_SUFFIXES = {'.cpp', '.h'}


def project_files(*subdirs):
    """Project .cpp/.h files (SDK headers under include/ excluded), stable order.

    With no arguments every project file is returned; otherwise only files under
    the given subdirectories of 'EFM Source Code' (a top-level file name also works).
    """
    roots = [SRC / s for s in subdirs] if subdirs else [SRC]
    found = []
    for root in roots:
        candidates = [root] if root.is_file() else sorted(root.rglob('*'))
        for path in candidates:
            if path.suffix in _SUFFIXES and 'include' not in path.relative_to(SRC).parts:
                found.append(path)
    return sorted(set(found), key=lambda p: (p.parent != SRC, p.as_posix()))


def load_source(*subdirs):
    """Concatenate project files into one text blob (BOM stripped, LF newlines)."""
    parts = []
    for path in project_files(*subdirs):
        parts.append(path.read_text(encoding='utf-8-sig', errors='replace').replace('\r\n', '\n'))
    return '\n'.join(parts)


def extract(text, signature, opener='{'):
    """Return text from `signature` through the matching close brace.

    A forward declaration (the signature followed by ';' before any '{') is skipped,
    so the definition is found even when a header declares the same function.
    """
    pos = 0
    while True:
        start = text.index(signature, pos)
        brace = text.index(opener, start)
        semicolon = text.find(';', start)
        if semicolon == -1 or semicolon > brace:
            break
        pos = start + 1
    end = brace + 1
    depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


def unit_body(*subdirs, keep_namespace=True):
    """Source of the given files without #include lines (and optionally without the
    outer 'namespace X { ... }' wrapper), for compiling a whole unit inside a test harness."""
    import re
    lines = load_source(*subdirs).splitlines()
    text = '\n'.join(line for line in lines if not line.startswith('#include'))
    if not keep_namespace:
        text = re.sub(r'^namespace \w+\s*\{', '', text, count=1, flags=re.M)
        text = re.sub(r'^\} // namespace \w+', '', text, count=1, flags=re.M)
    return text
