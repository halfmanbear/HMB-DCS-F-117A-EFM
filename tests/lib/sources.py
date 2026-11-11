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
    """Return text from `signature` through the matching close brace."""
    start = text.index(signature)
    end = text.index(opener, start) + 1
    depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]
