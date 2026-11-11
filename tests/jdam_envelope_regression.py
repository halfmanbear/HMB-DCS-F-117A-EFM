"""Checks that JdamEnvelope.h still matches the stock DCS JDAM definitions.

Usage: jdam_envelope_regression.py [path to AircraftWeaponPack/JDAM.lua]
Skips when DCS is not installed. A failure usually means a DCS update changed
the stock release ranges, so the header needs copying again. Also checks the
JSOW (AGM-154A/B) tables against glide_bombs.lua.
"""
from pathlib import Path
import re
import sys

DEFAULT = r'E:\Eagle Dynamics\DCS World\CoreMods\aircraft\AircraftWeaponPack\JDAM.lua'
HEADER = Path(__file__).resolve().parents[1] / 'EFM Source Code/Avionics/JdamEnvelope.h'
PROFILES = [('GBU_31_V_1B', 'GBU_31_V_2B'), ('GBU_31_V_3B', 'GBU_31_V_4B'), ('GBU_32_V_2B', 'bombs_data')]
HEIGHTS, SPEEDS = 21, 10
NUMBER = r'-?\d+(?:\.\d+)?'


def stock_table(block, name, heights=HEIGHTS):
    match = re.search(r'\b' + name + r'\s*=\s*\{(.*?)\}', block, re.S)
    assert match, f'{name} not found in stock definition'
    values = [float(v) for v in re.findall(NUMBER, re.sub(r'--[^\n]*', '', match.group(1)))]
    assert values[:2] == [heights, SPEEDS], f'{name} grid is {values[:2]}'
    rows = values[2 + SPEEDS:]
    assert len(rows) == heights * (SPEEDS + 1), f'{name} has {len(rows)} values'
    grid = [rows[i + 1:i + 1 + SPEEDS] for i in range(0, len(rows), SPEEDS + 1)]
    return values[2:2 + SPEEDS], rows[::SPEEDS + 1], grid


def header_array(text, declaration, start=0):
    match = re.compile(re.escape(declaration) + r'\s*=\s*\{(.*?)\};', re.S).search(text, start)
    assert match, f'{declaration} not found in header'
    return [float(v) for v in re.findall(NUMBER, match.group(1))]


def main():
    path = Path(sys.argv[1] if len(sys.argv) > 1 else DEFAULT)
    if not path.exists():
        print(f'SKIP: {path} not found')
        return
    stock = path.read_text(encoding='latin-1')
    header = HEADER.read_text(encoding='utf-8')
    maximum = header_array(header, f'maximum[3][{HEIGHTS}][{SPEEDS}]')
    minimum = header_array(header, f'minimum[3][{HEIGHTS}][{SPEEDS}]')
    speeds = header_array(header, f'speeds[{SPEEDS}]')
    heights = header_array(header, f'heights[{HEIGHTS}]')
    size = HEIGHTS * SPEEDS
    for profile, (start, end) in enumerate(PROFILES):
        first = re.search(r'^' + start + r'\s*=', stock, re.M)
        last = re.search(r'^' + end + r'\s*=', stock, re.M)
        assert first and last, f'{start} block not found'
        block = stock[first.start():last.start()]
        for name, values in (('LaunchDistData', maximum), ('MinLaunchDistData', minimum)):
            stock_speeds, stock_heights, grid = stock_table(block, name)
            assert stock_speeds == speeds, f'{start} {name} speeds differ'
            assert stock_heights == heights, f'{start} {name} heights differ'
            expected = [v for row in grid for v in row]
            actual = values[profile * size:(profile + 1) * size]
            assert actual == expected, f'{start} {name} differs from JdamEnvelope.h'
    print('PASS: JdamEnvelope.h matches stock min/max release ranges for all three JDAMs')
    check_glide(path.with_name('glide_bombs.lua'), header)


# AGM-154A and AGM-154B blocks in glide_bombs.lua; the AGM-154C uses table 0.
GLIDE_PROFILES = [('AGM_154A', 'AGM_154B'), ('AGM_154B', 'JSOW_data')]
GLIDE_HEIGHTS = 18


def check_glide(path, header):
    stock = path.read_text(encoding='latin-1')
    start = header.index('namespace Glide_envelope')
    maximum = header_array(header, f'maximum[2][{GLIDE_HEIGHTS}][{SPEEDS}]', start)
    minimum = header_array(header, f'minimum[2][{GLIDE_HEIGHTS}][{SPEEDS}]', start)
    speeds = header_array(header, f'speeds[{SPEEDS}]', start)
    heights = header_array(header, f'heights[{GLIDE_HEIGHTS}]', start)
    size = GLIDE_HEIGHTS * SPEEDS
    for table, (first_name, last_name) in enumerate(GLIDE_PROFILES):
        first = re.search(r'^' + first_name + r'\s*=', stock, re.M)
        last = re.search(r'^' + last_name + r'\s*=', stock, re.M)
        assert first and last, f'{first_name} block not found'
        block = stock[first.start():last.start()]
        for name, values in (('LaunchDistData', maximum), ('MinLaunchDistData', minimum)):
            stock_speeds, stock_heights, grid = stock_table(block, name, GLIDE_HEIGHTS)
            assert stock_speeds == speeds, f'{first_name} {name} speeds differ'
            assert stock_heights == heights, f'{first_name} {name} heights differ'
            assert values[table * size:(table + 1) * size] == [v for row in grid for v in row],                 f'{first_name} {name} differs from Glide_envelope'
    print('PASS: Glide_envelope matches stock min/max release ranges for the AGM-154A and AGM-154B')


if __name__ == '__main__':
    main()
