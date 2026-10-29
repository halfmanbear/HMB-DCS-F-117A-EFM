#!/usr/bin/env python3
"""
Symmetrize _CzData and _CmData tables in AeroData_PrimaryTables.h.

For a symmetric aircraft, both Cz and Cm must be EVEN functions of sideslip beta:
  Cz(alpha, +beta, ele) == Cz(alpha, -beta, ele)
  Cm(alpha, +beta, ele) == Cm(alpha, -beta, ele)

Data layout (column-major, matching getLinIndex in UtilityFunctions.h):
  index = alpha_idx + beta_idx * 20 + ele_idx * 380

  alpha1[20] = {-20,-15,-10,-5, 0, 5,10,15,20,25,30,35,40,45,50,55,60,70,80,90}
  beta1[19]  = {-30,-25,-20,-15,-10,-8,-6,-4,-2, 0, 2, 4, 6, 8,10,15,20,25,30}
  dh1[5]     = {-25,-10, 0, 10, 25}

Beta symmetric pairs (i <-> 18-i):
  (0,18): -30 <-> +30
  (1,17): -25 <-> +25
  (2,16): -20 <-> +20
  (3,15): -15 <-> +15
  (4,14): -10 <-> +10
  (5,13):  -8 <-> +8
  (6,12):  -6 <-> +6
  (7,11):  -4 <-> +4
  (8,10):  -2 <-> +2
  (9):      0  (centre, unchanged)
"""

import re
import sys
import os

HEADER_FILE = os.path.join(os.path.dirname(__file__), 'AeroData_PrimaryTables.h')

N_ALPHA = 20
N_BETA  = 19
N_ELE   = 5
N_TOTAL = N_ALPHA * N_BETA * N_ELE   # 1900


def idx(alpha_i, beta_i, ele_i):
    return alpha_i + beta_i * N_ALPHA + ele_i * N_ALPHA * N_BETA


def find_array_in_lines(lines, name):
    """Return (start_line, end_line) for a static double array, 0-indexed."""
    start = None
    for i, line in enumerate(lines):
        if f'static double {name}[{N_TOTAL}]' in line:
            start = i
        if start is not None and i > start and '};' in line:
            return start, i
    raise ValueError(f'Could not find {name} in file')


def parse_values(lines, start, end):
    """Parse float values from lines[start+1 .. end-1]."""
    text = ' '.join(lines[start + 1:end])
    tokens = re.split(r'[\s,]+', text)
    values = []
    for t in tokens:
        t = t.strip()
        if t:
            values.append(float(t))
    if len(values) != N_TOTAL:
        raise ValueError(f'Expected {N_TOTAL} values, got {len(values)}')
    return values


def symmetrize_even(data):
    """Return a new list with even beta symmetry applied (full strength)."""
    result = list(data)
    for e in range(N_ELE):
        for b in range(9):        # pairs (0,18)..(8,10)
            b_mirror = 18 - b
            for a in range(N_ALPHA):
                i_neg = idx(a, b, e)
                i_pos = idx(a, b_mirror, e)
                avg = (data[i_neg] + data[i_pos]) / 2.0
                result[i_neg] = avg
                result[i_pos] = avg
    return result


def report_asymmetry(data, name):
    pairs = [(b, 18 - b) for b in range(9)]
    diffs = []
    for e in range(N_ELE):
        for b_neg, b_pos in pairs:
            for a in range(N_ALPHA):
                d = abs(data[idx(a, b_neg, e)] - data[idx(a, b_pos, e)])
                diffs.append(d)
    max_d = max(diffs)
    mean_d = sum(diffs) / len(diffs)
    print(f'  {name}: max asymmetry = {max_d:.6f}, mean = {mean_d:.6f}')


def format_values(name, values):
    """Format values as C static array declaration, 5 per line."""
    lines = [f'static double {name}[{N_TOTAL}] = {{']
    i = 0
    while i < N_TOTAL:
        # Each (ele, beta) group = N_ALPHA = 20 values = 4 lines of 5
        group_end = i + N_ALPHA
        j = i
        while j < group_end:
            chunk = values[j:j + 5]
            is_last = (j + 5 >= N_TOTAL)
            formatted = ', '.join(f'{v:.6f}' for v in chunk)
            if not is_last:
                formatted += ','
            lines.append(f'    {formatted}')
            j += 5
        i += N_ALPHA
    lines.append('};')
    return lines


def main():
    with open(HEADER_FILE, 'r') as f:
        file_lines = f.read().splitlines()

    # Locate arrays
    cz_start, cz_end = find_array_in_lines(file_lines, '_CzData')
    cm_start, cm_end = find_array_in_lines(file_lines, '_CmData')
    print(f'_CzData: file lines {cz_start + 1}–{cz_end + 1}')
    print(f'_CmData: file lines {cm_start + 1}–{cm_end + 1}')

    # Parse
    cz = parse_values(file_lines, cz_start, cz_end)
    cm = parse_values(file_lines, cm_start, cm_end)

    # Asymmetry before
    print('\nAsymmetry BEFORE correction:')
    report_asymmetry(cz, '_CzData')
    report_asymmetry(cm, '_CmData')

    # Symmetrize
    cz_sym = symmetrize_even(cz)
    cm_sym = symmetrize_even(cm)

    # Asymmetry after
    print('\nAsymmetry AFTER correction:')
    report_asymmetry(cz_sym, '_CzData')
    report_asymmetry(cm_sym, '_CmData')

    # Rebuild file
    new_cz_lines = format_values('_CzData', cz_sym)
    new_cm_lines = format_values('_CmData', cm_sym)

    out_lines = []
    i = 0
    while i < len(file_lines):
        if i == cz_start:
            out_lines.extend(new_cz_lines)
            i = cz_end + 1          # skip original block
        elif i == cm_start:
            out_lines.extend(new_cm_lines)
            i = cm_end + 1
        else:
            out_lines.append(file_lines[i])
            i += 1

    output = '\n'.join(out_lines) + '\n'
    with open(HEADER_FILE, 'w') as f:
        f.write(output)

    print(f'\nWrote corrected file: {HEADER_FILE}')


if __name__ == '__main__':
    main()
