#!/usr/bin/env python3
"""Refuse publication when the version's qualification report is on hold."""
import json
from pathlib import Path
import sys


def verify(root):
    version = (root / 'VERSION').read_text().strip()
    report = root / 'docs' / 'releases' / f'{version}-qualification.json'
    data = json.loads(report.read_text())
    if data.get('version') != version:
        raise ValueError('qualification version does not match VERSION')
    decision = data.get('decision')
    if decision not in {'PASS', 'LOCAL_PASS_PENDING_PLATFORM_CI'}:
        raise ValueError(f'{version} is not qualified for publication: {decision}')
    return decision


if __name__ == '__main__':
    try:
        decision = verify(Path(__file__).resolve().parents[1])
    except (OSError, ValueError) as error:
        sys.exit(str(error))
    print(f'Release qualification accepted: {decision}')
