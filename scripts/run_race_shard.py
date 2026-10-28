#!/usr/bin/env python3
"""Run a deterministic partition of the complete Go test inventory under race.

Partition only top-level tests; all their subtests remain together. Each package
runs in a separate process with its ordinary TestMain and test ordering.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys


def selected_tests(package, names, index, count):
    if count < 1 or not 0 <= index < count:
        raise ValueError('invalid shard index/count')
    return [name for name in names
            if int.from_bytes(hashlib.sha256(f'{package}:{name}'.encode()).digest()[:8], 'big') % count == index]


def discover():
    packages = subprocess.check_output(['go', 'list', './...'], text=True).splitlines()
    inventory = []
    for package in packages:
        listing = subprocess.check_output(['go', 'test', '-list', '.', package], text=True)
        names = [line for line in listing.splitlines()
                 if re.fullmatch(r'(?:Test|Example|Fuzz)[A-Za-z0-9_]*', line)]
        if len(names) != len(set(names)):
            raise ValueError(f'duplicate test names in {package}')
        inventory.append({'package': package, 'tests': names})
    return inventory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--index', type=int, required=True)
    parser.add_argument('--count', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plan-only', action='store_true')
    args = parser.parse_args()
    selected_tests('', [], args.index, args.count)  # validate even an empty inventory
    args.output.mkdir(parents=True, exist_ok=True)
    plan = []
    for item in discover():
        names = item['tests']
        selected = selected_tests(item['package'], names, args.index, args.count)
        plan.append({**item, 'selected': selected,
                     'run': bool(selected) or (not names and args.index == 0)})
    (args.output / 'manifest.json').write_text(json.dumps({
        'index': args.index, 'count': args.count, 'packages': plan}, indent=2))
    print(f'Shard {args.index}/{args.count}: {sum(len(p["selected"]) for p in plan)} top-level tests', flush=True)
    if args.plan_only:
        return 0
    status = {}
    for number, item in enumerate(plan):
        if not item['run']:
            continue
        pattern = '^(' + '|'.join(re.escape(n) for n in item['selected']) + ')$' if item['selected'] else '^$'
        log = args.output / f'package-{number}.json'
        with log.open('w') as output:
            proc = subprocess.run(['go', 'test', '-race', '-json', '-count=1', '-timeout=60m',
                                   '-run', pattern, item['package']], stdout=output, stderr=subprocess.STDOUT)
        status[item['package']] = proc.returncode
        print(f'{item["package"]}: exit {proc.returncode} ({len(item["selected"])} tests)', flush=True)
        if proc.returncode:
            print('\n'.join(log.read_text(errors='replace').splitlines()[-80:]), flush=True)
    (args.output / 'status.json').write_text(json.dumps(status, indent=2))
    return int(any(status.values()))


if __name__ == '__main__':
    sys.exit(main())
