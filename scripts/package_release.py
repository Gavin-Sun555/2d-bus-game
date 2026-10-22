#!/usr/bin/env python3
"""Build and package one release target using the local Go toolchain."""
import argparse
import hashlib
import os
from pathlib import Path
import plistlib
import re
import shutil
import subprocess
import tempfile
import zipfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--os', required=True, choices=['darwin', 'windows', 'linux'])
    parser.add_argument('--arch', required=True, choices=['arm64', 'amd64'])
    parser.add_argument('--version', help='Release version (defaults to VERSION)')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    source_version = (root / 'VERSION').read_text().strip()
    args.version = args.version or source_version
    if args.version != source_version:
        parser.error('version must match VERSION so archive, binary and bundle agree')
    if not re.fullmatch(r'[0-9]+\.[0-9]+\.[0-9]+(?:-[A-Za-z0-9.-]+)?', args.version):
        parser.error('version must be a semantic version without the v prefix')
    dist = root / 'dist'
    dist.mkdir(exist_ok=True)
    name = f'busgame-{args.version}-{args.os}-{args.arch}'
    with tempfile.TemporaryDirectory(prefix='busgame-release-') as tmp:
        package = Path(tmp) / name
        package.mkdir()
        binary = package / ('busgame.exe' if args.os == 'windows' else 'busgame')
        env = {**os.environ, 'GOOS': args.os, 'GOARCH': args.arch}
        if args.os == 'windows':
            env['CGO_ENABLED'] = '0'
        subprocess.run(['go', 'build', '-trimpath', '-ldflags=-s -w', '-o', str(binary), '.'],
                       cwd=root, env=env, check=True)
        if args.os == 'darwin':
            bundle = package / '2D Bus Simulator.app' / 'Contents'
            (bundle / 'MacOS').mkdir(parents=True)
            (bundle / 'Resources').mkdir()
            shutil.move(str(binary), bundle / 'MacOS' / 'busgame')
            shutil.copy2(root / 'assets' / 'AppIcon.icns', bundle / 'Resources')
            with (bundle / 'Info.plist').open('wb') as output:
                plistlib.dump({
                    'CFBundlePackageType': 'APPL',
                    'CFBundleInfoDictionaryVersion': '6.0',
                    'CFBundleName': '2D Bus Simulator',
                    'CFBundleDisplayName': '2D Bus Simulator',
                    'CFBundleIdentifier': 'com.gavinsun.2dbussimulator',
                    'CFBundleVersion': args.version,
                    'CFBundleShortVersionString': args.version,
                    'CFBundleExecutable': 'busgame',
                    'CFBundleIconFile': 'AppIcon',
                    'NSHighResolutionCapable': True,
                }, output)
        shutil.copy2(root / 'VERSION', package)
        shutil.copy2(root / 'README.md', package)
        shutil.copy2(root / 'CHANGELOG.md', package)
        shutil.copy2(root / 'custom_map.json', package)
        shutil.copytree(root / 'docs', package / 'docs')
        with zipfile.ZipFile(dist / f'{name}.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
            for path in sorted(package.rglob('*')):
                if path.is_file():
                    archive.write(path, path.relative_to(package.parent))
    archive = dist / f'{name}.zip'
    checksum = hashlib.sha256(archive.read_bytes()).hexdigest()
    (dist / f'{name}.sha256').write_text(f'{checksum}  {archive.name}\n')
    print(f'Packaged {archive.name}: {archive.stat().st_size} bytes; SHA256 {checksum}')


if __name__ == '__main__':
    main()
