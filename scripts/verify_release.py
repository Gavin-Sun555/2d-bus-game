#!/usr/bin/env python3
"""Verify packaged 0.0.x archives, executable architecture and SHA-256 files."""
import argparse
import hashlib
import json
from pathlib import Path
import plistlib
import struct
import zipfile


def verify(path, version):
    assert path.name.startswith(f'busgame-{version}-'), 'archive version mismatch'
    checksum_file = path.with_suffix('.sha256')
    digest, filename = checksum_file.read_text().strip().split()
    assert filename == path.name, 'checksum filename mismatch'
    assert hashlib.sha256(path.read_bytes()).hexdigest() == digest, 'checksum mismatch'
    with zipfile.ZipFile(path) as archive:
        assert archive.testzip() is None, 'ZIP integrity check failed'
        prefix = path.stem + '/'
        assert archive.read(prefix + 'VERSION').decode().strip() == version
        for name in ['README.md', 'custom_map.json', 'docs/architecture.md',
                     f'docs/releases/{version}.md', f'docs/releases/{version}-testing.md']:
            assert prefix + name in archive.namelist(), f'missing {name}'
        json.loads(archive.read(prefix + 'custom_map.json'))
        if '-darwin-' in path.name:
            bundle = prefix + '2D Bus Simulator.app/Contents/'
            info = plistlib.loads(archive.read(bundle + 'Info.plist'))
            assert info['CFBundleVersion'] == version
            assert info['CFBundleShortVersionString'] == version
            executable = bundle + 'MacOS/' + info['CFBundleExecutable']
            binary = archive.read(executable)
            magic, cpu = struct.unpack_from('<II', binary)
            assert magic == 0xFEEDFACF, 'not a 64-bit Mach-O'
            expected_cpu = 0x0100000C if '-arm64' in path.name else 0x01000007
            assert cpu == expected_cpu, 'wrong Mach-O architecture'
            assert (archive.getinfo(executable).external_attr >> 16) & 0o111, 'not executable'
            assert len(archive.read(bundle + 'Resources/AppIcon.icns')) > 8
        elif '-windows-' in path.name:
            binary = archive.read(prefix + 'busgame.exe')
            assert binary[:2] == b'MZ', 'not a Windows executable'
            pe_offset = struct.unpack_from('<I', binary, 0x3C)[0]
            assert binary[pe_offset:pe_offset + 4] == b'PE\0\0'
            expected_cpu = 0xAA64 if '-arm64' in path.name else 0x8664
            assert struct.unpack_from('<H', binary, pe_offset + 4)[0] == expected_cpu
        else:
            binary = archive.read(prefix + 'busgame')
            assert binary[:4] == b'\x7fELF', 'not a Linux executable'
            expected_cpu = 183 if '-arm64' in path.name else 62
            assert struct.unpack_from('<H', binary, 18)[0] == expected_cpu
    print(f'OK {path.name}: archive, architecture, metadata and SHA-256')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archives', nargs='+', type=Path)
    args = parser.parse_args()
    version = (Path(__file__).resolve().parents[1] / 'VERSION').read_text().strip()
    for path in args.archives:
        verify(path, version)


if __name__ == '__main__':
    main()
