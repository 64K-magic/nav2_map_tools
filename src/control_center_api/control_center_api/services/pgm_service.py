"""Discover, parse, and serve local PGM occupancy maps (+ YAML sidecar)."""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List

import yaml


@dataclass
class PgmMapInfo:
    name: str
    pgm_path: Path
    yaml_path: Path
    image: str
    mode: str
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    negate: int
    occupied_thresh: float
    free_thresh: float
    width: int
    height: int

    def to_dict(self) -> Dict[str, Any]:
        return {
            'name': self.name,
            'image': self.image,
            'mode': self.mode,
            'resolution': self.resolution,
            'origin': [self.origin_x, self.origin_y, self.origin_yaw],
            'negate': self.negate,
            'occupied_thresh': self.occupied_thresh,
            'free_thresh': self.free_thresh,
            'width': self.width,
            'height': self.height,
            'yaml_path': str(self.yaml_path),
            'pgm_path': str(self.pgm_path),
        }


def _read_token(data: bytes, idx: int) -> tuple[Optional[bytes], int]:
    while idx < len(data) and data[idx] in b' \t\r\n':
        idx += 1
    if idx >= len(data):
        return None, idx
    while idx < len(data) and data[idx] == ord('#'):
        while idx < len(data) and data[idx] != ord('\n'):
            idx += 1
        while idx < len(data) and data[idx] in b' \t\r\n':
            idx += 1
    start = idx
    while idx < len(data) and data[idx] not in b' \t\r\n':
        idx += 1
    if start == idx:
        return None, idx
    return data[start:idx], idx


def load_pgm(path: Path) -> tuple[int, int, bytes]:
    """Load P2/P5 PGM → (width, height, grayscale pixels row-major)."""
    raw = path.read_bytes()
    idx = 0
    while idx < len(raw) and raw[idx] in b' \t\r\n':
        idx += 1
    if idx + 1 >= len(raw) or raw[idx : idx + 2] not in (b'P2', b'P5'):
        raise ValueError(f'Not a PGM file: {path}')
    magic = raw[idx : idx + 2]
    idx += 2

    tok, idx = _read_token(raw, idx)
    if tok is None:
        raise ValueError(f'Invalid PGM header: {path}')
    width = int(tok)
    tok, idx = _read_token(raw, idx)
    if tok is None:
        raise ValueError(f'Invalid PGM header: {path}')
    height = int(tok)
    tok, idx = _read_token(raw, idx)
    if tok is None:
        raise ValueError(f'Invalid PGM header: {path}')
    maxval = int(tok)
    if idx < len(raw) and raw[idx] in b' \t\r\n':
        idx += 1

    if magic == b'P5':
        expected = width * height
        pixels = raw[idx : idx + expected]
        if len(pixels) < expected:
            raise ValueError(f'PGM data truncated: {path}')
        if maxval != 255:
            scale = 255.0 / maxval
            pixels = bytes(min(255, int(v * scale)) for v in pixels)
        return width, height, pixels

    # P2 ASCII
    values: List[int] = []
    while idx < len(raw) and len(values) < width * height:
        tok, idx = _read_token(raw, idx)
        if tok is None:
            break
        val = int(tok)
        if maxval != 255:
            val = int(val * 255.0 / maxval)
        values.append(min(255, max(0, val)))
    if len(values) < width * height:
        raise ValueError(f'PGM P2 data truncated: {path}')
    return width, height, bytes(values)


def pgm_to_png_bytes(width: int, height: int, pixels: bytes) -> bytes:
    """Encode grayscale pixels as PNG (stdlib only)."""

    def chunk(tag: bytes, data: bytes) -> bytes:
        body = tag + data
        crc = zlib.crc32(body) & 0xFFFFFFFF
        return struct.pack('>I', len(data)) + body + struct.pack('>I', crc)

    sig = b'\x89PNG\r\n\x1a\n'
    ihdr = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)
    raw_rows = b''.join(b'\x00' + pixels[y * width : (y + 1) * width] for y in range(height))
    idat = zlib.compress(raw_rows, 9)
    return sig + chunk(b'IHDR', ihdr) + chunk(b'IDAT', idat) + chunk(b'IEND', b'')


def parse_map_yaml(yaml_path: Path) -> Dict[str, Any]:
    with yaml_path.open('r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    origin = data.get('origin') or [0.0, 0.0, 0.0]
    if len(origin) < 2:
        origin = [0.0, 0.0, 0.0]
    return {
        'image': str(data.get('image') or ''),
        'mode': str(data.get('mode') or 'trinary'),
        'resolution': float(data.get('resolution') or 0.05),
        'origin_x': float(origin[0]),
        'origin_y': float(origin[1]),
        'origin_yaw': float(origin[2]) if len(origin) > 2 else 0.0,
        'negate': int(data.get('negate') or 0),
        'occupied_thresh': float(data.get('occupied_thresh') or 0.65),
        'free_thresh': float(data.get('free_thresh') or 0.196),
    }


def resolve_pgm_path(yaml_path: Path, image_field: str) -> Path:
    if image_field:
        candidate = yaml_path.parent / image_field
        if candidate.is_file():
            return candidate
    stem = yaml_path.stem
    for ext in ('.pgm', '.PGM'):
        candidate = yaml_path.with_suffix(ext)
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(f'PGM image not found for {yaml_path}')


class PgmService:
    def __init__(self, maps_dir: str | Path):
        self.maps_dir = Path(maps_dir).expanduser().resolve()

    def list_maps(self) -> List[PgmMapInfo]:
        if not self.maps_dir.is_dir():
            return []
        found: Dict[str, PgmMapInfo] = {}
        for yaml_path in sorted(self.maps_dir.glob('*.yaml')):
            try:
                info = self.load_map(yaml_path.stem)
                found[info.name] = info
            except (OSError, ValueError, yaml.YAMLError):
                continue
        for yaml_path in sorted(self.maps_dir.glob('*.yml')):
            name = yaml_path.stem
            if name in found:
                continue
            try:
                info = self.load_map(name)
                found[info.name] = info
            except (OSError, ValueError, yaml.YAMLError):
                continue
        return list(found.values())

    def load_map(self, name: str) -> PgmMapInfo:
        yaml_path = self.maps_dir / f'{name}.yaml'
        if not yaml_path.is_file():
            yaml_path = self.maps_dir / f'{name}.yml'
        if not yaml_path.is_file():
            raise FileNotFoundError(f'PGM map YAML not found: {name}')
        meta = parse_map_yaml(yaml_path)
        pgm_path = resolve_pgm_path(yaml_path, meta['image'])
        width, height, _ = load_pgm(pgm_path)
        return PgmMapInfo(
            name=name,
            pgm_path=pgm_path,
            yaml_path=yaml_path,
            image=meta['image'] or pgm_path.name,
            mode=meta['mode'],
            resolution=meta['resolution'],
            origin_x=meta['origin_x'],
            origin_y=meta['origin_y'],
            origin_yaw=meta['origin_yaw'],
            negate=meta['negate'],
            occupied_thresh=meta['occupied_thresh'],
            free_thresh=meta['free_thresh'],
            width=width,
            height=height,
        )

    def load_png(self, name: str) -> bytes:
        info = self.load_map(name)
        _, _, pixels = load_pgm(info.pgm_path)
        return pgm_to_png_bytes(info.width, info.height, pixels)
