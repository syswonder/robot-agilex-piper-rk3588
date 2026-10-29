#!/usr/bin/env python3
"""Import every generated protobuf module as a post-codegen smoke test."""

from __future__ import annotations

import importlib
from pathlib import Path
import sys


def generated_modules(codegen_root: Path) -> list[tuple[str, Path]]:
    modules: list[tuple[str, Path]] = []
    roots = [
        codegen_root / root_name
        for root_name in ("proto_gen", "robonix_mcp_types")
        if (codegen_root / root_name).is_dir()
    ]
    sys.path[:0] = [str(root) for root in roots if str(root) not in sys.path]
    for root in roots:
        for path in sorted(root.rglob("*_pb2.py")) + sorted(root.rglob("*_pb2_grpc.py")):
            module = ".".join(path.relative_to(root).with_suffix("").parts)
            modules.append((module, path))
    return modules


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit(f"usage: {sys.argv[0]} CODEGEN_ROOT")
    modules = generated_modules(Path(sys.argv[1]).resolve())
    if not modules:
        raise SystemExit("no generated *_pb2.py or *_pb2_grpc.py modules found")
    for module, path in modules:
        imported = importlib.import_module(module)
        if Path(imported.__file__).resolve() != path.resolve():
            raise RuntimeError(f"{module} resolved to {imported.__file__}, expected {path}")
    print(f"[build] imported {len(modules)} generated protobuf modules")


if __name__ == "__main__":
    main()
