from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class BuildPolicyTests(unittest.TestCase):
    def test_runtime_venv_is_ready_before_codegen(self):
        text = (ROOT / "scripts/build.sh").read_text()
        codegen = text.rindex("rbnx codegen")
        self.assertLess(text.index("uv venv"), codegen)
        self.assertLess(text.index("uv pip install"), codegen)
        self.assertIn('RBNX_CODEGEN_PYTHON="$VENV/bin/python"', text[:codegen])
        self.assertIn('PATH="$VENV/bin:$PATH"', text[:codegen])
        self.assertNotIn("--system-site-packages", text)

    def test_protobuf_toolchain_is_exactly_pinned(self):
        requirements = (ROOT / "requirements.txt").read_text().splitlines()
        for requirement in (
            "grpcio==1.80.0",
            "grpcio-tools==1.76.0",
            "protobuf==6.33.6",
        ):
            self.assertIn(requirement, requirements)

    def test_build_imports_generated_modules(self):
        text = (ROOT / "scripts/build.sh").read_text()
        self.assertIn('"$VENV/bin/python" "$PKG/scripts/import_generated.py"', text)

    def test_generated_import_smoke_test_checks_both_module_kinds(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            generated = root / "proto_gen"
            generated.mkdir(parents=True)
            mcp_generated = root / "robonix_mcp_types" / "nested"
            mcp_generated.mkdir(parents=True)
            (generated / "example_pb2.py").write_text("VALUE = 1\n")
            (generated / "example_pb2_grpc.py").write_text("import example_pb2\n")
            (mcp_generated / "contract_pb2.py").write_text("VALUE = 2\n")
            (mcp_generated / "contract_pb2_grpc.py").write_text(
                "from nested import contract_pb2\n"
            )
            result = subprocess.run(
                [sys.executable, str(ROOT / "scripts/import_generated.py"), str(root)],
                check=True,
                capture_output=True,
                text=True,
            )
            self.assertIn("imported 4 generated protobuf modules", result.stdout)

    def test_start_reuses_venv_and_both_generated_roots(self):
        text = (ROOT / "scripts/start.sh").read_text()
        self.assertIn('PYTHON="$PKG_ROOT/rbnx-build/venv/bin/python"', text)
        self.assertIn("$CODEGEN_ROOT/proto_gen:$CODEGEN_ROOT/robonix_mcp_types", text)

    def test_clean_removes_only_the_package_build_directory(self):
        text = (ROOT / "scripts/build.sh").read_text()
        self.assertIn('rm -rf "$BUILD"', text)
        self.assertIn('BUILD="$PKG/rbnx-build"', text)
        self.assertIn("uv venv --allow-existing", text)


if __name__ == "__main__":
    unittest.main()
