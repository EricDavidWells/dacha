"""Compile the mocap protos (and everything they import) to Python in gen/."""
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).parent
ROOT = HERE.parents[4]  # dacha repo root
GEN = HERE / "gen"
TOP = ["pkg/vision/mocap/proto/manager.proto", "pkg/vision/mocap/proto/simulation.proto"]


def closure():
    seen, todo = set(), list(TOP)
    while todo:
        p = todo.pop()
        if p in seen:
            continue
        seen.add(p)
        todo += re.findall(r'^import\s+"([^"]+)";', (ROOT / p).read_text(), flags=re.M)
    return sorted(seen)


def build():
    GEN.mkdir(exist_ok=True)
    files = closure()
    subprocess.run([sys.executable, "-m", "grpc_tools.protoc", f"-I{ROOT}", f"--python_out={GEN}",
                    f"--grpc_python_out={GEN}", *files], check=True)
    print(f"compiled {len(files)} protos -> {GEN}")


if __name__ == "__main__":
    build()
