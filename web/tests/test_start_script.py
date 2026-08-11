import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "start_web.sh"


def run_script(*arguments):
    return subprocess.run(
        ["bash", str(SCRIPT), *arguments],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )


def test_help_documents_default_and_custom_port():
    result = run_script("--help")
    assert result.returncode == 0
    assert "9000" in result.stdout
    assert "--port" in result.stdout


def test_invalid_port_is_rejected_before_startup():
    result = run_script("--port", "0")
    assert result.returncode != 0
    assert "1-65535" in result.stderr
