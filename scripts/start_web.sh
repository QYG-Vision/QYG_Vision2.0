#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
用法: ./scripts/start_web.sh [--port N]

启动庆园阁视觉调试平台，固定监听 0.0.0.0。
默认端口: 9000
EOF
}

port=9000
while (($#)); do
  case "$1" in
    --port)
      if (($# < 2)); then
        echo "错误: --port 需要一个端口号" >&2
        exit 2
      fi
      port="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "错误: 未知参数 $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! "$port" =~ ^[0-9]+$ ]] || ((port < 1 || port > 65535)); then
  echo "错误: 端口必须在 1-65535 之间" >&2
  exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "$script_dir/.." && pwd -P)"
python_bin="${QYG_WEB_PYTHON:-python3}"

if [[ ! -d /dev/shm || ! -w /dev/shm ]]; then
  echo "错误: /dev/shm 不存在或不可写" >&2
  exit 1
fi

if ! "$python_bin" -c 'import flask' >/dev/null 2>&1; then
  echo "错误: Python 缺少 Flask，请运行: python3 -m pip install -r web/requirements.txt" >&2
  exit 1
fi

if ! "$python_bin" - "$port" <<'PY'
import socket
import sys

sock = socket.socket()
try:
    sock.bind(("0.0.0.0", int(sys.argv[1])))
except OSError:
    raise SystemExit(1)
finally:
    sock.close()
PY
then
  echo "错误: 端口 $port 已被占用" >&2
  exit 1
fi

cd "$project_root"
exec "$python_bin" -m web.app --port "$port"
