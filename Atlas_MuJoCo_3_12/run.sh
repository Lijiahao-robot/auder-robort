#!/usr/bin/env sh
set -eu
cd "$(dirname "$0")"
if [ ! -x .venv/bin/python ]; then
  python3 -m venv .venv
  .venv/bin/python -m pip install -r requirements.txt
fi
case "$(uname -s)" in
  Darwin) exec .venv/bin/mjpython run_atlas.py "$@" ;;
  *) exec .venv/bin/python run_atlas.py "$@" ;;
esac
