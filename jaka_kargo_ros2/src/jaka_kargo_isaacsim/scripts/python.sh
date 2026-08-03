#!/usr/bin/env bash

set -u

ISAAC_ROOT="${ISAAC_SIM_ROOT:-$HOME/isaacsim}"

SCRIPT_DIR="$(
    cd "$(dirname "${BASH_SOURCE[0]}")" &&
    pwd
)"

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <python-script> [script arguments...]"
    exit 2
fi

SCRIPT_PATH="$1"
shift

if [[ "$SCRIPT_PATH" != /* ]]; then
    SCRIPT_PATH="$SCRIPT_DIR/$SCRIPT_PATH"
fi

if [[ ! -f "$ISAAC_ROOT/python.sh" ]]; then
    echo "Isaac Sim python.sh not found: $ISAAC_ROOT/python.sh"
    exit 1
fi

if [[ ! -f "$SCRIPT_PATH" ]]; then
    echo "Python script not found: $SCRIPT_PATH"
    exit 1
fi

cd "$ISAAC_ROOT" || exit 1

# exec preserves Isaac Sim's real exit status.
exec ./python.sh "$SCRIPT_PATH" "$@"