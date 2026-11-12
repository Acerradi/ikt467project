#!/usr/bin/env bash
# Run every BehaviorSpace experiment headless; tables go to work/<experiment>.csv.
# Usage: ./run-experiments.sh [experiment-name ...]   (default: all)
set -euo pipefail

NETLOGO_HOME="${NETLOGO_HOME:-/opt/NetLogo-7.0.4}"
export JAVA_HOME="${JAVA_HOME:-$NETLOGO_HOME/lib/runtime}"
THREADS="${THREADS:-$(nproc)}"
ROOT="$(cd "$(dirname "$0")" && pwd)"

# model file : experiment name
EXPERIMENTS=(
  "Pedestrians.nlogox:figure-3-lanes"
  "scenarios/Door.nlogox:door-width-sweep"
  "scenarios/TrafficLight.nlogox:light-cycle"
  "scenarios/Crossing.nlogox:crossing-flow-sweep"
)

mkdir -p "$ROOT/work"
for entry in "${EXPERIMENTS[@]}"; do
  model="${entry%%:*}"
  exp="${entry##*:}"
  if (( $# > 0 )) && [[ ! " $* " == *" $exp "* ]]; then continue; fi
  echo "=== $exp ($model) ==="
  start=$SECONDS
  "$NETLOGO_HOME/netlogo-headless.sh" \
    --model "$ROOT/$model" --experiment "$exp" \
    --table "$ROOT/work/$exp.csv" --threads "$THREADS"
  echo "=== $exp done in $((SECONDS - start)) s -> work/$exp.csv ==="
done
