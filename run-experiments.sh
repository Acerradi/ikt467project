#!/usr/bin/env bash
# Run every BehaviorSpace experiment headless; tables go to work/<experiment>.csv.
# Usage: ./run-experiments.sh [experiment-name ...]   (default: all)
#
# Works on Linux, macOS (including its old bash 3.2) and Windows (Git Bash,
# MSYS2, Cygwin or WSL) with NetLogo 7 installed. The NetLogo install is
# found automatically in the usual places; NetLogo's own launcher script is
# not used (Windows has none), Java is started directly instead.
#
# Environment overrides:
#   NETLOGO_HOME  NetLogo install directory      (default: auto-detected)
#   NETLOGO_JAVA  java executable to use         (default: the Java bundled
#                 with NetLogo, else $JAVA_HOME/bin/java, else java on PATH)
#   THREADS       parallel BehaviorSpace runs    (default: number of CPU cores)
#
# Examples:
#   ./run-experiments.sh                                # all four experiments
#   ./run-experiments.sh door-width-sweep light-cycle   # only the named ones
#   THREADS=8 ./run-experiments.sh                      # limit parallelism
#   NETLOGO_HOME="/c/Program Files/NetLogo 7.0.4" ./run-experiments.sh

# Stop on the first failing command (-e), on unset variables (-u) and on a
# failure anywhere in a pipeline (-o pipefail).
set -euo pipefail

die() { echo "run-experiments.sh: $*" >&2; exit 1; }

# Absolute path of the repository root (the directory holding this script),
# so the script works from any current directory.
ROOT="$(cd "$(dirname "$0")" && pwd)"

# model file : experiment name
# Each experiment is defined in the BehaviorSpace section of its model file.
EXPERIMENTS=(
  "Pedestrians.nlogox:figure-3-lanes"
  "scenarios/Door.nlogox:door-width-sweep"
  "scenarios/TrafficLight.nlogox:light-cycle"
  "scenarios/Crossing.nlogox:crossing-flow-sweep"
)

# Reject unknown experiment names instead of silently running nothing.
for name in "$@"; do
  known=false
  for entry in "${EXPERIMENTS[@]}"; do
    [[ "${entry##*:}" == "$name" ]] && known=true
  done
  $known || die "unknown experiment '$name' (known: figure-3-lanes door-width-sweep light-cycle crossing-flow-sweep)"
done

# ------------------------------------------------------------ locate NetLogo

# First line of a command's output, or "" (no head: with pipefail, head
# closing the pipe early could make the whole pipeline fail).
first_line() { local out; out="$("$@" 2>/dev/null || true)"; printf '%s' "${out%%$'\n'*}"; }

# The NetLogo 7 application jar inside directory $1 matching pattern $2.
# Linux: lib/app/, Windows: app/, macOS: inside the .app bundle.
find_jar() { first_line find -H "$1" -type f -name "$2"; }

if [[ -z "${NETLOGO_HOME:-}" ]]; then
  # Usual install locations on Linux, macOS, Windows (Git Bash/MSYS/Cygwin)
  # and WSL. Unmatched globs stay literal and fail the -d test below.
  candidates=(
    "$HOME"/NetLogo* /opt/NetLogo* /usr/local/NetLogo* /usr/share/NetLogo*
    /Applications/NetLogo* "$HOME"/Applications/NetLogo*
    "/c/Program Files"/NetLogo* "/c/Program Files (x86)"/NetLogo*
    "/cygdrive/c/Program Files"/NetLogo* "/mnt/c/Program Files"/NetLogo*
  )
  # Prefer the exact version the models were saved with, then any NetLogo 7.
  for pattern in 'netlogo-7.0.4.jar' 'netlogo-7*.jar'; do
    for dir in "${candidates[@]}"; do
      if [[ -d "$dir" && -n "$(find_jar "$dir" "$pattern")" ]]; then
        NETLOGO_HOME="$dir"
        break 2
      fi
    done
  done
  [[ -n "${NETLOGO_HOME:-}" ]] || die "NetLogo 7 not found. Set NETLOGO_HOME to its install directory."
fi
[[ -d "$NETLOGO_HOME" ]] || die "NETLOGO_HOME '$NETLOGO_HOME' is not a directory."

NETLOGO_JAR="$(find_jar "$NETLOGO_HOME" 'netlogo-7*.jar')"
[[ -n "$NETLOGO_JAR" ]] || die "no netlogo-7*.jar under '$NETLOGO_HOME'. The models need NetLogo 7."

# Java: NetLogo bundles a runtime of the right version, so prefer it over a
# system Java (NetLogo 7 does not run on older Java versions such as 8 or 11).
if [[ -z "${NETLOGO_JAVA:-}" ]]; then
  NETLOGO_JAVA="$(first_line find -H "$NETLOGO_HOME" -type f \( -name java -o -name java.exe \) -path '*/bin/*')"
fi
if [[ -z "${NETLOGO_JAVA:-}" && -n "${JAVA_HOME:-}" && -x "$JAVA_HOME/bin/java" ]]; then
  NETLOGO_JAVA="$JAVA_HOME/bin/java"
fi
if [[ -z "${NETLOGO_JAVA:-}" ]]; then
  NETLOGO_JAVA="$(command -v java || true)"
fi
[[ -n "$NETLOGO_JAVA" && -x "$NETLOGO_JAVA" ]] || die "no Java found. Set NETLOGO_JAVA to a java executable (version 17 or newer)."

# A Windows java.exe (from Git Bash, Cygwin or WSL) needs Windows-style paths.
native_path() {
  if [[ "$NETLOGO_JAVA" == *.exe ]]; then
    if command -v cygpath >/dev/null 2>&1; then cygpath -w "$1"; return; fi
    if command -v wslpath >/dev/null 2>&1; then wslpath -w "$1"; return; fi
  fi
  printf '%s' "$1"
}

# Number of CPU cores: nproc (Linux), getconf (Linux/macOS), sysctl (macOS),
# NUMBER_OF_PROCESSORS (Windows), else 1.
cpu_count() {
  local n
  n="$(nproc 2>/dev/null || getconf _NPROCESSORS_ONLN 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || true)"
  [[ -z "$n" ]] && n="${NUMBER_OF_PROCESSORS:-1}"
  printf '%s' "$n"
}
THREADS="${THREADS:-$(cpu_count)}"

# The same JVM options as NetLogo's own netlogo-headless.sh, plus
# java.awt.headless so no window or dock icon appears and no display is needed.
JVM_OPTS=(
  -XX:MaxRAMPercentage=50
  -Dfile.encoding=UTF-8
  -Djava.awt.headless=true
  --add-exports=java.base/java.lang=ALL-UNNAMED
  --add-exports=java.desktop/sun.awt=ALL-UNNAMED
  --add-exports=java.desktop/sun.java2d=ALL-UNNAMED
)
if [[ -d "$NETLOGO_HOME/extensions" ]]; then
  JVM_OPTS+=("-Dnetlogo.extensions.dir=$(native_path "$NETLOGO_HOME/extensions")")
fi

echo "NetLogo: $NETLOGO_JAR"
echo "Java:    $NETLOGO_JAVA"
echo "Threads: $THREADS"

# ------------------------------------------------------------ run

mkdir -p "$ROOT/work"
for entry in "${EXPERIMENTS[@]}"; do
  # Split "model:experiment" at the colon.
  model="${entry%%:*}"
  exp="${entry##*:}"
  # When names were given on the command line, skip experiments not listed.
  # (Padding with spaces makes the match whole-word.)
  if (( $# > 0 )) && [[ ! " $* " == *" $exp "* ]]; then continue; fi
  echo "=== $exp ($model) ==="
  start=$SECONDS
  # org.nlogo.headless.Main is NetLogo's command-line BehaviorSpace runner.
  # --table writes one row per run with the experiment's metrics; an existing
  # file of the same name is overwritten.
  "$NETLOGO_JAVA" "${JVM_OPTS[@]}" -classpath "$(native_path "$NETLOGO_JAR")" \
    org.nlogo.headless.Main \
    --model "$(native_path "$ROOT/$model")" --experiment "$exp" \
    --table "$(native_path "$ROOT/work/$exp.csv")" --threads "$THREADS"
  echo "=== $exp done in $((SECONDS - start)) s -> work/$exp.csv ==="
done
