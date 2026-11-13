#!/usr/bin/env bash
# Builds the flight code for your PC and flies simulated flights.
# Needs only g++ (Linux, macOS, or Windows with WSL / MSYS2).
#
#   tools/desktop_sim/run.sh          one full flight with all messages, then 10 quick flights
#   tools/desktop_sim/run.sh --quick  only the 10 quick flights (what CI runs)
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
sketch="$here/../../TVC_Flight_Computer"
build="$here/build"
flags=(-std=gnu++17 -O2 -Wall -Wextra -Wno-unused-parameter -I"$here/mock")

mkdir -p "$build/sdcard" "$build/hw_sketch"

echo "== Building the simulator build (SIM_MODE 1)"
g++ "${flags[@]}" -I"$sketch" -x c++ "$here/sim_runner.cpp" -o "$build/sim_runner"

echo "== Building the hardware build (SIM_MODE 0) to check it compiles"
cp "$sketch"/* "$build/hw_sketch/"
sed -i.bak 's/^#define SIM_MODE 1/#define SIM_MODE 0/' "$build/hw_sketch/config.h"
grep -q '^#define SIM_MODE 0' "$build/hw_sketch/config.h"
g++ "${flags[@]}" -I"$build/hw_sketch" -x c++ "$here/sim_runner.cpp" -o "$build/hw_runner"
(cd "$build" && ./hw_runner | grep RESULT)

cd "$build"
if [[ "${1:-}" != "--quick" ]]; then
  echo "== One full flight (telemetry lines hidden)"
  ./sim_runner | grep -v '^T,'
fi

echo "== 10 flights with different gusts, sensor noise and igniter jitter"
fail=0
for seed in 1 2 3 4 5 6 7 8 9 10; do
  line="$(./sim_runner -q "$seed" 0.35 | grep RESULT)"
  echo "$line"
  # pass = landed upright (under 10 degrees of tilt)
  if ! echo "$line" | awk '{ for (i = 1; i <= NF; i++) { split($i, kv, "="); r[kv[1]] = kv[2] } }
       END { exit !(r["state"] == "LANDED" && r["td"] == 1 && r["tilt"] + 0 < 10) }'; then
    echo "   ^ FAILED"
    fail=1
  fi
done
[[ $fail -eq 0 ]] && echo "All simulated flights landed upright." || echo "Some simulated flights failed."
exit $fail
