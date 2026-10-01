#!/usr/bin/env bash
# Offline x86 test (ASan/UBSan) of riffbox_vst.cpp, then a CPU benchmark (-O2): run after build.sh
# (needs build/params.h, build/popup.h). Prints PASSED/FAILED; exit code follows.
set -euo pipefail
cd "$(dirname "$0")"
docker run --rm -v "$PWD":/b -w /b gcc:12 bash -euc '
  mkdir -p build/x86
  SAN="-fsanitize=address,undefined"
  g++ -O0 -g $SAN -std=c++17 -fPIC -shared -Ibuild -o build/x86/riffbox-x86.so riffbox_vst.cpp -lpthread
  g++ -O0 -g $SAN -std=c++17 -o build/x86/host_test host_test.cpp -ldl
  ASAN_OPTIONS=detect_leaks=0 ./build/x86/host_test ./build/x86/riffbox-x86.so
  g++ -O2 -std=c++17 -fPIC -shared -fvisibility=hidden -Ibuild -o build/x86/riffbox-o2.so riffbox_vst.cpp -lpthread
  g++ -O2 -std=c++17 -o build/x86/bench host_test.cpp -ldl
  ./build/x86/bench ./build/x86/riffbox-o2.so bench
'
