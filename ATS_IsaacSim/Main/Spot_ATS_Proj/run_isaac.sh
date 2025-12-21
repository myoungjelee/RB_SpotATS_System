#!/bin/bash
cd /home/geunpilpark/isaacsim_ws/isaacsim/_build/linux-x86_64/release

case $1 in
  "spot_with_ats")
    ./python.sh ~/isaacsim_ws/ATS_IsaacSim/4.5/lecture_12/main.py
    ;;
  *)
    echo "Usage: ./run_isaac.sh {spot|lecture12}"
    ;;
esac
