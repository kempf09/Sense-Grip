#!/usr/bin/env bash
# Builds a tiny recording (1.5 s MONITOR hold, then a -60 % force drop on one pad) and checks that
# slip_replay reports exactly one fsr slip event. Run from the repo root.
set -euo pipefail
dir=$(mktemp -d); trap 'rm -rf "$dir"' EXIT
hdr='seq,t_ms,state,step,armed,pos,speed,current_mA,load_pct,f1,f2,f3,f4,f5,f6,r1,r2,r3,r4,r5,r6,c1,c2,c3,tof_seq,obj_valid,width_mm,range_mm,height_mm,rad1,rad2,rad3,cls,shape,event'
{ echo "# synthetic"; echo "$hdr"
  for i in $(seq 0 299); do
    f=2.000; [ "$i" -ge 200 ] && f=0.800
    echo "$i,$((1000+i*10)),MONITOR,-,1,9000,0,0,0.0,$f,1.500,0.000,0.000,0.000,0.000,$f,1.500,0,0,0,0,1,1,0,0,0,,,,,,,medium,,"
  done; } > "$dir/r.csv"
out=$(tools/slip_replay/slip_replay --rows "$dir/r.csv")
n=$(echo "$out" | awk -F, 'NR>1 && $10==1' | wc -l)
first=$(echo "$out" | awk -F, 'NR>1 && $10==1 {print $1","$11; exit}')
[ "$n" = 1 ] || { echo "FAIL: expected 1 slip event, got $n"; exit 1; }
[ "${first#*,}" = fsr ] || { echo "FAIL: reason ${first#*,}"; exit 1; }
seq=${first%,*}; [ "$seq" -ge 200 ] && [ "$seq" -le 210 ] || { echo "FAIL: event at seq $seq"; exit 1; }
echo "slip_replay selftest OK (event at seq $seq)"
