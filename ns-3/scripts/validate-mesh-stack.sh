#!/usr/bin/env bash
# Four-node and Eight-node 802.11s mesh validation (HWMP disabled).
# Runs the five-floor-building-mesh binary with 4-node and 8-node configs,
# using Dot11sStack with UseHwmp=false so the custom L2 routing protocol
# (Dot11sL3OnlyRouting) is used instead of HWMP.
#
# Requirements:
#   - 4-node: AODV, AODVv2, OLSR
#   - 8-node: AODV, AODVv2, OLSR
#   - Same topology/node positions as the validated 6B experiments
#   - Custom L2 routing protocol active
#   - Peer management retained
#   - No HWMP, no FLAME
#   - Output to results/80211s-no-hwmp-validation/

set -u

NS3_DIR="$(cd "$(dirname "$0")/.." && pwd)"
ROOT="results/80211s-no-hwmp-validation"
BIN="$NS3_DIR/build/scratch/five-floor-building-mesh"

mkdir -p "$ROOT"
mkdir -p "$ROOT/4node"
mkdir -p "$ROOT/8node"

echo "============================================================"
echo "802.11s Mesh Validation (HWMP disabled, custom L2 routing)"
echo "============================================================"

if [ ! -x "$BIN" ]; then
    echo "Building scenario binary..."
    cd "$NS3_DIR"
    ./waf --run "scratch/five-floor-building-mesh --pcap=false --flowmon=false" 2>&1 | tail -20
    if [ ! -f "$NS3_DIR/build/scratch/five-floor-building-mesh" ]; then
        echo "Build FAILED"
        exit 1
    fi
fi
echo "Build OK."

run_one ()
{
    local nodes="$1" protocol="$2" exp_name="$3" peer_a="$4" peer_b="$5"

    local tag="n${nodes}_${protocol}"
    local dir="$ROOT/${exp_name}/${tag}"
    mkdir -p "$dir/sim"

    LD_LIBRARY_PATH=build "$BIN" \
        --nodes="$nodes" \
        --protocol="$protocol" \
        --peerFilterEnabled=1 \
        --peerFilterNodeA="$peer_a" \
        --peerFilterNodeB="$peer_b" \
        --resultsRoot="$dir/sim" \
        --pcap=true --flowmon=true > "$dir/run.log" 2>&1

    local status=$?
    local exit_code=$status

    if [ "$status" -ne 0 ]; then
        echo "FAILED $tag (exit $status)"
    else
        echo "OK $tag"
    fi

    {
        echo "experiment=$exp_name"
        echo "nodes=$nodes"
        echo "protocol=$protocol"
        echo "peer_filter_node_a=$peer_a"
        echo "peer_filter_node_b=$peer_b"
        echo "l2_mode=dot11s-mesh-l3-HWMP-disabled"
        echo "exit_status=$exit_code"
    } > "$dir/run-config.txt"

    if [ -f "$dir/sim/${protocol}/flowmon.xml" ]; then
        python3 - "$dir/sim/${protocol}/flowmon.xml" << 'PYEOF'
import sys, xml.etree.ElementTree as ET
path = sys.argv[1]
try:
    tree = ET.parse(path)
    root = tree.getroot()
    for flow in root.findall('.//flow'):
        tx = int(flow.get('txPackets', 0))
        rx = int(flow.get('rxPackets', 0))
        lost = int(flow.get('lostPackets', 0))
        dur = float(flow.get('timeLastRxPacket', 0)) if flow.get('timeLastRxPacket') else 0
        delay = float(flow.get('delaySum', 0)) if flow.get('delaySum') else 0
        print(f"  tx={tx} rx={rx} lost={lost} pdr={100.0*rx/tx if tx>0 else 0:.2f}% delay={delay:.4f}s")
except Exception as e:
    print(f"  flowmon parse error: {e}")
PYEOF
    fi
}

# ---- Four-node validation (AODV / AODVv2 / OLSR) ----
echo ""
echo "--- Four-node validation ---"
run_one 4 aodv "four-node" 1 4
run_one 4 aodvv2 "four-node" 1 4
run_one 4 olsr "four-node" 1 4

# ---- Eight-node multihop validation (AODV / AODVv2 / OLSR) ----
echo ""
echo "--- Eight-node multihop validation ---"
echo "Primary path: N1 -> N2 -> N3 -> N8"
echo "Alternate path: N1 -> N4 -> N5 -> N8"
run_one 8 aodv "eight-node" 0 7
run_one 8 aodvv2 "eight-node" 0 7
run_one 8 olsr "eight-node" 0 7

echo ""
echo "============================================================"
echo "Done. Results in $ROOT/"
echo "============================================================"
