#!/bin/bash
# Run the five-floor building with the 802.11s mesh stack (HWMP disabled)
# for all three routing protocols (AODV, AODVv2, OLSR).

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
NS3_DIR="$(dirname "$SCRIPT_DIR")"

mkdir -p "$NS3_DIR/results/AODV-mesh/pcap"
mkdir -p "$NS3_DIR/results/AODVv2-mesh/pcap"
mkdir -p "$NS3_DIR/results/OLSR-mesh/pcap"

echo "=== Building five-floor-building-mesh ==="
cd "$NS3_DIR"
./waf --run "scratch/five-floor-building-mesh --pcap=true --flowmon=true" 2>&1 | tail -20

if [ ! -f "$NS3_DIR/build/scratch/five-floor-building-mesh" ]; then
    echo "Build FAILED"
    exit 1
fi

echo ""
echo "=== Running AODV (802.11s mesh, HWMP disabled) ==="
cd "$NS3_DIR"
./waf --run "scratch/five-floor-building-mesh --protocol=aodv --pcap=true --flowmon=true" 2>&1 | tail -20

echo ""
echo "=== Running AODVv2 (802.11s mesh, HWMP disabled) ==="
cd "$NS3_DIR"
./waf --run "scratch/five-floor-building-mesh --protocol=aodvv2 --pcap=true --flowmon=true" 2>&1 | tail -20

echo ""
echo "=== Running OLSR (802.11s mesh, HWMP disabled) ==="
cd "$NS3_DIR"
./waf --run "scratch/five-floor-building-mesh --protocol=olsr --pcap=true --flowmon=true" 2>&1 | tail -20

echo ""
echo "=== Results ==="
echo "AODV:   $NS3_DIR/results/AODV-mesh/"
echo "AODVv2: $NS3_DIR/results/AODVv2-mesh/"
echo "OLSR:   $NS3_DIR/results/OLSR-mesh/"
