# WiFi HaLow Mesh Routing (NS-3)

Simulation framework for studying AODV, AODVv2, OLSR, and a link-quality
extension of AODVv2 (AODVv2-LQ) over an IEEE 802.11ah (Wi-Fi HaLow)
physical layer combined with an IEEE 802.11s mesh data plane, built on
NS-3.

## 1. Project overview

This repository contains the NS-3 source tree, custom protocol
modifications, and simulation scenario code used for an M.Tech research
project titled **"Performance Study of AODVv2 over a Wi-Fi HaLow-Based
Mesh Network."** It is published as a reusable source-code reference for
other researchers working on routing over sub-GHz, long-range IEEE
802.11ah mesh networks — not as a results archive. No experimental
datasets, plots, or performance conclusions are included here; see
"Known limitations" below and the project's published paper (if any) for
results.

## 2. Research motivation

IEEE 802.11ah (Wi-Fi HaLow) is a sub-GHz amendment to the 802.11 family
designed for long range and low power, making it attractive for indoor
IoT and building-scale mesh deployments. Combining it with an IEEE
802.11s mesh data plane and a MANET routing protocol raises concrete
engineering questions — which routing protocol integrates cleanly with
mesh forwarding, whether link-quality-aware routing metrics are worth
their added complexity — that this project investigates by simulation
in a controlled, multi-floor building scenario.

## 3. Project objectives

- Provide a working NS-3 integration of IEEE 802.11ah PHY/MAC with an
  IEEE 802.11s mesh data plane, with HWMP replaced by a minimal, custom
  Layer-2 forwarding path.
- Run and compare AODV, AODVv2, and OLSR as the network-layer routing
  protocol over that mesh.
- Extend AODVv2 with an optional SNR-derived link-quality (ETX) routing
  metric (AODVv2-LQ) as an alternative to plain hop count.
- Package the above as a reusable, documented simulation scenario
  (a five-floor indoor building topology) that other researchers can
  rerun, adapt, or extend.

## 4. Technologies used

- **NS-3** (pre-3.27 development snapshot, `waf` build system), forked
  from imec-idlab's IEEE 802.11ah NS-3 module.
- **C++11**, compiled with GCC/G++.
- **Python 3** for the build system (`waf`) and simulation configuration
  helpers.
- **IEEE 802.11ah** PHY/MAC (`src/wifi`, with HaLow-specific
  configuration in the simulation scenario).
- **IEEE 802.11s** mesh networking (`src/mesh`), with HWMP disabled.

## 5. Supported routing protocols

| Protocol | Module | Notes |
|---|---|---|
| AODV | `src/aodv` | Standard ns-3 AODV, used as-is. |
| AODVv2 | `src/aodvv2` | Standard hop-count metric by default. |
| OLSR | `src/olsr` | Standard ns-3 OLSR, used as-is. |
| AODVv2-LQ | `src/aodvv2` | AODVv2 with an optional SNR-derived ETX metric in place of hop count (`--enableLinkQualityMetric=true`). |

See [docs/ROUTING_PROTOCOLS.md](docs/ROUTING_PROTOCOLS.md) for how each
protocol is integrated and selected.

## 6. Wi-Fi HaLow overview

IEEE 802.11ah operates in sub-1 GHz unlicensed bands and defines
reduced-bandwidth OFDM PHY modes (as narrow as 1 MHz) aimed at extended
range and low-power operation, at the cost of lower peak data rates
compared to 2.4/5 GHz Wi-Fi. This project configures the ns-3 `wifi`
module's PHY/channel parameters (TX power, receiver sensitivity, noise
floor, and per-material attenuation) to approximate an indoor HaLow
deployment; see [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the
exact parameters exposed.

## 7. Project architecture

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the full
PHY → MAC → mesh forwarding → routing → application layering, including
how packets move through the custom Layer-2 forwarding path.

## 8. Repository directory structure

```
wifi-halow-mesh-routing-ns3/
├── README.md                    This file
├── INSTALLATION.md               Setup, build, and verification steps
├── LICENSE                       GNU GPLv2 (inherited from ns-3)
├── AUTHORS                       Upstream ns-3 / module authors
├── .gitignore
├── docs/
│   ├── ARCHITECTURE.md
│   ├── ROUTING_PROTOCOLS.md
│   └── SOURCE_CODE_STRUCTURE.md
└── ns-3/                          The buildable ns-3 tree
    ├── waf, wscript, wutils.py, utils.py, utils/   Build system
    ├── waf-tools/                                  Waf tool plugins
    ├── src/                                        NS-3 modules (incl. custom aodvv2/mesh/network changes)
    ├── examples/                                    Standard ns-3 module examples
    ├── bindings/                                    Python binding generation sources (optional feature)
    ├── scratch/                                      Simulation scenarios (see below)
    └── scripts/                                       Reusable execution/validation shell scripts
```

### Simulation scenarios (`ns-3/scratch/`)

- `five-floor-building-mesh.cc` — the main scenario: a 20-node,
  five-floor indoor building topology, IEEE 802.11s mesh data plane with
  HWMP disabled, selectable routing protocol (AODV/AODVv2/OLSR,
  optionally AODVv2-LQ), configurable PHY/propagation parameters.
- `five-floor-building.cc` — an earlier/simpler variant of the same
  topology without the custom mesh L2 forwarding layer, used for
  protocol-compatibility checks.
- `80211s-no-hwmp-validation.cc` — a small standalone scenario used to
  validate that the custom L2 forwarding (HWMP disabled) behaves
  correctly before running it in the full building topology.

## 9. Installation requirements

See [INSTALLATION.md](INSTALLATION.md) for full, verified instructions.
In short: Ubuntu 24.04 LTS (or a comparably recent Linux distribution),
GCC/G++ with C++11 support, Python 3, and the standard ns-3 waf build
dependencies (`pkg-config`, optionally `libxml2`, `sqlite3`).

## 10. Installation instructions

See [INSTALLATION.md](INSTALLATION.md).

## 11. Build instructions

```bash
cd ns-3
python3 ./waf configure
python3 ./waf build
```

See [INSTALLATION.md](INSTALLATION.md) for the verified configure
options and troubleshooting.

## 12. How to execute the simulation

From inside `ns-3/`:

```bash
python3 ./waf --run "scratch/five-floor-building-mesh --protocol=aodvv2 --l2Mode=mesh11s-l3 --validationNodes=20"
```

Or use the bundled helper scripts:

```bash
cd ns-3
bash scripts/run-five-floor-building.sh          # runs AODV, AODVv2, OLSR in sequence
bash scripts/validate-mesh-stack.sh              # 4-node / 8-node mesh L2 validation
```

## 13. Configuration options

The scenario exposes its parameters through ns-3's standard
`CommandLine` mechanism (`--name=value`). The most relevant are:

| Flag | Purpose |
|---|---|
| `--protocol` | `aodv` \| `aodvv2` \| `olsr` |
| `--l2Mode` | `adhoc` \| `mesh11s` \| `mesh11s-l3` (custom L2 forwarding) \| `mesh-flame` |
| `--validationNodes` | Node-count stage: `4`, `8`, or `20` |
| `--enableLinkQualityMetric` | `true` to use AODVv2-LQ's SNR-derived ETX metric instead of hop count (requires `--protocol=aodvv2`) |
| `--sourceNodeNumber`, `--destNodeNumber` | Which N-numbered node is the UDP client/server |
| `--improvedPeerManagement`, `--peerMaxBeaconLoss`, `--peerMaxPacketFailure`, `--peerMaxRetries` | IEEE 802.11s PeerLink failure-tolerance tuning |
| `--txPowerDbm`, `--rxSensitivityDbm`, `--noiseFloorDbm` | PHY-level radio parameters |
| `--brickLossDb`, `--concreteLossDb`, `--partitionLossDb`, `--floorLossDb` | Per-material propagation attenuation for the building model |
| `--pcap`, `--flowmon` | Enable PCAP / FlowMonitor tracing |

Run `./waf --run "scratch/five-floor-building-mesh --PrintHelp"` for the
complete, authoritative list (the table above is not exhaustive).

## 14. Explanation of custom implementations

- **Custom Layer-2 forwarding without HWMP** (`src/mesh/model/dot11s/no-hwmp-protocol.{h,cc}`):
  a minimal `MeshL2RoutingProtocol` implementation
  (`NoHwmpRoutingProtocol` / `ReceiverAddressPlugin`) that forwards
  frames using the network-layer routing decision instead of HWMP path
  discovery, while still using the IEEE 802.11s peer-management state
  machine.
- **AODVv2 link-quality metric (AODVv2-LQ)** (`src/aodvv2/model/aodvv2-routing-protocol.{h,cc}`):
  an additional metric type (alongside AODVv2's existing hop-count
  metric) computed from PHY-layer SNR observations and propagated
  through RREQ/RREP using AODVv2's existing generic multi-metric route
  machinery.
- **Mesh previous-hop tagging** (`src/network/model/mesh-previous-hop-tag.{h,cc}`,
  `src/network/model/mesh-last-hop-mac-tag.{h,cc}`): packet tags used to
  carry per-hop relay identity information through the mesh forwarding
  path.

See [docs/SOURCE_CODE_STRUCTURE.md](docs/SOURCE_CODE_STRUCTURE.md) for
the complete file-by-file breakdown.

## 15. How other researchers can reuse the source code

- The `src/aodvv2` module's multi-metric route infrastructure
  (`Metric<T>`, `AddMetric()`, `LookupBestRoute()`) is generic and can
  carry additional metric types beyond hop count and SNR-ETX.
- `src/mesh/model/dot11s/no-hwmp-protocol.{h,cc}` is a self-contained
  reference for replacing HWMP with an external/network-layer routing
  decision while keeping 802.11s peer management.
- The five-floor building scenario's propagation/attenuation model
  (`scratch/five-floor-building-mesh.cc`) is parameterized by material
  type and can be adapted to other indoor floor plans by changing node
  placement and per-material loss values on the command line.
- Build and run the included scenarios first (see Installation/Build
  above) to confirm your environment reproduces a working baseline
  before modifying anything.

## 16. Known limitations

- This is an NS-3 **pre-3.27 development snapshot** (`VERSION` file:
  `3-dev`), not a current NS-3 release; module APIs may differ from
  upstream ns-3's current `src/aodv`, `src/olsr`, etc.
- The bundled `waf` 1.8.19 build system predates Python 3's removal of
  the `imp` module and the `'rU'` file mode; this repository ships a
  patched copy of waf's self-extracted runtime library
  (`ns-3/.waf3-1.8.19-*/`) with the minimal changes needed to run under
  Python ≥ 3.12 — see [INSTALLATION.md](INSTALLATION.md).
- Python bindings, the ns-3 test suite, and example binaries are
  disabled by default in the verified configuration (no `pybindgen` /
  `python-config` dependency was installed when this was last verified);
  they are optional ns-3 features, not required to build or run the
  routing scenarios.
- This repository intentionally excludes experimental result data,
  logs, and performance figures. It is a source-code reference, not a
  results archive.

## 17. References

- IEEE Std 802.11ah-2016, "Wireless LAN Medium Access Control (MAC) and
  Physical Layer (PHY) Specifications — Amendment 2: Sub 1 GHz License
  Exempt Operation."
- IEEE Std 802.11s-2011, "Wireless LAN Medium Access Control (MAC) and
  Physical Layer (PHY) Specifications — Amendment 10: Mesh Networking."
- RFC 3561, "Ad hoc On-Demand Distance Vector (AODV) Routing."
- IETF draft/RFC material for AODVv2 (DYMO successor), as implemented by
  the ns-3 `aodvv2` module this project builds on.
- RFC 3626, "Optimized Link State Routing Protocol (OLSR)."
- The NS-3 Network Simulator, <https://www.nsnam.org/>.
- imec IDLab's IEEE 802.11ah ns-3 module (upstream base for the PHY/MAC
  work in this repository), <https://github.com/imec-idlab/IEEE-802.11ah-ns-3>.

## 18. Author and research project information

M.Tech research project: *Performance Study of AODVv2 over a Wi-Fi
HaLow-Based Mesh Network*.

Repository: <https://github.com/praveenreddy4512/wifi-halow-mesh-routing-ns3>
