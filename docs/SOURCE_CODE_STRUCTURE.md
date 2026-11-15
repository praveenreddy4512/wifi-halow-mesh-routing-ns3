# Source Code Structure

This document explains the purpose of the directories and files most
relevant to this project's own work, so another researcher can locate
and reuse the implementation without reading the entire ns-3 tree.
Directories not listed here (most of `ns-3/src/*`) are standard,
unmodified upstream ns-3 modules, present because the ns-3 build system
expects the full module set to be available, not because they were
modified for this project.

## Top level

```
README.md            Project overview, usage, references
INSTALLATION.md       Setup, build, and verification steps
LICENSE               GNU GPLv2 (inherited from ns-3)
AUTHORS               Upstream ns-3 / IEEE 802.11ah module authors
docs/                 This directory
ns-3/                 The buildable ns-3 tree (see below)
```

## `ns-3/` — build system

- `waf`, `waf.bat`, `wscript`, `wutils.py`, `utils.py`, `utils/`,
  `waf-tools/` — the ns-3 waf build system, unmodified except as noted
  next.
- `ns-3/.waf3-1.8.19-b1fc8f7baef51bd2db4c2971909a568d/` — waf's
  self-extracted internal runtime library (normally regenerated
  automatically from a blob embedded in the `waf` script on first run).
  This copy is **pre-patched** for Python ≥ 3.12 compatibility, with
  exactly four changes relative to what a fresh extraction on an
  unpatched system produces:

  1. `waflib/Context.py`: removed `import imp` (module removed in
     Python 3.12) and replaced `imp.new_module(WSCRIPT_FILE)` with
     `type(sys)("module")`.
  2. `waflib/Context.py` and `waflib/ConfigSet.py`: replaced the
     deprecated universal-newline file mode `'rU'` with `'r'`.
  3. `waflib/Node.py`: replaced a bare `raise StopIteration` inside a
     generator with `return` (PEP 479, enforced since Python 3.7).
  4. `waflib/Tools/python.py`: replaced
     `` conf.env.PYTHON+['-c',"import imp;print(imp.get_tag())"] `` with
     a `sys.implementation.cache_tag`-based equivalent that does not
     depend on the removed `imp` module.

  If this directory is ever deleted and `waf` re-extracts a fresh,
  unpatched copy, reapply these four changes (or restore the directory
  from version control) — see `INSTALLATION.md` for symptoms.

## `ns-3/src/aodv/` — AODV

Standard ns-3 AODV implementation, unmodified by this project.

## `ns-3/src/aodvv2/` — AODVv2 and AODVv2-LQ

- `model/aodvv2-routing-protocol.{h,cc}` — the main protocol
  implementation. This project's additions live here:
  - `HookLinkQualitySnooping()` — attaches a trace callback to the
    node's `WifiPhy` (unwrapping a `MeshPointDevice`'s interfaces when
    necessary, since a `MeshPointDevice` does not itself expose a
    `WifiNetDevice` via `GetObject<WifiNetDevice>()`) to observe
    received-frame SNR.
  - `SnoopPhyRx(...)` — the trace callback itself. Rejects frames with
    no usable transmitter address (ACK/CTS, which lack an `Addr2`
    field) and zero/broadcast/group addresses before computing SNR and
    ETX; records the result keyed by `packet->GetUid()` in
    `m_recentPhyEtxByUid` (a short, 2-second windowed cache).
  - `RecvAodvv2(...)`'s hop-ETX resolution block — looks up the
    incoming AODVv2 control message's own packet UID in
    `m_recentPhyEtxByUid` to retrieve the ETX measured for that exact
    reception event. This UID-based correlation is used specifically
    because this mesh's custom forwarding path
    (`no-hwmp-protocol.cc`, below) does not update the 802.11
    transmitter address (`Addr2`) on relay, so any MAC-address-based
    neighbor-identity scheme would silently attribute a relayed
    frame's measurement to the original source rather than the actual
    relay. `ResolveNeighborMac()` (ARP-based) and
    `ResolveLastHopMacFromTag()` (reading `MeshLastHopMacTag`) remain
    in the file for comparison/diagnostic purposes; neither feeds the
    metric used for routing decisions.
  - `RecvRequest()` — standard AODVv2 RREQ handling; see
    `ROUTING_PROTOCOLS.md` for a disclosed, unfixed off-by-one-hop
    characteristic in this function's `LocalRoute` installation.
  - `AODVV2_METRIC_SNR_ETX` (alongside the existing
    `AODVV2_METRIC_HOP`) — defined in `model/aodvv2-packet.h`.
- `model/aodvv2-multi-msg-set.cc` — `MulticastMessageSet::IsDuplicate()`
  governs RREQ duplicate suppression; see `ROUTING_PROTOCOLS.md`.
- `model/aodvv2-local-route-set.*`, `aodvv2-neighbor-set.*`,
  `aodvv2-rerr-set.*`, `aodvv2-route-client-set.*`, `aodvv2-rqueue.*` —
  standard AODVv2 route-table/queue machinery, unmodified.

## `ns-3/src/olsr/` — OLSR

Standard ns-3 OLSR implementation, unmodified by this project.

## `ns-3/src/mesh/` — IEEE 802.11s mesh

- `model/mesh-point-device.{h,cc}` — the mesh network device. Attaches
  `MeshPreviousHopTag` (and, in this project's additions,
  `MeshLastHopMacTag`) to each frame in `ReceiveFromDevice()` before
  delivering it up to the IP stack.
- `model/dot11s/peer-management-protocol.*` — standard 802.11s peer
  management (OPEN/CONFIRM handshake, peer-link state machine),
  unmodified; its failure-tolerance thresholds are tuned from the
  scenario's own command line, not by editing this file.
- `model/dot11s/hwmp-protocol.*` — standard HWMP, present but not used
  by the main scenario (`--l2Mode=mesh11s-l3` bypasses it).
- `model/dot11s/no-hwmp-protocol.{h,cc}` — **this project's custom
  Layer-2 forwarding**, added in place of HWMP. `ReceiverAddressPlugin::
  UpdateOutcomingFrame()` sets each outgoing frame's 802.11 receiver
  address (`Addr1`) to the next hop the network-layer routing protocol
  selected. It does not modify the transmitter address (`Addr2`), which
  is why `Addr2` on a relayed frame reflects the frame's *original*
  sender for its entire forwarding lifetime, not the immediate relay —
  a confirmed, source-level characteristic of this plugin's design
  (it was built to answer "who should receive this," not "who is
  transmitting this," since no routing protocol needed `Addr2` for
  anything until the AODVv2-LQ work above). This is why AODVv2-LQ's ETX
  identity correlation uses packet UID rather than any MAC address read
  from a relayed frame.

## `ns-3/src/network/` — packet tags

- `model/mesh-previous-hop-tag.{h,cc}` — pre-existing tag, carries an
  `Ipv4Address`; set by a relay to describe *itself* right before
  forwarding. Reliable for identifying the immediate previous hop.
- `model/mesh-last-hop-mac-tag.{h,cc}` — this project's addition,
  carries a `Mac48Address` captured from a received frame's `Addr2`.
  Present and functional as a tag mechanism, but — per the `Addr2`
  characteristic above — not reliable as a *relay* identity source for
  frames that have been forwarded more than once; kept for
  diagnostic/comparison use, not for routing decisions.
- `wscript` — registers both tags' source/header files for the build.

## `ns-3/scratch/` — simulation scenarios

See `README.md`'s "Simulation scenarios" section.

## `ns-3/scripts/` — execution helpers

- `run-five-floor-building.sh` — runs the `five-floor-building`
  scenario for AODV, AODVv2, and OLSR in sequence, with basic
  PCAP/FlowMonitor output verification. Run from inside `ns-3/`.
- `validate-mesh-stack.sh` — runs 4-node and 8-node configurations of
  `five-floor-building-mesh` to validate the custom L2 forwarding path
  independently of the full 20-node topology. Run from inside `ns-3/`.
