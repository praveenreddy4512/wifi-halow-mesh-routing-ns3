# Architecture

This document describes how a packet moves through the simulated
network, from the physical layer up to the application, and where this
project's custom code sits relative to standard NS-3 modules.

## 1. Layering overview

```
Application (UDP client/server, ns-3 applications module)
        |
Network-layer routing (AODV | AODVv2[-LQ] | OLSR)
        |
Mesh Layer-2 forwarding (IEEE 802.11s peer management +
                          either HWMP or this project's NoHwmpRoutingProtocol)
        |
IEEE 802.11 MAC (ns-3 wifi module)
        |
IEEE 802.11ah-configured PHY (ns-3 wifi module, HaLow-like parameters)
        |
Propagation model (per-material indoor attenuation)
```

Only the two middle layers (mesh L2 forwarding and network-layer
routing) and parts of the PHY configuration are modified or added by
this project; the application layer, MAC frame handling, and the
underlying `wifi`/`propagation` modules are standard NS-3.

## 2. Wi-Fi HaLow PHY/MAC

The scenario code (`ns-3/scratch/five-floor-building-mesh.cc`) configures
the standard ns-3 `wifi` module's PHY and channel with parameters
intended to approximate an indoor IEEE 802.11ah deployment:

- Transmit power (`--txPowerDbm`) and receiver sensitivity
  (`--rxSensitivityDbm`), set to values representative of HaLow's
  extended-range, reduced-bandwidth operation rather than 2.4/5 GHz
  Wi-Fi defaults.
- A noise floor (`--noiseFloorDbm`) used for SNR estimation.
- A propagation loss model composed of per-material attenuation terms
  — brick, concrete, and partition wall crossings, plus floor
  crossings (direct, via staircase, or via atrium) — applied according
  to the node layout of the five-floor building topology.

The 802.11 MAC itself (frame exchange, retransmission, rate control) is
the unmodified ns-3 `wifi` module.

## 3. Mesh networking components

IEEE 802.11s mesh functionality comes from the ns-3 `mesh` module
(`src/mesh`), which provides:

- Peer management (`src/mesh/model/dot11s/peer-management-protocol.*`)
  — the OPEN/CONFIRM handshake and peer-link state machine that
  establishes mesh neighbor relationships. This project tunes its
  failure-tolerance parameters (`--peerMaxBeaconLoss`,
  `--peerMaxPacketFailure`, `--peerMaxRetries`,
  `--improvedPeerManagement`) but does not modify the state machine
  itself.
- HWMP (`src/mesh/model/dot11s/hwmp-protocol.*`) — ns-3's standard mesh
  path-discovery protocol, still available (`--l2Mode=mesh11s`) but not
  used by this project's main scenario.

## 4. Custom Layer-2 forwarding (no HWMP)

This project adds `NoHwmpRoutingProtocol` / `ReceiverAddressPlugin`
(`src/mesh/model/dot11s/no-hwmp-protocol.{h,cc}`), selected with
`--l2Mode=mesh11s-l3`. Instead of HWMP's own path-discovery frames, this
plugin forwards each frame using the receiver address the network-layer
routing protocol (AODV/AODVv2/OLSR) has already decided on — the mesh
layer becomes a thin forwarding shim over the routing protocol's own
next-hop decision, while 802.11s peer management (above) still runs
underneath it to maintain the mesh's neighbor relationships.

Concretely, `ReceiverAddressPlugin::UpdateOutcomingFrame()` sets the
802.11 receiver address (`Addr1`) of each outgoing frame to the next hop
the routing layer selected; it does not alter the transmitter address
(`Addr2`), which is a source-level characteristic documented in
`docs/SOURCE_CODE_STRUCTURE.md` under `no-hwmp-protocol.cc`.

## 5. Network-layer routing

See [ROUTING_PROTOCOLS.md](ROUTING_PROTOCOLS.md) for how AODV, AODVv2,
OLSR, and AODVv2-LQ are each integrated and selected.

## 6. Application traffic

The scenario installs a standard ns-3 UDP client/server application
pair (`ns3::UdpEchoClient`/`ns3::UdpEchoServer`-style applications from
the `applications` module) between two configurable nodes
(`--sourceNodeNumber`, `--destNodeNumber`). Traffic generation itself is
not custom.

## 7. Packet forwarding flow (five-floor-building-mesh scenario, `mesh11s-l3` mode)

1. The application layer generates a UDP packet addressed to the
   destination node's IP.
2. The active routing protocol (AODV/AODVv2[-LQ]/OLSR) determines the
   next hop and hands the packet to the mesh device.
3. `MeshPointDevice` (`src/mesh/model/mesh-point-device.cc`) passes the
   frame to `NoHwmpRoutingProtocol`, which sets the 802.11 receiver
   address to the chosen next hop and transmits over the IEEE
   802.11ah-configured PHY/MAC.
4. At the receiving node, `MeshPointDevice::ReceiveFromDevice()` tags
   the packet (`MeshPreviousHopTag`, carrying the relay's own IP; see
   `src/network/model/mesh-previous-hop-tag.{h,cc}`) before delivering
   it up to the IP stack, where the routing protocol processes it (and,
   for AODVv2-LQ, also reads an SNR-derived link-quality observation —
   see ROUTING_PROTOCOLS.md).
5. If this node is not the final destination, steps 2-4 repeat at each
   intermediate relay until the packet reaches the destination's
   application layer.
