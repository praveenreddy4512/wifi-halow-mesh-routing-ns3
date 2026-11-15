# Routing Protocols

This document explains how each of the four routing protocols is
integrated into the simulation and how a run selects one.

## Protocol selection

The scenario (`ns-3/scratch/five-floor-building-mesh.cc`) selects the
routing protocol from a single command-line flag:

```bash
--protocol=aodv      # standard AODV
--protocol=aodvv2    # AODVv2, hop-count metric by default
--protocol=aodvv2 --enableLinkQualityMetric=true   # AODVv2-LQ
--protocol=olsr       # standard OLSR
```

Internally, the scenario installs the corresponding ns-3 routing helper
(`AodvHelper`, `Aodvv2Helper`, or `OlsrHelper`) on the `InternetStackHelper`
before assigning IP addresses, so exactly one network-layer routing
protocol is active per run.

## AODV

Standard ns-3 AODV (`src/aodv`), implementing RFC 3561 (Ad hoc
On-Demand Distance Vector Routing) unmodified by this project. Used as
a baseline comparison point.

## AODVv2

Standard ns-3 AODVv2 (`src/aodvv2`), the ns-3 implementation of the
AODVv2/DYMO-successor IETF work. Its route-table machinery
(`aodvv2-local-route-set.*`, `aodvv2-multi-msg-set.*`,
`aodvv2-neighbor-set.*`, `aodvv2-rerr-set.*`,
`aodvv2-route-client-set.*`) and packet formats (`aodvv2-packet.*`) are
standard ns-3, built around a generic `Metric<T>` abstraction that
already supported more than one metric type before this project's
changes (`AODVV2_METRIC_HOP` for hop count).

By default (`--enableLinkQualityMetric=false`, the default), AODVv2 uses
its standard hop-count metric, with behavior unmodified by this
project.

## OLSR

Standard ns-3 OLSR (`src/olsr`), implementing RFC 3626 (Optimized Link
State Routing), unmodified by this project. Its `HelloInterval` and
`TcInterval` (also controlling `MidInterval`/`HnaInterval`) are exposed
as scenario command-line flags (`--olsrHelloInterval`,
`--olsrTcInterval`) rather than hardcoded, but the protocol
implementation itself is standard ns-3.

## AODVv2-LQ (link-quality extension)

An additional routing metric for AODVv2, enabled with
`--protocol=aodvv2 --enableLinkQualityMetric=true`, implemented entirely
inside `src/aodvv2/model/aodvv2-routing-protocol.{h,cc}`:

- **PHY observation**: a trace callback hooked onto the underlying
  `WifiPhy`'s receive path (`HookLinkQualitySnooping()`) observes each
  received frame's SNR and, together with the configured
  `ErrorRateModel`, derives a per-reception expected transmission count
  (ETX, `1 / success_probability`, capped at a fixed ceiling for very
  poor links).
- **Identity**: each ETX observation is correlated to the AODVv2
  control message it was measured for via the packet's own ns-3 UID
  (`Packet::GetUid()`), not via a MAC address — see
  `SOURCE_CODE_STRUCTURE.md` for why this project does not use the
  802.11 transmitter address (`Addr2`) for this purpose on this mesh
  forwarding path.
- **Propagation**: the measured ETX is added to AODVv2's existing
  generic metric machinery (`AODVV2_METRIC_SNR_ETX`, alongside the
  existing `AODVV2_METRIC_HOP`) via the same `AddMetric()` /
  `GetMetrics()` calls AODVv2 already used for hop count, so RREQ/RREP
  processing and `LookupBestRoute()`'s route comparison did not need
  protocol-specific special-casing for the new metric type.
- **Scope**: this metric only activates when `--protocol=aodvv2` and
  `--enableLinkQualityMetric=true` are both set; AODV, OLSR, and AODVv2
  in its default hop-count mode are unaffected by this code.

See `src/aodvv2/model/aodvv2-routing-protocol.cc` for the exact
implementation, and `SOURCE_CODE_STRUCTURE.md` for file-level detail.

## Known, disclosed implementation characteristics

- AODVv2's `RecvRequest()` installs a relay's `LocalRoute` metric value
  one hop count-increment before that same iteration's own
  `AddMetric()` call updates it, so the value stored at a relay is one
  hop "behind" the path's true accumulated cost at that point. This is
  a constant, ordering-preserving offset for hop count, but not for a
  variable-cost metric like ETX. Not fixed in this repository's current
  state; see `aodvv2-routing-protocol.cc`'s `RecvRequest()`.
- `aodvv2-multi-msg-set.cc`'s `MulticastMessageSet::IsDuplicate()`
  treats an incoming RREQ with a metric value equal to (not strictly
  better than) the already-stored one as a duplicate and suppresses it.
  This is an intentional loop/duplicate-prevention design, not a defect.

No performance claims about any of these protocols, or about AODVv2-LQ
relative to the others, are made in this repository.
