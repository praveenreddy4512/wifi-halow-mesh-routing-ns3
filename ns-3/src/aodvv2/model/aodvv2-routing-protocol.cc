/*
 * Copyright (c) 2024 University of Florence
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Based on
 *      NS-3 AODV model developed by Elena Buchatskaya and Pavel Boyko of IITP RAS
 *
 * Authors: Francesco Todino <todinofrancesco97@gmail.com>
 *          Tommaso Pecorella <tommaso.pecorella@unifi.it>
 */

#include "aodvv2-routing-protocol.h"

#include "ns3/adhoc-wifi-mac.h"
#include "ns3/boolean.h"
#include "ns3/inet-socket-address.h"
#include "ns3/log.h"
#include "ns3/packetbb.h"
#include "ns3/pointer.h"
#include "ns3/random-variable-stream.h"
#include "ns3/string.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/udp-header.h"
#include "ns3/udp-l4-protocol.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/wifi-net-device.h"
#include "ns3/abort.h"
#include "ns3/mesh-point-device.h"
#include "ns3/mesh-previous-hop-tag.h"
#include "ns3/mesh-last-hop-mac-tag.h"
#include "ns3/yans-wifi-phy.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sys/stat.h>

namespace ns3
{

namespace aodvv2
{
NS_LOG_COMPONENT_DEFINE("Aodvv2RoutingProtocol");

namespace
{
struct Aodvv2RoutingProtocolIpv4EnsureRegistered
{
    Aodvv2RoutingProtocolIpv4EnsureRegistered()
    {
        Aodvv2RoutingProtocol<Ipv4RoutingProtocol>::GetTypeId();
    }
} g_aodvv2RoutingProtocolIpv4EnsureRegistered;

} // namespace

/**
 * @ingroup aodvv2
 * @brief Tag used by AODVv2 implementation
 */
class DeferredRouteOutputTag : public Tag
{
  public:
    /**
     * @brief Constructor
     * @param o the output interface
     */
    DeferredRouteOutputTag(int32_t o = -1)
        : Tag(),
          m_oif(o)
    {
    }

    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::aodvv2::DeferredRouteOutputTag")
                                .SetParent<Tag>()
                                .SetGroupName("Aodvv2")
                                .AddConstructor<DeferredRouteOutputTag>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override
    {
        return GetTypeId();
    }

    /**
     * @brief Get the output interface
     * @return the output interface
     */
    int32_t GetInterface() const
    {
        return m_oif;
    }

    /**
     * @brief Set the output interface
     * @param oif the output interface
     */
    void SetInterface(int32_t oif)
    {
        m_oif = oif;
    }

    uint32_t GetSerializedSize() const override
    {
        return sizeof(int32_t);
    }

    void Serialize(TagBuffer i) const override
    {
        i.WriteU32(m_oif);
    }

    void Deserialize(TagBuffer i) override
    {
        m_oif = i.ReadU32();
    }

    void Print(std::ostream& os) const override
    {
        os << "DeferredRouteOutputTag: output interface = " << m_oif;
    }

  private:
    /// Positive if output device is fixed in RouteOutput
    int32_t m_oif;
};

NS_OBJECT_ENSURE_REGISTERED(DeferredRouteOutputTag);

template <typename T>
const Time Aodvv2RoutingProtocol<T>::kPhyEtxUidWindow = Seconds(2.0);

//-----------------------------------------------------------------------------
template <typename T>
Aodvv2RoutingProtocol<T>::Aodvv2RoutingProtocol()
    : m_discoveryAttemptsMax(10),
      m_rrepRetries(2),
      m_maxHopLimit(100),
      m_timeoutBuffer(2),
      m_controlTrafficLimit(0.1),
      m_rreqRateLimit(1 / m_controlTrafficLimit),
      m_rrepRateLimit(1 / m_controlTrafficLimit),
      m_rerrRateLimit(1 / m_controlTrafficLimit),
      m_activeInterval(Seconds(5)),
      m_nodeTraversalTime(MilliSeconds(40)),
      m_netTraversalTime(Seconds(2)),
      m_nextHopWait(m_nodeTraversalTime + MilliSeconds(10)),
      m_rreqWaitTime(Seconds(2)),
      m_rreqHolddownTime(Seconds(10)),
      m_rrepAckSentTimeout(Seconds(1)),
      m_rerrTimeout(Seconds(3)),
      m_maxIdleTime(Seconds(3)),
      m_maxBlacklistTime(Seconds(5)),
      m_maxSeqnumLifetime(Seconds(300)),
      m_maxQueueLen(64),
      m_maxQueueTime(Seconds(30)),
      m_rtemsgEntryTime(Seconds(12)),
      m_destinationOnly(false),
      m_routingTable(m_activeInterval, m_maxBlacklistTime, m_maxSeqnumLifetime),
      m_queue(m_maxQueueLen, m_maxQueueTime),
      m_requestId(0),
      m_seqNo(0),
      m_mms(m_maxSeqnumLifetime),
      m_nb(m_maxBlacklistTime),
      m_RouteErrorSet(),
      m_rreqCount(0),
      m_rrepCount(0),
      m_rerrCount(0),
      m_useDefaultMetric(true),
      m_enableLinkQualityMetric(false),
      m_currentHopEtx(255.0),
      m_rreqRateLimitTimer(Timer::CANCEL_ON_DESTROY),
      m_rrepRateLimitTimer(Timer::CANCEL_ON_DESTROY),
      m_rerrRateLimitTimer(Timer::CANCEL_ON_DESTROY),
      m_lastBcastTime(Seconds(0)),
      m_bidirectionalLinks(false),
      m_resetPeriodically(false)
{
    m_routingTable.SetCallback(
        MakeCallback(&Aodvv2RoutingProtocol<T>::SendRerrWhenBreaksLinkToNextHop, this));

    Simulator::Schedule(Seconds(50), &Aodvv2RoutingProtocol<T>::ResetProtocol, this);

    std::ofstream file;
    file.open("output-packets.csv");
    file << "type,size\n";
    file.close();
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::ResetProtocol()
{
    if (GetResetPeriodically())
    {
        m_routingTable.Clear();
        m_nb.Clear();
        Simulator::Schedule(Seconds(50), &Aodvv2RoutingProtocol<T>::ResetProtocol, this);
    }
}

template <typename T>
TypeId
Aodvv2RoutingProtocol<T>::GetTypeId()
{
    std::string name;
    if constexpr (IsIpv4)
    {
        name = "Ipv4";
    }
    else
    {
        name = "Ipv6";
    }

    static TypeId tid =
        TypeId(("ns3::aodvv2::" + name + "Aodvv2RoutingProtocol").c_str())
            .SetParent<T>()
            .SetGroupName("Aodvv2")
            .template AddConstructor<Aodvv2RoutingProtocol<T>>()
            .AddAttribute("TimeoutBuffer",
                          "Provide a buffer for the timeout.",
                          UintegerValue(2),
                          MakeUintegerAccessor(&Aodvv2RoutingProtocol<T>::m_timeoutBuffer),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("RreqRetries",
                          "Maximum number of retransmissions of RREQ to discover a route",
                          UintegerValue(10),
                          MakeUintegerAccessor(&Aodvv2RoutingProtocol<T>::m_discoveryAttemptsMax),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("RreqRateLimit",
                          "Maximum number of RREQ per second.",
                          UintegerValue(10),
                          MakeUintegerAccessor(&Aodvv2RoutingProtocol<T>::m_rreqRateLimit),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("RerrRateLimit",
                          "Maximum number of RERR per second.",
                          UintegerValue(10),
                          MakeUintegerAccessor(&Aodvv2RoutingProtocol<T>::m_rerrRateLimit),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("NodeTraversalTime",
                          "Conservative estimate of the average one hop traversal time for packets "
                          "and should include "
                          "queuing delays, interrupt processing times and transfer times.",
                          TimeValue(MilliSeconds(40)),
                          MakeTimeAccessor(&Aodvv2RoutingProtocol<T>::m_nodeTraversalTime),
                          MakeTimeChecker())
            .AddAttribute(
                "NextHopWait",
                "Period of our waiting for the neighbour's RREP_ACK = 10 ms + NodeTraversalTime",
                TimeValue(MilliSeconds(50)),
                MakeTimeAccessor(&Aodvv2RoutingProtocol<T>::m_nextHopWait),
                MakeTimeChecker())
            .AddAttribute("NetTraversalTime",
                          "Estimate of the average net traversal time = 2 * NodeTraversalTime * "
                          "m_maxHopLimit",
                          TimeValue(Seconds(2)),
                          MakeTimeAccessor(&Aodvv2RoutingProtocol<T>::m_netTraversalTime),
                          MakeTimeChecker())
            .AddAttribute("MaxQueueLen",
                          "Maximum number of packets that we allow a routing protocol to buffer.",
                          UintegerValue(64),
                          MakeUintegerAccessor(&Aodvv2RoutingProtocol<T>::SetMaxQueueLen,
                                               &Aodvv2RoutingProtocol<T>::GetMaxQueueLen),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("MaxQueueTime",
                          "Maximum time packets can be queued (in seconds)",
                          TimeValue(Seconds(30)),
                          MakeTimeAccessor(&Aodvv2RoutingProtocol<T>::SetMaxQueueTime,
                                           &Aodvv2RoutingProtocol<T>::GetMaxQueueTime),
                          MakeTimeChecker())
            .AddAttribute("DestinationOnly",
                          "Indicates only the destination may respond to this RREQ.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&Aodvv2RoutingProtocol<T>::SetDestinationOnlyFlag,
                                              &Aodvv2RoutingProtocol<T>::GetDestinationOnlyFlag),
                          MakeBooleanChecker())
            .AddAttribute("EnableBroadcast",
                          "Indicates whether a broadcast data packets forwarding enable.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&Aodvv2RoutingProtocol<T>::SetBroadcastEnable,
                                              &Aodvv2RoutingProtocol<T>::GetBroadcastEnable),
                          MakeBooleanChecker())
            .AddAttribute("UniformRv",
                          "Access to the underlying UniformRandomVariable",
                          StringValue("ns3::UniformRandomVariable"),
                          MakePointerAccessor(&Aodvv2RoutingProtocol<T>::m_uniformRandomVariable),
                          MakePointerChecker<UniformRandomVariable>())
            .AddAttribute("EnableLinkQualityMetric",
                          "AODVv2-LQ (future-work variant): select routes using an "
                          "SNR-derived ETX metric instead of hop count. Default false "
                          "leaves this module's behavior identical to the unmodified "
                          "AODVv2 baseline; see AODVV2LQ_DESIGN_NOTES.md.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&Aodvv2RoutingProtocol<T>::SetEnableLinkQualityMetric,
                                              &Aodvv2RoutingProtocol<T>::GetEnableLinkQualityMetric),
                          MakeBooleanChecker());
    return tid;
}

// --- AODVv2-LQ: SNR-derived ETX link-quality metric. See the member
// declarations in aodvv2-routing-protocol.h for the overall design and
// AODVV2LQ_DESIGN_NOTES.md for the full derivation and limitations.
//
// Diagnostic logging (docs/AODVV2_LQ_DIAGNOSTIC_REPORT.md investigation):
// every call below is a no-op unless the NS3_DIAGNOSTIC_ROOT environment
// variable is set, matching the convention already used throughout this
// project (src/wifi/model/diagnostic-attempt-registry.h; the harness's own
// DiagnosticPath() in scratch/five-floor-building-mesh.cc). It only ever
// appends a CSV row -- it never changes a return value, a control-flow
// decision, or anything that could affect packet delivery, RNG
// consumption, or event ordering in a run where diagnostics are disabled.
static std::string
AodvV2LqDiagnosticPath(const char* filename)
{
    const char* root = std::getenv("NS3_DIAGNOSTIC_ROOT");
    if (!root)
    {
        return std::string();
    }
    std::string dir = std::string(root) + "/aodvv2lq";
    ::mkdir(dir.c_str(), 0777);
    return dir + "/" + filename;
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::SetEnableLinkQualityMetric(bool enable)
{
    // Attribute setters are invoked by ns-3's attribute system during
    // ObjectFactory::Create(), i.e. while this object is still being
    // constructed -- m_ip is not assigned yet at that point (it is set
    // afterwards by InternetStackHelper::Install() via SetIpv4()/SetIpv6()).
    // So this setter must not, directly or indirectly, touch m_ip, this
    // node's devices, or anything derived from them: doing so previously
    // dereferenced a null m_ip and crashed with SIGSEGV. The device/PHY
    // hookup those need lives in HookLinkQualitySnooping() instead, called
    // from NotifyInterfaceUp() below, where m_ip is already valid (every
    // other device access in this file already relies on that).
    m_enableLinkQualityMetric = enable;
    if (!enable)
    {
        return;
    }
    // AddMetric() is a no-op if this metric type was already registered
    // (e.g. a second attribute set on the same instance); SetUseDefaultMetric
    // (false) is what actually switches route selection over to it, by
    // making GetMetrics() return only the metric(s) explicitly added here
    // instead of {hop-count, ...explicit}. This is the only state change
    // SetEnableLinkQualityMetric makes -- it never touches the hop-count
    // metric's own behavior, which remains reachable (and is still what
    // runs) whenever this attribute is left at its default of false.
    AddMetric(BuildLinkQualityMetric());
    SetUseDefaultMetric(false);
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::HookLinkQualitySnooping(Ptr<NetDevice> device)
{
    if (!m_enableLinkQualityMetric)
    {
        return;
    }
    // Fix (docs/AODVV2_LQ_FIX1_IMPLEMENTATION_REPORT.md): `device` is a
    // WifiNetDevice directly under adhoc L2 mode, but under this project's
    // mesh11s-l3 mode it is a MeshPointDevice, which does not aggregate a
    // WifiNetDevice at all -- its real radio interfaces live in a separate
    // list, MeshPointDevice::GetInterfaces(). The original code only tried
    // device->GetObject<WifiNetDevice>(), which is always null for a
    // MeshPointDevice, so the trace below was never connected in this
    // project's own scenario. Collect every WifiNetDevice this call could
    // plausibly mean (the device itself, or each of a MeshPointDevice's
    // interfaces) and hook each one exactly once.
    std::vector<Ptr<WifiNetDevice> > wifiDevicesToHook;
    Ptr<WifiNetDevice> directWifiDev = device->GetObject<WifiNetDevice>();
    if (directWifiDev)
    {
        wifiDevicesToHook.push_back(directWifiDev);
    }
    else
    {
        Ptr<MeshPointDevice> meshDev = device->GetObject<MeshPointDevice>();
        if (meshDev)
        {
            // This project's scenario always configures exactly one
            // interface per mesh point (mesh.SetNumberOfInterfaces(1) in
            // scratch/five-floor-building-mesh.cc, confirmed by
            // inspection, not assumed); every interface is hooked here
            // regardless, so this is correct even if that configuration
            // ever changes.
            for (const Ptr<NetDevice>& iface : meshDev->GetInterfaces())
            {
                Ptr<WifiNetDevice> ifaceWifiDev = iface->GetObject<WifiNetDevice>();
                if (ifaceWifiDev)
                {
                    wifiDevicesToHook.push_back(ifaceWifiDev);
                }
            }
        }
    }
    for (const Ptr<WifiNetDevice>& wifiDev : wifiDevicesToHook)
    {
        if (m_hookedPhys.count(PeekPointer(wifiDev)) > 0)
        {
            // Already connected (e.g. NotifyInterfaceUp firing more than
            // once for the same device); avoid a duplicate trace
            // connection, which would otherwise double-count every
            // observation in m_neighborEtx and in the diagnostic CSVs.
            continue;
        }
        m_hookedPhys.insert(PeekPointer(wifiDev));
        if (!m_phyErrorRateModel)
        {
            Ptr<YansWifiPhy> yansPhy = DynamicCast<YansWifiPhy>(wifiDev->GetPhy());
            if (yansPhy)
            {
                m_phyErrorRateModel = yansPhy->GetErrorRateModel();
            }
        }
        wifiDev->GetPhy()->TraceConnectWithoutContext(
            "MonitorSnifferRx",
            MakeCallback(&Aodvv2RoutingProtocol<T>::SnoopPhyRx, this));
    }
}

template <typename T>
Metric<typename Aodvv2RoutingProtocol<T>::IpAddress>
Aodvv2RoutingProtocol<T>::BuildLinkQualityMetric()
{
    Aodvv2RoutingProtocol<T>* self = this;
    auto clampToByte = [](double value) -> uint8_t {
        if (!std::isfinite(value))
        {
            value = 255.0;
        }
        value = std::round(value);
        value = std::max(1.0, std::min(255.0, value));
        return static_cast<uint8_t>(value);
    };
    return Metric<IpAddress>(
        AODVV2_METRIC_SNR_ETX,
        1,   // metricSize: one byte, the same wire width as hop count
        255, // maxMetric: the same bound as hop count
        [self, clampToByte](const MetricNode&) -> uint8_t* {
            // Cost of the single radio hop this message was just received
            // over: the predicted ETX of its sender, from m_currentHopEtx
            // (set once per received message in RecvAodvv2 -- see that
            // function and the member's own declaration for why MetricNode
            // itself cannot be used here).
            uint8_t* cost = new uint8_t[1];
            cost[0] = clampToByte(self->m_currentHopEtx);
            std::string diagPath = AodvV2LqDiagnosticPath("metric-cost-calc.csv");
            if (!diagPath.empty())
            {
                uint32_t nodeId = self->m_ip->template GetObject<Node>()->GetId();
                std::ofstream out(diagPath.c_str(), std::ios::app);
                out << Simulator::Now().GetSeconds() << "," << nodeId << ",LinkCost,,"
                    << self->m_currentHopEtx << "," << static_cast<uint32_t>(cost[0]) << ","
                    << (cost[0] == 255 ? 1 : 0) << "\n";
            }
            return cost;
        },
        [self, clampToByte](const uint8_t* routeCost, const MetricNode&) -> uint8_t* {
            // Path cost accumulates additively: the sum of per-link ETX
            // values along the path, which is the standard definition of
            // path ETX, in the same "add this hop's contribution" shape as
            // hop count's own RouteCost (routeCost[0] + 1).
            uint8_t* newCost = new uint8_t[1];
            newCost[0] = clampToByte(static_cast<double>(routeCost[0]) + self->m_currentHopEtx);
            std::string diagPath = AodvV2LqDiagnosticPath("metric-cost-calc.csv");
            if (!diagPath.empty())
            {
                uint32_t nodeId = self->m_ip->template GetObject<Node>()->GetId();
                std::ofstream out(diagPath.c_str(), std::ios::app);
                out << Simulator::Now().GetSeconds() << "," << nodeId << ",RouteCost,"
                    << static_cast<uint32_t>(routeCost[0]) << "," << self->m_currentHopEtx << ","
                    << static_cast<uint32_t>(newCost[0]) << ","
                    << (newCost[0] == 255 ? 1 : 0) << "\n";
            }
            return newCost;
        },
        [](const uint8_t* r1, const uint8_t* r2) {
            // Lower accumulated ETX is the better (loop-free-preferred)
            // route, the same "smaller is better" comparison hop count
            // already uses.
            return r1[0] <= r2[0];
        });
}

template <typename T>
Mac48Address
Aodvv2RoutingProtocol<T>::ResolveNeighborMac(Ipv4Address ip) const
{
    // Fix-3 investigation: every status label below names a real,
    // currently-existing condition in this function or a real state
    // from ns3::ArpCache::Entry (IsAlive/IsDead/IsWaitReply/IsPermanent --
    // src/internet/model/arp-cache.h); none are invented. This block is
    // diagnostic-only: it reads m_etxArpCache and the ArpCache::Entry it
    // finds (both already read by the unmodified logic below), and writes
    // a gated CSV row. It does not change which branch is taken, what is
    // returned, or anything about the ARP cache's own state.
    std::string diagPath = AodvV2LqDiagnosticPath("arp-resolution-attempts.csv");
    std::string status;
    Mac48Address resolvedMac;
    bool hasCache = static_cast<bool>(m_etxArpCache);
    bool hasEntry = false;
    if (!hasCache)
    {
        status = "NO_ARP_CACHE";
    }
    else
    {
        ArpCache::Entry* entry = m_etxArpCache->Lookup(ip);
        hasEntry = (entry != nullptr);
        if (!hasEntry)
        {
            status = "NO_ARP_ENTRY";
        }
        else if (entry->IsAlive())
        {
            status = "ARP_ENTRY_ALIVE";
            resolvedMac = Mac48Address::ConvertFrom(entry->GetMacAddress());
        }
        else if (entry->IsPermanent())
        {
            status = "ARP_ENTRY_PERMANENT";
            resolvedMac = Mac48Address::ConvertFrom(entry->GetMacAddress());
        }
        else if (entry->IsWaitReply())
        {
            status = "ARP_ENTRY_WAIT_REPLY";
        }
        else if (entry->IsDead())
        {
            status = "ARP_ENTRY_DEAD";
        }
        else
        {
            // Every state ArpCache::Entry can report (Alive, Dead,
            // WaitReply, Permanent) is checked above; this branch is
            // unreachable under the current ns3::ArpCache::Entry
            // implementation, kept only so an unrecognized future state
            // is reported honestly instead of silently mis-sorted.
            status = "UNKNOWN_STATE";
        }
    }

    if (!diagPath.empty())
    {
        uint32_t nodeId = m_ip ? m_ip->template GetObject<Node>()->GetId() : 0;
        std::ofstream out(diagPath.c_str(), std::ios::app);
        out << Simulator::Now().GetSeconds() << "," << nodeId << "," << ip << "," << hasCache
            << "," << hasEntry << "," << status << "," << resolvedMac << "\n";
    }

    // Unmodified from before Fix 3: only Alive or Permanent entries are
    // treated as resolved; every other case (no cache, no entry,
    // WaitReply, Dead, or an unrecognized state) returns the same
    // all-zero sentinel it always did.
    if (status == "ARP_ENTRY_ALIVE" || status == "ARP_ENTRY_PERMANENT")
    {
        return resolvedMac;
    }
    return Mac48Address();
}

template <typename T>
Mac48Address
Aodvv2RoutingProtocol<T>::ResolveLastHopMacFromTag(Ptr<const Packet> packet) const
{
    // Fix 4 (docs/AODVV2_LQ_FIX4_IMPLEMENTATION_REPORT.md): replaces ARP
    // (Fix 3 found it architecturally unable to supply this, 98.83%
    // failure) as the source of the immediate previous hop's MAC. Reads
    // MeshLastHopMacTag (src/network/model/mesh-last-hop-mac-tag.h),
    // attached by MeshPointDevice::ReceiveFromDevice() from that
    // function's own src48 -- the authoritative, no-indirection answer to
    // "who physically transmitted this exact frame to me" -- for every
    // reception, including a packet's first hop from its true origin
    // (where MeshPreviousHopTag, which only a relay sets, is absent).
    // Never uses ARP. Validated the same way Fix 2 validates SnoopPhyRx's
    // observations: a zero, broadcast, or group address is never treated
    // as resolved, regardless of what the tag happens to contain.
    MeshLastHopMacTag lastHopTag;
    if (!packet->PeekPacketTag(lastHopTag))
    {
        return Mac48Address();
    }
    Mac48Address mac = lastHopTag.GetAddress();
    if (mac == Mac48Address() || mac.IsBroadcast() || mac.IsGroup())
    {
        return Mac48Address();
    }
    return mac;
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::SnoopPhyRx(Ptr<const Packet> packet,
                                     uint16_t channelFreqMhz,
                                     uint16_t channelNumber,
                                     uint32_t rate,
                                     bool isShortPreamble,
                                     WifiTxVector txVector,
                                     double signalDbm,
                                     double noiseDbm)
{
    WifiMacHeader hdr;
    if (!packet->PeekHeader(hdr))
    {
        return;
    }
    // Fix 2 (docs/AODVV2_LQ_FIX2_IMPLEMENTATION_REPORT.md): ACK and CTS
    // frames do not carry a transmitter address at all in their over-the-
    // air format (confirmed from this fork's own WifiMacHeader::GetSize():
    // both are FrameControl+Duration+Addr1 only, 10 bytes, no Addr2 field
    // whatsoever -- unlike RTS/BlockAckReq/BlockAck, which do include
    // Addr2). hdr.GetAddr2() on one of these two types therefore returns
    // whatever the header object's Addr2 happened to hold before this
    // Deserialize() call, observed in practice to be the all-zero
    // Mac48Address(). Skip them outright, by frame type, rather than by
    // guessing from the resulting address alone.
    std::string rejectReason;
    if (hdr.IsAck() || hdr.IsCts())
    {
        rejectReason = "no-addr2-frame-type";
    }
    Mac48Address sender;
    if (rejectReason.empty())
    {
        sender = hdr.GetAddr2();
        // Defense in depth, independent of the frame-type check above:
        // never accept an observation under a key that is not a genuine
        // single neighbor's own unicast address. Mac48Address() (all-
        // zero) is also the sentinel ResolveNeighborMac() returns for
        // "ARP resolution failed" (aodvv2-routing-protocol.cc) -- storing
        // a real observation under that exact key is what caused Fix 1's
        // validation to find 98.8% of hop-ETX resolutions were spurious
        // (matching this unrelated data, not the intended neighbor).
        // Rejecting it here, at the only place anything is ever written
        // into m_neighborEtx, removes the collision without touching
        // ResolveNeighborMac()'s sentinel, the ETX formula, the fallback
        // value, or route selection at all.
        if (sender == Mac48Address())
        {
            rejectReason = "zero-address";
        }
        else if (sender.IsBroadcast())
        {
            rejectReason = "broadcast-address";
        }
        else if (sender.IsGroup())
        {
            rejectReason = "multicast-address";
        }
    }

    std::string obsDiagPath = AodvV2LqDiagnosticPath("snr-etx-observations.csv");
    if (!rejectReason.empty())
    {
        if (!obsDiagPath.empty())
        {
            uint32_t nodeId = m_ip ? m_ip->template GetObject<Node>()->GetId() : 0;
            std::ofstream out(obsDiagPath.c_str(), std::ios::app);
            // Same 10-column layout as the ACCEPTED row below (time,node,
            // status,senderMac,signalDbm,noiseDbm,snrDb,successRate,etx,
            // rejectReason), with the 5 PHY/ETX fields left empty here --
            // never guessed or filled with a placeholder number.
            out << Simulator::Now().GetSeconds() << "," << nodeId << ",REJECTED," << sender
                << ",,,,,," << rejectReason << "\n";
        }
        return; // never write a rejected observation into m_neighborEtx
    }

    double snrDb = signalDbm - noiseDbm;
    double snrLinear = std::pow(10.0, snrDb / 10.0);
    double successRate = 1.0;
    if (m_phyErrorRateModel)
    {
        successRate =
            m_phyErrorRateModel->GetChunkSuccessRate(txVector.GetMode(), snrLinear,
                                                      packet->GetSize() * 8);
    }
    successRate = std::max(1.0e-6, std::min(1.0, successRate));
    double etx = 1.0 / successRate;
    m_neighborEtx[sender] = etx;

    // Fix 5: the actually-used record. Keyed by this exact packet's UID,
    // not by `sender` (which Fix 5's root-cause report confirmed is the
    // *original source's* address for any relayed frame, not the true
    // immediate transmitter -- see m_recentPhyEtxByUid's own declaration
    // in the header for the full reasoning). Pruned on every write using
    // the same windowed-cache pattern already used elsewhere in this
    // project (NoHwmpRoutingProtocol::m_recentlyRelayedGroupPackets).
    for (auto it = m_recentPhyEtxByUid.begin(); it != m_recentPhyEtxByUid.end();)
    {
        if (Simulator::Now() - it->second.second > kPhyEtxUidWindow)
        {
            it = m_recentPhyEtxByUid.erase(it);
        }
        else
        {
            ++it;
        }
    }
    m_recentPhyEtxByUid[packet->GetUid()] = std::make_pair(etx, Simulator::Now());

    if (!obsDiagPath.empty())
    {
        uint32_t nodeId = m_ip ? m_ip->template GetObject<Node>()->GetId() : 0;
        std::ofstream out(obsDiagPath.c_str(), std::ios::app);
        out << Simulator::Now().GetSeconds() << "," << nodeId << ",ACCEPTED," << sender << ","
            << signalDbm << "," << noiseDbm << "," << snrDb << "," << successRate << "," << etx
            << ",\n";
    }
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::SetMaxQueueLen(uint32_t len)
{
    m_maxQueueLen = len;
    m_queue.SetMaxQueueLen(len);
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::SetMaxQueueTime(Time t)
{
    m_maxQueueTime = t;
    m_queue.SetQueueTimeout(t);
}

template <typename T>
Aodvv2RoutingProtocol<T>::~Aodvv2RoutingProtocol()
{
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::DoDispose()
{
    m_ip = nullptr;
    for (auto iter = m_socketAddresses.begin(); iter != m_socketAddresses.end(); iter++)
    {
        iter->first->Close();
    }
    m_socketAddresses.clear();
    for (auto iter = m_socketSubnetBroadcastAddresses.begin();
         iter != m_socketSubnetBroadcastAddresses.end();
         iter++)
    {
        iter->first->Close();
    }
    m_socketSubnetBroadcastAddresses.clear();
    IpRoutingProtocol::DoDispose();
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::PrintRoutingTable(Ptr<OutputStreamWrapper> stream) const
{
    Time::Unit unit = Time::S;
    *stream->GetStream() << "Node: " << m_ip->template GetObject<Node>()->GetId()
                         << "; IP: " << GetInterfaceAddress(m_ip->GetAddress(1, 0))
                         << "; Time: " << Now().As(unit) << ", Local time: "
                         << m_ip->template GetObject<Node>()->GetLocalTime().As(unit)
                         << ", AODVv2 Routing table" << std::endl;

    m_routingTable.Print(stream, unit);
    m_nb.Print(stream, unit);
    *stream->GetStream() << std::endl;
}

template <typename T>
int64_t
Aodvv2RoutingProtocol<T>::AssignStreams(int64_t stream)
{
    NS_LOG_FUNCTION(this << stream);
    m_uniformRandomVariable->SetStream(stream);
    return 1;
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::Start()
{
    NS_LOG_FUNCTION(this);
    m_routingTable.ScheduleTimer();

    m_rreqRateLimitTimer.SetFunction(&Aodvv2RoutingProtocol<T>::RreqRateLimitTimerExpire, this);
    m_rreqRateLimitTimer.Schedule(Seconds(1));

    m_rrepRateLimitTimer.SetFunction(&Aodvv2RoutingProtocol<T>::RrepRateLimitTimerExpire, this);
    m_rrepRateLimitTimer.Schedule(Seconds(1));

    m_rerrRateLimitTimer.SetFunction(&Aodvv2RoutingProtocol<T>::RerrRateLimitTimerExpire, this);
    m_rerrRateLimitTimer.Schedule(Seconds(1));
}

template <typename T>
Ptr<typename Aodvv2RoutingProtocol<T>::IpRoute>
Aodvv2RoutingProtocol<T>::RouteOutput(Ptr<Packet> p,
                                      const IpHeader& header,
                                      Ptr<NetDevice> oif,
                                      Socket::SocketErrno& sockerr)
{
    NS_LOG_FUNCTION(this << header << (oif ? oif->GetIfIndex() : 0));
    if (!p)
    {
        NS_LOG_DEBUG("Packet is == 0");
        return LoopbackRoute(header, oif); // later
    }
    if (m_socketAddresses.empty())
    {
        sockerr = Socket::ERROR_NOROUTETOHOST;
        NS_LOG_LOGIC("No aodvv2 interfaces");
        Ptr<IpRoute> route;
        return route;
    }
    sockerr = Socket::ERROR_NOTERROR;
    Ptr<IpRoute> route;
    IpAddress dst = header.GetDestination();
    LocalRoute<IpAddress> rt;
    if (m_routingTable.LookupBestRoute(dst, rt))
    {
        route = rt.GetRoute();
        NS_ASSERT(route);
        NS_LOG_DEBUG("Exist route to " << route->GetDestination() << " from interface "
                                       << route->GetSource());
        if (oif && route->GetOutputDevice() != oif)
        {
            NS_LOG_DEBUG("Output device doesn't match. Dropped.");
            sockerr = Socket::ERROR_NOROUTETOHOST;
            return Ptr<IpRoute>();
        }
        UpdateRouteLifeTime(dst, m_activeInterval);
        UpdateRouteLifeTime(route->GetGateway(), m_activeInterval);
        return route;
    }

    // Valid route not found, in this case we return loopback.
    // Actual route request will be deferred until packet will be fully formed,
    // routed to loopback, received from loopback and passed to RouteInput (see below)
    uint32_t iif = (oif ? m_ip->GetInterfaceForDevice(oif) : -1);
    DeferredRouteOutputTag tag(iif);
    NS_LOG_DEBUG("Valid Route not found");
    if (!p->PeekPacketTag(tag))
    {
        p->AddPacketTag(tag);
    }
    return LoopbackRoute(header, oif);
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::DeferredRouteOutput(Ptr<const Packet> p,
                                              const IpHeader& header,
                                              UnicastForwardCallback ucb,
                                              ErrorCallback ecb)
{
    NS_LOG_FUNCTION(this << p << header);
    NS_ASSERT(p && p != Ptr<Packet>());

    QueueEntry<IpHeader> newEntry(p, header, ucb, ecb);
    bool result = m_queue.Enqueue(newEntry);
    if (result)
    {
        NS_LOG_LOGIC("Add packet " << p->GetUid() << " to queue.");
        std::vector<LocalRoute<IpAddress>> routes;
        bool result = m_routingTable.LookupRoutes(header.GetDestination(), routes);
        if (result)
        {
            for (LocalRoute<IpAddress> route : routes)
            {
                if (route.GetState() != UNCONFIRMED)
                {
                    NS_LOG_LOGIC("Send new RREQ for outbound packet to "
                                 << header.GetDestination());
                    SendRequest(header.GetDestination());
                    return;
                }
            }
        }
        else
        {
            NS_LOG_LOGIC("Send new RREQ for outbound packet to " << header.GetDestination());
            SendRequest(header.GetDestination());
        }
    }
}

template <typename T>
bool
Aodvv2RoutingProtocol<T>::RouteInput(Ptr<const Packet> p,
                                     const IpHeader& header,
                                     Ptr<const NetDevice> idev,
                                     UnicastForwardCallback ucb,
                                     MulticastForwardCallback mcb,
                                     LocalDeliverCallback lcb,
                                     ErrorCallback ecb)
{
    NS_LOG_FUNCTION(this << p->GetUid() << header.GetDestination() << idev->GetAddress());
    if (m_socketAddresses.empty())
    {
        NS_LOG_LOGIC("No aodvv2 interfaces");
        return false;
    }
    NS_ASSERT(m_ip);
    NS_ASSERT(p);
    // Check if input device supports IP
    NS_ASSERT(m_ip->GetInterfaceForDevice(idev) >= 0);
    int32_t iif = m_ip->GetInterfaceForDevice(idev);

    IpAddress dst = header.GetDestination();
    IpAddress origin = header.GetSource();

    // Deferred route request
    if (idev == m_lo)
    {
        DeferredRouteOutputTag tag;
        if (p->PeekPacketTag(tag))
        {
            DeferredRouteOutput(p, header, ucb, ecb);
            return true;
        }
    }

    // Duplicate of own packet
    if (IsMyOwnAddress(origin))
    {
        return true;
    }

    // AODVv2 is not a multicast routing protocol
    if (dst.IsMulticast())
    {
        return false;
    }

    // Broadcast local delivery/forwarding
    for (auto j = m_socketAddresses.begin(); j != m_socketAddresses.end(); ++j)
    {
        IpInterfaceAddress iface = j->second;
        if (m_ip->GetInterfaceForAddress(GetInterfaceAddress(iface)) == iif)
        {
            if constexpr (std::is_same<T, Ipv4RoutingProtocol>::value)
            {
                if (dst == iface.GetBroadcast() || dst.IsBroadcast())
                {
                    UpdateRouteLifeTime(origin, m_activeInterval);
                    Ptr<Packet> packet = p->Copy();
                    if (!lcb.IsNull())
                    {
                        NS_LOG_LOGIC("Broadcast local delivery to " << GetInterfaceAddress(iface));
                        lcb(p, header, iif);
                        // Fall through to additional processing
                    }
                    else
                    {
                        NS_LOG_ERROR("Unable to deliver packet locally due to null callback "
                                     << p->GetUid() << " from " << origin);
                        ecb(p, header, Socket::ERROR_NOROUTETOHOST);
                    }
                    if (!m_enableBroadcast)
                    {
                        return true;
                    }
                    if (header.GetProtocol() == UdpL4Protocol::PROT_NUMBER)
                    {
                        UdpHeader udpHeader;
                        p->PeekHeader(udpHeader);
                        if (udpHeader.GetDestinationPort() == AODVV2_PORT)
                        {
                            // AODVv2 packets sent in broadcast are already managed
                            return true;
                        }
                    }
                    if (header.GetTtl() > 1)
                    {
                        NS_LOG_LOGIC("Forward broadcast. TTL " << (uint16_t)header.GetTtl());
                        std::vector<LocalRoute<IpAddress>> routes;
                        if (m_routingTable.LookupRoutes(dst, routes))
                        {
                            Ptr<IpRoute> route = routes[0].GetRoute();
                            ucb(route, packet, header);
                        }
                        else
                        {
                            NS_LOG_DEBUG("No route to forward broadcast. Drop packet "
                                         << p->GetUid());
                        }
                    }
                    else
                    {
                        NS_LOG_DEBUG("TTL exceeded. Drop packet " << p->GetUid());
                    }
                    return true;
                }

                // Unicast local delivery
                if (m_ip->IsDestinationAddress(dst, iif))
                {
                    UpdateRouteLifeTime(origin, m_activeInterval);
                    std::vector<LocalRoute<IpAddress>> routes;
                    if (m_routingTable.LookupValidRoutes(origin, routes))
                    {
                        for (LocalRoute<IpAddress>& toOrigin : routes)
                        {
                            UpdateRouteLifeTime(toOrigin.GetNextHop(), m_activeInterval);
                            m_nb.AddNeighbor(toOrigin.GetNextHop(), iface);
                        }
                    }
                    if (!lcb.IsNull())
                    {
                        NS_LOG_LOGIC("Unicast local delivery to " << dst);
                        lcb(p, header, iif);
                    }
                    else
                    {
                        NS_LOG_ERROR("Unable to deliver packet locally due to null callback "
                                     << p->GetUid() << " from " << origin);
                        ecb(p, header, Socket::ERROR_NOROUTETOHOST);
                    }
                    return true;
                }
            }
            else
            {
                // TODO Ipv6 multicast and unicast
            }
        }
    }

    // Check if input device supports IP forwarding
    if (!m_ip->IsForwarding(iif))
    {
        NS_LOG_LOGIC("Forwarding disabled for this interface");
        ecb(p, header, Socket::ERROR_NOROUTETOHOST);
        return true;
    }

    // Forwarding
    return Forwarding(p, header, ucb, ecb);
}

template <typename T>
bool
Aodvv2RoutingProtocol<T>::Forwarding(Ptr<const Packet> p,
                                     const IpHeader& header,
                                     UnicastForwardCallback ucb,
                                     ErrorCallback ecb)
{
    NS_LOG_FUNCTION(this);
    IpAddress dst = header.GetDestination();
    IpAddress origin = header.GetSource();
    m_routingTable.Purge();
    std::vector<LocalRoute<IpAddress>> routes;
    if (m_routingTable.LookupValidRoutes(dst, routes))
    {
        LocalRoute<IpAddress> toDst = routes[0];
        if (toDst.IsValid())
        {
            Ptr<IpRoute> route = toDst.GetRoute();
            NS_LOG_LOGIC(route->GetSource() << " forwarding to " << dst << " from " << origin
                                            << " packet " << p->GetUid());

            /*
             *  Each time a route is used to forward a data packet, its Active Route
             *  Lifetime field of the source, destination and the next hop on the
             *  path to the destination is updated to be no less than the current
             *  time plus ActiveRouteTimeout.
             */
            UpdateRouteLifeTime(origin, m_activeInterval);
            UpdateRouteLifeTime(dst, m_activeInterval);
            UpdateRouteLifeTime(route->GetGateway(), m_activeInterval);
            /*
             *  Since the route between each originator and destination pair is expected to be
             * symmetric, the Active Route Lifetime for the previous hop, along the reverse path
             * back to the IP source, is also updated to be no less than the current time plus
             * ActiveRouteTimeout
             */
            std::vector<LocalRoute<IpAddress>> routes;
            m_routingTable.LookupRoutes(origin, routes);
            for (LocalRoute<IpAddress>& toOrigin : routes)
            {
                UpdateRouteLifeTime(toOrigin.GetNextHop(), m_activeInterval);

                m_nb.AddNeighbor(route->GetGateway(), toDst.GetInterface());
                m_nb.AddNeighbor(toOrigin.GetNextHop(), toDst.GetInterface());
                if constexpr (std::is_same<T, Ipv4RoutingProtocol>::value)
                {
                    ucb(route, p, header);
                }
                else
                {
                    // TODO Ipv6
                }
            }
            return true;
        }
        else
        {
            if (toDst.GetValidSeqNo())
            {
                SendRerrWhenNoRouteToForward(dst, toDst.GetSeqNo(), origin);
                NS_LOG_DEBUG("Drop packet " << p->GetUid() << " because no route to forward it.");
                return false;
            }
        }
    }
    NS_LOG_LOGIC("route not found to " << dst << ". Send RERR message.");
    NS_LOG_DEBUG("Drop packet " << p->GetUid() << " because no route to forward it.");
    SendRerrWhenNoRouteToForward(dst, 0, origin);
    return false;
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::SetIpv4(Ptr<Ipv4> ipv4)
{
    if constexpr (std::is_same<T, Ipv4RoutingProtocol>::value)
    {
        NS_ASSERT(ipv4);
        NS_ASSERT(!m_ip);

        m_ip = ipv4;

        // Create lo route. It is asserted that the only one interface up for now is loopback
        NS_ASSERT(m_ip->GetNInterfaces() == 1 &&
                  GetInterfaceAddress(m_ip->GetAddress(0, 0)) == IpAddress::GetLoopback());
        m_lo = m_ip->GetNetDevice(0);
        NS_ASSERT(m_lo);
        // Remember lo route

        /* removed localhost local route
         for (Metric<IpAddress> metric : GetMetrics())
         {
             LocalRoute<Ipv4Address> rt(
                 m_lo,
                 Ipv4Address::GetLoopback(),
                 0,
                 Ipv4InterfaceAddress(Ipv4Address::GetLoopback(), Ipv4Mask("255.0.0.0")),
                 1,
                 Ipv4Address::GetLoopback(),
                 Simulator::GetMaximumSimulationTime(),
                 m_maxIdleTime,
                 metric,
                 metric.LinkCost(GetMetricNode(m_ip->template GetObject<Node>())));
             m_routingTable.AddRoute(rt);
         } */
        Simulator::ScheduleNow(&Aodvv2RoutingProtocol<T>::Start, this);
    }
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::SetIpv6(Ptr<Ipv6> ipv6)
{
    if constexpr (std::is_same<T, Ipv6RoutingProtocol>::value)
    {
        NS_ASSERT(ipv6);
        NS_ASSERT(!m_ip);

        m_ip = ipv6;

        // TODO Ipv6
    }
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::NotifyInterfaceUp(uint32_t i)
{
    NS_LOG_FUNCTION(this << GetInterfaceAddress(m_ip->GetAddress(i, 0)));
    Ptr<IpL3Protocol> l3 = m_ip->template GetObject<IpL3Protocol>();
    if (l3->GetNAddresses(i) > 1)
    {
        NS_LOG_WARN("AODVv2 does not work with more than one address per each interface.");
    }
    IpInterfaceAddress iface = l3->GetAddress(i, 0);
    if (GetInterfaceAddress(iface) == IpAddress::GetLoopback())
    {
        return;
    }

    // Create a socket to listen only on this interface
    Ptr<Socket> socket =
        Socket::CreateSocket(m_ip->template GetObject<Node>(), UdpSocketFactory::GetTypeId());
    NS_ASSERT(socket);
    socket->SetRecvCallback(MakeCallback(&Aodvv2RoutingProtocol<T>::RecvAodvv2, this));
    socket->Bind(InetVxSocketAddress(GetInterfaceAddress(iface), AODVV2_PORT));
    socket->BindToNetDevice(l3->GetNetDevice(i));
    socket->SetAllowBroadcast(true);
    socket->SetIpRecvTtl(true);
    m_socketAddresses.insert(std::make_pair(socket, iface));

    // create also a subnet broadcast socket
    socket = Socket::CreateSocket(m_ip->template GetObject<Node>(), UdpSocketFactory::GetTypeId());
    NS_ASSERT(socket);
    socket->SetRecvCallback(MakeCallback(&Aodvv2RoutingProtocol<T>::RecvAodvv2, this));
    socket->SetAllowBroadcast(true);
    socket->SetIpRecvTtl(true);
    if constexpr (std::is_same<T, Ipv4RoutingProtocol>::value)
    {
        // iface.GetBroadcast() resolves to this node's own unicast address
        // under the /32 (all-ones) addressing this harness requires for
        // correct next-hop selection (see EXPERIMENT_METHODOLOGY.md), so
        // binding this receive-only socket to it would leave AODVv2 unable
        // to ever receive a real 255.255.255.255-addressed broadcast -- the
        // same defect already found and fixed for OLSR
        // (src/olsr/model/olsr-routing-protocol.cc). The send side already
        // guards this correctly a few lines above; this socket bind needs
        // the same guard.
        Ipv4Address bcastBindAddr = (iface.GetMask() == Ipv4Mask::GetOnes ())
          ? Ipv4Address::GetBroadcast ()
          : iface.GetBroadcast ();
        socket->Bind(InetVxSocketAddress(bcastBindAddr, AODVV2_PORT));
        socket->BindToNetDevice(l3->GetNetDevice(i));
        if (l3->GetInterface(i)->GetArpCache())
        {
            m_routingTable.AddArpCache(l3->GetInterface(i)->GetArpCache());
            // AODVv2-LQ (see aodvv2-routing-protocol.h): cache this
            // interface's ARP cache for resolving a received message's
            // sender IP to the MAC address m_neighborEtx is keyed by.
            // Harmless to set when the link-quality metric is disabled --
            // ResolveNeighborMac() is only ever called from the code path
            // that is itself gated on m_enableLinkQualityMetric.
            m_etxArpCache = l3->GetInterface(i)->GetArpCache();
        }
        // AODVv2-LQ: a no-op unless the link-quality metric is enabled
        // (checked inside HookLinkQualitySnooping itself).
        HookLinkQualitySnooping(l3->GetNetDevice(i));

        // Fix-3 investigation (docs/AODVV2_LQ_FIX3_ARP_DIAGNOSTIC_REPORT.md):
        // one-time, per-node record of exactly which device/ARP-cache
        // ResolveNeighborMac() will end up using, logged here (not inside
        // ResolveNeighborMac() itself) since this information is fixed at
        // setup time and does not need to be repeated on every lookup.
        // Pure diagnostic I/O, gated, no state read or written beyond
        // what NotifyInterfaceUp() already touches.
        if (m_enableLinkQualityMetric)
        {
            std::string setupDiagPath = AodvV2LqDiagnosticPath("arp-interface-setup.csv");
            if (!setupDiagPath.empty())
            {
                Ptr<NetDevice> dev = l3->GetNetDevice(i);
                Ptr<MeshPointDevice> meshDev = dev->GetObject<MeshPointDevice>();
                Ptr<WifiNetDevice> directWifiDev = dev->GetObject<WifiNetDevice>();
                uint32_t nodeId = m_ip->template GetObject<Node>()->GetId();
                std::ofstream out(setupDiagPath.c_str(), std::ios::app);
                out << nodeId << "," << (meshDev ? 1 : 0) << "," << (directWifiDev ? 1 : 0) << ","
                    << GetInterfaceAddress(iface) << "," << iface.GetMask() << ","
                    << (m_etxArpCache ? 1 : 0) << ","
                    << (m_etxArpCache ? m_etxArpCache->GetDevice()->GetAddress() : Address())
                    << "\n";
            }
        }
    }
    else
    {
        // TODO Ipv6
    }
    m_socketSubnetBroadcastAddresses.insert(std::make_pair(socket, iface));

    Ptr<NetDevice> dev = m_ip->GetNetDevice(m_ip->GetInterfaceForAddress(GetInterfaceAddress(iface)));

    // Allow neighbor manager use this interface for layer 2 feedback if possible
    Ptr<WifiNetDevice> wifi = dev->GetObject<WifiNetDevice>();
    if (!wifi)
    {
        return;
    }
    Ptr<WifiMac> mac = wifi->GetMac();
    if (!mac)
    {
        return;
    }

   // mac->TraceConnectWithoutContext("DroppedMpdu",
     //                               MakeCallback(&Aodvv2RoutingProtocol<T>::NotifyTxError, this));
}

//template <typename T>
//void
//Aodvv2RoutingProtocol<T>::NotifyTxError(WifiMacDropReason reason, Ptr<const WifiMpdu> mpdu)
//{
//    m_routingTable.GetTxErrorCallback()(mpdu->GetHeader());
//}

template <typename T>
void
Aodvv2RoutingProtocol<T>::NotifyInterfaceDown(uint32_t i)
{
    NS_LOG_FUNCTION(this << GetInterfaceAddress(m_ip->GetAddress(i, 0)));

    // Disable layer 2 link state monitoring (if possible)
    Ptr<IpL3Protocol> l3 = m_ip->template GetObject<IpL3Protocol>();
    Ptr<NetDevice> dev = l3->GetNetDevice(i);
    Ptr<WifiNetDevice> wifi = dev->GetObject<WifiNetDevice>();
    if (wifi)
    {
        Ptr<WifiMac> mac = wifi->GetMac()->GetObject<AdhocWifiMac>();
        if (mac)
        {
//            mac->TraceDisconnectWithoutContext(
//                "DroppedMpdu",
//                MakeCallback(&Aodvv2RoutingProtocol<T>::NotifyTxError, this));
            if constexpr (std::is_same<T, Ipv4RoutingProtocol>::value)
            {
                m_routingTable.DelArpCache(l3->GetInterface(i)->GetArpCache());
            }
            else
            {
                // TODO Ipv6
            }
        }
    }

    // Close socket
    Ptr<Socket> socket = FindSocketWithInterfaceAddress(m_ip->GetAddress(i, 0));
    NS_ASSERT(socket);
    socket->Close();
    m_socketAddresses.erase(socket);

    // Close socket
    socket = FindSubnetBroadcastSocketWithInterfaceAddress(m_ip->GetAddress(i, 0));
    NS_ASSERT(socket);
    socket->Close();
    m_socketSubnetBroadcastAddresses.erase(socket);

    if (m_socketAddresses.empty())
    {
        NS_LOG_LOGIC("No aodvv2 interfaces");
        m_nb.Clear();
        m_routingTable.Clear();
        return;
    }
    m_routingTable.DeleteAllRoutesFromInterface(m_ip->GetAddress(i, 0));
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::NotifyAddAddress(uint32_t i, IpInterfaceAddress address)
{
    NS_LOG_FUNCTION(this << " interface " << i << " address " << address);
    Ptr<IpL3Protocol> l3 = m_ip->template GetObject<IpL3Protocol>();
    if (!l3->IsUp(i))
    {
        return;
    }
    if (l3->GetNAddresses(i) == 1)
    {
        IpInterfaceAddress iface = l3->GetAddress(i, 0);
        Ptr<Socket> socket = FindSocketWithInterfaceAddress(iface);
        if (!socket)
        {
            if (GetInterfaceAddress(iface) == IpAddress::GetLoopback())
            {
                return;
            }
            // Create a socket to listen only on this interface
            Ptr<Socket> socket =
                Socket::CreateSocket(DynamicCast<Node>(m_ip), UdpSocketFactory::GetTypeId());
            NS_ASSERT(socket);
            socket->SetRecvCallback(MakeCallback(&Aodvv2RoutingProtocol<T>::RecvAodvv2, this));
            socket->Bind(InetVxSocketAddress(GetInterfaceAddress(iface), AODVV2_PORT));
            socket->BindToNetDevice(l3->GetNetDevice(i));
            socket->SetAllowBroadcast(true);
            m_socketAddresses.insert(std::make_pair(socket, iface));

            // create also a subnet directed broadcast socket
            socket = Socket::CreateSocket(DynamicCast<Node>(m_ip), UdpSocketFactory::GetTypeId());
            NS_ASSERT(socket);
            socket->SetRecvCallback(MakeCallback(&Aodvv2RoutingProtocol<T>::RecvAodvv2, this));
            socket->SetAllowBroadcast(true);
            socket->SetIpRecvTtl(true);
            if constexpr (std::is_same<T, Ipv4RoutingProtocol>::value)
            {
                // See the matching comment in NotifyInterfaceUp() above --
                // iface.GetBroadcast() is useless under /32 addressing.
                Ipv4Address bcastBindAddr = (iface.GetMask() == Ipv4Mask::GetOnes ())
                  ? Ipv4Address::GetBroadcast ()
                  : iface.GetBroadcast ();
                socket->Bind(InetVxSocketAddress(bcastBindAddr, AODVV2_PORT));
                socket->BindToNetDevice(l3->GetNetDevice(i));
            }
            else
            {
                // TODO Ipv6
            }
            m_socketSubnetBroadcastAddresses.insert(std::make_pair(socket, iface));
        }
    }
    else
    {
        NS_LOG_LOGIC("AODVv2 does not work with more then one address per each interface. Ignore "
                     "added address");
    }
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::NotifyRemoveAddress(uint32_t i, IpInterfaceAddress address)
{
    NS_LOG_FUNCTION(this);
    Ptr<Socket> socket = FindSocketWithInterfaceAddress(address);
    if (socket)
    {
        m_routingTable.DeleteAllRoutesFromInterface(address);
        socket->Close();
        m_socketAddresses.erase(socket);

        Ptr<Socket> unicastSocket = FindSubnetBroadcastSocketWithInterfaceAddress(address);
        if (unicastSocket)
        {
            unicastSocket->Close();
            m_socketAddresses.erase(unicastSocket);
        }

        Ptr<IpL3Protocol> l3 = m_ip->template GetObject<IpL3Protocol>();
        if (l3->GetNAddresses(i))
        {
            IpInterfaceAddress iface = l3->GetAddress(i, 0);
            // Create a socket to listen only on this interface
            Ptr<Socket> socket =
                Socket::CreateSocket(DynamicCast<Node>(m_ip), UdpSocketFactory::GetTypeId());
            NS_ASSERT(socket);
            socket->SetRecvCallback(MakeCallback(&Aodvv2RoutingProtocol<T>::RecvAodvv2, this));
            // Bind to any IP address so that broadcasts can be received
            socket->Bind(InetVxSocketAddress(GetInterfaceAddress(iface), AODVV2_PORT));
            socket->BindToNetDevice(l3->GetNetDevice(i));
            socket->SetAllowBroadcast(true);
            socket->SetIpRecvTtl(true);
            m_socketAddresses.insert(std::make_pair(socket, iface));

            // create also a unicast socket
            socket = Socket::CreateSocket(DynamicCast<Node>(m_ip), UdpSocketFactory::GetTypeId());
            NS_ASSERT(socket);
            socket->SetRecvCallback(MakeCallback(&Aodvv2RoutingProtocol<T>::RecvAodvv2, this));
            socket->SetAllowBroadcast(true);
            socket->SetIpRecvTtl(true);
            if constexpr (std::is_same<T, Ipv4RoutingProtocol>::value)
            {
                // See the matching comment in NotifyInterfaceUp() above --
                // iface.GetBroadcast() is useless under /32 addressing.
                Ipv4Address bcastBindAddr = (iface.GetMask() == Ipv4Mask::GetOnes ())
                  ? Ipv4Address::GetBroadcast ()
                  : iface.GetBroadcast ();
                socket->Bind(InetVxSocketAddress(bcastBindAddr, AODVV2_PORT));
                socket->BindToNetDevice(l3->GetNetDevice(i));
            }
            else
            {
                // TODO Ipv6
            }
            m_socketSubnetBroadcastAddresses.insert(std::make_pair(socket, iface));
        }
        if (m_socketAddresses.empty())
        {
            NS_LOG_LOGIC("No aodvv2 interfaces");
            m_nb.Clear();
            m_routingTable.Clear();
            return;
        }
    }
    else
    {
        NS_LOG_LOGIC("Remove address not participating in AODVv2 operation");
    }
}

//template <typename T>
//void
//Aodvv2RoutingProtocol<T>::NotifyTxError(WifiMacDropReason reason, Ptr<const WifiMpdu> mpdu)
//{
//    m_routingTable.GetTxErrorCallback()(mpdu->GetHeader());
//}

template <typename T>
void
Aodvv2RoutingProtocol<T>::NotifyAddRoute(IpAddress dst,
                                         Ipv6Prefix mask,
                                         IpAddress nextHop,
                                         uint32_t interface,
                                         IpAddress prefixToUse)
{
    // TODO Ipv6
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::NotifyRemoveRoute(IpAddress dst,
                                            Ipv6Prefix mask,
                                            IpAddress nextHop,
                                            uint32_t interface,
                                            IpAddress prefixToUse)
{
    // TODO Ipv6
}

template <typename T>
bool
Aodvv2RoutingProtocol<T>::IsMyOwnAddress(IpAddress src)
{
    NS_LOG_FUNCTION(this << src);
    for (auto j = m_socketAddresses.begin(); j != m_socketAddresses.end(); ++j)
    {
        IpInterfaceAddress iface = j->second;
        if (src == GetInterfaceAddress(iface))
        {
            return true;
        }
    }
    return false;
}

template <typename T>
Ptr<typename Aodvv2RoutingProtocol<T>::IpRoute>
Aodvv2RoutingProtocol<T>::LoopbackRoute(const IpHeader& hdr, Ptr<NetDevice> oif) const
{
    NS_LOG_FUNCTION(this << hdr);
    NS_ASSERT(m_lo);
    Ptr<IpRoute> rt = Create<IpRoute>();
    rt->SetDestination(hdr.GetDestination());
    //
    // Source address selection here is tricky.  The loopback route is
    // returned when AODVv2 does not have a route; this causes the packet
    // to be looped back and handled (cached) in RouteInput() method
    // while a route is found. However, connection-oriented protocols
    // like TCP need to create an endpoint four-tuple (src, src port,
    // dst, dst port) and create a pseudo-header for checksumming.  So,
    // AODVv2 needs to guess correctly what the eventual source address
    // will be.
    //
    // For single interface, single address nodes, this is not a problem.
    // When there are possibly multiple outgoing interfaces, the policy
    // implemented here is to pick the first available AODVv2 interface.
    // If RouteOutput() caller specified an outgoing interface, that
    // further constrains the selection of source address
    //
    auto j = m_socketAddresses.begin();
    if (oif)
    {
        // Iterate to find an address on the oif device
        for (j = m_socketAddresses.begin(); j != m_socketAddresses.end(); ++j)
        {
            IpAddress addr = GetInterfaceAddress(j->second);
            int32_t interface = m_ip->GetInterfaceForAddress(addr);
            if (oif == m_ip->GetNetDevice(static_cast<uint32_t>(interface)))
            {
                rt->SetSource(addr);
                break;
            }
        }
    }
    else
    {
        rt->SetSource(GetInterfaceAddress(j->second));
    }
    NS_ASSERT_MSG(rt->GetSource() != IpAddress(), "Valid AODVv2 source address not found");
    rt->SetGateway(IpAddress::GetLoopback());
    rt->SetOutputDevice(m_lo);
    return rt;
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::SendRequest(IpAddress dst)
{
    NS_LOG_FUNCTION(this << dst);
    // A node SHOULD NOT originate more than RREQ_RATELIMIT RREQ messages per second.
    if (m_rreqCount == m_rreqRateLimit)
    {
        Time delayLeft = m_rreqRateLimitTimer.GetDelayLeft();
        if (delayLeft.IsNegative())
        {
            delayLeft = Seconds(0);
        }
        Simulator::Schedule(delayLeft + MicroSeconds(100),
                            &Aodvv2RoutingProtocol<T>::SendRequest,
                            this,
                            dst);
        return;
    }
    else
    {
        m_rreqCount++;
    }
    // Create RREQ header
    RreqHeader<IpAddress> rreqHeader;
    rreqHeader.SetTargIp(dst);
    rreqHeader.SetTargMask(32); // TODO me: update if needed

    LocalRoute<IpAddress> rt;
    // Using the Hop field in Routing Table to store the number of hops to the destination
    uint16_t hops = 1;
    for (Metric<IpAddress> metric : GetMetrics())
    {
        if (m_routingTable.LookupRoute(dst, metric.GetMetricType(), rt))
        {
            if (rt.GetState() != UNCONFIRMED)
            {
                hops = std::min<uint16_t>(rt.GetHop() + 2, m_maxHopLimit);
            }
            else
            {
                hops = rt.GetHop() + 2;
                if (hops > m_maxHopLimit)
                {
                    hops = m_maxHopLimit;
                }
            }
            rt.IncrementRreqCnt();
            rt.SetHop(hops);
            rt.SetState(UNCONFIRMED);
            rt.SetLastUsed();
            rreqHeader.AddMetric(metric.GetMetricType(),
                                 metric.LinkCost(GetMetricNode(m_ip->template GetObject<Node>())),
                                 metric.GetMetricSize());
            m_routingTable.Update(rt);
        }
        else
        {
            Ptr<NetDevice> dev = nullptr;
            for (Metric<IpAddress> metric : GetMetrics())
            {
                LocalRoute<IpAddress> newEntry(
                    /*dev=*/dev,
                    /*dst=*/dst,
                    /*seqNo=*/0,
                    /*iface=*/IpInterfaceAddress(),
                    /*hops=*/hops,
                    /*nextHop=*/IpAddress(),
                    /*maxIdleTime=*/m_maxIdleTime,
                    /*metric=*/metric,
                    /*metricValue=*/
                    metric.LinkCost(GetMetricNode(m_ip->template GetObject<Node>())));

                newEntry.SetState(UNCONFIRMED);
                rreqHeader.AddMetric(newEntry.GetMetricType(),
                                     newEntry.GetMetricValue(),
                                     newEntry.GetMetricSize());
                m_routingTable.AddRoute(newEntry);
            }
        }
    }
    m_seqNo++;
    rreqHeader.SetSeqNo(m_seqNo);
    rreqHeader.SetHopLimit(m_maxHopLimit);
    m_requestId++;

    // Send RREQ as subnet directed broadcast from each interface used by aodvv2
    for (auto j = m_socketAddresses.begin(); j != m_socketAddresses.end(); ++j)
    {
        Ptr<Socket> socket = j->first;
        IpInterfaceAddress iface = j->second;

        rreqHeader.SetOrigIp(GetInterfaceAddress(iface));
        rreqHeader.SetOrigMask(32); // TODO me: update if needed
        rreqHeader.SetRtrIp(
            GetInterfaceAddress(m_ip->GetAddress(m_ip->GetInterfaceForAddress(GetInterfaceAddress(iface)), 0)));
        rreqHeader.SetRtrMask(32); // TODO me: update if needed

        for (uint8_t type : rreqHeader.GetMetricTypes())
        {
            m_mms.IsDuplicate(
                GetInterfaceAddress(iface),
                32,
                dst,
                GetInterfaceAddress(m_ip->GetAddress(m_ip->GetInterfaceForAddress(GetInterfaceAddress(iface)), 0)),
                m_seqNo,
                iface,
                GetMetric(type),
                GetMetric(type).LinkCost(GetMetricNode(m_ip->template GetObject<Node>())));
        }

        Ptr<Packet> packet = Create<Packet>();

        packet->AddHeader(rreqHeader);

        // Send to all-hosts broadcast if on /32 addr, subnet-directed otherwise
        IpAddress destination;
        if constexpr (std::is_same<T, Ipv4RoutingProtocol>::value)
        {
            if (iface.GetMask() == Ipv4Mask::GetOnes())
            {
                destination = Ipv4Address("255.255.255.255");
            }
            else
            {
                destination = iface.GetBroadcast();
            }
        }
        else
        {
            // TODO Ipv6
        }
        NS_LOG_DEBUG("Send RREQ with seqNo " << rreqHeader.GetSeqNo() << " to socket");
        m_lastBcastTime = Simulator::Now();
        Simulator::Schedule(Time(MilliSeconds(m_uniformRandomVariable->GetInteger(0, 10))),
                            &Aodvv2RoutingProtocol<T>::SendTo,
                            this,
                            socket,
                            packet,
                            destination);
    }
    ScheduleRreqRetry(dst);
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::SendTo(Ptr<Socket> socket, Ptr<Packet> packet, IpAddress destination)
{
    socket->SendTo(packet, 0, InetVxSocketAddress(destination, AODVV2_PORT));
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::ScheduleRreqRetry(IpAddress dst)
{
    NS_LOG_FUNCTION(this << dst);
    if (m_addressReqTimer.find(dst) == m_addressReqTimer.end())
    {
        Timer timer(Timer::CANCEL_ON_DESTROY);
        m_addressReqTimer[dst] = timer;
    }
    m_addressReqTimer[dst].SetFunction(&Aodvv2RoutingProtocol<T>::RouteRequestTimerExpire, this);
    m_addressReqTimer[dst].Cancel();
    m_addressReqTimer[dst].SetArguments(dst);

    Time retry;
    std::vector<LocalRoute<IpAddress>> routes;
    m_routingTable.LookupRoutes(dst, routes);
    LocalRoute<IpAddress> rt = routes[0];
    if (rt.GetHop() < m_maxHopLimit)
    {
        retry = 2 * m_nodeTraversalTime * (rt.GetHop() + m_timeoutBuffer);
    }
    else
    {
        NS_ABORT_MSG_UNLESS(rt.GetRreqCnt() > 0, "Unexpected value for GetRreqCount ()");
        uint16_t backoffFactor = rt.GetRreqCnt() - 1;
        NS_LOG_LOGIC("Applying binary exponential backoff factor " << backoffFactor);
        retry = m_netTraversalTime * (1 << backoffFactor);
    }
    m_addressReqTimer[dst].Schedule(retry);
    NS_LOG_LOGIC("Scheduled RREQ retry in " << retry.As(Time::S));
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::ScheduleRrepRetry(const RreqHeader<IpAddress>& rreqHeader,
                                            const LocalRoute<IpAddress>& toOrigin,
                                            uint8_t hopCount)
{
    IpAddress dst = rreqHeader.GetOrigIp();
    NS_LOG_FUNCTION(this << dst);
    if (m_addressRepTimer.find(dst) == m_addressRepTimer.end())
    {
        Timer timer(Timer::CANCEL_ON_DESTROY);
        m_addressRepTimer[dst] = timer;
    }
    m_addressRepTimer[dst].SetFunction(&Aodvv2RoutingProtocol<T>::RouteReplyTimerExpire, this);
    m_addressRepTimer[dst].Cancel();
    m_addressRepTimer[dst].SetArguments(rreqHeader, toOrigin, hopCount);

    Time retry;
    std::vector<LocalRoute<IpAddress>> routes;
    if (m_routingTable.LookupRoutes(dst, routes))
    {
        LocalRoute<IpAddress> rt = routes[0];
        if (rt.GetHop() < m_maxHopLimit)
        {
            retry =
                m_rrepAckSentTimeout + 2 * m_nodeTraversalTime * (rt.GetHop() + m_timeoutBuffer);
        }
        else
        {
            NS_ABORT_MSG_UNLESS(rt.GetRrepCnt() > 0, "Unexpected value for GetRrepCount ()");
            uint16_t rrepCnt = rt.GetRrepCnt();
            if (rrepCnt == 0) rrepCnt = 1;
            uint16_t backoffFactor = rrepCnt - 1;
            if (backoffFactor > 30) backoffFactor = 30;
            NS_LOG_LOGIC("Applying binary exponential backoff factor " << backoffFactor);
            retry = m_netTraversalTime * (1LL << backoffFactor);
        }
        m_addressRepTimer[dst].Schedule(retry);
        NS_LOG_LOGIC("Scheduled RREP retry in " << retry.As(Time::S));
    }
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::RecvAodvv2(Ptr<Socket> socket)
{
    NS_LOG_FUNCTION(this << socket);
    Address sourceAddress;
    Ptr<Packet> packet = socket->RecvFrom(sourceAddress);
    InetVxSocketAddress inetSourceAddr = InetVxSocketAddress::ConvertFrom(sourceAddress);
    IpAddress sender;
    if constexpr (std::is_same<T, Ipv4RoutingProtocol>::value)
    {
        sender = inetSourceAddr.GetIpv4();
        // This project's custom mesh forwarding (NoHwmpRoutingProtocol::
        // RequestRoute, since HWMP/FLAME are disabled) relays group-addressed
        // frames without rewriting the IP header, so the socket-level source
        // address above is always the RREQ/RREP's *original* IP source, never
        // the node that actually relayed it over the last radio hop -- for a
        // multihop RREQ/RREP this means every receiving node (including the
        // true destination) would otherwise treat the original sender as if
        // it were a direct one-hop neighbor. MeshPointDevice::
        // ReceiveFromDevice() already tags every relayed frame with a
        // MeshPreviousHopTag carrying the real immediate relayer's address
        // (src/mesh/model/mesh-point-device.cc); plain AODV already consumes
        // this same tag for the same reason (aodv-routing-protocol.cc
        // RecvAodv()). Do the same here, once, for every AODVv2 message type.
        MeshPreviousHopTag previousHopTag;
        if (packet->PeekPacketTag(previousHopTag) &&
            previousHopTag.GetAddress() != Ipv4Address())
        {
            sender = previousHopTag.GetAddress();
        }
    }
    else
    {
        sender = inetSourceAddr.GetIpv6();
    }
    if constexpr (std::is_same<T, Ipv4RoutingProtocol>::value)
    {
        // AODVv2-LQ, Fix 5: record the ETX for the LinkCost/RouteCost
        // lambdas built in BuildLinkQualityMetric() to read while this
        // message is processed. A no-op when the link-quality metric is
        // disabled. The *actual* source is now m_recentPhyEtxByUid,
        // looked up by this exact packet's own UID -- Fix 5's root-cause
        // report confirmed that no MAC read from this (or any relayed)
        // frame identifies the true immediate transmitter, so UID
        // correlation is used instead of any MAC identity at all for
        // this lookup. `sender` (the already-trusted MeshPreviousHopTag/
        // raw-socket IP, unaffected by the Addr2 characteristic) is only
        // used for logging below, not for the lookup itself. The Fix 3
        // (ARP) and Fix 4 (MeshLastHopMacTag) paths are still computed
        // here too, purely to log all three side by side for direct
        // before/after evidence; neither feeds m_currentHopEtx anymore.
        auto phyEtxIt = m_recentPhyEtxByUid.find(packet->GetUid());
        bool hasEtxObservation = (phyEtxIt != m_recentPhyEtxByUid.end());
        m_currentHopEtx = hasEtxObservation ? phyEtxIt->second.first : 255.0;

        std::string diagPath = AodvV2LqDiagnosticPath("hop-etx-resolution.csv");
        if (!diagPath.empty())
        {
            const char* etxStatus = hasEtxObservation ? "ETX_OBSERVATION_FOUND" : "NO_ETX_OBSERVATION";
            const char* finalStatus = hasEtxObservation ? "RESOLVED" : "FALLBACK_255";
            // Fix 3 (ARP) and Fix 4 (tag) paths, logged purely for
            // before/after comparison -- neither feeds m_currentHopEtx.
            Mac48Address tagMac = ResolveLastHopMacFromTag(packet);
            bool mappingFound = (tagMac != Mac48Address());
            const char* mappingStatus = !mappingFound ? "MAPPING_NOT_FOUND" : "MAPPING_FOUND";
            Mac48Address arpMac = ResolveNeighborMac(sender);
            bool arpResolved = (arpMac != Mac48Address());
            uint32_t nodeId = m_ip->template GetObject<Node>()->GetId();
            std::ofstream out(diagPath.c_str(), std::ios::app);
            out << Simulator::Now().GetSeconds() << "," << nodeId << "," << sender << ","
                << tagMac << "," << mappingStatus << "," << etxStatus << "," << finalStatus << ","
                << m_currentHopEtx << "," << (arpResolved ? 1 : 0) << "," << arpMac << "," << packet->GetUid() << "\n";
        }
    }
    IpAddress receiver;

    if (m_socketAddresses.find(socket) != m_socketAddresses.end())
    {
        receiver = GetInterfaceAddress(m_socketAddresses[socket]);
    }
    else if (m_socketSubnetBroadcastAddresses.find(socket) !=
             m_socketSubnetBroadcastAddresses.end())
    {
        receiver = GetInterfaceAddress(m_socketSubnetBroadcastAddresses[socket]);
    }
    else
    {
        NS_ASSERT_MSG(false, "Received a packet from an unknown socket");
    }
    NS_LOG_DEBUG("AODVv2 node " << this << " received a AODVv2 packet from " << sender << " to "
                                << receiver);

    UpdateRouteToNeighbor(sender, receiver);

    PbbPacket tlvHeader;
    packet->RemoveHeader(tlvHeader);

    if (tlvHeader.MessageSize() == 0)
    {
        NS_LOG_DEBUG("AODVv2 message " << packet->GetUid()
                                       << " with no tlv message received. Drop");
        return; // drop
    }

    uint8_t rreqMessages = 0;
    uint8_t rrepMessages = 0;
    uint8_t rerrMessages = 0;
    uint8_t rrepAckMessages = 0;

    for (auto i = tlvHeader.MessageBegin(); i != tlvHeader.MessageEnd(); ++i)
    {
        auto tlvMessage = *i;
        switch (tlvMessage->GetType())
        {
        case AODVV2_TYPE_RREQ:
            rreqMessages++;
            break;
        case AODVV2_TYPE_RREP:
            rrepMessages++;
            break;
        case AODVV2_TYPE_RERR:
            rerrMessages++;
            break;
        case AODVV2_TYPE_RREP_ACK:
            rrepAckMessages++;
            break;
        default:
            NS_LOG_DEBUG("Unknown AODVv2 message type. Drop");
            return; // drop
        }
    }

    if (rreqMessages > 0)
    {
        RecvRequest(packet, receiver, sender, tlvHeader);
        SavePacketData("RREQ", packet->GetSize() + tlvHeader.GetSerializedSize());
    }
    else if (rrepMessages > 0 || (rrepMessages > 0 && rrepAckMessages > 0))
    {
        RecvReply(packet, receiver, sender, tlvHeader);
        SavePacketData("RREP", packet->GetSize() + tlvHeader.GetSerializedSize());
    }
    else if (rrepAckMessages > 0)
    {
        RecvReplyAck(sender);
        SavePacketData("RREP_ACK", packet->GetSize() + tlvHeader.GetSerializedSize());
    }
    else if (rerrMessages > 0)
    {
        RecvError(packet, sender, tlvHeader);
        SavePacketData("RERR", packet->GetSize() + tlvHeader.GetSerializedSize());
    }
}

template <typename T>
std::vector<uint8_t>
Aodvv2RoutingProtocol<T>::GetRouteMetricTypes(IpAddress addr)
{
    NS_LOG_FUNCTION(this << addr);
    std::vector<LocalRoute<IpAddress>> routes;
    std::vector<uint8_t> metricTypes;
    if (m_routingTable.LookupRoutes(addr, routes))
    {
        for (const auto& route : routes)
        {
            metricTypes.push_back(route.GetMetricType());
        }
    }
    return metricTypes;
}

template <typename T>
bool
Aodvv2RoutingProtocol<T>::UpdateRouteLifeTime(IpAddress addr, Time lifetime)
{
    NS_LOG_FUNCTION(this << addr << lifetime);
    std::vector<LocalRoute<IpAddress>> routes;
    bool result = false;
    if (m_routingTable.LookupRoutes(addr, routes))
    {
        for (LocalRoute<IpAddress>& rt : routes)
        {
            if (rt.GetState() == ACTIVE)
            {
                NS_LOG_DEBUG("Updating ACTIVE route");
                rt.SetRreqCnt(0);
                rt.SetRrepCnt(0);
                rt.SetLastUsed();
                m_routingTable.Update(rt);
                result = true;
            }
        }
    }
    return result;
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::UpdateRouteToNeighbor(IpAddress sender, IpAddress receiver)
{
    NS_LOG_FUNCTION(this << "sender " << sender << " receiver " << receiver);
    m_nb.AddNeighbor(sender, m_ip->GetAddress(m_ip->GetInterfaceForAddress(receiver), 0));
    std::vector<LocalRoute<IpAddress>> routes;
    if (!m_routingTable.LookupRoutes(sender, routes))
    {
        Ptr<NetDevice> dev = m_ip->GetNetDevice(m_ip->GetInterfaceForAddress(receiver));
        for (Metric<IpAddress> metric : GetMetrics())
        {
            LocalRoute<IpAddress> newEntry(
                /*dev=*/dev,
                /*dst=*/sender,
                /*seqNo=*/0,
                /*iface=*/m_ip->GetAddress(m_ip->GetInterfaceForAddress(receiver), 0),
                /*hops=*/1,
                /*nextHop=*/sender,
                /*maxIdleTime=*/m_maxIdleTime,
                /*metric=*/metric,
                /*metricValue=*/
                metric.LinkCost(GetMetricNode(m_ip->template GetObject<Node>())));
            m_routingTable.AddRoute(newEntry);
        }
    }
    else
    {
        LocalRoute<IpAddress> toNeighbor = routes[0];
        Ptr<NetDevice> dev = m_ip->GetNetDevice(m_ip->GetInterfaceForAddress(receiver));
        if (toNeighbor.GetValidSeqNo() && (toNeighbor.GetHop() == 1) &&
            (toNeighbor.GetOutputDevice() == dev))
        {
            for (LocalRoute<IpAddress>& rt : routes)
            {
                rt.SetLastUsed();
            }
        }
        else
        {
            for (Metric<IpAddress> metric : GetMetrics())
            {
                LocalRoute<IpAddress> newEntry(
                    /*dev=*/dev,
                    /*dst=*/sender,
                    /*seqNo=*/0,
                    /*iface=*/m_ip->GetAddress(m_ip->GetInterfaceForAddress(receiver), 0),
                    /*hops=*/1,
                    /*nextHop=*/sender,
                    /*maxIdleTime=*/m_maxIdleTime,
                    /*metric=*/metric,
                    /*metricValue=*/
                    metric.LinkCost(GetMetricNode(m_ip->template GetObject<Node>())));
                m_routingTable.Update(newEntry);
            }
        }
    }
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::RecvRequest(Ptr<Packet> p,
                                      IpAddress receiver,
                                      IpAddress src,
                                      PbbPacket tlvHeader)
{
    NS_LOG_FUNCTION(this);
    RreqHeader<IpAddress> rreqHeader(tlvHeader);

    // A node ignores all RREQs received from any node in its blacklist
    LocalRoute<IpAddress> toPrev;
    for (uint8_t type : rreqHeader.GetMetricTypes())
    {
        if (m_routingTable.LookupRoute(src, type, toPrev))
        {
            if (toPrev.GetState() == INVALID)
            {
                NS_LOG_DEBUG("Ignoring RREQ from node in blacklist");
                return;
            }
        }
    }

    IpAddress origin = rreqHeader.GetOrigIp();

    /*
     *  Node checks to determine whether it has received a RREQ with the same Originator IP Address,
     * Mask, Target IP Address and SeqNoRtr. If such a RREQ has been received, the node silently
     * discards the newly received RREQ.
     */

    for (uint8_t type : rreqHeader.GetMetricTypes())
    {
        if (m_mms.IsDuplicate(origin,
                              rreqHeader.GetOrigMask(),
                              rreqHeader.GetTargIp(),
                              rreqHeader.GetRtrIp(),
                              rreqHeader.GetSeqNo(),
                              m_ip->GetAddress(m_ip->GetInterfaceForAddress(receiver), 0),
                              GetMetric(type),
                              rreqHeader.GetMetricValue(type)))
        {
            NS_LOG_DEBUG("Ignoring RREQ due to duplicate");
            return;
        }
    }

    // Decrement RREQ hop count
    uint8_t hop = rreqHeader.GetHopLimit() - 1;
    rreqHeader.SetHopLimit(hop);

    Metric<IpAddress> metric = GetMetric(rreqHeader.GetMetricTypes()[0]);
    for (uint8_t metricType : rreqHeader.GetMetricTypes())
    {
        metric = GetMetric(metricType);
        /*
         *  When the reverse route is created or updated, the following actions on the route are
         * also carried out:
         *  1. the Originator Sequence Number from the RREQ is compared to the corresponding
         * destination sequence number in the route table entry and copied if greater than the
         * existing value there
         *  2. the valid sequence number field is set to true;
         *  3. the next hop in the routing table becomes the node from which the RREQ was
         * received
         *  4. the hop count is copied from the Hop Count in the RREQ message;
         *  5. the Lifetime is set to be the maximum of (ExistingLifetime, MinimalLifetime),
         * where MinimalLifetime = current time + 2*NetTraversalTime -
         * 2*HopCount*NodeTraversalTime
         */
        if (origin != receiver)
        {
            LocalRoute<IpAddress> toOrigin;
            if (!m_routingTable.LookupRoute(origin, metricType, toOrigin))
            {
                Ptr<NetDevice> dev = m_ip->GetNetDevice(m_ip->GetInterfaceForAddress(receiver));
                LocalRoute<IpAddress> newEntry(
                    /*dev=*/dev,
                    /*dst=*/origin,
                    /*seqNo=*/rreqHeader.GetSeqNo(),
                    /*iface=*/m_ip->GetAddress(m_ip->GetInterfaceForAddress(receiver), 0),
                    /*hops=*/m_maxHopLimit - hop,
                    /*nextHop=*/src,
                    /*maxIdleTime=*/m_maxIdleTime,
                    /*metric=*/metric,
                    /*metricValue=*/rreqHeader.GetMetricValue(metric.GetMetricType()));
                m_routingTable.AddRoute(newEntry);
            }
            else
            {
                if (toOrigin.GetValidSeqNo())
                {
                    if (uint16_t(rreqHeader.GetOrigSeqNo()) - uint16_t(toOrigin.GetSeqNo()) > 0)
                    {
                        toOrigin.SetSeqNo(rreqHeader.GetOrigSeqNo());
                    }
                }
                else
                {
                    rreqHeader.SetSendTargSeqNum(true);
                    toOrigin.SetSeqNo(rreqHeader.GetOrigSeqNo());
                }
                toOrigin.SetNextHop(src);
                toOrigin.SetOutputDevice(
                    m_ip->GetNetDevice(m_ip->GetInterfaceForAddress(receiver)));
                toOrigin.SetInterface(m_ip->GetAddress(m_ip->GetInterfaceForAddress(receiver), 0));
                toOrigin.SetHop(m_maxHopLimit - hop);
                toOrigin.SetLastUsed();
                m_routingTable.Update(toOrigin);
            }
        }

        LocalRoute<IpAddress> toNeighbor;
        if (!m_routingTable.LookupRoute(src, metricType, toNeighbor))
        {
            NS_LOG_DEBUG("Neighbor:" << src << " not found in routing table. Creating an entry");
            Ptr<NetDevice> dev = m_ip->GetNetDevice(m_ip->GetInterfaceForAddress(receiver));
            LocalRoute<IpAddress> newEntry(
                dev,
                src,
                rreqHeader.GetOrigSeqNo(),
                m_ip->GetAddress(m_ip->GetInterfaceForAddress(receiver), 0),
                1,
                src,
                m_maxIdleTime,
                metric,
                rreqHeader.GetMetricValue(metric.GetMetricType()));
            m_routingTable.AddRoute(newEntry);
        }
        else
        {
            toNeighbor.SetLastUsed();
            toNeighbor.SetInvalidSeqNo();
            toNeighbor.SetSeqNo(rreqHeader.GetOrigSeqNo());
            // toNeighbor.SetState(ACTIVE);
            toNeighbor.SetOutputDevice(m_ip->GetNetDevice(m_ip->GetInterfaceForAddress(receiver)));
            toNeighbor.SetInterface(m_ip->GetAddress(m_ip->GetInterfaceForAddress(receiver), 0));
            toNeighbor.SetHop(1);
            toNeighbor.SetNextHop(src);
            m_routingTable.Update(toNeighbor);
        }

        rreqHeader.AddMetric(metric.GetMetricType(),
                             metric.RouteCost(rreqHeader.GetMetricValue(metric.GetMetricType()),
                                              GetMetricNode(m_ip->template GetObject<Node>())),
                             metric.GetMetricSize());
    }
    m_nb.AddNeighbor(src, m_ip->GetAddress(m_ip->GetInterfaceForAddress(receiver), 0));

    NS_LOG_LOGIC(receiver << " receive RREQ with hop count "
                          << static_cast<uint32_t>(rreqHeader.GetHopLimit()) << " SeqNo "
                          << rreqHeader.GetSeqNo() << " to destination " << rreqHeader.GetTargIp()
                          << " from " << src);

    LocalRoute<IpAddress> toOrigin;
    //  A node generates a RREP if either:
    //  (i)  it is itself the destination,
    if (IsMyOwnAddress(rreqHeader.GetTargIp()))
    {
        m_routingTable.LookupRoute(origin, rreqHeader.GetMetricTypes()[0], toOrigin);
        NS_LOG_DEBUG("Send reply since I am the destination " << toOrigin.GetDestination());
        SendReply(rreqHeader, toOrigin, rreqHeader.GetHopLimit() - 1);
        return;
    }
    /*
     * (ii) or it has an active route to the destination, the destination sequence number in the
     * node's existing route table entry for the destination is valid and greater than or equal
     * to the Destination Sequence Number of the RREQ, and the "destination only" flag is NOT
     * set.
     */
    LocalRoute<IpAddress> toDst;
    IpAddress dst = rreqHeader.GetTargIp();
    if (m_routingTable.LookupRoute(dst, metric.GetMetricType(), toDst))
    {
        /*
         * Drop RREQ, This node RREP will make a loop.
         */
        if (toDst.GetNextHop() == src)
        {
            NS_LOG_DEBUG("Drop RREQ from " << src << ", dest next hop " << toDst.GetNextHop());
            return;
        }
        /*
         * The Destination Sequence number for the requested destination is set to the maximum
         * of the corresponding value received in the RREQ message, and the destination sequence
         * value currently maintained by the node for the requested destination. However, the
         * forwarding node MUST NOT modify its maintained value for the destination sequence
         * number, even if the value received in the incoming RREQ is larger than the value
         * currently maintained by the forwarding node.
         */
        if (((uint16_t(toDst.GetSeqNo()) - uint16_t(rreqHeader.GetSeqNo()) >= 0)) &&
            toDst.GetValidSeqNo())
        {
        if (toDst.GetState() == ACTIVE &&
            m_routingTable.LookupRoute(origin, metric.GetMetricType(), toOrigin) &&
            !toOrigin.GetNextHop().IsAny())
        {
            SendReplyByIntermediateNode(toDst, toOrigin);
            return;
        }
            rreqHeader.SetSeqNo(toDst.GetSeqNo());
        }
    }
    else
    {
        for (uint8_t metricType : rreqHeader.GetMetricTypes())
        {
            metric = GetMetric(metricType);
            Ptr<NetDevice> dev = m_ip->GetNetDevice(m_ip->GetInterfaceForAddress(receiver));
            LocalRoute<IpAddress> newEntry(
                dev,
                dst,
                rreqHeader.GetOrigSeqNo(),
                m_ip->GetAddress(m_ip->GetInterfaceForAddress(receiver), 0),
                m_maxHopLimit - rreqHeader.GetHopLimit(),
                dst,
                m_maxIdleTime,
                metric,
                rreqHeader.GetMetricValue(metric.GetMetricType()));
            m_routingTable.AddRoute(newEntry);
        }
    }

    for (auto j = m_socketAddresses.begin(); j != m_socketAddresses.end(); ++j)
    {
        Ptr<Socket> socket = j->first;
        IpInterfaceAddress iface = j->second;
        Ptr<Packet> packet = Create<Packet>();
        packet->AddHeader(rreqHeader);
        // Send to all-hosts broadcast if on /32 addr, subnet-directed otherwise
        IpAddress destination;
        if constexpr (std::is_same<T, Ipv4RoutingProtocol>::value)
        {
            if (iface.GetMask() == Ipv4Mask::GetOnes())
            {
                destination = IpAddress("255.255.255.255");
            }
            else
            {
                destination = iface.GetBroadcast();
            }
        }
        else
        {
            // TODO Ipv6
        }
        m_lastBcastTime = Simulator::Now();
        Simulator::Schedule(Time(MilliSeconds(m_uniformRandomVariable->GetInteger(0, 10))),
                            &Aodvv2RoutingProtocol<T>::SendTo,
                            this,
                            socket,
                            packet,
                            destination);
    }
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::SendReply(const RreqHeader<IpAddress>& rreqHeader,
                                    const LocalRoute<IpAddress>& toOrigin,
                                    uint8_t hopCount)
{
    NS_LOG_FUNCTION(this << toOrigin.GetDestination());
    // A node SHOULD NOT originate more than RREP_RATELIMIT RREP messages per second.
    if (m_rrepCount == m_rrepRateLimit)
    {
        Simulator::Schedule(m_rrepRateLimitTimer.GetDelayLeft() + MicroSeconds(100),
                            &Aodvv2RoutingProtocol<T>::SendReply,
                            this,
                            rreqHeader,
                            toOrigin,
                            hopCount);
        return;
    }
    else
    {
        m_rrepCount++;
    }

    /*
     * Destination node MUST increment its own sequence number by one if the sequence number in
     * the RREQ packet is equal to that incremented value. Otherwise, the destination does not
     * change its sequence number before generating the  RREP message.
     */
    if (rreqHeader.GetSeqNo() == m_seqNo + 1)
    {
        m_seqNo++;
    }

    std::vector<LocalRoute<IpAddress>> routes;
    if (m_routingTable.LookupRoutes(rreqHeader.GetTargIp(), routes))
    {
        for (LocalRoute<IpAddress>& rt : routes)
        {
            rt.IncrementRrepCnt();
            m_routingTable.Update(rt);
        }
    }

    RrepHeader rrepHeader(
        /*origIp=*/toOrigin.GetDestination(),
        /*origMask=*/32,
        /*targIp=*/rreqHeader.GetTargIp(),
        /*targMask=*/32,
        /*seqNo=*/m_seqNo,
        /*hopLimit=*/m_maxHopLimit - hopCount);

    for (uint8_t type : rreqHeader.GetMetricTypes())
    {
        Metric<IpAddress> metric = GetMetric(type);

        rrepHeader.AddMetric(metric.GetMetricType(),
                             metric.LinkCost(GetMetricNode(m_ip->template GetObject<Node>())),
                             metric.GetMetricSize());
    }

    Ptr<Packet> packet = Create<Packet>();

    if (m_routingTable.LookupRoutes(toOrigin.GetNextHop(), routes) &&
        routes[0].GetState() == UNCONFIRMED)
    {
        if (m_bidirectionalLinks)
        {
            RecvReplyAck(toOrigin.GetNextHop());
        }
        else
        {
            rrepHeader.SetHasRrepAck(true);
        }
    }

    packet->AddHeader(rrepHeader);
    Ptr<Socket> socket = FindSocketWithInterfaceAddress(toOrigin.GetInterface());
    NS_ASSERT(socket);
    socket->SendTo(packet, 0, InetVxSocketAddress(toOrigin.GetNextHop(), AODVV2_PORT));
    ScheduleRrepRetry(rreqHeader, toOrigin, hopCount);
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::SendReplyByIntermediateNode(LocalRoute<IpAddress>& toDst,
                                                      LocalRoute<IpAddress>& toOrigin)
{
    NS_LOG_FUNCTION(this);
    RrepHeader rrepHeader(
        /*origIp=*/toOrigin.GetDestination(),
        /*origMask=*/32,
        /*targIp=*/toDst.GetDestination(),
        /*targMask=*/32,
        /*seqNo=*/toDst.GetSeqNo(),
        /*hopLimit=*/toDst.GetHop());

    /* If the node we received a RREQ for is a neighbor we are
     * probably facing a unidirectional link... Better request a RREP-ack
     */
    if (toDst.GetHop() == 1)
    {
        ScheduleRrepAckCheck(toOrigin);
    }
    toDst.InsertPrecursor(toOrigin.GetNextHop());
    toOrigin.InsertPrecursor(toDst.GetNextHop());
    m_routingTable.Update(toDst);
    m_routingTable.Update(toOrigin);

    Ptr<Packet> packet = Create<Packet>();
    packet->AddHeader(rrepHeader);
    Ptr<Socket> socket = FindSocketWithInterfaceAddress(toOrigin.GetInterface());
    NS_ASSERT(socket);
    socket->SendTo(packet, 0, InetVxSocketAddress(toOrigin.GetNextHop(), AODVV2_PORT));
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::ScheduleRrepAckCheck(LocalRoute<IpAddress> toOrigin)
{
    std::vector<LocalRoute<IpAddress>> routes;
    if (m_routingTable.LookupRoutes(toOrigin.GetNextHop(), routes))
    {
        for (LocalRoute<IpAddress>& toNextHop : routes)
        {
            toNextHop.m_ackTimer.SetFunction(&Aodvv2RoutingProtocol<T>::AckTimerExpire, this);
            toNextHop.m_ackTimer.SetArguments(toNextHop.GetDestination(), m_maxIdleTime);
            toNextHop.m_ackTimer.SetDelay(m_rrepAckSentTimeout);
        }
    }
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::SendReplyAck(IpAddress neighbor, RrepHeader<IpAddress> rrepHeader)
{
    NS_LOG_FUNCTION(this << " to " << neighbor);
    RrepAckHeader<IpAddress> h;
    Ptr<Packet> packet = Create<Packet>();
    packet->AddHeader(h);
    std::vector<LocalRoute<IpAddress>> routes;
    if (m_routingTable.LookupRoutes(neighbor, routes))
    {
        for (LocalRoute<IpAddress>& toNeighbor : routes)
        {
            toNeighbor.SetState(ACTIVE);
            m_routingTable.Update(toNeighbor);
            m_nb.AddNeighbor(toNeighbor.GetDestination(), toNeighbor.GetInterface());
            Ptr<Socket> socket = FindSocketWithInterfaceAddress(toNeighbor.GetInterface());
            NS_ASSERT(socket);
            socket->SendTo(packet, 0, InetVxSocketAddress(neighbor, AODVV2_PORT));
        }
    }
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::RecvReply(Ptr<Packet> p,
                                    IpAddress receiver,
                                    IpAddress sender,
                                    PbbPacket tlvHeader)
{
    NS_LOG_FUNCTION(this << " src " << sender);
    RrepHeader<IpAddress> rrepHeader(tlvHeader);

    IpAddress dst = rrepHeader.GetTargIp();
    NS_LOG_LOGIC("RREP destination " << dst << " RREP origin " << rrepHeader.GetOrigIp());

    if (m_newDiscoveryTimer.find(dst) != m_newDiscoveryTimer.end())
    {
        m_newDiscoveryTimer[dst].Cancel();
    }

    uint8_t hop = rrepHeader.GetHopLimit() - 1;
    rrepHeader.SetHopLimit(hop);

    LocalRoute<IpAddress> toDst;
    Metric<IpAddress> metric = GetMetric(rrepHeader.GetMetricTypes()[0]);

    for (uint8_t metricType : rrepHeader.GetMetricTypes())
    {
        metric = GetMetric(metricType);

        /*
         * If the route table entry to the destination is created or updated, then the following
         * actions occur:
         * -  the route is marked as active,
         * -  the destination sequence number is marked as valid,
         * -  the next hop in the route entry is assigned to be the node from which the RREP is
         * received, which is indicated by the source IP address field in the IP header,
         * -  the hop count is set to the value of the hop count from RREP message + 1
         * -  the expiry time is set to the current time plus the value of the Lifetime in the
         * RREP message,
         * -  and the destination sequence number is the Destination Sequence Number in the RREP
         * message.
         */
        Ptr<NetDevice> dev = m_ip->GetNetDevice(m_ip->GetInterfaceForAddress(receiver));
        LocalRoute<IpAddress> newEntry(
            /*dev=*/dev,
            /*dst=*/dst,
            /*seqNo=*/rrepHeader.GetSeqNo(),
            /*iface=*/m_ip->GetAddress(m_ip->GetInterfaceForAddress(receiver), 0),
            /*hops=*/hop,
            /*nextHop=*/sender,
            /*maxIdleTime=*/m_maxIdleTime,
            /*metric=*/metric,
            /*metricValue=*/rrepHeader.GetMetricValue(metric.GetMetricType()),
            /*state=*/ACTIVE);

        m_nb.AddNeighbor(sender, m_ip->GetAddress(m_ip->GetInterfaceForAddress(receiver), 0));
        m_nb.UpdateState(sender,
                         m_ip->GetAddress(m_ip->GetInterfaceForAddress(receiver), 0),
                         m_rreqWaitTime);

        if (m_nb.GetState(sender) == HEARD)
        {
            newEntry.SetState(UNCONFIRMED);
        }
        if (m_nb.GetState(sender) == BLACKLISTED) // drop
        {
            return;
        }

        newEntry.SetHop(rrepHeader.GetMetricValue(metric.GetMetricType())[0]);

        rrepHeader.AddMetric(metric.GetMetricType(),
                             metric.RouteCost(rrepHeader.GetMetricValue(metric.GetMetricType()),
                                              GetMetricNode(m_ip->template GetObject<Node>())),
                             metric.GetMetricSize());

        if (m_routingTable.LookupRoute(dst, metric.GetMetricType(), toDst))
        {
            // The existing entry is updated only in the following circumstances:
            if (
                // (i) the sequence number in the routing table is marked as invalid in route
                // table entry.
                (!toDst.GetValidSeqNo()) ||

                // (ii) the Destination Sequence Number in the RREP is greater than the node's
                // copy of the destination sequence number and the known value is valid,
                ((uint16_t(rrepHeader.GetSeqNo()) - uint16_t(toDst.GetSeqNo())) > 0) ||

                // (iii) the sequence numbers are the same, but the route is marked as inactive.
                (rrepHeader.GetSeqNo() == toDst.GetSeqNo() && toDst.GetState() != ACTIVE) ||

                // (iv) the sequence numbers are the same, and the New Hop Count is smaller than
                // the hop count in route table entry.
                (rrepHeader.GetSeqNo() == toDst.GetSeqNo() && hop < toDst.GetHop()))
            {
                m_routingTable.Update(newEntry);
            }
        }
        else
        {
            // The forward route for this destination is created if it does not already exist.
            // This AddRoute() call was left commented out upstream (with a
            // "TODO me: understand why this is not working" note) -- that
            // left every first-time RREP for a new destination unable to
            // install a usable forward route: LookupRoute() just above
            // confirmed no entry exists, so skipping AddRoute() here means
            // the route table never gains one, and the SendPacketFromQueue()
            // call a few lines down (and in the mirrored branch further
            // below) is given an empty/invalid route. AddRoute() itself is
            // the same call already used safely by six other call sites in
            // this file; it only appends to the local route set (see
            // LocalRouteSet<T>::AddRoute(), aodvv2-local-route-set.cc) and
            // has no precondition this newEntry fails to satisfy.
            NS_LOG_LOGIC("add new route");
            m_routingTable.AddRoute(newEntry);
        }
        NS_LOG_LOGIC("receiver " << receiver << " origin " << rrepHeader.GetOrigIp());
        if (IsMyOwnAddress(rrepHeader.GetOrigIp()))
        {
            if (toDst.GetState() == UNCONFIRMED)
            {
                m_routingTable.Update(newEntry);
                m_addressReqTimer[dst].Cancel();
                m_addressReqTimer.erase(dst);
            }
        }
    }

    if (IsMyOwnAddress(rrepHeader.GetOrigIp()))
    {
        m_routingTable.LookupRoute(dst, metric.GetMetricType(), toDst);
        if (m_bidirectionalLinks)
        {
            RecvReplyAck(sender);
        }
        else
        {
            if (rrepHeader.HasRrepAck())
            {
                SendReplyAck(sender, rrepHeader);
            }
        }
        SendPacketFromQueue(dst, toDst.GetRoute());
        return;
    }

    std::vector<LocalRoute<IpAddress>> routes;
    if (!m_routingTable.LookupValidRoutes(rrepHeader.GetOrigIp(), routes))
    {
        return; // Impossible! drop.
    }
    for (LocalRoute<IpAddress>& toOrigin : routes)
    {
        toOrigin.SetLastUsed();
        toOrigin.SetState(ACTIVE);
        m_routingTable.Update(toOrigin);

        // Update information about precursors
        if (m_routingTable.LookupRoute(rrepHeader.GetTargIp(), metric.GetMetricType(), toDst))
        {
            toDst.SetNextHop(sender);
            toDst.InsertPrecursor(toOrigin.GetNextHop());
            toDst.SetLastUsed();
            toDst.SetState(ACTIVE);
            m_routingTable.Update(toDst);

            std::vector<LocalRoute<IpAddress>> routes;
            if (m_routingTable.LookupRoutes(toDst.GetNextHop(), routes))
            {
                for (LocalRoute<IpAddress>& toNextHopToDst : routes)
                {
                    toNextHopToDst.InsertPrecursor(toOrigin.GetNextHop());
                    m_routingTable.Update(toNextHopToDst);
                }
            }

            toOrigin.InsertPrecursor(toDst.GetNextHop());
            m_routingTable.Update(toOrigin);

            if (m_routingTable.LookupRoutes(toOrigin.GetNextHop(), routes))
            {
                for (LocalRoute<IpAddress>& toNextHopToOrigin : routes)
                {
                    toNextHopToOrigin.InsertPrecursor(toDst.GetNextHop());
                    m_routingTable.Update(toNextHopToOrigin);
                }
            }
        }
        if (m_bidirectionalLinks)
        {
            RecvReplyAck(sender);
        }
        else
        {
            if (rrepHeader.HasRrepAck())
            {
                SendReplyAck(sender, rrepHeader);
            }
        }

        Ptr<Packet> packet = Create<Packet>();
        packet->AddHeader(rrepHeader);
        Ptr<Socket> socket = FindSocketWithInterfaceAddress(toOrigin.GetInterface());
        NS_ASSERT(socket);
        socket->SendTo(packet, 0, InetVxSocketAddress(toOrigin.GetNextHop(), AODVV2_PORT));
    }
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::RecvReplyAck(IpAddress neighbor)
{
    NS_LOG_FUNCTION(this);

    std::vector<LocalRoute<IpAddress>> routes;
    if (m_routingTable.LookupRoutes(neighbor, routes))
    {
        for (LocalRoute<IpAddress>& rt : routes)
        {
            if (rt.IsValid())
            {
                rt.m_ackTimer.Cancel();
                rt.SetState(ACTIVE);
                m_routingTable.Update(rt);
                m_addressRepTimer[rt.GetDestination()].Cancel();
                m_addressRepTimer.erase(rt.GetDestination());

                m_nb.UpdateState(neighbor, rt.GetInterface(), Simulator::Now());
            }
        }
    }
    m_routingTable.ActivateRouteWithNextHop(neighbor);
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::RecvError(Ptr<Packet> p, IpAddress src, PbbPacket tlvHeader)
{
    NS_LOG_FUNCTION(this << " from " << src);
    RerrHeader<IpAddress> rerrHeader(tlvHeader);

    std::map<IpAddress, UnreachableDst> dstWithNextHopSrc;
    std::map<IpAddress, UnreachableDst> unreachable;
    m_routingTable.GetListOfDestinationWithNextHop(src, dstWithNextHopSrc);
    std::pair<IpAddress, UnreachableDst> un;
    while (rerrHeader.RemoveUnDestination(un))
    {
        for (auto i = dstWithNextHopSrc.begin(); i != dstWithNextHopSrc.end(); ++i)
        {
            if (i->first == un.first && !un.first.IsAny() && !un.first.IsBroadcast() &&
                !un.first.IsMulticast())
            {
                unreachable.insert(un);
            }
        }
    }

    std::vector<IpAddress> precursors;
    for (auto i = unreachable.begin(); i != unreachable.end();)
    {
        if (!rerrHeader.AddUnDestination(i->first, i->second.m_seqNo, i->second.m_metricType))
        {
            Ptr<Packet> packet = Create<Packet>();
            packet->AddHeader(rerrHeader);
            SendRerrMessage(packet, precursors);
            rerrHeader.Clear();
        }
        else
        {
            std::vector<LocalRoute<IpAddress>> toDstRoutes;
            if (m_routingTable.LookupRoutes(i->first, toDstRoutes))
            {
                for (LocalRoute<IpAddress>& toDst : toDstRoutes)
                {
                    toDst.SetState(INVALID);
                    m_routingTable.Update(toDst);
                    toDst.GetPrecursors(precursors);
                }
            }
            ++i;
        }
        m_nb.SetState(i->first, HEARD);
    }
    if (rerrHeader.GetDestCount() != 0)
    {
        Ptr<Packet> packet = Create<Packet>();
        packet->AddHeader(rerrHeader);
        SendRerrMessage(packet, precursors);
    }
    m_routingTable.InvalidateRoutesWithDst(unreachable);
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::SavePacketData(std::string type, uint32_t size)
{
    std::ofstream file;
    file.open("output-packets.csv", std::ios_base::app);
    file << type << "," << size << std::endl;
    file.close();
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::RouteRequestTimerExpire(IpAddress dst)
{
    NS_LOG_LOGIC(this);
    std::vector<LocalRoute<IpAddress>> routes;
    if (m_routingTable.LookupValidRoutes(dst, routes))
    {
        for (LocalRoute<IpAddress>& toDst : routes)
        {
            if (toDst.GetState() == UNCONFIRMED)
            {
                break;
            }
            else
            {
                SendPacketFromQueue(dst, toDst.GetRoute());
                NS_LOG_LOGIC("route to " << dst << " found");
                return;
            }
        }
    }
    // LookupValidRoutes() only ever returns routes that already have a
    // concrete output device and next hop (by design -- see its
    // implementation in aodvv2-local-route-set.cc). The placeholder entry
    // SendRequest() creates while a discovery is still pending has neither,
    // so LookupValidRoutes() returns false for it and the retry logic below
    // (which is specifically meant to handle that pending-discovery case)
    // would otherwise never run -- the destination's route request would
    // silently stop retrying after its first attempt. Fall back to the
    // unfiltered LookupRoutes() here, purely to find that pending-discovery
    // placeholder for the retry-count/backoff bookkeeping below; this does
    // not weaken route validity for actually sending data (SendPacketFromQueue
    // above is only ever reached via LookupValidRoutes's real validity check).
    else if (!m_routingTable.LookupRoutes(dst, routes))
    {
        NS_LOG_LOGIC("route to " << dst << " not found");
        return;
    }
    /*
     *  If a route discovery has been attempted RreqRetries times without receiving any
     * RREP, all data packets destined for the corresponding destination SHOULD be dropped
     * from the buffer and a Destination Unreachable message SHOULD be delivered to the
     * application.
     */
    LocalRoute<IpAddress> toDst = routes[0];
    if (toDst.GetRreqCnt() == m_discoveryAttemptsMax)
    {
        Timer timer(Timer::CANCEL_ON_DESTROY);
        m_newDiscoveryTimer[dst] = timer;
        m_newDiscoveryTimer[dst].SetFunction(&Aodvv2RoutingProtocol<T>::SendRequest, this);
        m_newDiscoveryTimer[dst].Schedule(m_rreqHolddownTime);
        return;
    }

    if (toDst.GetState() == UNCONFIRMED)
    {
        NS_LOG_LOGIC("Resend RREQ to " << dst);
        SendRequest(dst);
    }
    else
    {
        NS_LOG_DEBUG("Route down. Stop search. Drop packet with destination " << dst);
        m_addressReqTimer.erase(dst);
        m_routingTable.DeleteRoute(dst);
        m_queue.DropPacketWithDst(dst);
    }
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::RouteReplyTimerExpire(const RreqHeader<IpAddress>& rreqHeader,
                                                const LocalRoute<IpAddress>& toOrigin,
                                                uint8_t hopCount)
{
    NS_LOG_LOGIC(this);
    IpAddress dst = rreqHeader.GetOrigIp();
    std::vector<LocalRoute<IpAddress>> routes;
    LocalRoute<IpAddress> toDst;
    if (m_routingTable.LookupValidRoutes(dst, routes))
    {
        toDst = routes[0];
        if (toDst.GetState() == UNCONFIRMED)
        {
            NS_LOG_LOGIC("Resend RREP to " << dst << " previous diameter " << toDst.GetHop());
            SendReply(rreqHeader, toOrigin, hopCount);
            return;
        }
    }
    /*
     *  If a route discovery has been attempted RrepRetries times without receiving any RREP
     * Ack, all data packets destined for the corresponding destination SHOULD be dropped
     * from the buffer
     */
    if (toDst.GetRrepCnt() == m_rrepRetries)
    {
        return;
    }

    if (toDst.GetState() == UNCONFIRMED)
    {
        NS_LOG_LOGIC("Resend RREP to " << dst << " previous diameter " << toDst.GetHop());
        SendReply(rreqHeader, toOrigin, hopCount);
    }
    if (toDst.GetState() == INVALID)
    {
        NS_LOG_DEBUG("Route down. Stop search. Drop packet with destination " << dst);
        m_addressRepTimer.erase(dst);
        m_routingTable.DeleteRoute(dst);
        m_queue.DropPacketWithDst(dst);
    }
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::RreqRateLimitTimerExpire()
{
    NS_LOG_FUNCTION(this);
    m_rreqCount = 0;
    m_rreqRateLimitTimer.Schedule(Seconds(1));
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::RrepRateLimitTimerExpire()
{
    NS_LOG_FUNCTION(this);
    m_rrepCount = 0;
    m_rrepRateLimitTimer.Schedule(Seconds(1));
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::RerrRateLimitTimerExpire()
{
    NS_LOG_FUNCTION(this);
    m_rerrCount = 0;
    m_rerrRateLimitTimer.Schedule(Seconds(1));
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::AckTimerExpire(IpAddress neighbor, Time blacklistTimeout)
{
    NS_LOG_FUNCTION(this);
    m_routingTable.MarkLinkAsUnidirectional(neighbor, blacklistTimeout);
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::SendPacketFromQueue(IpAddress dst, Ptr<IpRoute> route)
{
    NS_LOG_FUNCTION(this);
    QueueEntry<IpHeader> queueEntry;
    while (m_queue.Dequeue(dst, queueEntry))
    {
        DeferredRouteOutputTag tag;
        Ptr<Packet> p = ConstCast<Packet>(queueEntry.GetPacket());
        if (p->RemovePacketTag(tag) && tag.GetInterface() != -1 &&
            tag.GetInterface() != m_ip->GetInterfaceForDevice(route->GetOutputDevice()))
        {
            NS_LOG_DEBUG("Output device doesn't match. Dropped.");
            return;
        }
        UnicastForwardCallback ucb = queueEntry.GetUnicastForwardCallback();
        IpHeader header = queueEntry.GetIpHeader();
        header.SetSource(route->GetSource());
        if constexpr (std::is_same<T, Ipv4RoutingProtocol>::value)
        {
            ucb(route, p, header);
        }
        else
        {
            // TODO Ipv6
        }
    }
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::SendRerrWhenBreaksLinkToNextHop(IpAddress nextHop)
{
    NS_LOG_FUNCTION(this << nextHop);
    RerrHeader<IpAddress> rerrHeader;
    std::vector<IpAddress> precursors;
    std::map<IpAddress, UnreachableDst> unreachable;

    if (nextHop.IsAny())
    {
        return;
    }

    std::vector<LocalRoute<IpAddress>> routes;
    if (!m_routingTable.LookupRoutes(nextHop, routes))
    {
        return;
    }

    m_nb.SetState(nextHop, HEARD);
    for (LocalRoute<IpAddress>& toNextHop : routes)
    {
        toNextHop.SetState(INVALID);
        m_routingTable.Update(toNextHop);
        toNextHop.GetPrecursors(precursors);
        rerrHeader.SetOrigIp(GetInterfaceAddress(toNextHop.GetInterface()));
        rerrHeader.SetOrigMask(32); // TODO me: update if needed
        rerrHeader.AddUnDestination(nextHop, toNextHop.GetSeqNo());
    }

    m_routingTable.GetListOfDestinationWithNextHop(nextHop, unreachable);
    for (auto i = unreachable.begin(); i != unreachable.end();)
    {
        if (!rerrHeader.AddUnDestination(i->first, i->second.m_seqNo, i->second.m_metricType))
        {
            NS_LOG_LOGIC("Send RERR message with maximum size.");
            Ptr<Packet> packet = Create<Packet>();
            packet->AddHeader(rerrHeader);
            SendRerrMessage(packet, precursors);
            rerrHeader.Clear();
        }
        else
        {
            std::vector<LocalRoute<IpAddress>> toDstRoutes;
            if (m_routingTable.LookupRoutes(i->first, toDstRoutes))
            {
                for (LocalRoute<IpAddress>& toDst : toDstRoutes)
                {
                    toDst.SetState(INVALID);
                    m_routingTable.Update(toDst);
                    toDst.GetPrecursors(precursors);
                }
            }
            ++i;
        }
    }
    if (rerrHeader.GetDestCount() != 0)
    {
        Ptr<Packet> packet = Create<Packet>();
        packet->AddHeader(rerrHeader);
        SendRerrMessage(packet, precursors);
    }
    for (LocalRoute<IpAddress>& route : routes)
    {
        unreachable.insert(
            std::make_pair(nextHop, UnreachableDst(route.GetSeqNo(), route.GetMetricType())));
    }
    m_routingTable.InvalidateRoutesWithDst(unreachable);
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::SendRerrWhenNoRouteToForward(IpAddress dst,
                                                       uint16_t dstSeqNo,
                                                       IpAddress origin)
{
    NS_LOG_FUNCTION(this);
    // A node SHOULD NOT originate more than RERR_RATELIMIT RERR messages per second.
    if (m_rerrCount == m_rerrRateLimit)
    {
        // Just make sure that the RerrRateLimit timer is running and will expire
        NS_ASSERT(m_rerrRateLimitTimer.IsRunning());
        // discard the packet and return
        NS_LOG_LOGIC("RerrRateLimit reached at "
                     << Simulator::Now().As(Time::S) << " with timer delay left "
                     << m_rerrRateLimitTimer.GetDelayLeft().As(Time::S) << "; suppressing RERR");
        return;
    }
    RerrHeader<IpAddress> rerrHeader;
    rerrHeader.AddUnDestination(dst, dstSeqNo);
    Ptr<Packet> packet = Create<Packet>();
    std::vector<LocalRoute<IpAddress>> routes;
    if (m_routingTable.LookupValidRoutes(origin, routes) && routes.size() > 0)
    {
        LocalRoute<IpAddress> toOrigin = routes[0];
        Ptr<Socket> socket = FindSocketWithInterfaceAddress(toOrigin.GetInterface());
        rerrHeader.SetOrigIp(GetInterfaceAddress(toOrigin.GetInterface()));
        rerrHeader.SetOrigMask(32); // TODO me: update if needed
        packet->AddHeader(rerrHeader);
        NS_ASSERT(socket);
        NS_LOG_LOGIC("Unicast RERR to the source of the data transmission");
        socket->SendTo(packet, 0, InetVxSocketAddress(toOrigin.GetNextHop(), AODVV2_PORT));
    }
    else
    {
        for (auto i = m_socketAddresses.begin(); i != m_socketAddresses.end(); ++i)
        {
            Ptr<Socket> socket = i->first;
            IpInterfaceAddress iface = i->second;
            rerrHeader.SetOrigIp(GetInterfaceAddress(iface));
            rerrHeader.SetOrigMask(32); // TODO me: update if needed
            packet->AddHeader(rerrHeader);
            NS_ASSERT(socket);
            NS_LOG_LOGIC("Broadcast RERR message from interface " << GetInterfaceAddress(iface));
            // Send to all-hosts broadcast if on /32 addr, subnet-directed otherwise
            IpAddress destination;
            if constexpr (std::is_same<T, Ipv4RoutingProtocol>::value)
            {
                if (iface.GetMask() == Ipv4Mask::GetOnes())
                {
                    destination = IpAddress("255.255.255.255");
                }
                else
                {
                    destination = iface.GetBroadcast();
                }
            }
            else
            {
                // TODO Ipv6
            }
            socket->SendTo(packet->Copy(), 0, InetVxSocketAddress(destination, AODVV2_PORT));
        }
    }
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::SendRerrMessage(Ptr<Packet> packet, std::vector<IpAddress> precursors)
{
    NS_LOG_FUNCTION(this);

    if (precursors.empty())
    {
        NS_LOG_LOGIC("No precursors");
        return;
    }
    // A node SHOULD NOT originate more than RERR_RATELIMIT RERR messages per second.
    if (m_rerrCount == m_rerrRateLimit)
    {
        // Just make sure that the RerrRateLimit timer is running and will expire
        NS_ASSERT(m_rerrRateLimitTimer.IsRunning());
        // discard the packet and return
        NS_LOG_LOGIC("RerrRateLimit reached at "
                     << Simulator::Now().As(Time::S) << " with timer delay left "
                     << m_rerrRateLimitTimer.GetDelayLeft().As(Time::S) << "; suppressing RERR");
        return;
    }
    // If there is only one precursor, RERR SHOULD be unicast toward that precursor
    if (precursors.size() == 1)
    {
        std::vector<LocalRoute<IpAddress>> routes;
        if (m_routingTable.LookupValidRoutes(precursors.front(), routes))
        {
            LocalRoute<IpAddress> toPrecursor = routes[0];
            if (m_RouteErrorSet.HasRerr(toPrecursor.GetDestination(),
                                  GetInterfaceAddress(toPrecursor.GetInterface())))
            {
                return;
            }
            else
            {
                m_RouteErrorSet.Add(toPrecursor.GetDestination(),
                              GetInterfaceAddress(toPrecursor.GetInterface()),
                              m_rerrTimeout);
            }
            Ptr<Socket> socket = FindSocketWithInterfaceAddress(toPrecursor.GetInterface());
            NS_ASSERT(socket);
            NS_LOG_LOGIC("one precursor => unicast RERR to "
                         << toPrecursor.GetDestination() << " from "
                         << GetInterfaceAddress(toPrecursor.GetInterface()));
            Simulator::Schedule(Time(MilliSeconds(m_uniformRandomVariable->GetInteger(0, 10))),
                                &Aodvv2RoutingProtocol<T>::SendTo,
                                this,
                                socket,
                                packet,
                                precursors.front());
            m_rerrCount++;
        }
        return;
    }

    //  Should only transmit RERR on those interfaces which have precursor nodes for the
    //  broken route
    std::vector<IpInterfaceAddress> ifaces;
    std::vector<LocalRoute<IpAddress>> routes;
    for (auto i = precursors.begin(); i != precursors.end(); ++i)
    {
        if (m_routingTable.LookupRoutes(*i, routes))
        {
            LocalRoute<IpAddress> toPrecursor = routes[0];
            if (std::find(ifaces.begin(), ifaces.end(), toPrecursor.GetInterface()) == ifaces.end())
            {
                ifaces.push_back(toPrecursor.GetInterface());
            }
        }
    }

    for (auto i = ifaces.begin(); i != ifaces.end(); ++i)
    {
        Ptr<Socket> socket = FindSocketWithInterfaceAddress(*i);
        NS_ASSERT(socket);
        NS_LOG_LOGIC("Broadcast RERR message from interface " << GetInterfaceAddress(*i));
        // std::cout << "Broadcast RERR message from interface " << i->GetLocal () <<
        // std::endl; Send to all-hosts broadcast if on /32 addr, subnet-directed otherwise
        Ptr<Packet> p = packet->Copy();
        IpAddress destination;
        if constexpr (std::is_same<T, Ipv4RoutingProtocol>::value)
        {
            if (i->GetMask() == Ipv4Mask::GetOnes())
            {
                destination = IpAddress("255.255.255.255");
            }
            else
            {
                destination = i->GetBroadcast();
            }
        }
        else
        {
            // TODO Ipv6
        }
        Simulator::Schedule(Time(MilliSeconds(m_uniformRandomVariable->GetInteger(0, 10))),
                            &Aodvv2RoutingProtocol<T>::SendTo,
                            this,
                            socket,
                            p,
                            destination);
    }
}

template <typename T>
Ptr<Socket>
Aodvv2RoutingProtocol<T>::FindSocketWithInterfaceAddress(IpInterfaceAddress addr) const
{
    NS_LOG_FUNCTION(this << addr);
    for (auto j = m_socketAddresses.begin(); j != m_socketAddresses.end(); ++j)
    {
        Ptr<Socket> socket = j->first;
        IpInterfaceAddress iface = j->second;
        if (iface == addr)
        {
            return socket;
        }
    }
    Ptr<Socket> socket;
    return socket;
}

template <typename T>
Ptr<Socket>
Aodvv2RoutingProtocol<T>::FindSubnetBroadcastSocketWithInterfaceAddress(
    IpInterfaceAddress addr) const
{
    NS_LOG_FUNCTION(this << addr);
    for (auto j = m_socketSubnetBroadcastAddresses.begin();
         j != m_socketSubnetBroadcastAddresses.end();
         ++j)
    {
        Ptr<Socket> socket = j->first;
        IpInterfaceAddress iface = j->second;
        if (iface == addr)
        {
            return socket;
        }
    }
    Ptr<Socket> socket;
    return socket;
}

template <typename T>
void
Aodvv2RoutingProtocol<T>::DoInitialize()
{
    NS_LOG_FUNCTION(this);
    IpRoutingProtocol::DoInitialize();
}

template int64_t Aodvv2RoutingProtocol<Ipv4RoutingProtocol>::AssignStreams(int64_t stream);
template int64_t Aodvv2RoutingProtocol<Ipv6RoutingProtocol>::AssignStreams(int64_t stream);

} // namespace aodvv2
} // namespace ns3
