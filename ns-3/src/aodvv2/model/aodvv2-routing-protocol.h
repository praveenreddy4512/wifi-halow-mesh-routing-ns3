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
#ifndef AODVV2_ROUTING_PROTOCOL_H
#define AODVV2_ROUTING_PROTOCOL_H

#include "aodvv2-local-route-set.h"
#include "aodvv2-metric.h"
#include "aodvv2-multi-msg-set.h"
#include "aodvv2-neighbor-set.h"
#include "aodvv2-packet.h"
#include "aodvv2-rerr-set.h"
#include "aodvv2-route-client-set.h"
#include "aodvv2-rqueue.h"

#include "ns3/ipv4-interface.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/ipv4-routing-helper.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv6-interface.h"
#include "ns3/ipv6-l3-protocol.h"
#include "ns3/ipv6-routing-helper.h"
#include "ns3/ipv6-routing-protocol.h"
#include "ns3/arp-cache.h"
#include "ns3/error-rate-model.h"
#include "ns3/mac48-address.h"
#include "ns3/node.h"
#include "ns3/output-stream-wrapper.h"
#include "ns3/random-variable-stream.h"
#include "ns3/wifi-mac-header.h"
#include "ns3/wifi-tx-vector.h"

#include <set>

#include <map>

namespace ns3
{

class WifiMpdu;
enum WifiMacDropReason : uint8_t; // opaque enum declaration

namespace aodvv2
{
/**
 * @ingroup aodvv2
 *
 * @brief AODVv2 routing protocol
 */

/// UDP Port for AODVv2 control traffic
static const uint32_t AODVV2_PORT = 269;

template <typename T>
class Aodvv2RoutingProtocol : public std::enable_if_t<std::is_same_v<Ipv4RoutingProtocol, T> ||
                                                          std::is_same_v<Ipv6RoutingProtocol, T>,
                                                      T>
{
    /// Alias for determining whether the parent is Ipv4RoutingProtocol or Ipv6RoutingProtocol
    static constexpr bool IsIpv4 = std::is_same_v<Ipv4RoutingProtocol, T>;
    /// Alias for Ipv4 and Ipv6 classes
    using Ip = typename std::conditional_t<IsIpv4, Ipv4, Ipv6>;
    /// Alias for Ipv4Address and Ipv6Address classes
    using IpAddress = typename std::conditional_t<IsIpv4, Ipv4Address, Ipv6Address>;
    /// Alias for Ipv4Route and Ipv6Route classes
    using IpRoute = typename std::conditional_t<IsIpv4, Ipv4Route, Ipv6Route>;

    /// Alias for Ipv4AddressHash and Ipv6AddressHash classes
    using IpAddressHash = typename std::conditional_t<IsIpv4, Ipv4AddressHash, Ipv6AddressHash>;
    /// Alias for Ipv4Header and Ipv6Header classes
    using IpHeader = typename std::conditional_t<IsIpv4, Ipv4Header, Ipv6Header>;
    /// Alias for Ipv4InterfaceAddress and Ipv6InterfaceAddress classes
    using IpInterfaceAddress =
        typename std::conditional_t<IsIpv4, Ipv4InterfaceAddress, Ipv6InterfaceAddress>;
    static IpAddress GetInterfaceAddress(const IpInterfaceAddress& address)
    {
        if constexpr (IsIpv4)
        {
            return address.GetLocal();
        }
        else
        {
            return address.GetAddress();
        }
    }
    /// Alias for Ipv4Interface and Ipv6Interface classes
    using IpInterface = typename std::conditional_t<IsIpv4, Ipv4Interface, Ipv6Interface>;
    /// Alias for Ipv4L3Protocol and Ipv6L3Protocol classes
    using IpL3Protocol = typename std::conditional_t<IsIpv4, Ipv4L3Protocol, Ipv6L3Protocol>;
    /// Alias for Ipv4RoutingProtocol and Ipv6RoutingProtocol classes
    using IpRoutingProtocol =
        typename std::conditional_t<IsIpv4, Ipv4RoutingProtocol, Ipv6RoutingProtocol>;
    /// Alias for InetSocketAddress and Inet6SocketAddress classes
    using InetVxSocketAddress =
        typename std::conditional_t<IsIpv4, InetSocketAddress, Inet6SocketAddress>;

    /// Callback for IPv4 unicast packets to be forwarded
    typedef Callback<void, Ptr<IpRoute>, Ptr<const Packet>, const IpHeader&>
        UnicastForwardCallbackv4;

    /// Callback for IPv6 unicast packets to be forwarded
    typedef Callback<void, Ptr<const NetDevice>, Ptr<IpRoute>, Ptr<const Packet>, const IpHeader&>
        UnicastForwardCallbackv6;

    /// Callback for unicast packets to be forwarded
    typedef typename std::conditional_t<IsIpv4, UnicastForwardCallbackv4, UnicastForwardCallbackv6>
        UnicastForwardCallback;

    /// Callback for IPv4 multicast packets to be forwarded
    typedef Callback<void, Ptr<Ipv4MulticastRoute>, Ptr<const Packet>, const IpHeader&>
        MulticastForwardCallbackv4;

    /// Callback for IPv6 multicast packets to be forwarded
    typedef Callback<void,
                     Ptr<const NetDevice>,
                     Ptr<Ipv6MulticastRoute>,
                     Ptr<const Packet>,
                     const IpHeader&>
        MulticastForwardCallbackv6;

    /// Callback for multicast packets to be forwarded
    typedef
        typename std::conditional_t<IsIpv4, MulticastForwardCallbackv4, MulticastForwardCallbackv6>
            MulticastForwardCallback;

    /// Callback for packets to be locally delivered
    typedef Callback<void, Ptr<const Packet>, const IpHeader&, uint32_t> LocalDeliverCallback;

    /// Callback for routing errors (e.g., no route found)
    typedef Callback<void, Ptr<const Packet>, const IpHeader&, Socket::SocketErrno> ErrorCallback;

  public:
    /**
     * @brief Get the type ID.
     * @return the object TypeId
     */
    static TypeId GetTypeId();

    /// constructor
    Aodvv2RoutingProtocol();
    ~Aodvv2RoutingProtocol();

    /**
     * \sa Ipv4RoutingProtocol::DoDispose
     * \sa Ipv6RoutingProtocol::DoDispose
     */
    void DoDispose();

    /* From Ipv4RoutingProtocol and Ipv6RoutingProtocol */
    /**
     * @brief Query routing cache for an existing route, for an outbound packet
     * @param p packet to be routed.  Note that this method may modify the packet.
     *          Callers may also pass in a null pointer.
     * @param header input parameter (used to form key to search for the route)
     * @param oif Output interface Netdevice.  May be zero, or may be bound via
     *            socket options to a particular output interface.
     * @param sockerr Output parameter; socket errno
     *
     * @returns a code that indicates what happened in the lookup
     *
     * \sa Ipv4RoutingProtocol::RouteOutput
     * \sa Ipv6RoutingProtocol::RouteOutput
     */
    virtual Ptr<IpRoute> RouteOutput(Ptr<Packet> p,
                                     const IpHeader& header,
                                     Ptr<NetDevice> oif,
                                     Socket::SocketErrno& sockerr);
    /**
     * @brief Route an input packet (to be forwarded or locally delivered)
     * @param p received packet
     * @param header input parameter used to form a search key for a route
     * @param idev Pointer to ingress network device
     * @param ucb Callback for the case in which the packet is to be forwarded
     *            as unicast
     * @param mcb Callback for the case in which the packet is to be forwarded
     *            as multicast
     * @param lcb Callback for the case in which the packet is to be locally
     *            delivered
     * @param ecb Callback to call if there is an error in forwarding
     *
     * @returns true if Aodvv2RoutingProtocol class takes responsibility for
     *          forwarding or delivering the packet, false otherwise
     *
     * \sa Ipv4RoutingProtocol::RouteInput
     * \sa Ipv6RoutingProtocol::RouteInput
     */
    virtual bool RouteInput(Ptr<const Packet> p,
                            const IpHeader& header,
                            Ptr<const NetDevice> idev,
                            UnicastForwardCallback ucb,
                            MulticastForwardCallback mcb,
                            LocalDeliverCallback lcb,
                            ErrorCallback ecb);
    /**
     * @param interface the index of the interface we are being notified about
     *
     * \sa Ipv4RoutingProtocol::NotifyInterfaceUp
     * \sa Ipv6RoutingProtocol::NotifyInterfaceUp
     */
    virtual void NotifyInterfaceUp(uint32_t interface);
    /**
     * @param interface the index of the interface we are being notified about
     *
     * \sa Ipv4RoutingProtocol::NotifyInterfaceDown
     * \sa Ipv6RoutingProtocol::NotifyInterfaceDown
     */
    virtual void NotifyInterfaceDown(uint32_t interface);
    /**
     * @param interface the index of the interface we are being notified about
     * @param address a new address being added to an interface
     *
     * \sa Ipv4RoutingProtocol::NotifyAddAddress
     * \sa Ipv6RoutingProtocol::NotifyAddAddress
     */
    virtual void NotifyAddAddress(uint32_t interface, IpInterfaceAddress address);
    /**
     * @param interface the index of the interface we are being notified about
     * @param address a new address being added to an interface
     *
     * \sa Ipv4RoutingProtocol::NotifyRemoveAddress
     * \sa Ipv6RoutingProtocol::NotifyRemoveAddress
     */
    virtual void NotifyRemoveAddress(uint32_t interface, IpInterfaceAddress address);
    /* From IPv4RoutingProtocol */
    /**
     * @brief Typically, invoked directly or indirectly from ns3::Ipv4::SetRoutingProtocol
     *
     * @param ipv4 the ipv4 object this routing protocol is being associated with
     *
     * \sa Ipv4RoutingProtocol::SetIpv4
     */
    virtual void SetIpv4(Ptr<Ipv4> ipv4);
    /* From IPv6RoutingProtocol */
    /**
     * @brief Typically, invoked directly or indirectly from ns3::Ipv6::SetRoutingProtocol
     *
     * @param ipv6 the ipv6 object this routing protocol is being associated with
     *
     * \sa Ipv6RoutingProtocol::SetIpv6
     */
    virtual void SetIpv6(Ptr<Ipv6> ipv6);

    /**
     * @brief Notify a new route.
     *
     * @param dst destination address
     * @param mask destination mask
     * @param nextHop nextHop for this destination
     * @param interface output interface
     * @param prefixToUse prefix to use as source with this route
     *
     * \sa Ipv6RoutingProtocol::NotifyAddRoute
     */
    virtual void NotifyAddRoute(IpAddress dst,
                                Ipv6Prefix mask,
                                IpAddress nextHop,
                                uint32_t interface,
                                IpAddress prefixToUse = IpAddress::GetZero());

    /**
     * @brief Notify route removing.
     *
     * @param dst destination address
     * @param mask destination mask
     * @param nextHop nextHop for this destination
     * @param interface output interface
     * @param prefixToUse prefix to use as source with this route
     *
     * \sa Ipv6RoutingProtocol::NotifyRemoveRoute
     */
    virtual void NotifyRemoveRoute(IpAddress dst,
                                   Ipv6Prefix mask,
                                   IpAddress nextHop,
                                   uint32_t interface,
                                   IpAddress prefixToUse = IpAddress::GetZero());
    /**
     * @brief Print the Routing Table entries
     *
     * @param stream The ostream the Routing table is printed to
     * @param unit The time unit to be used in the report
     *
     * \sa Ipv4RoutingProtocol::PrintRoutingTable
     * \sa Ipv6RoutingProtocol::PrintRoutingTable
     */
    virtual void PrintRoutingTable(Ptr<OutputStreamWrapper> stream) const;

    // Handle protocol parameters
    /**
     * Get m_resetPeriodically flag
     */
    bool GetResetPeriodically() const
    {
        return m_resetPeriodically;
    }

    /**
     * Set reset periodically flag
     */
    void SetResetPeriodically(bool resetPeriodically)
    {
        m_resetPeriodically = resetPeriodically;
    }

    /**
     * Reset protocol
     */
    void ResetProtocol();

    /**
     * Get maximum queue time
     * @returns the maximum queue time
     */
    Time GetMaxQueueTime() const
    {
        return m_maxQueueTime;
    }

    /**
     * Set the maximum queue time
     * @param t the maximum queue time
     */
    void SetMaxQueueTime(Time t);

    /**
     * Get the maximum queue length
     * @returns the maximum queue length
     */
    uint32_t GetMaxQueueLen() const
    {
        return m_maxQueueLen;
    }

    /**
     * Set the maximum queue length
     * @param len the maximum queue length
     */
    void SetMaxQueueLen(uint32_t len);

    /**
     * Get destination only flag
     * @returns the destination only flag
     */
    bool GetDestinationOnlyFlag() const
    {
        return m_destinationOnly;
    }

    /**
     * Set destination only flag
     * @param f the destination only flag
     */
    void SetDestinationOnlyFlag(bool f)
    {
        m_destinationOnly = f;
    }

    /**
     * Set broadcast enable flag
     * @param f enable broadcast flag
     */
    void SetBroadcastEnable(bool f)
    {
        m_enableBroadcast = f;
    }

    /**
     * Get broadcast enable flag
     * @returns the broadcast enable flag
     */
    bool GetBroadcastEnable() const
    {
        return m_enableBroadcast;
    }

    /**
     * Set the use of the default metric (hop)
     * @param useDefaultMetric the use of the default metric
     */
    void SetUseDefaultMetric(bool useDefaultMetric)
    {
        m_useDefaultMetric = useDefaultMetric;
    }

    /**
     * @brief Add a metric to the metrics list
     * @param metric The metric to add
     * @returns true if the metric is added
     */
    bool AddMetric(const Metric<IpAddress>& metric)
    {
        // add if the metricType attribute is not already in the list
        for (auto& m : m_metrics)
        {
            if (m.GetMetricType() == metric.GetMetricType())
            {
                return false;
            }
        }
        m_metrics.push_back(metric);
        return true;
    }

    /**
     * @brief Add a metric node to the metric nodes list
     * @param node The node to add
     * @param metricNode The metric node to add
     * @returns true if the pair is added
     */
    bool AddMetricNode(Ptr<Node> node, const MetricNode& metricNode)
    {
        auto result = m_metricNodes.insert(std::make_pair(node, metricNode));
        return result.second;
    }

    /**
     * @brief Get the metricNode given a node
     * @param node The node
     * @returns the metricNode
     */
    MetricNode GetMetricNode(Ptr<Node> node) const
    {
        auto it = m_metricNodes.find(node);
        if (it != m_metricNodes.end())
        {
            return it->second;
        }
        return MetricNode(node);
    }

    /**
     * @brief Get the list of metrics, if empty return the default metric
     * @returns the list of metrics
     */
    std::vector<Metric<IpAddress>> GetMetrics() const
    {
        if (m_metrics.empty() || m_useDefaultMetric)
        {
            Metric<IpAddress> defaultMetric{};

            if (m_metrics.empty())
            {
                return {defaultMetric};
            }
            else
            {
                std::vector<Metric<IpAddress>> metrics = {defaultMetric};
                metrics.insert(metrics.end(), m_metrics.begin(), m_metrics.end());
                return metrics;
            }
        }
        return m_metrics;
    }

    /**
     * @brief Get the metric based on the metric type
     * @param metricType The metric type
     * @returns the metric
     */
    Metric<IpAddress> GetMetric(uint8_t metricType) const
    {
        for (auto& m : GetMetrics())
        {
            if (m.GetMetricType() == metricType)
            {
                return m;
            }
        }
        throw std::runtime_error("Metric not found");
    }

    /**
     * Assign a fixed random variable stream number to the random variables
     * used by this model.  Return the number of streams (possibly zero) that
     * have been assigned.
     *
     * @param stream first stream index to use
     * @return the number of stream indices assigned by this model
     */
    int64_t AssignStreams(int64_t stream);

  protected:
    /**
     * \sa Ipv4RoutingProtocol::DoInitialize
     * \sa Ipv6RoutingProtocol::DoInitialize
     */
    void DoInitialize();

  private:
    /**
     * Notify that an MPDU was dropped.
     *
     * @param reason the reason why the MPDU was dropped
     * @param mpdu the dropped MPDU
     */
   // void NotifyTxError(WifiMacDropReason reason, Ptr<const WifiMpdu> mpdu);

    // Protocol parameters.
    uint32_t m_discoveryAttemptsMax; ///< Maximum number of retransmissions of RREQ
    uint32_t m_rrepRetries;          ///< Maximum number of retransmissions of RREP
    uint32_t m_maxHopLimit;          ///< Maximum number of hops allowed for a route
    uint16_t m_timeoutBuffer;        ///< Provide a buffer for the timeout.
    double_t m_controlTrafficLimit;  ///< Maximum number of control packets that can be sent
    uint16_t m_rreqRateLimit;        ///< Maximum number of RREQ per second.
    uint16_t m_rrepRateLimit;        ///< Maximum number of RREP per second.
    uint16_t m_rerrRateLimit;        ///< Maximum number of REER per second.
    Time m_activeInterval; ///< Period of time during which the route is considered to be valid.
    /**
     * NodeTraversalTime is a conservative estimate of the average one hop traversal time for
     * packets and should include queuing delays, interrupt processing times and transfer times.
     */
    Time m_nodeTraversalTime;
    Time m_netTraversalTime;  ///< Estimate of the average net traversal time.
    Time m_pathDiscoveryTime; ///< Estimate of maximum time needed to find route in network.
    Time m_myRouteTimeout;    ///< Value of lifetime field in RREP generating by this node.
    /**
     * DeletePeriod is intended to provide an upper bound on the time for which an upstream node A
     * can have a neighbor B as an active next hop for destination D, while B has invalidated the
     * route to D.
     */
    Time m_nextHopWait;        ///< Period of our waiting for the neighbour's RREP_ACK
    Time m_rreqWaitTime;       ///< Period of time to wait for a RREQ reply
    Time m_rreqHolddownTime;   ///< Period of time to wait before another route discovery
    Time m_rrepAckSentTimeout; ///< Period of time to consider an RREP_ACK expired
    Time m_rerrTimeout;        ///< Period of time to consider an RERR expired
    Time m_maxIdleTime;        ///< Time for which the node is put into the blacklist
    Time m_maxBlacklistTime;   ///< Time for which the node is removed from the blacklist
    Time m_maxSeqnumLifetime;  ///< Maximum time a sequence number is considered valid
    uint32_t m_maxQueueLen; ///< The maximum number of packets that we allow a routing protocol to
                            ///< buffer.
    Time m_maxQueueTime;    ///< The maximum period of time that a routing protocol is allowed to
                            ///< buffer a packet for.
    Time m_rtemsgEntryTime; ///< The maximum period of time that a routing protocol is allowed to
                            ///< buffer a multicast packet for.
    bool m_destinationOnly; ///< Indicates only the destination may respond to this RREQ.
    bool m_gratuitousReply; ///< Indicates whether a gratuitous RREP should be unicast to the node
                            ///< originated route discovery.
    bool m_enableBroadcast; ///< Indicates whether a a broadcast data packets forwarding enable

    /// IP protocol
    Ptr<Ip> m_ip;
    /// Raw unicast socket per each IP interface, map socket -> iface address (IP + mask)
    std::map<Ptr<Socket>, IpInterfaceAddress> m_socketAddresses;
    /// Raw subnet directed broadcast socket per each IP interface, map socket -> iface address (IP
    /// + mask)
    std::map<Ptr<Socket>, IpInterfaceAddress> m_socketSubnetBroadcastAddresses;
    /// Loopback device used to defer RREQ until packet will be fully formed
    Ptr<NetDevice> m_lo;

    /// Routing table
    LocalRouteSet<IpAddress> m_routingTable;
    /// A "drop-front" queue used by the routing layer to buffer packets to which it does not have a
    /// route.
    RequestQueue<IpAddress> m_queue;
    /// Broadcast ID
    uint32_t m_requestId;
    /// Request sequence number
    uint16_t m_seqNo;
    /// Handle duplicated RREQ
    MulticastMessageSet<IpAddress> m_mms;
    /// Handle neighbors
    NeighborSet<IpAddress> m_nb;
    /// Handle route clients
    RouterClientSet<IpAddress> m_rcs;
    /// Handle rerrs
    RouteErrorSet<IpAddress> m_RouteErrorSet;
    /// Number of RREQs used for RREQ rate control
    uint16_t m_rreqCount;
    /// Number of RREPs used for RREP rate control
    uint16_t m_rrepCount;
    /// Number of RERRs used for RERR rate control
    uint16_t m_rerrCount;
    /// List of metrics
    std::vector<Metric<IpAddress>> m_metrics;
    /// Use default metric
    bool m_useDefaultMetric;
    /// Map of nodes and their corresponding MetricNode
    std::map<Ptr<Node>, MetricNode> m_metricNodes;

    // --- AODVv2-LQ: SNR-derived ETX link-quality metric (future work,
    // Section V-D of the project's paper). Everything below is additive:
    // when m_enableLinkQualityMetric is false (its default), none of this
    // is reachable and the protocol's behavior is byte-identical to the
    // unmodified baseline. See AODVV2LQ_DESIGN_NOTES.md for the full
    // derivation and the limitations of this first implementation.

    /// AODVv2-LQ on/off switch. Default false preserves the baseline.
    bool m_enableLinkQualityMetric;
    /// This node's own PHY error-rate model, captured once from its
    /// WifiNetDevice when the link-quality metric is enabled. Null (and
    /// therefore a worst-case ETX fallback) if the device is not a
    /// YansWifiPhy, or before the metric is enabled.
    Ptr<ErrorRateModel> m_phyErrorRateModel;
    /// This node's own ARP cache, cached once for resolving a received
    /// message's IP-layer sender (already corrected to the true one-hop
    /// relayer via MeshPreviousHopTag) to the MAC address this project's
    /// PHY-level ETX observations are keyed by.
    Ptr<ArpCache> m_etxArpCache;
    /// Most recently observed predicted ETX per neighbor MAC address,
    /// updated passively on every frame received from that neighbor (no
    /// extra probe traffic is sent). Absent entries are treated as
    /// worst-case (255), never as a good link, since no observation is not
    /// evidence of a good link.
    std::map<Mac48Address, double> m_neighborEtx;
    /// Fix 5 (docs/AODVV2_LQ_FIX5_NEIGHBOR_IDENTITY_IMPLEMENTATION_REPORT.md):
    /// the actual, used source of ETX for AODVv2-LQ. Keyed by packet UID,
    /// not by any MAC address -- Fix 5's root-cause investigation
    /// (AODVV2_LQ_FIX5_SRC48_ROOT_CAUSE_REPORT.md) confirmed that this
    /// project's custom L2 forwarding never rewrites a relayed frame's
    /// 802.11 transmitter address (Addr2), so no MAC read from a frame
    /// -- not src48, not MeshLastHopMacTag, not hdr.GetAddr2() in
    /// SnoopPhyRx -- identifies the true immediate relayer for anything
    /// but a packet's first, direct hop. Packet UID is unaffected by
    /// that characteristic: SnoopPhyRx() records the ETX it measures for
    /// *this exact packet instance's* reception here, by UID; RecvAodvv2()
    /// looks up the same UID for the same packet once it reaches the IP
    /// layer, pairing the physically-correct SNR/ETX measurement with the
    /// independently-trusted MeshPreviousHopTag identity, without ever
    /// needing a MAC identity for this purpose at all. Pruned the same
    /// way NoHwmpRoutingProtocol's own duplicate-suppression cache is
    /// (src/mesh/model/dot11s/no-hwmp-protocol.cc,
    /// m_recentlyRelayedGroupPackets), the same precedented pattern.
    std::map<uint64_t, std::pair<double, Time> > m_recentPhyEtxByUid;
    static const Time kPhyEtxUidWindow;
    /// Scratch value: the ETX of the one-hop sender of the AODVv2 message
    /// currently being processed inside RecvAodvv2(). Set once per
    /// received message (ns-3's event loop is single-threaded, so this is
    /// safe) and read by the link-quality Metric's LinkCost/RouteCost
    /// lambdas, which otherwise have no way to see which physical link a
    /// given call relates to (MetricNode, as used throughout this file,
    /// always wraps this node itself, never the neighbor).
    double m_currentHopEtx;
    /// Raw WifiPhy pointers (as opaque identity, never dereferenced as a
    /// key) already hooked by HookLinkQualitySnooping(), so a second call
    /// for the same device -- e.g. if NotifyInterfaceUp() ever fires more
    /// than once for it -- does not connect the same trace twice and
    /// double-count observations. Fix 1, see
    /// docs/AODVV2_LQ_FIX1_IMPLEMENTATION_REPORT.md.
    std::set<const void*> m_hookedPhys;

    /**
     * @brief Enable or disable AODVv2-LQ (SNR-derived ETX route selection).
     * @param enable true to register and switch to the link-quality metric.
     */
    void SetEnableLinkQualityMetric(bool enable);

    /**
     * @brief Whether AODVv2-LQ is enabled.
     */
    bool GetEnableLinkQualityMetric() const
    {
        return m_enableLinkQualityMetric;
    }

    /**
     * @brief Build the AODVv2-LQ Metric<IpAddress> instance (LinkCost,
     * RouteCost, LoopFree), to be registered via AddMetric().
     */
    Metric<IpAddress> BuildLinkQualityMetric();

    /**
     * @brief Resolve an IP-layer neighbor address to the MAC address this
     * project's PHY-level ETX observations (m_neighborEtx) are keyed by,
     * via this node's own ARP cache. Returns the all-zero address if the
     * mapping is not (yet) known.
     */
    Mac48Address ResolveNeighborMac(Ipv4Address ip) const;

    /**
     * @brief Fix 4: resolve the immediate previous hop's MAC directly
     * from MeshLastHopMacTag on the received packet, bypassing ARP
     * entirely (see AODVV2_LQ_FIX4_IMPLEMENTATION_REPORT.md for why Fix 3
     * found ARP unusable for this). Returns the all-zero address if the
     * tag is absent or carries an invalid (zero/broadcast/group) value.
     */
    Mac48Address ResolveLastHopMacFromTag(Ptr<const Packet> packet) const;

    /**
     * @brief PHY receive sniffer, connected to this node's own
     * WifiNetDevice when AODVv2-LQ is enabled. Converts the observed
     * signal/noise into a predicted ETX via this project's own PHY
     * ErrorRateModel (not an arbitrary formula, and not a classical
     * active-probe ETX estimate -- see AODVV2LQ_DESIGN_NOTES.md) and
     * records it for whichever neighbor sent the frame. Runs on every
     * received frame (beacons, data, AODVv2 control messages themselves);
     * sends no additional traffic of its own.
     */
    void SnoopPhyRx(Ptr<const Packet> packet,
                    uint16_t channelFreqMhz,
                    uint16_t channelNumber,
                    uint32_t rate,
                    bool isShortPreamble,
                    WifiTxVector txVector,
                    double signalDbm,
                    double noiseDbm);

    /**
     * @brief Connect SnoopPhyRx() to this device's WifiPhy and capture its
     * ErrorRateModel, if AODVv2-LQ is enabled and this is a WifiNetDevice.
     * Called from NotifyInterfaceUp(), where m_ip (and therefore this
     * node's device list) is already valid -- see the comment in
     * SetEnableLinkQualityMetric() for why that logic cannot live there.
     * @param device The interface's NetDevice, as already resolved by
     * NotifyInterfaceUp() at its call site.
     */
    void HookLinkQualitySnooping(Ptr<NetDevice> device);

  private:
    /// Start protocol operation
    void Start();
    /**
     * Queue packet and send route request
     *
     * @param p the packet to route
     * @param header the IP header
     * @param ucb the UnicastForwardCallback function
     * @param ecb the ErrorCallback function
     */
    void DeferredRouteOutput(Ptr<const Packet> p,
                             const IpHeader& header,
                             UnicastForwardCallback ucb,
                             ErrorCallback ecb);
    /**
     * If route exists and is valid, forward packet.
     *
     * @param p the packet to route
     * @param header the IP header
     * @param ucb the UnicastForwardCallback function
     * @param ecb the ErrorCallback function
     * @returns true if forwarded
     */
    bool Forwarding(Ptr<const Packet> p,
                    const IpHeader& header,
                    UnicastForwardCallback ucb,
                    ErrorCallback ecb);
    /**
     * Repeated attempts by a source node at route discovery for a single destination
     * @param dst the destination IP address
     */
    void ScheduleRreqRetry(IpAddress dst);
    /**
     * Repeated attempts by a destination node at route discovery for a single source
     * @param rreqHeader route request header
     * @param toOrigin routing table entry to originator
     * @param hopCount hop count
     */
    void ScheduleRrepRetry(const RreqHeader<IpAddress>& rreqHeader,
                           const LocalRoute<IpAddress>& toOrigin,
                           uint8_t hopCount);
    /**
     * Get route metric type
     * @param addr destination address
     * @returns the route metric type
     */
    std::vector<uint8_t> GetRouteMetricTypes(IpAddress addr);
    /**
     * Set lifetime field in routing table entry to the maximum of existing lifetime and lt, if the
     * entry exists
     * @param addr destination address
     * @param lt proposed time for lifetime field in routing table entry for destination with
     * address addr.
     * @return true if route to destination address addr exist
     */
    bool UpdateRouteLifeTime(IpAddress addr, Time lt);
    /**
     * UpdateTimeout neighbor record.
     * @param receiver is supposed to be my interface
     * @param sender is supposed to be IP address of my neighbor.
     */
    void UpdateRouteToNeighbor(IpAddress sender, IpAddress receiver);
    /**
     * Test whether the provided address is assigned to an interface on this node
     * @param src the source IP address
     * @returns true if the IP address is the node's IP address
     */
    bool IsMyOwnAddress(IpAddress src);
    /**
     * Find unicast socket with local interface address iface
     *
     * @param iface the interface
     * @returns the socket associated with the interface
     */
    Ptr<Socket> FindSocketWithInterfaceAddress(IpInterfaceAddress iface) const;
    /**
     * Find subnet directed broadcast socket with local interface address iface
     *
     * @param iface the interface
     * @returns the socket associated with the interface
     */
    Ptr<Socket> FindSubnetBroadcastSocketWithInterfaceAddress(IpInterfaceAddress iface) const;
    /**
     * Create loopback route for given header
     *
     * @param header the IP header
     * @param oif the output interface net device
     * @returns the route
     */
    Ptr<IpRoute> LoopbackRoute(const IpHeader& header, Ptr<NetDevice> oif) const;

    /**
     * @name Receive control packets
     * @{
     */
    /**
     * Receive and process control packet
     * @param socket input socket
     */
    void RecvAodvv2(Ptr<Socket> socket);
    /**
     * Receive RREQ
     * @param p packet
     * @param receiver receiver address
     * @param src sender address
     * @param tlvHeader TLV header
     */
    void RecvRequest(Ptr<Packet> p, IpAddress receiver, IpAddress src, PbbPacket tlvHeader);
    /**
     * Receive RREP
     * @param p packet
     * @param my destination address
     * @param src sender address
     * @param tlvHeader TLV header
     */
    void RecvReply(Ptr<Packet> p, IpAddress my, IpAddress src, PbbPacket tlvHeader);
    /**
     * Receive RREP_ACK
     * @param neighbor neighbor address
     */
    void RecvReplyAck(IpAddress neighbor);
    /**
     * Receive RERR
     * @param p packet
     * @param src sender address
     * @param tlvHeader TLV header
     */
    /// Receive  from node with address src
    void RecvError(Ptr<Packet> p, IpAddress src, PbbPacket tlvHeader);
    /** @} */

    /**
     * Save packet data when received
     * @param type packet type
     * @param size packet size
     */
    void SavePacketData(std::string type, uint32_t size);

    /**
     * @name Send
     * @{
     */
    /** Forward packet from route request queue
     * @param dst destination address
     * @param route route to use
     */
    void SendPacketFromQueue(IpAddress dst, Ptr<IpRoute> route);
    /** Send RREQ
     * @param dst destination address
     */
    void SendRequest(IpAddress dst);
    /** Add TLV headers to packet
     * @param packet packet
     * @param socket socket
     * @param dst destination address
     * @param sequenceNumber sequence number
     */
    void AddTlvHeaders(Ptr<Packet> packet,
                       Ptr<Socket> socket,
                       IpAddress dst,
                       uint32_t sequenceNumber);
    /** Send RREP
     * @param rreqHeader route request header
     * @param toOrigin routing table entry to originator
     * @param hopCount hop count
     */
    void SendReply(const RreqHeader<IpAddress>& rreqHeader,
                   const LocalRoute<IpAddress>& toOrigin,
                   uint8_t hopCount);
    /** Send RREP by intermediate node
     * @param toDst routing table entry to destination
     * @param toOrigin routing table entry to originator
     */
    void SendReplyByIntermediateNode(LocalRoute<IpAddress>& toDst, LocalRoute<IpAddress>& toOrigin);
    /** Schedule RREP_ACK check
     * @param toOrigin routing table entry to originator
     */
    void ScheduleRrepAckCheck(LocalRoute<IpAddress> toOrigin);
    /** Send RREP_ACK
     * @param neighbor neighbor address
     * @param rrepHeader route reply header
     */
    void SendReplyAck(IpAddress neighbor, RrepHeader<IpAddress> rrepHeader);
    /** Initiate RERR
     * @param nextHop next hop address
     */
    void SendRerrWhenBreaksLinkToNextHop(IpAddress nextHop);
    /** Forward RERR
     * @param packet packet
     * @param precursors list of addresses of the visited nodes
     */
    void SendRerrMessage(Ptr<Packet> packet, std::vector<IpAddress> precursors);
    /**
     * Send RERR message when no route to forward input packet. Unicast if there is reverse
     * route to originating node, broadcast otherwise. @param dst destination node IP address
     * @param dstSeqNo destination node sequence number
     * @param origin originating node IP address
     */
    void SendRerrWhenNoRouteToForward(IpAddress dst, uint16_t dstSeqNo, IpAddress origin);
    /** @} */

    /**
     * Send packet to destination socket
     * @param socket destination node socket
     * @param packet packet to send
     * @param destination destination node IP address
     */
    void SendTo(Ptr<Socket> socket, Ptr<Packet> packet, IpAddress destination);

    /// RREQ rate limit timer
    Timer m_rreqRateLimitTimer;
    /// Reset RREQ count and schedule RREQ rate limit timer with delay 1 sec.
    void RreqRateLimitTimerExpire();
    /// RREP rate limit timer
    Timer m_rrepRateLimitTimer;
    /// Reset RREP count and schedule RREP rate limit timer with delay 1 sec.
    void RrepRateLimitTimerExpire();
    /// RERR rate limit timer
    Timer m_rerrRateLimitTimer;
    /// Reset RERR count and schedule RERR rate limit timer with delay 1 sec.
    void RerrRateLimitTimerExpire();
    /// Map IP address + RREQ timer.
    std::map<IpAddress, Timer> m_addressReqTimer;
    /// Map IP address + RREP timer.
    std::map<IpAddress, Timer> m_addressRepTimer;
    /// Map IP address + New discovery timer after x attempts
    std::map<IpAddress, Timer> m_newDiscoveryTimer;
    /**
     * Handle route discovery process
     * @param dst the destination IP address
     */
    void RouteRequestTimerExpire(IpAddress dst);
    /**
     * Handle route discovery process
     * @param rreqHeader route request header
     * @param toOrigin routing table entry to originator
     * @param hopCount hop count
     */
    void RouteReplyTimerExpire(const RreqHeader<IpAddress>& rreqHeader,
                               const LocalRoute<IpAddress>& toOrigin,
                               uint8_t hopCount);
    /**
     * Mark link to neighbor node as unidirectional for blacklistTimeout
     *
     * @param neighbor the IP address of the neighbor node
     * @param blacklistTimeout the black list timeout time
     */
    void AckTimerExpire(IpAddress neighbor, Time blacklistTimeout);

    /// Provides uniform random variables.
    Ptr<UniformRandomVariable> m_uniformRandomVariable;
    /// Keep track of the last bcast time
    Time m_lastBcastTime;
    /// Set all links as bidirectional (avoid rrep_acks)
    bool m_bidirectionalLinks;
    /// Reset periodically
    bool m_resetPeriodically;
};

typedef Aodvv2RoutingProtocol<Ipv4RoutingProtocol> Ipv4Aodvv2RoutingProtocol;
typedef Aodvv2RoutingProtocol<Ipv6RoutingProtocol> Ipv6Aodvv2RoutingProtocol;

} // namespace aodvv2
} // namespace ns3

#endif /* AODVV2_ROUTING_PROTOCOL_H */
