/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#ifndef DOT11S_NO_HWMP_PROTOCOL_H
#define DOT11S_NO_HWMP_PROTOCOL_H

#include "ns3/mesh-l2-routing-protocol.h"
#include "ns3/mesh-wifi-interface-mac-plugin.h"
#include "ns3/nstime.h"
#include <map>

namespace ns3 {
namespace dot11s {

/**
 * \brief Sets the IEEE 802.11 receiver address from the L2 route decision.
 *
 * This plugin is deliberately unicast-only for data frames. It does not
 * create a fallback broadcast route.
 */
class ReceiverAddressPlugin : public MeshWifiInterfaceMacPlugin
{
public:
  void SetParent (Ptr<MeshWifiInterfaceMac> parent) override;
  bool Receive (Ptr<Packet> packet, const WifiMacHeader& header) override;
  bool UpdateOutcomingFrame (Ptr<Packet> packet,
                             WifiMacHeader& header,
                             Mac48Address from,
                             Mac48Address to) override;
  void UpdateBeacon (MeshWifiBeacon& beacon) const override;
  int64_t AssignStreams (int64_t stream) override;
};

/**
 * \brief L2 adapter for routes selected by the IPv4/ARP stack.
 *
 * IPv4 routing selects the next-hop IP address and ARP supplies its MAC
 * address. The address passed to RequestRoute is therefore the exact
 * receiver for this wireless hop.
 */
class NoHwmpRoutingProtocol : public MeshL2RoutingProtocol
{
public:
  static TypeId GetTypeId ();

  bool Install (Ptr<MeshPointDevice> mp);
  bool RequestRoute (uint32_t sourceIface,
                     const Mac48Address source,
                     const Mac48Address destination,
                     Ptr<const Packet> packet,
                     uint16_t protocolType,
                     RouteReplyCallback routeReply) override;
  bool RemoveRoutingStuff (uint32_t fromIface,
                           const Mac48Address source,
                           const Mac48Address destination,
                           Ptr<Packet> packet,
                           uint16_t& protocolType) override;

private:
  // Phase 42: group-addressed (broadcast/multicast) relay duplicate
  // suppression. Without this, a node that hears its own already-relayed
  // frame echoed back by a neighbor (a normal occurrence: broadcast has
  // no unicast ACK/ownership, so every neighbor that hears a relay treats
  // it as a fresh incoming frame) re-relays it again, and the neighbor
  // does the same -- a two-node ping-pong that silently consumes the
  // entire MeshTtlTag hop budget without ever reaching a third node, even
  // one with an established direct peer link to the origin. Each node
  // therefore remembers packet UIDs (stable across Copy(), confirmed via
  // diagnostic trace) it has itself already relayed, and declines to
  // relay the same UID twice within a short window.
  std::map<uint64_t, Time> m_recentlyRelayedGroupPackets;
  static const Time kDuplicateSuppressWindow;
};

} // namespace dot11s
} // namespace ns3

#endif /* DOT11S_NO_HWMP_PROTOCOL_H */
