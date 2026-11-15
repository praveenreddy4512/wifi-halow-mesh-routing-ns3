/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "no-hwmp-protocol.h"

#include "ns3/log.h"
#include "ns3/mesh-point-device.h"
#include "ns3/mesh-wifi-interface-mac.h"
#include "ns3/mesh-ttl-tag.h"
#include "ns3/simulator.h"
#include "ns3/wifi-net-device.h"

namespace ns3 {
namespace dot11s {

NS_LOG_COMPONENT_DEFINE ("NoHwmpRoutingProtocol");
NS_OBJECT_ENSURE_REGISTERED (NoHwmpRoutingProtocol);

const Time NoHwmpRoutingProtocol::kDuplicateSuppressWindow = Seconds (2.0);

TypeId
NoHwmpRoutingProtocol::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::dot11s::NoHwmpRoutingProtocol")
    .SetParent<MeshL2RoutingProtocol> ()
    .SetGroupName ("Mesh")
    .AddConstructor<NoHwmpRoutingProtocol> ();
  return tid;
}

void
ReceiverAddressPlugin::SetParent (Ptr<MeshWifiInterfaceMac>)
{
}

bool
ReceiverAddressPlugin::Receive (Ptr<Packet>, const WifiMacHeader&)
{
  return true;
}

bool
ReceiverAddressPlugin::UpdateOutcomingFrame (Ptr<Packet>,
                                             WifiMacHeader& header,
                                             Mac48Address,
                                             Mac48Address to)
{
  if (header.IsData ())
    {
      if (to.IsGroup ())
        {
          header.SetAddr1 (to);
        }
      else if (to != Mac48Address ())
        {
          header.SetAddr1 (to);
        }
    }
  return true;
}

void
ReceiverAddressPlugin::UpdateBeacon (MeshWifiBeacon&) const
{
}

int64_t
ReceiverAddressPlugin::AssignStreams (int64_t)
{
  return 0;
}

bool
NoHwmpRoutingProtocol::Install (Ptr<MeshPointDevice> mp)
{
  SetMeshPoint (mp);
  const std::vector<Ptr<NetDevice> > interfaces = mp->GetInterfaces ();
  for (std::vector<Ptr<NetDevice> >::const_iterator i = interfaces.begin ();
       i != interfaces.end (); ++i)
    {
      Ptr<WifiNetDevice> wifi = (*i)->GetObject<WifiNetDevice> ();
      if (wifi == 0)
        {
          return false;
        }
      Ptr<MeshWifiInterfaceMac> mac =
        wifi->GetMac ()->GetObject<MeshWifiInterfaceMac> ();
      if (mac == 0)
        {
          return false;
        }
      mac->InstallPlugin (Create<ReceiverAddressPlugin> ());
    }
  mp->SetRoutingProtocol (this);
  return true;
}

bool
NoHwmpRoutingProtocol::RequestRoute (uint32_t sourceIface,
                                     const Mac48Address source,
                                     const Mac48Address destination,
                                     Ptr<const Packet> packet,
                                     uint16_t protocolType,
                                     RouteReplyCallback routeReply)
{
  if (packet == 0 || routeReply.IsNull () || destination == Mac48Address ())
    {
      NS_LOG_WARN ("Cannot route packet without a resolved receiver address");
      return false;
    }

  // HWMP/FLAME, which would normally bound how many hops a flooded frame
  // may travel, are disabled by design. Without any hop limit, every
  // group-addressed (broadcast/multicast) frame -- including routing
  // protocol HELLO messages whose own semantics assume single-hop
  // delivery -- would otherwise be relayed by every node that hears it,
  // indefinitely. This block restores a standard, protocol-agnostic hop
  // limit (the same concept as an IP TTL, or the Mesh TTL field in a real
  // 802.11s Mesh Control header) so broadcast still reaches the whole
  // network, just not forever. It never suppresses a frame outright on
  // its first MeshTtlTag::kInitialTtl hops, and it treats every protocol's
  // broadcast traffic identically.
  Ptr<Packet> routedPacket = packet->Copy ();
  if (destination.IsGroup ())
    {
      // Phase 42 fix: drop a group-addressed frame this node has already
      // relayed itself, rather than re-relaying an echo of its own prior
      // relay heard back from a neighbor. Packet UIDs are stable across
      // Copy() (confirmed via this project's own diagnostic trace), so
      // GetUid() is a safe, precise de-duplication key -- no risk of
      // conflating distinct packets. Without this, two nodes that both
      // hear each other's relay of the same frame ping-pong it back and
      // forth, consuming the entire MeshTtlTag hop budget before a third,
      // equally-reachable neighbor ever gets a copy.
      uint64_t uid = packet->GetUid ();
      for (std::map<uint64_t, Time>::iterator it = m_recentlyRelayedGroupPackets.begin ();
           it != m_recentlyRelayedGroupPackets.end (); )
        {
          if (Simulator::Now () - it->second > kDuplicateSuppressWindow)
            {
              it = m_recentlyRelayedGroupPackets.erase (it);
            }
          else
            {
              ++it;
            }
        }
      if (m_recentlyRelayedGroupPackets.find (uid) != m_recentlyRelayedGroupPackets.end ())
        {
          NS_LOG_DEBUG ("Dropping group-addressed frame: already relayed (duplicate suppression)");
          return false;
        }
      m_recentlyRelayedGroupPackets[uid] = Simulator::Now ();

      MeshTtlTag ttlTag;
      if (routedPacket->PeekPacketTag (ttlTag))
        {
          if (ttlTag.GetTtl () == 0)
            {
              NS_LOG_DEBUG ("Dropping group-addressed frame: mesh TTL expired");
              return false;
            }
          ttlTag.SetTtl (ttlTag.GetTtl () - 1);
          routedPacket->ReplacePacketTag (ttlTag);
        }
      else
        {
          ttlTag.SetTtl (MeshTtlTag::kInitialTtl - 1);
          routedPacket->AddPacketTag (ttlTag);
        }
    }

  const std::vector<Ptr<NetDevice> > interfaces = GetMeshPoint ()->GetInterfaces ();
  if (interfaces.empty ())
    {
      return false;
    }
  // MeshPointDevice passes its own ifIndex for locally generated packets;
  // only an underlying interface index is valid for DoSend/GetInterface.
  uint32_t outIface = interfaces.front ()->GetIfIndex ();
  for (std::vector<Ptr<NetDevice> >::const_iterator i = interfaces.begin ();
       i != interfaces.end (); ++i)
    {
      if ((*i)->GetIfIndex () == sourceIface)
        {
          outIface = sourceIface;
          break;
        }
    }
  routeReply (true, routedPacket, source, destination, protocolType, outIface);
  return true;
}

bool
NoHwmpRoutingProtocol::RemoveRoutingStuff (uint32_t,
                                           const Mac48Address,
                                           const Mac48Address,
                                           Ptr<Packet>,
                                           uint16_t&)
{
  return true;
}

} // namespace dot11s
} // namespace ns3
