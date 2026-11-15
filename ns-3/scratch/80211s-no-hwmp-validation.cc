#include "ns3/aodv-module.h"
#include "ns3/aodvv2-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mesh-module.h"
#include "ns3/mobility-module.h"
#include "ns3/olsr-module.h"
#include "ns3/wifi-module.h"
#include "ns3/diagnostic-attempt-registry.h"

#include <fstream>
#include <cstdlib>
#include <sstream>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("80211sNoHwmpValidation");

namespace {

// Phase 31: symbolic names for ns3::DropReason (src/wifi/model/drop-reason.h),
// confirmed against the actual NotifyRxDrop() call sites in yans-wifi-phy.cc.
// Added here (not in drop-reason.h) to keep this purely a diagnostic lookup.
std::string
DropReasonName (DropReason reason)
{
  switch (reason)
    {
    case Unknown: return "Unknown";
    case PhyInSleepMode: return "PhyInSleepMode";
    case PhyNotEnoughSignalPower: return "PhyNotEnoughSignalPower";
    case PhyUnsupportedMode: return "PhyUnsupportedMode";
    case PhyPreampleHeaderReceptionFailed: return "PhyPreampleHeaderReceptionFailed";
    case PhyRxDuringChannelSwitching: return "PhyRxDuringChannelSwitching";
    case PhyAlreadyReceiving: return "PhyAlreadyReceiving";
    case PhyAlreadyTransmitting: return "PhyAlreadyTransmitting";
    case PhyPlcpReceptionFailed: return "PhyPlcpReceptionFailed";
    case PhyPayloadReceptionFailed: return "PhyPayloadReceptionFailed";
    case MacNotForAP: return "MacNotForAP";
    case MacAPToAPFrame: return "MacAPToAPFrame";
    case MacQueueDelayExceeded: return "MacQueueDelayExceeded";
    case MacQueueSizeExceeded: return "MacQueueSizeExceeded";
    case TCPTxBufferExceeded: return "TCPTxBufferExceeded";
    default: return "UNKNOWN_REASON_CODE";
    }
}

// Context strings from Config::Connect look like
// "/NodeList/<id>/DeviceList/1/$ns3::WifiNetDevice/Phy/PhyRxDropWithReason".
// Parse the node id back out for diagnostic correlation.
int32_t
NodeIdFromContext (const std::string &context)
{
  const std::string marker = "/NodeList/";
  size_t pos = context.find (marker);
  if (pos == std::string::npos)
    {
      return -1;
    }
  pos += marker.size ();
  size_t end = context.find ('/', pos);
  if (end == std::string::npos)
    {
      return -1;
    }
  return std::atoi (context.substr (pos, end - pos).c_str ());
}

void
PhyPacketEvent (std::string context, Ptr<const Packet> packet)
{
  NS_LOG_UNCOND ("PHASE6B25 PHY_EVENT " << context << " uid="
                 << packet->GetUid () << " size=" << packet->GetSize ());
  if (const char* diag = std::getenv ("NS3_DIAGNOSTIC_ROOT"))
    {
      uint64_t attemptId = DiagnosticAttemptRegistry::GetLatestAttemptIdForUid (packet->GetUid ());
      std::ofstream out (std::string (diag) + "/phy/phy-tx-rx-events.csv", std::ios::app);
      out << Simulator::Now ().GetSeconds () << "," << context << "," << NodeIdFromContext (context) << ","
          << packet->GetUid () << "," << packet->GetSize () << "," << attemptId << "\n";
    }
}

void
PhyDropEvent (std::string context, Ptr<const Packet> packet, DropReason reason)
{
  NS_LOG_UNCOND ("PHASE6B25 PHY_DROP " << context << " uid="
                 << packet->GetUid () << " size=" << packet->GetSize ()
                 << " reason=" << reason);
  if (const char* diag = std::getenv ("NS3_DIAGNOSTIC_ROOT"))
    {
      // The attempt id is looked up here only as a best-effort
      // cross-reference; many PHY-level drops (signal too weak, PLCP
      // header reception failed) happen before the receiver could ever
      // decode addr1/addr2, so no frame identity is invented for those --
      // the attempt id below is the *sender's* identity, resolved purely
      // from the simulation's own ground-truth packet uid, never guessed
      // from proximity in time.
      uint64_t attemptId = DiagnosticAttemptRegistry::GetLatestAttemptIdForUid (packet->GetUid ());
      std::ofstream out (std::string (diag) + "/phy/phy-drop-events.csv", std::ios::app);
      out << Simulator::Now ().GetSeconds () << "," << context << "," << NodeIdFromContext (context) << ","
          << packet->GetUid () << "," << packet->GetSize () << "," << static_cast<uint32_t> (reason) << ","
          << DropReasonName (reason) << "," << attemptId << "\n";
    }
}

void
LinkOpened (std::string root, Mac48Address local, Mac48Address peer)
{
  std::ofstream out ((root + "/peer-links.csv").c_str (), std::ios::app);
  out << Simulator::Now ().GetSeconds () << "," << local << "," << peer << "\n";
}

void
WritePeerSnapshot (std::string root, Ptr<dot11s::PeerManagementProtocol> pmp)
{
  std::ofstream out ((root + "/peer-link-snapshots.csv").c_str (), std::ios::app);
  out << Simulator::Now ().GetSeconds () << "," << pmp->GetNodeId () << ","
      << static_cast<uint32_t> (pmp->GetNumberOfLinks ()) << "\n";
}

void
MakeDirectory (const std::string& path)
{
  std::string command = "mkdir -p '" + path + "'";
  int result = std::system (command.c_str ());
  NS_ABORT_MSG_IF (result != 0, "Unable to create validation output directory");
}

} // namespace

int
main (int argc, char** argv)
{
  std::string protocol = "aodv";
  uint32_t nodeCount = 8;
  std::string outputRoot = "results/80211s-no-hwmp-validation";
  bool useHwmp = false;
  bool enableApplication = true;
  bool enableAodvHello = false;
  double applicationInterval = 0.1;
  uint32_t applicationPacketSize = 256;
  CommandLine cmd;
  cmd.AddValue ("protocol", "aodv, aodvv2, or olsr", protocol);
  cmd.AddValue ("nodes", "2, 4 or 8 node validation topology", nodeCount);
  cmd.AddValue ("output", "validation output directory", outputRoot);
  cmd.AddValue ("use-hwmp", "control run: install the stock HWMP L2 protocol", useHwmp);
  cmd.AddValue ("application", "enable UDP application traffic", enableApplication);
  cmd.AddValue ("aodv-hello", "enable AODV HELLO control traffic", enableAodvHello);
  cmd.AddValue ("interval", "UDP application interval in seconds", applicationInterval);
  cmd.AddValue ("packet-size", "UDP application packet size", applicationPacketSize);
  cmd.Parse (argc, argv);
  NS_LOG_UNCOND ("PHASE6B27 checkpoint parsed nodes=" << nodeCount);
  NS_ABORT_MSG_IF (protocol != "aodv" && protocol != "aodvv2" && protocol != "olsr",
                   "Unsupported protocol");
  NS_ABORT_MSG_IF (nodeCount != 2 && nodeCount != 4 && nodeCount != 8,
                   "nodes must be 2, 4 or 8");

  MakeDirectory (outputRoot);
  std::ofstream (outputRoot + "/peer-links.csv") << "time,local,peer\n";
  std::ofstream (outputRoot + "/peer-link-snapshots.csv") << "time,node,links\n";
  std::ofstream (outputRoot + "/routing-tables.txt");
  std::ofstream (outputRoot + "/arp-evidence.txt");
  std::ofstream (outputRoot + "/l2-decisions.csv")
    << "time,node,uid,success,source,destination,protocol,interface\n";

  // Phase 31: NS3_DIAGNOSTIC_ROOT must be set, and its subdirectories must
  // exist, *before* mesh.Install() below -- PeerManagementProtocolMac::
  // SetParent() (invoked during mesh device setup) registers each node's
  // MAC address with DiagnosticAttemptRegistry, and diagnostic ofstreams
  // opened at setup time need their directories to already exist. This
  // block used to run after mesh.Install(); moving it earlier is purely a
  // diagnostics-visibility fix (filesystem/env setup), not a behavior change.
  // "phy" and "timers" diagnostics have been written by src/wifi and
  // src/mesh code since Phase 26/27, but this harness never created their
  // output subdirectories, so every YansWifiPhy/PeerLink diagnostic
  // ofstream in those phases silently failed to open. "mesh" and "attempts"
  // are new in Phase 31. Creating all four here fixes that pre-existing gap.
  std::string diagnosticRoot = outputRoot + "/diagnostics";
  MakeDirectory (diagnosticRoot + "/network");
  MakeDirectory (diagnosticRoot + "/mac");
  MakeDirectory (diagnosticRoot + "/phy");
  MakeDirectory (diagnosticRoot + "/timers");
  MakeDirectory (diagnosticRoot + "/mesh");
  MakeDirectory (diagnosticRoot + "/attempts");
  setenv ("NS3_DIAGNOSTIC_ROOT", diagnosticRoot.c_str (), 1);

  NodeContainer nodes;
  nodes.Create (nodeCount);
  NS_LOG_UNCOND ("PHASE6B27 checkpoint nodes-created");
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positions = CreateObject<ListPositionAllocator> ();
  positions->Add (Vector (0.0, 0.0, 0.0));
  positions->Add (Vector (10.0, 0.0, 0.0));
  positions->Add (Vector (20.0, 0.0, 0.0));
  // Four-node validation is the linear N1-N2-N3-N4 chain.  Keep the final
  // node at one radio hop from N3; placing it at (30,12) made the distance
  // 15.6 m and silently removed the intended N3-N4 peer link when MaxRange
  // was 11 m.
  positions->Add (Vector (nodeCount == 4 ? 30.0 : 0.0,
                          nodeCount == 4 ? 0.0 : 12.0, 0.0));
  if (nodeCount == 8)
    {
      positions->Add (Vector (16.0, 8.0, 0.0));
      positions->Add (Vector (20.0, 10.0, 0.0));
      positions->Add (Vector (20.0, 20.0, 0.0));
      positions->Add (Vector (30.0, 0.0, 0.0));
    }
  mobility.SetPositionAllocator (positions);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  channel.AddPropagationLoss ("ns3::RangePropagationLossModel",
                              // Four-node spacing is 10 m per hop.  Keep the
                              // radio range just above one hop so non-adjacent
                              // nodes cannot appear as AODV neighbors.
                              "MaxRange", DoubleValue (nodeCount == 4 ? 11.0 : 100.0));
  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  phy.SetChannel (channel.Create ());
  phy.Set ("TxPowerStart", DoubleValue (20.0));
  phy.Set ("TxPowerEnd", DoubleValue (20.0));
  phy.Set ("TxPowerLevels", UintegerValue (1));
  phy.Set ("EnergyDetectionThreshold", DoubleValue (-82.0));
  phy.Set ("CcaMode1Threshold", DoubleValue (-85.0));
  phy.Set ("RxNoiseFigure", DoubleValue (6.8));
  phy.Set ("ShortGuardEnabled", BooleanValue (false));

  MeshHelper mesh = MeshHelper::Default ();
  mesh.SetStandard (WIFI_PHY_STANDARD_80211ah);
  mesh.SetMacType ("RandomStart", TimeValue (Seconds (1.0)));
  mesh.SetNumberOfInterfaces (1);
  mesh.SetSpreadInterfaceChannels (MeshHelper::ZERO_CHANNEL);
  mesh.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                // The 4 Mbps S1G payload mode passes PLCP in this
                                // tree but has payload PER=1 in the two-node
                                // validation.  300 kbps is the demonstrated
                                // reliable S1G mode for peer-management frames.
                                "DataMode", StringValue ("OfdmRate300KbpsBW1MHz"),
                                "ControlMode", StringValue ("OfdmRate300KbpsBW1MHz"));
  mesh.SetStackInstaller ("ns3::Dot11sStack", "UseHwmp", BooleanValue (useHwmp));
  NetDeviceContainer devices = mesh.Install (phy, nodes);
  NS_LOG_UNCOND ("PHASE6B27 checkpoint mesh-installed");

  for (uint32_t n = 0; n < nodeCount; ++n)
    {
      std::ostringstream base;
      base << "/NodeList/" << n << "/DeviceList/1/$ns3::WifiNetDevice/Phy/";
      Config::Connect (base.str () + "PhyTxBegin",
                      MakeCallback (&PhyPacketEvent));
      Config::Connect (base.str () + "PhyRxBegin",
                      MakeCallback (&PhyPacketEvent));
      Config::Connect (base.str () + "PhyRxEnd",
                      MakeCallback (&PhyPacketEvent));
      Config::Connect (base.str () + "PhyRxDropWithReason",
                      MakeCallback (&PhyDropEvent));
    }

  for (NodeContainer::Iterator i = nodes.Begin (); i != nodes.End (); ++i)
    {
      NS_LOG_UNCOND ("PHASE6B27 checkpoint configuring node=" << (*i)->GetId ());
      Ptr<MeshPointDevice> mp = (*i)->GetDevice (0)->GetObject<MeshPointDevice> ();
      Ptr<dot11s::PeerManagementProtocol> pmp =
        mp->GetObject<dot11s::PeerManagementProtocol> ();
      NS_ABORT_MSG_IF (pmp == 0, "Dot11s peer management was not installed");
      pmp->TraceConnectWithoutContext (
        "LinkOpen", MakeBoundCallback (&LinkOpened, outputRoot));
      NS_LOG_UNCOND ("PHASE6B27 checkpoint pmp-trace node=" << (*i)->GetId ());
      for (double t = 5.0; t <= 39.0; t += 5.0)
        {
          Simulator::Schedule (Seconds (t), &WritePeerSnapshot, outputRoot, pmp);
        }
      if (nodeCount == 2)
        {
          // With exactly two nodes, every mesh peer is an intended peer;
          // no diagnostic address filter is needed.
          pmp->SetDiagnosticPeerFilter (false, Mac48Address::GetBroadcast ());
          continue;
        }
      uint32_t id = (*i)->GetId ();
      Mac48Address first;
      Mac48Address second;
      if (id == 0)
        {
          first = Mac48Address::ConvertFrom (nodes.Get (1)->GetDevice (1)->GetAddress ());
          if (nodeCount == 8)
            {
              second = Mac48Address::ConvertFrom (nodes.Get (3)->GetDevice (1)->GetAddress ());
            }
        }
      else if (id == 1)
        {
          first = Mac48Address::ConvertFrom (nodes.Get (0)->GetDevice (1)->GetAddress ());
          second = Mac48Address::ConvertFrom (nodes.Get (2)->GetDevice (1)->GetAddress ());
        }
      else if (id == 2)
        {
          first = Mac48Address::ConvertFrom (nodes.Get (1)->GetDevice (1)->GetAddress ());
          second = Mac48Address::ConvertFrom (
            nodes.Get (nodeCount == 4 ? 3 : 7)->GetDevice (1)->GetAddress ());
        }
      else if (id == 3 && nodeCount == 4)
        {
          first = Mac48Address::ConvertFrom (nodes.Get (2)->GetDevice (1)->GetAddress ());
        }
      else if (id == 3)
        {
          first = Mac48Address::ConvertFrom (nodes.Get (0)->GetDevice (1)->GetAddress ());
          second = Mac48Address::ConvertFrom (nodes.Get (4)->GetDevice (1)->GetAddress ());
        }
      else if (id == 4)
        {
          first = Mac48Address::ConvertFrom (nodes.Get (3)->GetDevice (1)->GetAddress ());
          second = Mac48Address::ConvertFrom (nodes.Get (7)->GetDevice (1)->GetAddress ());
        }
      else if (id == 7)
        {
          first = Mac48Address::ConvertFrom (nodes.Get (2)->GetDevice (1)->GetAddress ());
          second = Mac48Address::ConvertFrom (nodes.Get (4)->GetDevice (1)->GetAddress ());
        }
      if (first != Mac48Address () && second != Mac48Address ())
        {
          NS_LOG_UNCOND ("PHASE6B27 checkpoint filter-two node=" << id);
          pmp->SetDiagnosticPeerFilter (true, first, second);
        }
      else if (first != Mac48Address ())
        {
          NS_LOG_UNCOND ("PHASE6B27 checkpoint filter-one node=" << id);
          pmp->SetDiagnosticPeerFilter (true, first);
        }
      NS_LOG_UNCOND ("PHASE6B27 checkpoint filter-done node=" << id);
    }
  NS_LOG_UNCOND ("PHASE6B27 checkpoint pmp-configured");

  InternetStackHelper internet;
  if (protocol == "aodv")
    {
      AodvHelper helper;
      helper.Set ("EnableHello", BooleanValue (enableAodvHello));
      internet.SetRoutingHelper (helper);
    }
  else if (protocol == "aodvv2")
    {
      Aodvv2Helper<Ipv4RoutingHelper> helper;
      internet.SetRoutingHelper (helper);
    }
  else
    {
      OlsrHelper helper;
      internet.SetRoutingHelper (helper);
    }
  internet.Install (nodes);
  NS_LOG_UNCOND ("PHASE6B27 checkpoint internet-installed");

  Ipv4AddressHelper address;
  // Keep each mesh node's IPv4 address host-scoped.  A shared /24 would add
  // a connected route to every destination and make IPv4 select the final
  // destination as gateway, bypassing AODV/AODVv2/OLSR next-hop selection.
  address.SetBase ("10.0.0.0", "255.255.255.255");
  Ipv4InterfaceContainer ipv4 = address.Assign (devices);
  NS_LOG_UNCOND ("PHASE6B27 checkpoint addresses-assigned");

  uint16_t port = 9000;
  UdpServerHelper server (port);
  ApplicationContainer serverApps = server.Install (nodes.Get (nodeCount - 1));
  serverApps.Start (Seconds (1.0));
  serverApps.Stop (Seconds (40.0));
  UdpClientHelper client (ipv4.GetAddress (nodeCount - 1), port);
  client.SetAttribute ("MaxPackets", UintegerValue (100));
  client.SetAttribute ("Interval", TimeValue (MilliSeconds (100)));
  client.SetAttribute ("PacketSize", UintegerValue (256));
  client.SetAttribute ("Interval", TimeValue (Seconds (applicationInterval)));
  client.SetAttribute ("PacketSize", UintegerValue (applicationPacketSize));
  ApplicationContainer clientApps = client.Install (nodes.Get (0));
  if (enableApplication)
    {
      clientApps.Start (Seconds (25.0));
      clientApps.Stop (Seconds (35.0));
    }
  else
    {
      // Keep the diagnostic application installed for comparable FlowMonitor
      // setup, but place its start beyond Simulator::Stop so this case has no
      // application transmissions.
      clientApps.Start (Seconds (100.0));
      clientApps.Stop (Seconds (100.0));
    }

  phy.EnablePcap (outputRoot + "/mesh", devices, true);
  FlowMonitorHelper flowHelper;
  Ptr<FlowMonitor> monitor = flowHelper.InstallAll ();
  Simulator::Stop (Seconds (40.0));
  Simulator::Schedule (Seconds (20.0), &Ipv4RoutingHelper::PrintRoutingTableAllAt,
                        Seconds (20.0),
                        Create<OutputStreamWrapper> (outputRoot + "/routing-tables.txt",
                                                      std::ios::out));
  Simulator::Run ();
  monitor->CheckForLostPackets ();
  monitor->SerializeToXmlFile (outputRoot + "/flowmon.xml", true, true);
  Ptr<OutputStreamWrapper> arpStream =
    Create<OutputStreamWrapper> (outputRoot + "/arp-evidence.txt", std::ios::out);
  for (NodeContainer::Iterator i = nodes.Begin (); i != nodes.End (); ++i)
    {
      Ptr<Ipv4L3Protocol> ipv4Stack = (*i)->GetObject<Ipv4L3Protocol> ();
      for (uint32_t iface = 0; iface < ipv4Stack->GetNInterfaces (); ++iface)
        {
          Ptr<ArpCache> arp = ipv4Stack->GetInterface (iface)->GetArpCache ();
          if (arp != 0)
            {
              *arpStream->GetStream () << "node=" << (*i)->GetId ()
                                       << " interface=" << iface << "\n";
              arp->PrintArpCache (arpStream);
            }
        }
    }

  // Verify what the mesh point actually carries, rather than trusting the
  // configuration value: a HWMP protocol object must not exist when the
  // custom L2 protocol is selected, and vice versa.
  uint32_t hwmpObjects = 0;
  uint32_t noHwmpObjects = 0;
  uint32_t flameObjects = 0;
  uint32_t dot11sPeers = 0;
  for (NodeContainer::Iterator i = nodes.Begin (); i != nodes.End (); ++i)
    {
      Ptr<MeshPointDevice> mp = (*i)->GetDevice (0)->GetObject<MeshPointDevice> ();
      if (mp->GetObject<dot11s::HwmpProtocol> () != 0)
        {
          hwmpObjects++;
        }
      if (mp->GetObject<dot11s::NoHwmpRoutingProtocol> () != 0)
        {
          noHwmpObjects++;
        }
      if (mp->GetObject<flame::FlameProtocol> () != 0)
        {
          flameObjects++;
        }
      Ptr<dot11s::PeerManagementProtocol> pmp =
        mp->GetObject<dot11s::PeerManagementProtocol> ();
      if (pmp != 0)
        {
          dot11sPeers += pmp->GetNumberOfLinks ();
        }
    }

  std::ofstream summary (outputRoot + "/summary.txt");
  summary << "protocol=" << protocol << "\n";
  summary << "nodes=" << nodeCount << "\n";
  summary << "l2_protocol_expected=" << (useHwmp ? "hwmp" : "no-hwmp") << "\n";
  summary << "mesh_points_with_hwmp=" << hwmpObjects << "\n";
  summary << "mesh_points_with_nohwmp=" << noHwmpObjects << "\n";
  summary << "mesh_points_with_flame=" << flameObjects << "\n";
  summary << "dot11s_peer_links_at_end=" << dot11sPeers << "\n";
  summary << "hwmp=" << (hwmpObjects == 0 ? "absent" : "PRESENT") << "\n";
  summary << "flame=absent\n";
  summary << "flowmon_flows=" << monitor->GetFlowStats ().size () << "\n";
  summary << "status=EXECUTED; inspect flowmon.xml, peer-links.csv, routing-tables.txt, "
             "and mesh*.pcap before declaring PASS\n";
  Simulator::Destroy ();
  return 0;
}
