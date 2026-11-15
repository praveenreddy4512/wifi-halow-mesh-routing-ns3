/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include "diagnostic-attempt-registry.h"
#include "ns3/simulator.h"
#include <cstdlib>
#include <sys/stat.h>
#include <map>
#include <fstream>
#include <sstream>

namespace ns3 {

namespace {

struct AttemptRecord
{
  uint64_t attemptId;
  uint64_t packetUid;
  uint32_t attemptNumber;
  int32_t nodeId;
  std::string frameType;
  std::string addr1;
  std::string addr2;
  double mintTime;
  double phyTxStart;
  double phyTxEnd;
  bool phyTxRecorded;
  bool ackGenerated;
  double ackGeneratedTime;
  std::string ackDestination;
  std::string outcome;
  double outcomeTime;
  bool finalized;
};

struct PendingAck
{
  uint64_t dataPacketUid;
  bool ackRequired;
};

// All state below is purely diagnostic (never read by any protocol logic)
// and only ever touched when NS3_DIAGNOSTIC_ROOT is set, see IsEnabled().
uint64_t g_nextAttemptId = 1;
std::map<uint64_t, uint32_t> g_attemptCountByUid;          // packetUid -> attempts so far
std::map<uint64_t, uint64_t> g_latestAttemptIdByUid;       // packetUid -> most recent attemptId
std::map<const void *, uint64_t> g_currentAttemptByLow;    // MacLow* -> in-flight attemptId
std::map<const void *, PendingAck> g_pendingAckByLow;      // receiver MacLow* -> uid awaiting ACK tx
std::map<uint64_t, AttemptRecord> g_attempts;              // attemptId -> record
std::map<std::string, uint32_t> g_nodeIdByAddress;         // Mac48Address.ToString() -> nodeId
bool g_destroyHookInstalled = false;

bool
EnabledCached (void)
{
  // Deliberately not cached: PeerManagementProtocolMac::SetParent() (which
  // registers node/address mappings) runs during device/mesh setup, before
  // the harness has necessarily called setenv("NS3_DIAGNOSTIC_ROOT", ...).
  // A one-shot cache taken on that first, pre-setenv call would permanently
  // lock this to "disabled" for the whole process. A plain getenv() on
  // every call matches the convention already used elsewhere in this
  // codebase's diagnostic code (e.g. mesh-wifi-interface-mac.cc,
  // edca-txop-n.cc) and costs nothing that affects simulated behavior.
  return std::getenv ("NS3_DIAGNOSTIC_ROOT") != 0;
}

std::string
RootCached (void)
{
  const char *root = std::getenv ("NS3_DIAGNOSTIC_ROOT");
  return root ? std::string (root) : std::string ();
}

void
EnsureDir (const std::string &path)
{
  ::mkdir (path.c_str (), 0777);
}

std::string
AddrToString (Mac48Address a)
{
  std::ostringstream oss;
  oss << a;
  return oss.str ();
}

} // anonymous namespace

bool
DiagnosticAttemptRegistry::IsEnabled (void)
{
  return EnabledCached ();
}

std::string
DiagnosticAttemptRegistry::Root (void)
{
  if (!EnabledCached ())
    {
      return std::string ();
    }
  return RootCached ();
}

void
DiagnosticAttemptRegistry::RegisterNodeAddress (Mac48Address addr, uint32_t nodeId)
{
  if (!EnabledCached ())
    {
      return;
    }
  g_nodeIdByAddress[AddrToString (addr)] = nodeId;
}

int32_t
DiagnosticAttemptRegistry::ResolveNodeId (Mac48Address addr)
{
  if (!EnabledCached ())
    {
      return -1;
    }
  std::map<std::string, uint32_t>::const_iterator it = g_nodeIdByAddress.find (AddrToString (addr));
  if (it == g_nodeIdByAddress.end ())
    {
      return -1;
    }
  return static_cast<int32_t> (it->second);
}

uint64_t
DiagnosticAttemptRegistry::BeginAttempt (const void *lowKey, uint64_t packetUid,
                                          uint32_t nodeId, const std::string &frameType,
                                          Mac48Address addr1, Mac48Address addr2)
{
  if (!EnabledCached ())
    {
      return 0;
    }
  if (!g_destroyHookInstalled)
    {
      g_destroyHookInstalled = true;
      Simulator::ScheduleDestroy (&DiagnosticAttemptRegistry::Flush);
    }
  uint32_t attemptNumber = g_attemptCountByUid[packetUid]++;
  uint64_t attemptId = g_nextAttemptId++;

  AttemptRecord rec;
  rec.attemptId = attemptId;
  rec.packetUid = packetUid;
  rec.attemptNumber = attemptNumber;
  rec.nodeId = static_cast<int32_t> (nodeId);
  rec.frameType = frameType;
  rec.addr1 = AddrToString (addr1);
  rec.addr2 = AddrToString (addr2);
  rec.mintTime = Simulator::Now ().GetSeconds ();
  rec.phyTxStart = -1.0;
  rec.phyTxEnd = -1.0;
  rec.phyTxRecorded = false;
  rec.ackGenerated = false;
  rec.ackGeneratedTime = -1.0;
  rec.ackDestination = "";
  rec.outcome = "IN_PROGRESS";
  rec.outcomeTime = -1.0;
  rec.finalized = false;
  g_attempts[attemptId] = rec;

  g_latestAttemptIdByUid[packetUid] = attemptId;
  g_currentAttemptByLow[lowKey] = attemptId;
  return attemptId;
}

uint64_t
DiagnosticAttemptRegistry::GetCurrentAttemptId (const void *lowKey)
{
  if (!EnabledCached ())
    {
      return 0;
    }
  std::map<const void *, uint64_t>::const_iterator it = g_currentAttemptByLow.find (lowKey);
  if (it == g_currentAttemptByLow.end ())
    {
      return 0;
    }
  return it->second;
}

uint64_t
DiagnosticAttemptRegistry::GetLatestAttemptIdForUid (uint64_t packetUid)
{
  if (!EnabledCached ())
    {
      return 0;
    }
  std::map<uint64_t, uint64_t>::const_iterator it = g_latestAttemptIdByUid.find (packetUid);
  if (it == g_latestAttemptIdByUid.end ())
    {
      return 0;
    }
  return it->second;
}

void
DiagnosticAttemptRegistry::SetPhyTx (uint64_t attemptId, double txStartSeconds, double txEndSeconds)
{
  if (!EnabledCached () || attemptId == 0)
    {
      return;
    }
  std::map<uint64_t, AttemptRecord>::iterator it = g_attempts.find (attemptId);
  if (it == g_attempts.end ())
    {
      return;
    }
  it->second.phyTxStart = txStartSeconds;
  it->second.phyTxEnd = txEndSeconds;
  it->second.phyTxRecorded = true;
}

void
DiagnosticAttemptRegistry::NotePendingAck (const void *lowKey, uint64_t dataPacketUid, bool ackRequired)
{
  if (!EnabledCached ())
    {
      return;
    }
  PendingAck pa;
  pa.dataPacketUid = dataPacketUid;
  pa.ackRequired = ackRequired;
  g_pendingAckByLow[lowKey] = pa;
}

uint64_t
DiagnosticAttemptRegistry::TakePendingAckUid (const void *lowKey)
{
  if (!EnabledCached ())
    {
      return 0;
    }
  std::map<const void *, PendingAck>::iterator it = g_pendingAckByLow.find (lowKey);
  if (it == g_pendingAckByLow.end ())
    {
      return 0;
    }
  uint64_t uid = it->second.dataPacketUid;
  g_pendingAckByLow.erase (it);
  return uid;
}

void
DiagnosticAttemptRegistry::SetAckGenerated (uint64_t attemptId, double timeSeconds, Mac48Address ackDestination)
{
  if (!EnabledCached () || attemptId == 0)
    {
      return;
    }
  std::map<uint64_t, AttemptRecord>::iterator it = g_attempts.find (attemptId);
  if (it == g_attempts.end ())
    {
      return;
    }
  it->second.ackGenerated = true;
  it->second.ackGeneratedTime = timeSeconds;
  it->second.ackDestination = AddrToString (ackDestination);
}

void
DiagnosticAttemptRegistry::FinalizeAttempt (uint64_t attemptId, const std::string &outcome, double timeSeconds)
{
  if (!EnabledCached () || attemptId == 0)
    {
      return;
    }
  std::map<uint64_t, AttemptRecord>::iterator it = g_attempts.find (attemptId);
  if (it == g_attempts.end ())
    {
      return;
    }
  it->second.outcome = outcome;
  it->second.outcomeTime = timeSeconds;
  it->second.finalized = true;
}

void
DiagnosticAttemptRegistry::FinalizeAttemptForLow (const void *lowKey, const std::string &outcome, double timeSeconds)
{
  if (!EnabledCached ())
    {
      return;
    }
  FinalizeAttempt (GetCurrentAttemptId (lowKey), outcome, timeSeconds);
}

void
DiagnosticAttemptRegistry::RecordDerivedPacket (uint64_t childUid, uint64_t parentUid, const std::string &relation)
{
  if (!EnabledCached ())
    {
      return;
    }
  EnsureDir (RootCached () + "/attempts");
  std::ofstream out (RootCached () + "/attempts/derived-packets.csv", std::ios::app);
  out << Simulator::Now ().GetSeconds () << "," << childUid << "," << parentUid << "," << relation << "\n";
}

void
DiagnosticAttemptRegistry::Flush (void)
{
  if (!EnabledCached ())
    {
      return;
    }
  std::string root = RootCached ();
  EnsureDir (root);
  EnsureDir (root + "/attempts");
  std::ofstream out (root + "/attempts/attempt-lifecycle.csv", std::ios::trunc);
  out << "attempt_id,packet_uid,attempt_number,node_id,frame_type,addr1,addr2,"
      << "mint_time,phy_tx_start,phy_tx_end,ack_generated,ack_generated_time,ack_destination,"
      << "outcome,outcome_time,finalized\n";
  for (std::map<uint64_t, AttemptRecord>::const_iterator it = g_attempts.begin ();
       it != g_attempts.end (); ++it)
    {
      const AttemptRecord &r = it->second;
      out << r.attemptId << "," << r.packetUid << "," << r.attemptNumber << "," << r.nodeId << ","
          << r.frameType << "," << r.addr1 << "," << r.addr2 << ","
          << r.mintTime << "," << r.phyTxStart << "," << r.phyTxEnd << ","
          << (r.ackGenerated ? 1 : 0) << "," << r.ackGeneratedTime << "," << r.ackDestination << ","
          << r.outcome << "," << r.outcomeTime << "," << (r.finalized ? 1 : 0) << "\n";
    }
}

} // namespace ns3
