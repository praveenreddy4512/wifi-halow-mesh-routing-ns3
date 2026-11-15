/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Phase 31 instrumentation-only transmission-attempt identity side-table.
 *
 * This class carries NO protocol state. It exists purely so that diagnostic
 * log lines written from EdcaTxopN, MacLow, YansWifiPhy and the dot11s
 * peer-management classes can be correlated into a single per-attempt
 * lifecycle, without mutating any Packet content or changing any NS-3
 * method signature.
 *
 * Every public method is a no-op (returns a sentinel value) unless the
 * NS3_DIAGNOSTIC_ROOT environment variable is set, matching the convention
 * already used by the diagnostic code added in earlier phases. This means
 * the registry can never influence simulated behavior, event ordering or
 * RNG consumption in a run where diagnostics are disabled.
 *
 * See PHASE31_TRANSMISSION_ATTEMPT_INSTRUMENTATION_REPORT.md for the full
 * design rationale (why a side-table instead of PacketTagList, how attempt
 * IDs are minted/propagated/finalized, etc).
 */
#ifndef DIAGNOSTIC_ATTEMPT_REGISTRY_H
#define DIAGNOSTIC_ATTEMPT_REGISTRY_H

#include <cstdint>
#include <string>
#include "ns3/mac48-address.h"

namespace ns3 {

class DiagnosticAttemptRegistry
{
public:
  /// True iff NS3_DIAGNOSTIC_ROOT is set. Cached after first call.
  static bool IsEnabled (void);

  /// Root diagnostics directory (NS3_DIAGNOSTIC_ROOT), or "" if disabled.
  static std::string Root (void);

  /// Record that a given MAC address belongs to a given node id. Called once
  /// per interface at setup time (dot11s peer-management-protocol-mac.cc),
  /// so that EdcaTxopN/MacLow/YansWifiPhy -- which only know their own MAC
  /// address, not their owning ns3::Node -- can resolve a node id for
  /// diagnostic output.
  static void RegisterNodeAddress (Mac48Address addr, uint32_t nodeId);

  /// Resolve a previously-registered node id. Returns -1 if unknown.
  static int32_t ResolveNodeId (Mac48Address addr);

  /// Mint a new MAC-transmission-attempt identity for packetUid on lowKey
  /// (the originating MacLow instance, used only as an opaque map key -- its
  /// address is never dereferenced by the registry). attemptNumber is
  /// 0 for the first attempt at this packetUid, 1, 2, ... for each
  /// subsequent retry of the *same* packet. Returns the new attempt id.
  static uint64_t BeginAttempt (const void *lowKey, uint64_t packetUid,
                                 uint32_t nodeId, const std::string &frameType,
                                 Mac48Address addr1, Mac48Address addr2);

  /// Fetch the attempt most recently begun on lowKey (the sender-side
  /// "currently in flight" attempt). Returns 0 if none / disabled.
  static uint64_t GetCurrentAttemptId (const void *lowKey);

  /// Fetch the most recent attempt minted for a given original packet uid,
  /// from any node (uids are simulation-global). Returns 0 if none.
  static uint64_t GetLatestAttemptIdForUid (uint64_t packetUid);

  /// Record the PHY transmission interval for an attempt.
  static void SetPhyTx (uint64_t attemptId, double txStartSeconds, double txEndSeconds);

  /// Stash the uid of a data/mgmt frame a receiver's MacLow has just decoded
  /// and is about to ACK, so SendAckAfterData (which only receives a
  /// Mac48Address, not a packet) can look up which attempt it is ACKing.
  static void NotePendingAck (const void *lowKey, uint64_t dataPacketUid, bool ackRequired);
  static uint64_t TakePendingAckUid (const void *lowKey);

  /// Record that an attempt's ACK was actually generated/transmitted.
  static void SetAckGenerated (uint64_t attemptId, double timeSeconds, Mac48Address ackDestination);

  /// Finalize an attempt with its terminal outcome, as observed at the
  /// sender. outcome is a short label, e.g. "ACK_RECEIVED", "ACK_TIMEOUT",
  /// "NO_ACK_EXPECTED_OK".
  static void FinalizeAttempt (uint64_t attemptId, const std::string &outcome, double timeSeconds);
  static void FinalizeAttemptForLow (const void *lowKey, const std::string &outcome, double timeSeconds);

  /// Record a derived-packet relationship (fragmentation / MSDU aggregation)
  /// if that code path is ever exercised. relation is "FRAGMENT" or
  /// "AGGREGATE".
  static void RecordDerivedPacket (uint64_t childUid, uint64_t parentUid, const std::string &relation);

  /// Write every attempt record collected so far to
  /// <root>/attempts/attempt-lifecycle.csv. Safe to call multiple times
  /// (e.g. hooked to Simulator::ScheduleDestroy) -- always (re)writes the
  /// full file from current in-memory state.
  static void Flush (void);
};

} // namespace ns3

#endif /* DIAGNOSTIC_ATTEMPT_REGISTRY_H */
