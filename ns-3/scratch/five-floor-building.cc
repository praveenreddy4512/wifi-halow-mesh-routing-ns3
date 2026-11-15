/* Five-floor research scenario with explicit floor-plan geometry and connectivity validation. */
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mesh-module.h"
#include "ns3/peer-management-protocol.h"
#include "ns3/buildings-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/aodv-module.h"
#include "ns3/aodvv2-module.h"
#include "ns3/olsr-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/propagation-module.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <cstdlib>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("FiveFloorBuildingResearch");

namespace
{
static std::string DiagnosticPath (const std::string& leaf)
{
  const char* root = std::getenv ("NS3_DIAGNOSTIC_ROOT");
  return std::string (root ? root : "results/diagnostics") + "/" + leaf;
}
static std::map<uint64_t, uint64_t> g_beaconCorrelation;
static uint64_t g_nextBeaconCorrelation = 1;

static void
WriteCorrelatedEvent (uint64_t id, uint32_t receiverNode, const std::string& event,
                      const std::string& phyResult, const std::string& macResult,
                      const std::string& pmResult, const std::string& reason)
{
  std::ofstream out (DiagnosticPath ("correlation/beacon-event-timeline.csv").c_str (), std::ios::app);
  out << id << ",unknown,unknown," << receiverNode << ",unknown," << event << ","
      << Simulator::Now ().GetSeconds () << "," << phyResult << "," << macResult
      << "," << pmResult << "," << reason << "\n";
}

static uint32_t
ContextNodeId (const std::string& context)
{
  std::string key = "/NodeList/";
  std::size_t p = context.find (key);
  if (p == std::string::npos) return 0;
  p += key.size ();
  return static_cast<uint32_t> (std::stoul (context.substr (p)));
}

static void
BeaconPhyRx (std::string context, Ptr<const Packet> packet, uint16_t frequency,
             uint16_t channel, uint32_t rate, bool shortPreamble,
             WifiTxVector txVector, double signalDbm, double noiseDbm)
{
  WriteCorrelatedEvent (packet->GetUid (), ContextNodeId (context), "PHY_RX_OK", "success", "UNKNOWN", "UNKNOWN", "");
  std::ofstream out (DiagnosticPath ("phy/phy-reception.csv").c_str (), std::ios::app);
  out << Simulator::Now ().GetSeconds () << "," << ContextNodeId (context) << ",rx-ok,"
      << packet->GetSize () << "," << frequency << "," << channel << "," << rate << ","
      << signalDbm << "," << noiseDbm << "," << (signalDbm - noiseDbm) << "\n";
}

static void
BeaconPhyRxDrop (std::string context, Ptr<const Packet> packet)
{
  WriteCorrelatedEvent (packet->GetUid (), ContextNodeId (context), "PHY_RX_DROP", "drop", "UNKNOWN", "UNKNOWN", "PHY_DROP");
  std::ofstream out (DiagnosticPath ("phy/phy-reception.csv").c_str (), std::ios::app);
  out << Simulator::Now ().GetSeconds () << "," << ContextNodeId (context) << ",rx-drop,"
      << packet->GetSize () << ",,,,,,,\n";
}

static const char* DropReasonName (DropReason reason)
{
  switch (reason)
    {
    case PhyInSleepMode: return "PhyInSleepMode";
    case PhyNotEnoughSignalPower: return "PhyNotEnoughSignalPower";
    case PhyUnsupportedMode: return "PhyUnsupportedMode";
    case PhyPreampleHeaderReceptionFailed: return "PhyPreampleHeaderReceptionFailed";
    case PhyRxDuringChannelSwitching: return "PhyRxDuringChannelSwitching";
    case PhyAlreadyReceiving: return "PhyAlreadyReceiving";
    case PhyAlreadyTransmitting: return "PhyAlreadyTransmitting";
    case PhyPlcpReceptionFailed: return "PhyPlcpReceptionFailed";
    case PhyPayloadReceptionFailed: return "PhyPayloadReceptionFailed";
    default: return "Unknown";
    }
}

static void
BeaconPhyRxDropReason (std::string context, Ptr<const Packet> packet, DropReason reason)
{
  std::string path = DiagnosticPath ("phy/phy-drop-reasons.csv");
  std::ofstream out (path.c_str (), std::ios::app);
  out << Simulator::Now ().GetSeconds () << "," << packet->GetUid () << ","
      << ContextNodeId (context) << "," << DropReasonName (reason) << "\n";
  WriteCorrelatedEvent (packet->GetUid (), ContextNodeId (context), "PHY_RX_DROP_REASON",
                        "drop", "UNKNOWN", "UNKNOWN", DropReasonName (reason));
}

static void
BeaconPhyTx (std::string context, Ptr<const Packet> packet, uint16_t frequency,
             uint16_t channel, uint32_t rate, bool shortPreamble,
             WifiTxVector txVector)
{
  WriteCorrelatedEvent (packet->GetUid (), ContextNodeId (context), "PHY_TX", "transmitted", "UNKNOWN", "UNKNOWN", "");
  std::ofstream out (DiagnosticPath ("phy/phy-reception.csv").c_str (), std::ios::app);
  out << Simulator::Now ().GetSeconds () << "," << ContextNodeId (context) << ",tx-monitor,"
      << packet->GetSize () << "," << frequency << "," << channel << "," << rate << "\n";
}

struct Rect
{
  std::string id;
  double x1;
  double y1;
  double x2;
  double y2;
  std::string type;
};

struct WallSegment
{
  std::string id;
  double x1;
  double y1;
  double x2;
  double y2;
  double lossDb;
  std::string material;
};

struct GeoPoint
{
  double x;
  double y;
  double z;
};

struct PropagationParams
{
  double frequencyMHz;
  double brickLossDb;
  double concreteLossDb;
  double partitionLossDb;
  double concreteObstacleLossDb;
  double elevatorLossDb;
  double floorLossDb;
  double staircaseFloorLossDb;
  double atriumFloorLossDb;
  double indoorBaselineLossDb;
  double pathExponent;
  double txPowerDbm;
  double rxSensitivityDbm;
  double noiseFloorDbm;
};

struct LinkQuality
{
  double distance;
  double pathLossDb;
  double rssiDbm;
  double snrDb;
  bool connected;
};

static const double kBuildingLength = 60.0;
static const double kBuildingWidth = 40.0;
static const double kFloorHeight = 3.5;
static const uint32_t kTotalNodes = 20;
static const uint16_t kUdpPort = 4000;
static const double kSimulationTime = 100.0;
static const uint32_t kPacketSize = 512;
static const double kPacketInterval = 0.04096; // 100 Kbps for 512-byte packets.

static const double kAllNodeLocations[kTotalNodes][3] = {
  {3,35,0},{21,35,0},{45,35,0},{21,5,0},
  {9,35,3.5},{33,35,3.5},{15,5,3.5},{39,5,3.5},
  {15,35,7},{39,35,7},{21,5,7},{45,5,7},
  {3,35,10.5},{27,35,10.5},{3,5,10.5},{33,5,10.5},
  {9,35,14},{33,35,14},{15,5,14},{45,5,14}
};

// Controlled Phase 6A topology.  N1--N2--N3--N8 and N1--N4--N5--N8 are
// spatially independent candidate paths; N6--N7 form a branch from N2.
static const double kEightNodeTopology[8][3] = {
  {3, 35, 0}, {9, 35, 3.5}, {15, 35, 7}, {3, 35, 3.5},
  {15, 35, 3.5}, {9, 25, 3.5}, {9, 15, 3.5}, {21, 35, 10.5}
};

static void
EnsureDir (const std::string &path)
{
  if (path.empty ())
    {
      return;
    }
  std::string partial;
  for (std::string::const_iterator it = path.begin (); it != path.end (); ++it)
    {
      partial.push_back (*it);
      if (*it == '/')
        {
          if (!partial.empty ())
            {
              mkdir (partial.c_str (), 0775);
            }
        }
    }
  mkdir (path.c_str (), 0775);
}

static std::vector<Rect>
BuildRoomLayout ()
{
  std::vector<Rect> rooms;
  // Row 1 (top): R1..R6
  rooms.push_back ((Rect){"R1", 0, 26, 10, 40, "room"});
  rooms.push_back ((Rect){"R2", 10, 26, 20, 40, "room"});
  rooms.push_back ((Rect){"R3", 20, 26, 30, 40, "room"});
  rooms.push_back ((Rect){"R4", 30, 26, 40, 40, "room"});
  rooms.push_back ((Rect){"R5", 40, 26, 50, 40, "room"});
  rooms.push_back ((Rect){"R6", 50, 26, 60, 40, "room"});
  // Row 2: R7, R8, ATRIUM, R9, R10
  rooms.push_back ((Rect){"R7", 0, 14, 10, 26, "room"});
  rooms.push_back ((Rect){"R8", 10, 14, 20, 26, "room"});
  rooms.push_back ((Rect){"R9", 40, 14, 50, 26, "room"});
  rooms.push_back ((Rect){"R10", 50, 14, 60, 26, "room"});
  // Row 3 (bottom): R11..R15
  rooms.push_back ((Rect){"R11", 0, 0, 10, 14, "room"});
  rooms.push_back ((Rect){"R12", 10, 0, 20, 14, "room"});
  rooms.push_back ((Rect){"R13", 20, 0, 30, 14, "room"});
  rooms.push_back ((Rect){"R14", 30, 0, 40, 14, "room"});
  rooms.push_back ((Rect){"R15", 40, 0, 50, 14, "room"});
  return rooms;
}

static std::vector<Rect>
BuildStaircases ()
{
  std::vector<Rect> stairs;
  stairs.push_back ((Rect){"S1", 0, 30, 5, 40, "staircase"});
  stairs.push_back ((Rect){"S2", 15, 30, 20, 40, "staircase"});
  stairs.push_back ((Rect){"S3", 45, 30, 50, 40, "staircase"});
  stairs.push_back ((Rect){"S4", 55, 30, 60, 40, "staircase"});
  return stairs;
}

static std::vector<Rect>
BuildCorridors ()
{
  std::vector<Rect> corridors;
  corridors.push_back ((Rect){"MainCorridor", 0, 18, 60, 22, "corridor"});
  corridors.push_back ((Rect){"CrossWest", 18, 0, 22, 40, "corridor"});
  corridors.push_back ((Rect){"CrossEast", 38, 0, 42, 40, "corridor"});
  return corridors;
}

static Rect
BuildAtrium ()
{
  return (Rect){"Atrium", 20, 14, 40, 26, "atrium"};
}

static std::vector<Rect>
BuildElevators ()
{
  std::vector<Rect> shafts;
  shafts.push_back ((Rect){"E1", 27, 0, 30, 6, "elevator"});
  shafts.push_back ((Rect){"E2", 30, 34, 33, 40, "elevator"});
  return shafts;
}

static std::vector<Rect>
BuildConcreteObstacles ()
{
  std::vector<Rect> obstacles;
  obstacles.push_back ((Rect){"C1", 24, 10, 26, 14, "concrete"});
  obstacles.push_back ((Rect){"C2", 34, 26, 36, 30, "concrete"});
  obstacles.push_back ((Rect){"C3", 52, 18, 56, 22, "concrete"});
  return obstacles;
}

static std::vector<WallSegment>
BuildInternalWalls (const PropagationParams &params)
{
  std::vector<WallSegment> walls;
  // External concrete shell.
  walls.push_back ((WallSegment){"W-ext-north", 0, 40, 60, 40, params.concreteLossDb, "concrete"});
  walls.push_back ((WallSegment){"W-ext-south", 0, 0, 60, 0, params.concreteLossDb, "concrete"});
  walls.push_back ((WallSegment){"W-ext-west", 0, 0, 0, 40, params.concreteLossDb, "concrete"});
  walls.push_back ((WallSegment){"W-ext-east", 60, 0, 60, 40, params.concreteLossDb, "concrete"});

  // Horizontal room separators.
  walls.push_back ((WallSegment){"W-row-top", 0, 26, 20, 26, params.partitionLossDb, "partition"});
  walls.push_back ((WallSegment){"W-row-top-right", 40, 26, 60, 26, params.partitionLossDb, "partition"});
  walls.push_back ((WallSegment){"W-row-bottom", 0, 14, 20, 14, params.partitionLossDb, "partition"});
  walls.push_back ((WallSegment){"W-row-bottom-center", 20, 14, 50, 14, params.partitionLossDb, "partition"});

  // Vertical partitions in top row.
  walls.push_back ((WallSegment){"W-x10-top", 10, 26, 10, 40, params.brickLossDb, "brick"});
  walls.push_back ((WallSegment){"W-x20-top", 20, 26, 20, 40, params.brickLossDb, "brick"});
  walls.push_back ((WallSegment){"W-x30-top", 30, 26, 30, 40, params.brickLossDb, "brick"});
  walls.push_back ((WallSegment){"W-x40-top", 40, 26, 40, 40, params.brickLossDb, "brick"});
  walls.push_back ((WallSegment){"W-x50-top", 50, 26, 50, 40, params.brickLossDb, "brick"});

  // Vertical partitions in middle row (atrium edges stay open).
  walls.push_back ((WallSegment){"W-x10-mid", 10, 14, 10, 26, params.brickLossDb, "brick"});
  walls.push_back ((WallSegment){"W-x20-mid", 20, 14, 20, 26, params.partitionLossDb, "partition"});
  walls.push_back ((WallSegment){"W-x40-mid", 40, 14, 40, 26, params.partitionLossDb, "partition"});
  walls.push_back ((WallSegment){"W-x50-mid", 50, 14, 50, 26, params.brickLossDb, "brick"});

  // Vertical partitions in bottom row.
  walls.push_back ((WallSegment){"W-x10-bottom", 10, 0, 10, 14, params.brickLossDb, "brick"});
  walls.push_back ((WallSegment){"W-x20-bottom", 20, 0, 20, 14, params.brickLossDb, "brick"});
  walls.push_back ((WallSegment){"W-x30-bottom", 30, 0, 30, 14, params.brickLossDb, "brick"});
  walls.push_back ((WallSegment){"W-x40-bottom", 40, 0, 40, 14, params.brickLossDb, "brick"});
  walls.push_back ((WallSegment){"W-x50-bottom", 50, 0, 50, 14, params.partitionLossDb, "partition"});
  return walls;
}

static bool
PointInRect (double x, double y, const Rect &r)
{
  return x >= r.x1 && x <= r.x2 && y >= r.y1 && y <= r.y2;
}

static int
Orientation (double ax, double ay, double bx, double by, double cx, double cy)
{
  double value = (by - ay) * (cx - bx) - (bx - ax) * (cy - by);
  const double eps = 1e-9;
  if (std::fabs (value) < eps)
    {
      return 0;
    }
  return value > 0.0 ? 1 : 2;
}

static bool
OnSegment (double ax, double ay, double bx, double by, double cx, double cy)
{
  return bx <= std::max (ax, cx) && bx >= std::min (ax, cx) &&
         by <= std::max (ay, cy) && by >= std::min (ay, cy);
}

static bool
SegmentsIntersect (double p1x, double p1y, double q1x, double q1y,
                   double p2x, double p2y, double q2x, double q2y)
{
  int o1 = Orientation (p1x, p1y, q1x, q1y, p2x, p2y);
  int o2 = Orientation (p1x, p1y, q1x, q1y, q2x, q2y);
  int o3 = Orientation (p2x, p2y, q2x, q2y, p1x, p1y);
  int o4 = Orientation (p2x, p2y, q2x, q2y, q1x, q1y);

  if (o1 != o2 && o3 != o4)
    {
      return true;
    }

  if (o1 == 0 && OnSegment (p1x, p1y, p2x, p2y, q1x, q1y))
    {
      return true;
    }
  if (o2 == 0 && OnSegment (p1x, p1y, q2x, q2y, q1x, q1y))
    {
      return true;
    }
  if (o3 == 0 && OnSegment (p2x, p2y, p1x, p1y, q2x, q2y))
    {
      return true;
    }
  if (o4 == 0 && OnSegment (p2x, p2y, q1x, q1y, q2x, q2y))
    {
      return true;
    }
  return false;
}

static bool
LineIntersectsRect (double x1, double y1, double x2, double y2, const Rect &rect)
{
  if (PointInRect (x1, y1, rect) || PointInRect (x2, y2, rect))
    {
      return true;
    }

  if (SegmentsIntersect (x1, y1, x2, y2, rect.x1, rect.y1, rect.x2, rect.y1))
    {
      return true;
    }
  if (SegmentsIntersect (x1, y1, x2, y2, rect.x2, rect.y1, rect.x2, rect.y2))
    {
      return true;
    }
  if (SegmentsIntersect (x1, y1, x2, y2, rect.x2, rect.y2, rect.x1, rect.y2))
    {
      return true;
    }
  if (SegmentsIntersect (x1, y1, x2, y2, rect.x1, rect.y2, rect.x1, rect.y1))
    {
      return true;
    }
  return false;
}

static bool
LineIntersectsWall (double x1, double y1, double x2, double y2, const WallSegment &wall)
{
  return SegmentsIntersect (x1, y1, x2, y2, wall.x1, wall.y1, wall.x2, wall.y2);
}

static double
Distance3d (const GeoPoint &a, const GeoPoint &b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt (dx * dx + dy * dy + dz * dz);
}

static LinkQuality
EvaluateLink (const GeoPoint &a, const GeoPoint &b,
              const std::vector<WallSegment> &walls,
              const std::vector<Rect> &staircases,
              const Rect &atrium,
              const std::vector<Rect> &elevators,
              const std::vector<Rect> &concreteObstacles,
              const PropagationParams &params)
{
  LinkQuality q;
  q.distance = std::max (Distance3d (a, b), 1.0);

  double freeSpaceLoss = 32.44 + 20.0 * std::log10 (params.frequencyMHz) + 10.0 * params.pathExponent * std::log10 (q.distance / 1000.0);
  if (!std::isfinite (freeSpaceLoss))
    {
      freeSpaceLoss = 120.0;
    }

  double wallLoss = 0.0;
  for (std::vector<WallSegment>::const_iterator it = walls.begin (); it != walls.end (); ++it)
    {
      if (LineIntersectsWall (a.x, a.y, b.x, b.y, *it))
        {
          wallLoss += it->lossDb;
        }
    }

  double obstacleLoss = 0.0;
  for (std::vector<Rect>::const_iterator it = concreteObstacles.begin (); it != concreteObstacles.end (); ++it)
    {
      if (LineIntersectsRect (a.x, a.y, b.x, b.y, *it))
        {
          obstacleLoss += params.concreteObstacleLossDb;
        }
    }

  double elevatorLoss = 0.0;
  for (std::vector<Rect>::const_iterator it = elevators.begin (); it != elevators.end (); ++it)
    {
      if (LineIntersectsRect (a.x, a.y, b.x, b.y, *it))
        {
          elevatorLoss += params.elevatorLossDb;
        }
    }

  uint32_t floorsCrossed = static_cast<uint32_t> (std::round (std::fabs (a.z - b.z) / kFloorHeight));
  double floorLoss = floorsCrossed * params.floorLossDb;
  if (floorsCrossed > 0)
    {
      bool intersectsAtrium = LineIntersectsRect (a.x, a.y, b.x, b.y, atrium);
      bool intersectsStair = false;
      for (std::vector<Rect>::const_iterator it = staircases.begin (); it != staircases.end (); ++it)
        {
          if (LineIntersectsRect (a.x, a.y, b.x, b.y, *it))
            {
              intersectsStair = true;
              break;
            }
        }

      if (intersectsAtrium)
        {
          floorLoss = floorsCrossed * params.atriumFloorLossDb;
        }
      if (intersectsStair)
        {
          floorLoss = std::min (floorLoss, floorsCrossed * params.staircaseFloorLossDb);
        }
    }

  q.pathLossDb = std::max (40.0, freeSpaceLoss + wallLoss + obstacleLoss + elevatorLoss + floorLoss + params.indoorBaselineLossDb);
  q.rssiDbm = params.txPowerDbm - q.pathLossDb;
  q.snrDb = q.rssiDbm - params.noiseFloorDbm;
  q.connected = q.rssiDbm >= params.rxSensitivityDbm;
  return q;
}

static std::vector<uint32_t>
BuildValidationNodeSet (uint32_t count)
{
  if (count == 4)
    {
      return std::vector<uint32_t> ({0, 4, 8, 12});
    }
  if (count == 8)
    {
      return std::vector<uint32_t> ({0, 4, 8, 12, 16, 17, 13, 9});
    }
  std::vector<uint32_t> ids;
  for (uint32_t i = 0; i < kTotalNodes; ++i)
    {
      ids.push_back (i);
    }
  return ids;
}

static void
WriteFloorPlanSvg (const std::string &svgPath,
                   const std::vector<Rect> &rooms,
                   const std::vector<Rect> &corridors,
                   const std::vector<Rect> &staircases,
                   const std::vector<Rect> &elevators,
                   const std::vector<Rect> &obstacles,
                   const Rect &atrium,
                   const std::vector<GeoPoint> &nodePoints,
                   const std::vector<uint32_t> &globalNodeIds)
{
  std::ofstream svg (svgPath.c_str ());
  svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 620 420\">\n";
  svg << "<rect x=\"10\" y=\"10\" width=\"600\" height=\"400\" fill=\"#f8f8f8\" stroke=\"#222\" stroke-width=\"2\"/>\n";

  for (std::vector<Rect>::const_iterator it = corridors.begin (); it != corridors.end (); ++it)
    {
      const Rect &r = *it;
      svg << "<rect x=\"" << (10 + r.x1 * 10) << "\" y=\"" << (410 - r.y2 * 10)
          << "\" width=\"" << ((r.x2 - r.x1) * 10) << "\" height=\"" << ((r.y2 - r.y1) * 10)
          << "\" fill=\"#e7f4ff\" stroke=\"#c5dff5\" stroke-width=\"1\"/>\n";
    }

  for (std::vector<Rect>::const_iterator it = rooms.begin (); it != rooms.end (); ++it)
    {
      const Rect &r = *it;
      svg << "<rect x=\"" << (10 + r.x1 * 10) << "\" y=\"" << (410 - r.y2 * 10)
          << "\" width=\"" << ((r.x2 - r.x1) * 10) << "\" height=\"" << ((r.y2 - r.y1) * 10)
          << "\" fill=\"none\" stroke=\"#444\" stroke-width=\"1.2\"/>\n";
      svg << "<text x=\"" << (10 + (r.x1 + r.x2) * 5.0) << "\" y=\"" << (410 - (r.y1 + r.y2) * 5.0)
          << "\" font-size=\"10\" text-anchor=\"middle\" fill=\"#333\">" << r.id << "</text>\n";
    }

  svg << "<rect x=\"" << (10 + atrium.x1 * 10) << "\" y=\"" << (410 - atrium.y2 * 10)
      << "\" width=\"" << ((atrium.x2 - atrium.x1) * 10) << "\" height=\"" << ((atrium.y2 - atrium.y1) * 10)
      << "\" fill=\"#d1ffd6\" stroke=\"#2c8a3f\" stroke-width=\"1.5\"/>\n";
  svg << "<text x=\"" << (10 + (atrium.x1 + atrium.x2) * 5.0) << "\" y=\"" << (410 - (atrium.y1 + atrium.y2) * 5.0)
      << "\" font-size=\"11\" text-anchor=\"middle\" fill=\"#17682a\">ATRIUM</text>\n";

  for (std::vector<Rect>::const_iterator it = staircases.begin (); it != staircases.end (); ++it)
    {
      const Rect &r = *it;
      svg << "<rect x=\"" << (10 + r.x1 * 10) << "\" y=\"" << (410 - r.y2 * 10)
          << "\" width=\"" << ((r.x2 - r.x1) * 10) << "\" height=\"" << ((r.y2 - r.y1) * 10)
          << "\" fill=\"#fff0ba\" stroke=\"#ad8b1e\" stroke-width=\"1.2\"/>\n";
      svg << "<text x=\"" << (10 + (r.x1 + r.x2) * 5.0) << "\" y=\"" << (410 - (r.y1 + r.y2) * 5.0)
          << "\" font-size=\"9\" text-anchor=\"middle\" fill=\"#7b6314\">" << r.id << "</text>\n";
    }

  for (std::vector<Rect>::const_iterator it = elevators.begin (); it != elevators.end (); ++it)
    {
      const Rect &r = *it;
      svg << "<rect x=\"" << (10 + r.x1 * 10) << "\" y=\"" << (410 - r.y2 * 10)
          << "\" width=\"" << ((r.x2 - r.x1) * 10) << "\" height=\"" << ((r.y2 - r.y1) * 10)
          << "\" fill=\"#dddddd\" stroke=\"#666\" stroke-width=\"1.2\"/>\n";
      svg << "<text x=\"" << (10 + (r.x1 + r.x2) * 5.0) << "\" y=\"" << (410 - (r.y1 + r.y2) * 5.0)
          << "\" font-size=\"8\" text-anchor=\"middle\" fill=\"#444\">" << r.id << "</text>\n";
    }

  for (std::vector<Rect>::const_iterator it = obstacles.begin (); it != obstacles.end (); ++it)
    {
      const Rect &r = *it;
      svg << "<rect x=\"" << (10 + r.x1 * 10) << "\" y=\"" << (410 - r.y2 * 10)
          << "\" width=\"" << ((r.x2 - r.x1) * 10) << "\" height=\"" << ((r.y2 - r.y1) * 10)
          << "\" fill=\"#bbb\" stroke=\"#555\" stroke-width=\"1.2\"/>\n";
    }

  for (uint32_t i = 0; i < nodePoints.size (); ++i)
    {
      const GeoPoint &p = nodePoints[i];
      svg << "<circle cx=\"" << (10 + p.x * 10) << "\" cy=\"" << (410 - p.y * 10)
          << "\" r=\"4\" fill=\"#d22\"/>\n";
      svg << "<text x=\"" << (16 + p.x * 10) << "\" y=\"" << (406 - p.y * 10)
          << "\" font-size=\"9\" fill=\"#111\">N" << (globalNodeIds[i] + 1) << "</text>\n";
    }

  svg << "</svg>\n";
}

static void
WriteGeometryCsv (const std::string &path,
                  const std::vector<Rect> &rooms,
                  const std::vector<Rect> &corridors,
                  const std::vector<Rect> &staircases,
                  const std::vector<Rect> &elevators,
                  const std::vector<Rect> &obstacles,
                  const Rect &atrium)
{
  std::ofstream out (path.c_str ());
  out << "type,id,x1,y1,x2,y2\n";
  for (std::vector<Rect>::const_iterator it = rooms.begin (); it != rooms.end (); ++it)
    {
      out << "room," << it->id << "," << it->x1 << "," << it->y1 << "," << it->x2 << "," << it->y2 << "\n";
    }
  for (std::vector<Rect>::const_iterator it = corridors.begin (); it != corridors.end (); ++it)
    {
      out << "corridor," << it->id << "," << it->x1 << "," << it->y1 << "," << it->x2 << "," << it->y2 << "\n";
    }
  for (std::vector<Rect>::const_iterator it = staircases.begin (); it != staircases.end (); ++it)
    {
      out << "staircase," << it->id << "," << it->x1 << "," << it->y1 << "," << it->x2 << "," << it->y2 << "\n";
    }
  for (std::vector<Rect>::const_iterator it = elevators.begin (); it != elevators.end (); ++it)
    {
      out << "elevator," << it->id << "," << it->x1 << "," << it->y1 << "," << it->x2 << "," << it->y2 << "\n";
    }
  for (std::vector<Rect>::const_iterator it = obstacles.begin (); it != obstacles.end (); ++it)
    {
      out << "concrete-obstacle," << it->id << "," << it->x1 << "," << it->y1 << "," << it->x2 << "," << it->y2 << "\n";
    }
  out << "atrium," << atrium.id << "," << atrium.x1 << "," << atrium.y1 << "," << atrium.x2 << "," << atrium.y2 << "\n";
}

static std::vector<int>
FindPathBfs (const std::vector<std::vector<bool> > &adj, int source, int destination)
{
  std::vector<int> prev (adj.size (), -1);
  std::queue<int> q;
  q.push (source);
  prev[source] = source;
  while (!q.empty ())
    {
      int u = q.front ();
      q.pop ();
      if (u == destination)
        {
          break;
        }
      for (uint32_t v = 0; v < adj[u].size (); ++v)
        {
          if (adj[u][v] && prev[v] == -1)
            {
              prev[v] = u;
              q.push (v);
            }
        }
    }
  if (prev[destination] == -1)
    {
      return std::vector<int> ();
    }
  std::vector<int> path;
  for (int at = destination; at != source; at = prev[at])
    {
      path.push_back (at);
    }
  path.push_back (source);
  std::reverse (path.begin (), path.end ());
  return path;
}

static double
SafeDivide (double a, double b)
{
  return b > 0.0 ? a / b : 0.0;
}
} // namespace

int
main (int argc, char *argv[])
{
  std::string protocol = "aodv";
  std::string l2Mode = "adhoc"; // adhoc | mesh11s | mesh11s-l3 | mesh-flame
  uint32_t validationNodes = 20; // 4, 8, 20
  bool enablePcap = true;
  bool enableFlowMonitor = true;
  bool connectivityOnly = false;
  std::string resultsRoot = "results/research";
  uint32_t runSeed = 7;
  double appStartTime = 2.0;
  std::string addressMask = "255.255.255.255";
  std::string diagnosticOutputRoot = "";
  std::string beaconValidation = "all"; // all | n3-n8 | n5-n8 | both
  bool beaconOnly = false;
  bool peerFilterEnabled = false;
  uint32_t peerFilterNodeA = 3;
  uint32_t peerFilterNodeB = 8;

  PropagationParams params;
  params.frequencyMHz = 900.0;
  params.brickLossDb = 6.0;
  params.concreteLossDb = 14.0;
  params.partitionLossDb = 4.0;
  params.concreteObstacleLossDb = 8.0;
  params.elevatorLossDb = 10.0;
  params.floorLossDb = 14.0;
  params.staircaseFloorLossDb = 6.0;
  params.atriumFloorLossDb = 5.0;
  params.indoorBaselineLossDb = 28.0;
  params.pathExponent = 2.0;
  params.txPowerDbm = 20.0;
  params.rxSensitivityDbm = -82.0;
  params.noiseFloorDbm = -104.0;

  CommandLine cmd;
  cmd.AddValue ("protocol", "Routing protocol: aodv | aodvv2 | olsr", protocol);
  cmd.AddValue ("l2Mode", "L2 mode: adhoc | mesh11s | mesh-flame", l2Mode);
  cmd.AddValue ("validationNodes", "Node-count stage: 4, 8, 20", validationNodes);
  cmd.AddValue ("pcap", "Enable PCAP tracing", enablePcap);
  cmd.AddValue ("flowmon", "Enable FlowMonitor", enableFlowMonitor);
  cmd.AddValue ("connectivityOnly", "Stop after physical topology validation", connectivityOnly);
  cmd.AddValue ("resultsRoot", "Results base directory", resultsRoot);
  cmd.AddValue ("runSeed", "Random seed", runSeed);
  cmd.AddValue ("appStartTime", "UDP client start time (s)", appStartTime);
  cmd.AddValue ("addressMask", "IPv4 interface mask used by the protocol compatibility test", addressMask);
  cmd.AddValue ("diagnosticOutputRoot", "Per-run diagnostic output root", diagnosticOutputRoot);
  cmd.AddValue ("beaconValidation", "Phase 6B7 beacon set: all | n3-n8 | n5-n8 | both", beaconValidation);
  cmd.AddValue ("beaconOnly", "Suppress UDP application traffic for beacon-only/control-only tests", beaconOnly);
  cmd.AddValue ("peerFilterEnabled", "Diagnostic Dot11s peer acceptance filter", peerFilterEnabled);
  cmd.AddValue ("peerFilterNodeA", "Diagnostic peer filter endpoint A (logical node number)", peerFilterNodeA);
  cmd.AddValue ("peerFilterNodeB", "Diagnostic peer filter endpoint B (logical node number)", peerFilterNodeB);
  cmd.AddValue ("txPowerDbm", "TX power (dBm)", params.txPowerDbm);
  cmd.AddValue ("rxSensitivityDbm", "Receiver sensitivity threshold (dBm)", params.rxSensitivityDbm);
  cmd.AddValue ("noiseFloorDbm", "Noise floor for SNR estimate (dBm)", params.noiseFloorDbm);
  cmd.AddValue ("brickLossDb", "Brick wall attenuation (dB/crossing)", params.brickLossDb);
  cmd.AddValue ("concreteLossDb", "Concrete wall attenuation (dB/crossing)", params.concreteLossDb);
  cmd.AddValue ("partitionLossDb", "Partition attenuation (dB/crossing)", params.partitionLossDb);
  cmd.AddValue ("floorLossDb", "Reinforced floor attenuation (dB/floor)", params.floorLossDb);
  cmd.AddValue ("staircaseFloorLossDb", "Floor attenuation via staircase paths (dB/floor)", params.staircaseFloorLossDb);
  cmd.AddValue ("atriumFloorLossDb", "Floor attenuation via atrium paths (dB/floor)", params.atriumFloorLossDb);
  cmd.AddValue ("indoorBaselineLossDb", "Additional baseline indoor attenuation (dB)", params.indoorBaselineLossDb);
  cmd.Parse (argc, argv);
  if (!diagnosticOutputRoot.empty ())
    {
      ::setenv ("NS3_DIAGNOSTIC_ROOT", diagnosticOutputRoot.c_str (), 1);
      std::string command = "mkdir -p " + diagnosticOutputRoot + "/correlation " + diagnosticOutputRoot + "/phy " + diagnosticOutputRoot + "/beacon " + diagnosticOutputRoot + "/peer " + diagnosticOutputRoot + "/timers " + diagnosticOutputRoot + "/mac " + diagnosticOutputRoot + "/network " + diagnosticOutputRoot + "/aodv";
      std::system (command.c_str ());
    }

  if (protocol != "aodv" && protocol != "aodvv2" && protocol != "olsr")
    {
      NS_LOG_ERROR ("Unsupported protocol: " << protocol);
      return 2;
    }
  if (l2Mode != "adhoc" && l2Mode != "mesh11s" && l2Mode != "mesh11s-l3" && l2Mode != "mesh-flame")
    {
      NS_LOG_ERROR ("Unsupported l2Mode: " << l2Mode);
      return 2;
    }
  if (validationNodes != 4 && validationNodes != 8 && validationNodes != 20)
    {
      NS_LOG_ERROR ("validationNodes must be one of 4, 8, 20");
      return 2;
    }

  RngSeedManager::SetSeed (runSeed);
  RngSeedManager::SetRun (1);

  const std::vector<uint32_t> selectedGlobalIds = BuildValidationNodeSet (validationNodes);
  const uint32_t nodeCount = selectedGlobalIds.size ();
  const uint32_t sourceIdx = 0;
  const uint32_t destinationIdx = nodeCount - 1;

  std::ostringstream runDirBuilder;
  runDirBuilder << resultsRoot << "/" << protocol << "/" << l2Mode << "/n" << validationNodes;
  const std::string runDir = runDirBuilder.str ();
  EnsureDir (runDir);
  EnsureDir (runDir + "/pcap");
  EnsureDir (runDir + "/geometry");
  EnsureDir (runDir + "/connectivity");

  std::vector<Rect> rooms = BuildRoomLayout ();
  std::vector<Rect> staircases = BuildStaircases ();
  std::vector<Rect> corridors = BuildCorridors ();
  Rect atrium = BuildAtrium ();
  std::vector<Rect> elevators = BuildElevators ();
  std::vector<Rect> concreteObstacles = BuildConcreteObstacles ();
  std::vector<WallSegment> walls = BuildInternalWalls (params);

  NodeContainer nodes;
  nodes.Create (nodeCount);

  Ptr<ListPositionAllocator> positions = CreateObject<ListPositionAllocator> ();
  std::vector<GeoPoint> points;
  points.reserve (nodeCount);
  for (uint32_t i = 0; i < nodeCount; ++i)
    {
      const uint32_t gid = selectedGlobalIds[i];
      GeoPoint p;
      if (validationNodes == 8)
        {
          p.x = kEightNodeTopology[i][0];
          p.y = kEightNodeTopology[i][1];
          p.z = kEightNodeTopology[i][2];
        }
      else
        {
          p.x = kAllNodeLocations[gid][0];
          p.y = kAllNodeLocations[gid][1];
          p.z = kAllNodeLocations[gid][2];
        }
      points.push_back (p);
      positions->Add (Vector (p.x, p.y, p.z));
    }

  MobilityHelper mobility;
  mobility.SetPositionAllocator (positions);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);

  Ptr<Building> building = CreateObject<Building> ();
  building->SetBoundaries (Box (0.0, kBuildingLength, 0.0, kBuildingWidth, 0.0, 5.0 * kFloorHeight));
  building->SetBuildingType (Building::Office);
  building->SetExtWallsType (Building::ConcreteWithoutWindows);
  building->SetNFloors (5);
  building->SetNRoomsX (6);
  building->SetNRoomsY (3);
  BuildingsHelper::Install (nodes);
  BuildingsHelper::MakeMobilityModelConsistent ();

  Ptr<MatrixPropagationLossModel> matrixLoss = CreateObject<MatrixPropagationLossModel> ();
  matrixLoss->SetDefaultLoss (250.0);

  std::vector<std::vector<LinkQuality> > links;
  links.resize (nodeCount, std::vector<LinkQuality> (nodeCount));
  std::vector<std::vector<bool> > connectivity;
  connectivity.resize (nodeCount, std::vector<bool> (nodeCount, false));

  for (uint32_t i = 0; i < nodeCount; ++i)
    {
      connectivity[i][i] = true;
      for (uint32_t j = i + 1; j < nodeCount; ++j)
        {
          LinkQuality q = EvaluateLink (points[i], points[j], walls, staircases, atrium, elevators, concreteObstacles, params);
          links[i][j] = q;
          links[j][i] = q;
          Ptr<MobilityModel> ma = nodes.Get (i)->GetObject<MobilityModel> ();
          Ptr<MobilityModel> mb = nodes.Get (j)->GetObject<MobilityModel> ();
          matrixLoss->SetLoss (ma, mb, q.pathLossDb, true);
          connectivity[i][j] = q.connected;
          connectivity[j][i] = q.connected;
        }
    }

  std::vector<int> path = FindPathBfs (connectivity, static_cast<int> (sourceIdx), static_cast<int> (destinationIdx));

  std::ofstream matrixCsv ((runDir + "/connectivity/connectivity-matrix.csv").c_str ());
  matrixCsv << "node";
  for (uint32_t j = 0; j < nodeCount; ++j)
    {
      matrixCsv << ",N" << (selectedGlobalIds[j] + 1);
    }
  matrixCsv << "\n";
  for (uint32_t i = 0; i < nodeCount; ++i)
    {
      matrixCsv << "N" << (selectedGlobalIds[i] + 1);
      for (uint32_t j = 0; j < nodeCount; ++j)
        {
          matrixCsv << "," << (connectivity[i][j] ? 1 : 0);
        }
      matrixCsv << "\n";
    }
  matrixCsv.close ();

  std::ofstream linksCsv ((runDir + "/connectivity/link-rssi-snr.csv").c_str ());
  linksCsv << "nodeA,nodeB,distanceM,pathLossDb,rssiDbm,snrDb,connected\n";
  for (uint32_t i = 0; i < nodeCount; ++i)
    {
      for (uint32_t j = i + 1; j < nodeCount; ++j)
        {
          const LinkQuality &q = links[i][j];
          linksCsv << "N" << (selectedGlobalIds[i] + 1) << ",N" << (selectedGlobalIds[j] + 1) << ","
                   << q.distance << "," << q.pathLossDb << "," << q.rssiDbm << "," << q.snrDb << ","
                   << (q.connected ? "yes" : "no") << "\n";
        }
    }
  linksCsv.close ();

  if (validationNodes == 8)
    {
      std::ofstream topologyCsv ((resultsRoot + "/connectivity.csv").c_str ());
      topologyCsv << "nodeA,nodeB,distanceM,forwardFeasible,reverseFeasible,bidirectional,rssiDbm,snrDb\n";
      for (uint32_t i = 0; i < nodeCount; ++i)
        {
          for (uint32_t j = i + 1; j < nodeCount; ++j)
            {
              const LinkQuality &q = links[i][j];
              topologyCsv << "N" << (i + 1) << ",N" << (j + 1) << "," << q.distance << ","
                          << (q.connected ? "yes" : "no") << "," << (q.connected ? "yes" : "no") << ","
                          << (q.connected ? "yes" : "no") << "," << q.rssiDbm << "," << q.snrDb << "\n";
            }
        }
      topologyCsv.close ();
      const bool primary = connectivity[0][1] && connectivity[1][2] && connectivity[2][7];
      const bool alternate = connectivity[0][3] && connectivity[3][4] && connectivity[4][7];
      const bool noDirect = !connectivity[0][7];
      const bool branchConnected = connectivity[1][5] && connectivity[5][6];
      const bool noBranchShortcut = !connectivity[5][7] && !connectivity[6][7];
      std::vector<std::vector<bool> > primaryFailure = connectivity;
      primaryFailure[1][2] = false;
      primaryFailure[2][1] = false;
      const bool alternateAfterFailure = primaryFailure[0][3] && primaryFailure[3][4] && primaryFailure[4][7]
                                         && !FindPathBfs (primaryFailure, 0, 7).empty ();
      std::ofstream gate ((resultsRoot + "/topology-validation.txt").c_str ());
      gate << "primary_N1_N2_N3_N8=" << (primary ? "PASS" : "FAIL") << "\n";
      gate << "alternate_N1_N4_N5_N8=" << (alternate ? "PASS" : "FAIL") << "\n";
      gate << "no_direct_N1_N8=" << (noDirect ? "PASS" : "FAIL") << "\n";
      gate << "N6_N7_branch_connected=" << (branchConnected ? "PASS" : "FAIL") << "\n";
      gate << "no_N6_N7_destination_shortcut=" << (noBranchShortcut ? "PASS" : "FAIL") << "\n";
      gate << "alternate_after_N2_N3_removal=" << (alternateAfterFailure ? "PASS" : "FAIL") << "\n";
      gate << "overall=" << (primary && alternate && noDirect && branchConnected && noBranchShortcut && alternateAfterFailure ? "PASS" : "FAIL") << "\n";
      gate.close ();
    }

  std::ofstream graphDot ((runDir + "/connectivity/connectivity-graph.dot").c_str ());
  graphDot << "graph Connectivity {\n";
  for (uint32_t i = 0; i < nodeCount; ++i)
    {
      graphDot << "  N" << (selectedGlobalIds[i] + 1) << ";\n";
    }
  for (uint32_t i = 0; i < nodeCount; ++i)
    {
      for (uint32_t j = i + 1; j < nodeCount; ++j)
        {
          if (connectivity[i][j])
            {
              graphDot << "  N" << (selectedGlobalIds[i] + 1) << " -- N" << (selectedGlobalIds[j] + 1)
                       << " [label=\"" << std::fixed << std::setprecision (1) << links[i][j].rssiDbm << " dBm\"];\n";
            }
        }
    }
  graphDot << "}\n";
  graphDot.close ();

  std::ofstream pathTxt ((runDir + "/connectivity/path-source-destination.txt").c_str ());
  pathTxt << "Source: N" << (selectedGlobalIds[sourceIdx] + 1) << "\n";
  pathTxt << "Destination: N" << (selectedGlobalIds[destinationIdx] + 1) << "\n";
  pathTxt << "Path available: " << (path.empty () ? "no" : "yes") << "\n";
  if (!path.empty ())
    {
      pathTxt << "Path: ";
      for (uint32_t k = 0; k < path.size (); ++k)
        {
          if (k > 0)
            {
              pathTxt << " -> ";
            }
          pathTxt << "N" << (selectedGlobalIds[path[k]] + 1);
        }
      pathTxt << "\n";
    }
  pathTxt.close ();

  WriteGeometryCsv (runDir + "/geometry/floor-plan-elements.csv", rooms, corridors, staircases, elevators, concreteObstacles, atrium);
  for (uint32_t floor = 0; floor < 5; ++floor)
    {
      std::vector<GeoPoint> floorNodes;
      std::vector<uint32_t> floorNodeIds;
      for (uint32_t i = 0; i < points.size (); ++i)
        {
          uint32_t f = static_cast<uint32_t> (std::round (points[i].z / kFloorHeight));
          if (f == floor)
            {
              floorNodes.push_back (points[i]);
              floorNodeIds.push_back (selectedGlobalIds[i]);
            }
        }
      std::ostringstream svgPath;
      svgPath << runDir << "/geometry/floor-" << (floor + 1) << "-layout.svg";
      WriteFloorPlanSvg (svgPath.str (), rooms, corridors, staircases, elevators, concreteObstacles, atrium, floorNodes, floorNodeIds);
    }

  if (connectivityOnly)
    {
      NS_LOG_UNCOND ("Connectivity-only validation complete; no routing protocol executed");
      Simulator::Destroy ();
      if (validationNodes == 8)
        {
          std::ifstream gate ((resultsRoot + "/topology-validation.txt").c_str ());
          std::string line;
          bool pass = false;
          while (std::getline (gate, line))
            {
              if (line == "overall=PASS")
                {
                  pass = true;
                }
            }
          return pass ? 0 : 3;
        }
      return path.empty () ? 3 : 0;
    }

  Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel> ();
  channel->SetPropagationDelayModel (CreateObject<ConstantSpeedPropagationDelayModel> ());
  channel->SetPropagationLossModel (matrixLoss);

  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  phy.SetChannel (channel);
  phy.Set ("TxPowerStart", DoubleValue (params.txPowerDbm));
  phy.Set ("TxPowerEnd", DoubleValue (params.txPowerDbm));
  phy.Set ("TxPowerLevels", UintegerValue (1));
  phy.Set ("EnergyDetectionThreshold", DoubleValue (params.rxSensitivityDbm));
  phy.Set ("CcaMode1Threshold", DoubleValue (params.rxSensitivityDbm - 3.0));
  phy.Set ("RxNoiseFigure", DoubleValue (6.8));
  phy.Set ("ShortGuardEnabled", BooleanValue (false));

  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211ah);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue ("OfdmRate1MbpsBW1MHz"),
                                "ControlMode", StringValue ("OfdmRate300KbpsBW1MHz"));

  NetDeviceContainer devices;
  if (l2Mode == "adhoc")
    {
      NqosWifiMacHelper mac = NqosWifiMacHelper::Default ();
      mac.SetType ("ns3::AdhocWifiMac");
      devices = wifi.Install (phy, mac, nodes);
      NS_LOG_UNCOND ("L2 mode: 802.11ah ad-hoc");
    }
  else
    {
      MeshHelper mesh = MeshHelper::Default ();
      mesh.SetStandard (WIFI_PHY_STANDARD_80211ah);
      // With eight simultaneous mesh points, a 100 ms beacon start window
      // causes peer-open collisions and leaves most peer tables empty.  Keep
      // the topology and PHY unchanged, but spread initial Dot11s beacons over
      // one second so peer-management convergence is observable.
      mesh.SetMacType ("RandomStart", TimeValue (Seconds (1.0)));
      mesh.SetNumberOfInterfaces (1);
      mesh.SetSpreadInterfaceChannels (MeshHelper::ZERO_CHANNEL);
      // MeshHelper::Default() sets ArfWifiManager, an adaptive rate
      // algorithm that was never validated against this fork's S1G rate
      // set. The four-node harness (scratch/80211s-no-hwmp-validation.cc)
      // explicitly overrides this to a fixed, already-demonstrated-reliable
      // rate for exactly this reason ("the 4 Mbps S1G payload mode ... has
      // payload PER=1 ... 300 kbps is the demonstrated reliable S1G mode").
      // This scratch file never applied that same override to the mesh
      // devices (a separate, unused WifiHelper a few lines below does set a
      // rate, but only for the "adhoc" l2Mode branch) -- every mesh11s*/
      // mesh-flame run was therefore attempting peer management at an
      // unvalidated, rate-adapted MCS. Matching the already-proven setting
      // here is a consistency fix (applies identically to every protocol),
      // not a parameter tuned to favor any one result.
      mesh.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                    "DataMode", StringValue ("OfdmRate300KbpsBW1MHz"),
                                    "ControlMode", StringValue ("OfdmRate300KbpsBW1MHz"));
      if (l2Mode == "mesh11s")
        {
          mesh.SetStackInstaller ("ns3::Dot11sStack");
          NS_LOG_UNCOND ("L2 mode: IEEE 802.11s Dot11sStack (includes HWMP in this tree)");
        }
      else if (l2Mode == "mesh11s-l3")
        {
          mesh.SetStackInstaller ("ns3::Dot11sStack", "UseHwmp", BooleanValue (false));
          NS_LOG_UNCOND ("L2 mode: IEEE 802.11s peer management + custom L3-only flooding (HWMP disabled)");
        }
      else
        {
          mesh.SetStackInstaller ("ns3::FlameStack");
          NS_LOG_UNCOND ("L2 mode: mesh with FLAME stack (no HWMP)");
        }
      devices = mesh.Install (phy, nodes);

      if (peerFilterEnabled && validationNodes == 8)
        {
          uint32_t a = peerFilterNodeA - 1;
          uint32_t b = peerFilterNodeB - 1;
          for (uint32_t i = 0; i < nodes.GetN (); ++i)
            {
              Ptr<MeshPointDevice> mp = devices.Get (i)->GetObject<MeshPointDevice> ();
              Ptr<dot11s::PeerManagementProtocol> pmp = mp->GetObject<dot11s::PeerManagementProtocol> ();
              Mac48Address allowed = Mac48Address::GetBroadcast ();
              Mac48Address secondAllowed = Mac48Address::GetBroadcast ();
              bool twoPairs = (beaconValidation == "both" && peerFilterNodeA == 3 && peerFilterNodeB == 8);
              if (twoPairs && i == 4)
                {
                  allowed = Mac48Address::ConvertFrom (devices.Get (7)->GetAddress ());
                }
              else if (twoPairs && i == 7)
                {
                  allowed = Mac48Address::ConvertFrom (devices.Get (2)->GetAddress ());
                  secondAllowed = Mac48Address::ConvertFrom (devices.Get (4)->GetAddress ());
                }
              else if (twoPairs && i != 2)
                {
                  allowed = Mac48Address::GetBroadcast ();
                }
              else if (i == a)
                {
                  allowed = Mac48Address::ConvertFrom (devices.Get (b)->GetAddress ());
                }
              else if (i == b)
                {
                  allowed = Mac48Address::ConvertFrom (devices.Get (a)->GetAddress ());
                }
              pmp->SetDiagnosticPeerFilter (true, allowed, secondAllowed);
            }
          NS_LOG_UNCOND ("Phase 6B8 peer filter enabled for logical nodes "
                         << peerFilterNodeA << " and " << peerFilterNodeB);
        }

      if (validationNodes == 8 && beaconValidation != "all")
        {
          std::set<uint32_t> keep;
          if (beaconValidation == "n3-n8")
            {
              keep.insert (2); keep.insert (7);
            }
          else if (beaconValidation == "n5-n8")
            {
              keep.insert (4); keep.insert (7);
            }
          else if (beaconValidation == "both")
            {
              keep.insert (2); keep.insert (4); keep.insert (7);
            }
          for (uint32_t i = 0; i < nodes.GetN (); ++i)
            {
              if (keep.find (i) != keep.end ())
                {
                  continue;
                }
              Ptr<MeshPointDevice> mp = devices.Get (i)->GetObject<MeshPointDevice> ();
              if (mp != 0 && !mp->GetInterfaces ().empty ())
                {
                  Ptr<WifiNetDevice> wifiDev = mp->GetInterfaces ().front ()->GetObject<WifiNetDevice> ();
                  if (wifiDev != 0)
                    {
                      Ptr<MeshWifiInterfaceMac> meshMac = wifiDev->GetMac ()->GetObject<MeshWifiInterfaceMac> ();
                      if (meshMac != 0)
                        {
                          meshMac->SetBeaconGeneration (false);
                        }
                    }
                }
            }
          NS_LOG_UNCOND ("Phase 6B7 beacon selection: " << beaconValidation);
        }
    }

  InternetStackHelper internet;
  if (protocol == "aodv")
    {
      AodvHelper helper;
      helper.Set ("NetDiameter", UintegerValue (40));
      helper.Set ("RreqRetries", UintegerValue (3));
      helper.Set ("RreqRateLimit", UintegerValue (20));
      internet.SetRoutingHelper (helper);
    }
  else if (protocol == "aodvv2")
    {
      Aodvv2Helper<Ipv4RoutingHelper> helper;
      helper.Set ("RreqRetries", UintegerValue (3));
      helper.Set ("RreqRateLimit", UintegerValue (20));
      internet.SetRoutingHelper (helper);
    }
  else
    {
      OlsrHelper helper;
      helper.Set ("HelloInterval", TimeValue (Seconds (1.0)));
      helper.Set ("TcInterval", TimeValue (Seconds (3.0)));
      helper.Set ("MidInterval", TimeValue (Seconds (3.0)));
      helper.Set ("HnaInterval", TimeValue (Seconds (3.0)));
      internet.SetRoutingHelper (helper);
    }
  internet.Install (nodes);

  Ipv4AddressHelper ipv4;
  // Treat each wireless interface as a routed host interface.  A /24 here
  // makes every destination appear directly attached, so the kernel ARPs for
  // N20 and never asks AODV/AODVv2/OLSR for a multi-hop route.
  ipv4.SetBase ("10.1.1.0", addressMask.c_str ());
  Ipv4InterfaceContainer interfaces = ipv4.Assign (devices);

  ApplicationContainer serverApps;
  ApplicationContainer clientApps;
  if (!beaconOnly)
    {
      UdpServerHelper server (kUdpPort);
      serverApps = server.Install (nodes.Get (destinationIdx));
      serverApps.Start (Seconds (1.0));
      serverApps.Stop (Seconds (kSimulationTime - 1.0));

      UdpClientHelper client (interfaces.GetAddress (destinationIdx), kUdpPort);
      client.SetAttribute ("MaxPackets", UintegerValue (2500));
      client.SetAttribute ("Interval", TimeValue (Seconds (kPacketInterval)));
      client.SetAttribute ("PacketSize", UintegerValue (kPacketSize));
      clientApps = client.Install (nodes.Get (sourceIdx));
      if (l2Mode != "adhoc" && appStartTime < 10.0)
        {
          appStartTime = 20.0;
        }
      clientApps.Start (Seconds (appStartTime));
      clientApps.Stop (Seconds (kSimulationTime - 1.0));
    }

  if (enablePcap)
    {
      phy.EnablePcapAll (runDir + "/pcap/node");
    }

  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
                   MakeCallback (&BeaconPhyRx));
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop",
                   MakeCallback (&BeaconPhyRxDrop));
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDropWithReason",
                   MakeCallback (&BeaconPhyRxDropReason));
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferTx",
                   MakeCallback (&BeaconPhyTx));

  FlowMonitorHelper flowHelper;
  Ptr<FlowMonitor> monitor;
  if (enableFlowMonitor)
    {
      monitor = flowHelper.InstallAll ();
    }

  std::ofstream preflight ((runDir + "/connectivity/preflight-summary.txt").c_str ());
  preflight << "Protocol: " << protocol << "\n";
  preflight << "L2 mode: " << l2Mode << "\n";
  preflight << "Validation nodes: " << validationNodes << "\n";
  preflight << "Source node: N" << (selectedGlobalIds[sourceIdx] + 1) << " (" << interfaces.GetAddress (sourceIdx) << ")\n";
  preflight << "Destination node: N" << (selectedGlobalIds[destinationIdx] + 1) << " (" << interfaces.GetAddress (destinationIdx) << ")\n";
  preflight << "Connectivity path exists: " << (path.empty () ? "NO" : "YES") << "\n";
  preflight << "HWMP status: ";
  if (l2Mode == "mesh11s")
    {
      preflight << "PRESENT (Dot11sStack installs HwmpProtocol internally)\n";
    }
  else if (l2Mode == "mesh11s-l3")
    {
      preflight << "ABSENT (Dot11s peer management retained; custom L2 protocol installed)\n";
    }
  else if (l2Mode == "mesh-flame")
    {
      preflight << "ABSENT (FlameStack)\n";
    }
  else
    {
      preflight << "N/A (ad-hoc)\n";
    }
  preflight.close ();

  Simulator::Stop (Seconds (kSimulationTime));
  Simulator::Run ();

  std::ofstream peerState ((runDir + "/peer-state.csv").c_str ());
  peerState << "node,establishedPeers\n";
  for (uint32_t i = 0; i < nodeCount; ++i)
    {
      Ptr<MeshPointDevice> mp = devices.Get (i)->GetObject<MeshPointDevice> ();
      Ptr<dot11s::PeerManagementProtocol> pmp = mp->GetObject<dot11s::PeerManagementProtocol> ();
      uint32_t iface = mp->GetInterfaces ().front ()->GetIfIndex ();
      peerState << (i + 1) << "," << pmp->GetPeers (iface).size () << "\n";
    }
  peerState.close ();

  std::ofstream metrics ((runDir + "/metrics.csv").c_str ());
  metrics << "flow,txPackets,rxPackets,lostPackets,pdrPercent,throughputKbps,avgDelaySeconds\n";

  std::ofstream summary ((runDir + "/summary.txt").c_str ());
  summary << "Five-floor research run\n";
  summary << "protocol=" << protocol << "\n";
  summary << "l2Mode=" << l2Mode << "\n";
  summary << "validationNodes=" << validationNodes << "\n";
  summary << "source=N" << (selectedGlobalIds[sourceIdx] + 1) << "\n";
  summary << "destination=N" << (selectedGlobalIds[destinationIdx] + 1) << "\n";

  if (enableFlowMonitor)
    {
      monitor->CheckForLostPackets ();
      monitor->SerializeToXmlFile ((runDir + "/flowmon.xml").c_str (), true, true);
      uint32_t flowId = 0;
      for (FlowMonitor::FlowStatsContainer::const_iterator it = monitor->GetFlowStats ().begin ();
           it != monitor->GetFlowStats ().end (); ++it, ++flowId)
        {
          const FlowMonitor::FlowStats &s = it->second;
          double pdr = SafeDivide (100.0 * s.rxPackets, s.txPackets);
          double duration = s.timeLastRxPacket.GetSeconds ();
          double throughput = SafeDivide (8.0 * s.rxBytes, 1024.0 * duration);
          double delay = SafeDivide (s.delaySum.GetSeconds (), s.rxPackets);
          metrics << flowId << "," << s.txPackets << "," << s.rxPackets << "," << s.lostPackets
                  << "," << pdr << "," << throughput << "," << delay << "\n";
          if (flowId == 0)
            {
              summary << "txPackets=" << s.txPackets << "\n";
              summary << "rxPackets=" << s.rxPackets << "\n";
              summary << "lostPackets=" << s.lostPackets << "\n";
              summary << "pdrPercent=" << pdr << "\n";
              summary << "throughputKbps=" << throughput << "\n";
              summary << "avgDelaySeconds=" << delay << "\n";
            }
        }
    }

  metrics.close ();
  summary.close ();

  Simulator::Destroy ();
  return 0;
}
