#ifndef MESH_LAST_HOP_MAC_TAG_H
#define MESH_LAST_HOP_MAC_TAG_H
#include "ns3/mac48-address.h"
#include "ns3/tag.h"
namespace ns3 {
/**
 * Fix 4 (docs/AODVV2_LQ_FIX4_IMPLEMENTATION_REPORT.md): carries the MAC
 * address of whoever physically transmitted this exact frame over the
 * single radio hop that just delivered it to the local node. Set once,
 * in MeshPointDevice::ReceiveFromDevice(), from that function's own
 * `src48` -- the one place in this project's mesh stack where the true,
 * authoritative, per-hop transmitting address is already known with no
 * indirection. Unlike MeshPreviousHopTag (which carries an *IP* address,
 * supplied by a relay node describing itself before re-forwarding, and
 * is therefore absent on a packet's very first, direct hop from its true
 * originator), this tag is attached on every reception, by the receiver
 * itself, from data it already has in hand -- so it is present for every
 * hop, including the first one, with no ARP lookup and no separate
 * lifecycle or staleness to manage: each packet carries its own, fresh,
 * single-hop answer.
 */
class MeshLastHopMacTag : public Tag
{
public:
  MeshLastHopMacTag ();
  explicit MeshLastHopMacTag (Mac48Address address);
  static TypeId GetTypeId (void);
  TypeId GetInstanceTypeId (void) const override;
  uint32_t GetSerializedSize (void) const override;
  void Serialize (TagBuffer i) const override;
  void Deserialize (TagBuffer i) override;
  void Print (std::ostream &os) const override;
  void SetAddress (Mac48Address address);
  Mac48Address GetAddress (void) const;
private:
  Mac48Address m_address;
};
}
#endif
