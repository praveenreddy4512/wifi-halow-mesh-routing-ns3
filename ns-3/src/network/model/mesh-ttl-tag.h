#ifndef MESH_TTL_TAG_H
#define MESH_TTL_TAG_H
#include "ns3/tag.h"
namespace ns3 {
/**
 * \brief Bounds how many L2 hops a group-addressed (broadcast/multicast)
 * mesh frame may be blindly relayed.
 *
 * NoHwmpRoutingProtocol::RequestRoute relays any group-addressed frame it
 * receives with no loop/hop-count protection of its own (HWMP/FLAME, which
 * would normally provide this, are disabled by design). This tag restores
 * a standard, protocol-agnostic hop limit -- the same concept as an IP TTL
 * or the Mesh TTL field in a real 802.11s Mesh Control header -- without
 * suppressing broadcast traffic itself: a frame is still relayed up to
 * MeshTtlTag::kInitialTtl hops, just not forwarded forever.
 */
class MeshTtlTag : public Tag
{
public:
  static const uint8_t kInitialTtl = 10;

  MeshTtlTag ();
  explicit MeshTtlTag (uint8_t ttl);
  static TypeId GetTypeId (void);
  TypeId GetInstanceTypeId (void) const override;
  uint32_t GetSerializedSize (void) const override;
  void Serialize (TagBuffer i) const override;
  void Deserialize (TagBuffer i) override;
  void Print (std::ostream &os) const override;
  void SetTtl (uint8_t ttl);
  uint8_t GetTtl (void) const;
private:
  uint8_t m_ttl;
};
}
#endif
