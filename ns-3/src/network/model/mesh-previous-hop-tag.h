#ifndef MESH_PREVIOUS_HOP_TAG_H
#define MESH_PREVIOUS_HOP_TAG_H
#include "ns3/ipv4-address.h"
#include "ns3/tag.h"
namespace ns3 {
class MeshPreviousHopTag : public Tag
{
public:
  MeshPreviousHopTag ();
  explicit MeshPreviousHopTag (Ipv4Address address);
  static TypeId GetTypeId (void);
  TypeId GetInstanceTypeId (void) const override;
  uint32_t GetSerializedSize (void) const override;
  void Serialize (TagBuffer i) const override;
  void Deserialize (TagBuffer i) override;
  void Print (std::ostream &os) const override;
  void SetAddress (Ipv4Address address);
  Ipv4Address GetAddress (void) const;
private:
  Ipv4Address m_address;
};
}
#endif
