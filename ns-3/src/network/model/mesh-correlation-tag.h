#ifndef MESH_CORRELATION_TAG_H
#define MESH_CORRELATION_TAG_H

#include "ns3/tag.h"

namespace ns3 {

class MeshCorrelationTag : public Tag
{
public:
  MeshCorrelationTag (uint64_t id = 0, uint32_t copy = 0, uint8_t kind = 0);
  static TypeId GetTypeId (void);
  TypeId GetInstanceTypeId (void) const override;
  uint32_t GetSerializedSize (void) const override;
  void Serialize (TagBuffer i) const override;
  void Deserialize (TagBuffer i) override;
  void Print (std::ostream &os) const override;
  uint64_t GetId () const;
  uint32_t GetCopy () const;
  uint8_t GetKind () const;
private:
  uint64_t m_id;
  uint32_t m_copy;
  uint8_t m_kind;
};

} // namespace ns3

#endif
