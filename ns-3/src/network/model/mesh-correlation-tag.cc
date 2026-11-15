#include "mesh-correlation-tag.h"
#include "ns3/uinteger.h"

namespace ns3 {

MeshCorrelationTag::MeshCorrelationTag (uint64_t id, uint32_t copy, uint8_t kind)
  : m_id (id), m_copy (copy), m_kind (kind) {}

TypeId MeshCorrelationTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MeshCorrelationTag")
    .SetParent<Tag> ().SetGroupName ("Network").AddConstructor<MeshCorrelationTag> ();
  return tid;
}
TypeId MeshCorrelationTag::GetInstanceTypeId (void) const { return GetTypeId (); }
uint32_t MeshCorrelationTag::GetSerializedSize (void) const { return 13; }
void MeshCorrelationTag::Serialize (TagBuffer i) const { i.WriteU64 (m_id); i.WriteU32 (m_copy); i.WriteU8 (m_kind); }
void MeshCorrelationTag::Deserialize (TagBuffer i) { m_id = i.ReadU64 (); m_copy = i.ReadU32 (); m_kind = i.ReadU8 (); }
void MeshCorrelationTag::Print (std::ostream &os) const { os << "id=" << m_id << " copy=" << m_copy << " kind=" << unsigned (m_kind); }
uint64_t MeshCorrelationTag::GetId () const { return m_id; }
uint32_t MeshCorrelationTag::GetCopy () const { return m_copy; }
uint8_t MeshCorrelationTag::GetKind () const { return m_kind; }

} // namespace ns3
