#include "mesh-ttl-tag.h"
namespace ns3 {
NS_OBJECT_ENSURE_REGISTERED (MeshTtlTag);
MeshTtlTag::MeshTtlTag () : m_ttl (kInitialTtl) {}
MeshTtlTag::MeshTtlTag (uint8_t ttl) : m_ttl (ttl) {}
TypeId MeshTtlTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MeshTtlTag").SetParent<Tag> ().AddConstructor<MeshTtlTag> ();
  return tid;
}
TypeId MeshTtlTag::GetInstanceTypeId (void) const { return GetTypeId (); }
uint32_t MeshTtlTag::GetSerializedSize (void) const { return 1; }
void MeshTtlTag::Serialize (TagBuffer i) const { i.WriteU8 (m_ttl); }
void MeshTtlTag::Deserialize (TagBuffer i) { m_ttl = i.ReadU8 (); }
void MeshTtlTag::Print (std::ostream &os) const { os << static_cast<uint32_t> (m_ttl); }
void MeshTtlTag::SetTtl (uint8_t ttl) { m_ttl = ttl; }
uint8_t MeshTtlTag::GetTtl (void) const { return m_ttl; }
}
