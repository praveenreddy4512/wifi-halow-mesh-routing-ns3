#include "mesh-previous-hop-tag.h"
namespace ns3 {
NS_OBJECT_ENSURE_REGISTERED (MeshPreviousHopTag);
MeshPreviousHopTag::MeshPreviousHopTag () : m_address (Ipv4Address::GetZero ()) {}
MeshPreviousHopTag::MeshPreviousHopTag (Ipv4Address address) : m_address (address) {}
TypeId MeshPreviousHopTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MeshPreviousHopTag").SetParent<Tag> ().AddConstructor<MeshPreviousHopTag> ();
  return tid;
}
TypeId MeshPreviousHopTag::GetInstanceTypeId (void) const { return GetTypeId (); }
uint32_t MeshPreviousHopTag::GetSerializedSize (void) const { return 4; }
void MeshPreviousHopTag::Serialize (TagBuffer i) const { i.WriteU32 (m_address.Get ()); }
void MeshPreviousHopTag::Deserialize (TagBuffer i) { m_address = Ipv4Address (i.ReadU32 ()); }
void MeshPreviousHopTag::Print (std::ostream &os) const { os << m_address; }
void MeshPreviousHopTag::SetAddress (Ipv4Address address) { m_address = address; }
Ipv4Address MeshPreviousHopTag::GetAddress (void) const { return m_address; }
}
