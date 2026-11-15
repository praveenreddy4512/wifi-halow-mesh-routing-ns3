#include "mesh-last-hop-mac-tag.h"
namespace ns3 {
NS_OBJECT_ENSURE_REGISTERED (MeshLastHopMacTag);
MeshLastHopMacTag::MeshLastHopMacTag () : m_address (Mac48Address ()) {}
MeshLastHopMacTag::MeshLastHopMacTag (Mac48Address address) : m_address (address) {}
TypeId MeshLastHopMacTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MeshLastHopMacTag").SetParent<Tag> ().AddConstructor<MeshLastHopMacTag> ();
  return tid;
}
TypeId MeshLastHopMacTag::GetInstanceTypeId (void) const { return GetTypeId (); }
uint32_t MeshLastHopMacTag::GetSerializedSize (void) const { return 6; }
void MeshLastHopMacTag::Serialize (TagBuffer i) const
{
  uint8_t buf[6];
  m_address.CopyTo (buf);
  i.Write (buf, 6);
}
void MeshLastHopMacTag::Deserialize (TagBuffer i)
{
  uint8_t buf[6];
  i.Read (buf, 6);
  m_address.CopyFrom (buf);
}
void MeshLastHopMacTag::Print (std::ostream &os) const { os << m_address; }
void MeshLastHopMacTag::SetAddress (Mac48Address address) { m_address = address; }
Mac48Address MeshLastHopMacTag::GetAddress (void) const { return m_address; }
}
