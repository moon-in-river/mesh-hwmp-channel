/*
 * Copyright (c) 2009 IITP RAS
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Kirill Andreev <andreev@iitp.ru>
 *          Pavel Boyko <boyko@iitp.ru>
 */

#include "ns3/mesh-wifi-interface-mac.h"

#include "ns3/boolean.h"
#include "ns3/channel-access-manager.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/mac-tx-middle.h"
#include "ns3/mesh-wifi-beacon.h"
#include "ns3/pointer.h"
#include "ns3/qos-txop.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"
#include "ns3/socket.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/wifi-mac-queue-scheduler.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-utils.h"
#include "ns3/yans-wifi-phy.h"

#include "ns3/mac-low.h"
#include "ns3/mobility-module.h"
#include "ns3/rx-power-tag.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MeshWifiInterfaceMac");

NS_OBJECT_ENSURE_REGISTERED(MeshWifiInterfaceMac);

TypeId
MeshWifiInterfaceMac::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::MeshWifiInterfaceMac")
            .SetParent<WifiMac>()
            .SetGroupName("Mesh")
            .AddConstructor<MeshWifiInterfaceMac>()
            .AddAttribute("BeaconInterval",
                          "Beacon Interval",
                          TimeValue(Seconds(0.5)),

                          MakeTimeAccessor(&MeshWifiInterfaceMac::m_beaconInterval),
                          MakeTimeChecker())
            .AddAttribute("RandomStart",
                          "Window when beacon generating starts (uniform random) in seconds",
                          TimeValue(Seconds(0.5)),
                          MakeTimeAccessor(&MeshWifiInterfaceMac::m_randomStart),
                          MakeTimeChecker())
            .AddAttribute("BeaconGeneration",
                          "Enable/Disable Beaconing.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&MeshWifiInterfaceMac::SetBeaconGeneration,
                                              &MeshWifiInterfaceMac::GetBeaconGeneration),
                          MakeBooleanChecker());
    return tid;
}

MeshWifiInterfaceMac::MeshWifiInterfaceMac()
    : m_standard(WIFI_STANDARD_80211a)
{
    NS_LOG_FUNCTION(this);

    // Let the lower layers know that we are acting as a mesh node
    SetTypeOfStation(MESH);
    m_coefficient = CreateObject<UniformRandomVariable>();
}

MeshWifiInterfaceMac::~MeshWifiInterfaceMac()
{
    NS_LOG_FUNCTION(this);
}

//-----------------------------------------------------------------------------
// WifiMac inherited
//-----------------------------------------------------------------------------
bool
MeshWifiInterfaceMac::CanForwardPacketsTo(Mac48Address to) const
{
    return true;
}

void
MeshWifiInterfaceMac::Enqueue(Ptr<Packet> packet, Mac48Address to, Mac48Address from)
{
    NS_LOG_FUNCTION(this << packet << to << from);
    ForwardDown(packet, from, to);
}

void
MeshWifiInterfaceMac::Enqueue(Ptr<Packet> packet, Mac48Address to)
{
    NS_LOG_FUNCTION(this << packet << to);
    ForwardDown(packet, GetAddress(), to);
}

bool
MeshWifiInterfaceMac::SupportsSendFrom() const
{
    return true;
}

void
MeshWifiInterfaceMac::SetLinkUpCallback(Callback<void> linkUp)
{
    NS_LOG_FUNCTION(this);
    WifiMac::SetLinkUpCallback(linkUp);

    // The approach taken here is that, from the point of view of a mesh
    // node, the link is always up, so we immediately invoke the
    // callback if one is set
    linkUp();
}

void
MeshWifiInterfaceMac::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_plugins.clear();
    m_beaconSendEvent.Cancel();

    WifiMac::DoDispose();
}

void
MeshWifiInterfaceMac::DoInitialize()
{
    NS_LOG_FUNCTION(this);
    m_coefficient->SetAttribute("Max", DoubleValue(m_randomStart.GetSeconds()));
    if (m_beaconEnable)
    {
        Time randomStart = Seconds(m_coefficient->GetValue());
        // Now start sending beacons after some random delay (to avoid collisions)
        NS_ASSERT(!m_beaconSendEvent.IsRunning());
        m_beaconSendEvent =
            Simulator::Schedule(randomStart, &MeshWifiInterfaceMac::SendBeacon, this);
        m_tbtt = Simulator::Now() + randomStart;
    }
    else
    {
        // stop sending beacons
        m_beaconSendEvent.Cancel();
    }
}

int64_t
MeshWifiInterfaceMac::AssignStreams(int64_t stream)
{
    NS_LOG_FUNCTION(this << stream);
    int64_t currentStream = stream;
    m_coefficient->SetStream(currentStream++);
    for (PluginList::const_iterator i = m_plugins.begin(); i < m_plugins.end(); i++)
    {
        currentStream += (*i)->AssignStreams(currentStream);
    }
    return (currentStream - stream);
}

//-----------------------------------------------------------------------------
// Plugins
//-----------------------------------------------------------------------------
void
MeshWifiInterfaceMac::InstallPlugin(Ptr<MeshWifiInterfaceMacPlugin> plugin)
{
    NS_LOG_FUNCTION(this);

    plugin->SetParent(this);
    m_plugins.push_back(plugin);
}

//-----------------------------------------------------------------------------
// Switch channels
//-----------------------------------------------------------------------------
uint16_t
MeshWifiInterfaceMac::GetFrequencyChannel() const
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(GetWifiPhy()); // need PHY to set/get channel
    return GetWifiPhy()->GetChannelNumber();
}

void
MeshWifiInterfaceMac::SwitchFrequencyChannel(uint16_t new_id)
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(GetWifiPhy()); // need PHY to set/get channel
    /**
     * \todo
     * Correct channel switching is:
     *
     * 1. Interface down, e.g. to stop packets from layer 3
     * 2. Wait before all output queues will be empty
     * 3. Switch PHY channel
     * 4. Interface up
     *
     * Now we use dirty channel switch -- just change frequency
     */
    GetWifiPhy()->SetOperatingChannel(
        WifiPhy::ChannelTuple{new_id, 0, GetWifiPhy()->GetPhyBand(), 0});
    // Don't know NAV on new channel
    GetLink(SINGLE_LINK_OP_ID).channelAccessManager->NotifyNavResetNow(Seconds(0));
}

//-----------------------------------------------------------------------------
// Forward frame down
//-----------------------------------------------------------------------------
void
MeshWifiInterfaceMac::ForwardDown(Ptr<Packet> const_packet, Mac48Address from, Mac48Address to)
{
    // copy packet to allow modifications
    Ptr<Packet> packet = const_packet->Copy ();
    WifiMacHeader hdr;
    hdr.SetType(WIFI_MAC_QOSDATA);
    hdr.SetAddr2(GetAddress());
    hdr.SetAddr3(to);
    hdr.SetAddr4(from);
    hdr.SetDsFrom();
    hdr.SetDsTo();
    // Fill QoS fields:
    hdr.SetQosAckPolicy(WifiMacHeader::NORMAL_ACK);
    hdr.SetQosNoEosp();
    hdr.SetQosNoAmsdu();
    hdr.SetQosTxopLimit(0);
    // Address 1 is unknown here. Routing plugin is responsible to correctly set it.
    hdr.SetAddr1(Mac48Address());
    // Filter packet through all installed plugins
    for (PluginList::const_iterator i = m_plugins.end() - 1; i != m_plugins.begin() - 1; i--)
    {
        bool drop = !((*i)->UpdateOutcomingFrame(packet, hdr, from, to));
        if (drop)
        {
            return; // plugin drops frame
        }
    }
    // Assert that address1 is set. Assert will fail e.g. if there is no installed routing plugin.
    NS_ASSERT(hdr.GetAddr1() != Mac48Address());
    // Queue frame
    if (GetWifiRemoteStationManager()->IsBrandNew(hdr.GetAddr1()))
    {
        // in adhoc mode, we assume that every destination
        // supports all the rates we support.
        for (const auto& mode : GetWifiPhy()->GetModeList())
        {
            GetWifiRemoteStationManager()->AddSupportedMode(hdr.GetAddr1(), mode);
        }
        GetWifiRemoteStationManager()->RecordDisassociated(hdr.GetAddr1());
    }
    // Classify: application may have set a tag, which is removed here
    AcIndex ac;
    SocketPriorityTag tag;
    if (packet->RemovePacketTag(tag))
    {
        hdr.SetQosTid(tag.GetPriority());
        ac = QosUtilsMapTidToAc(tag.GetPriority());
    }
    else
    {
        // No tag found; set to best effort
        ac = AC_BE;
        hdr.SetQosTid(0);
    }
    m_stats.sentFrames++;
    m_stats.sentBytes += packet->GetSize();
    NS_ASSERT(GetQosTxop(ac) != nullptr);
    GetQosTxop(ac)->Queue(packet, hdr);
}

void
MeshWifiInterfaceMac::SendManagementFrame(Ptr<Packet> packet, const WifiMacHeader& hdr)
{
    // Filter management frames:
    WifiMacHeader header = hdr;
    for (PluginList::const_iterator i = m_plugins.end() - 1; i != m_plugins.begin() - 1; i--)
    {
        bool drop = !((*i)->UpdateOutcomingFrame(packet, header, Mac48Address(), Mac48Address()));
        if (drop)
        {
            return; // plugin drops frame
        }
    }
    m_stats.sentFrames++;
    m_stats.sentBytes += packet->GetSize();
    if ((GetQosTxop(AC_VO) == nullptr) || (GetQosTxop(AC_BK) == nullptr))
    {
        NS_FATAL_ERROR("Voice or Background queue is not set up!");
    }
    /*
     * When we send a management frame - it is better to enqueue it to
     * priority queue. But when we send a broadcast management frame,
     * like PREQ, little MinCw value may cause collisions during
     * retransmissions (two neighbor stations may choose the same window
     * size, and two packets will be collided). So, broadcast management
     * frames go to BK queue.
     */
    if (hdr.GetAddr1() != Mac48Address::GetBroadcast())
    {
        GetQosTxop(AC_VO)->Queue(packet, header);
    }
    else
    {
        GetQosTxop(AC_BK)->Queue(packet, header);
    }
}

SupportedRates
MeshWifiInterfaceMac::GetSupportedRates() const
{
    // set the set of supported rates and make sure that we indicate
    // the Basic Rate set in this set of supported rates.
    SupportedRates rates;
    for (const auto& mode : GetWifiPhy()->GetModeList())
    {
        uint16_t gi = ConvertGuardIntervalToNanoSeconds(mode, GetWifiPhy()->GetDevice());
        rates.AddSupportedRate(mode.GetDataRate(GetWifiPhy()->GetChannelWidth(), gi, 1));
    }
    // set the basic rates
    for (uint32_t j = 0; j < GetWifiRemoteStationManager()->GetNBasicModes(); j++)
    {
        WifiMode mode = GetWifiRemoteStationManager()->GetBasicMode(j);
        uint16_t gi = ConvertGuardIntervalToNanoSeconds(mode, GetWifiPhy()->GetDevice());
        rates.SetBasicRate(mode.GetDataRate(GetWifiPhy()->GetChannelWidth(), gi, 1));
    }
    return rates;
}

bool
MeshWifiInterfaceMac::CheckSupportedRates(SupportedRates rates) const
{
    for (uint32_t i = 0; i < GetWifiRemoteStationManager()->GetNBasicModes(); i++)
    {
        WifiMode mode = GetWifiRemoteStationManager()->GetBasicMode(i);
        uint16_t gi = ConvertGuardIntervalToNanoSeconds(mode, GetWifiPhy()->GetDevice());
        if (!rates.IsSupportedRate(mode.GetDataRate(GetWifiPhy()->GetChannelWidth(), gi, 1)))
        {
            return false;
        }
    }
    return true;
}

//-----------------------------------------------------------------------------
// Beacons
//-----------------------------------------------------------------------------
void
MeshWifiInterfaceMac::SetRandomStartDelay(Time interval)
{
    NS_LOG_FUNCTION(this << interval);
    m_randomStart = interval;
}

void
MeshWifiInterfaceMac::SetBeaconInterval(Time interval)
{
    NS_LOG_FUNCTION(this << interval);
    m_beaconInterval = interval;
}

Time
MeshWifiInterfaceMac::GetBeaconInterval() const
{
    return m_beaconInterval;
}

void
MeshWifiInterfaceMac::SetBeaconGeneration(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_beaconEnable = enable;
}

bool
MeshWifiInterfaceMac::GetBeaconGeneration() const
{
    return m_beaconSendEvent.IsRunning();
}

Time
MeshWifiInterfaceMac::GetTbtt() const
{
    return m_tbtt;
}

void
MeshWifiInterfaceMac::ShiftTbtt(Time shift)
{
    // User of ShiftTbtt () must take care don't shift it to the past
    NS_ASSERT(GetTbtt() + shift > Simulator::Now());

    m_tbtt += shift;
    // Shift scheduled event
    Simulator::Cancel(m_beaconSendEvent);
    m_beaconSendEvent =
        Simulator::Schedule(GetTbtt() - Simulator::Now(), &MeshWifiInterfaceMac::SendBeacon, this);
}

void
MeshWifiInterfaceMac::ScheduleNextBeacon()
{
    m_tbtt += GetBeaconInterval();
    m_beaconSendEvent =
        Simulator::Schedule(GetBeaconInterval(), &MeshWifiInterfaceMac::SendBeacon, this);
}

void
MeshWifiInterfaceMac::SendBeacon()
{
    NS_LOG_FUNCTION(this);
    NS_LOG_DEBUG(GetAddress() << " is sending beacon");

    NS_ASSERT(!m_beaconSendEvent.IsRunning());

    // Form & send beacon
    MeshWifiBeacon beacon(GetSsid(), GetSupportedRates(), m_beaconInterval.GetMicroSeconds());

    // Ask all plugins to add their specific information elements to beacon
    for (PluginList::const_iterator i = m_plugins.begin(); i != m_plugins.end(); ++i)
    {
        (*i)->UpdateBeacon(beacon);
    }
    m_txop->Queue(beacon.CreatePacket(), beacon.CreateHeader(GetAddress(), GetMeshPointAddress()));

    ScheduleNextBeacon();
}

void
MeshWifiInterfaceMac::Receive(Ptr<const WifiMpdu> mpdu, uint8_t linkId)
{
    const WifiMacHeader* hdr = &mpdu->GetHeader();
    Ptr<Packet> packet = mpdu->GetPacket()->Copy();
    // Process beacon
    if ((hdr->GetAddr1() != GetAddress()) && (hdr->GetAddr1() != Mac48Address::GetBroadcast()))
    {
        return;
    }
    if (hdr->IsBeacon())
    {
        m_stats.recvBeacons++;
        MgtBeaconHeader beacon_hdr;

        packet->PeekHeader(beacon_hdr);

        NS_LOG_DEBUG("Beacon received from " << hdr->GetAddr2() << " I am " << GetAddress()
                                             << " at " << Simulator::Now().GetMicroSeconds()
                                             << " microseconds");

        // update supported rates
        if (beacon_hdr.GetSsid().IsEqual(GetSsid()))
        {
            SupportedRates rates = beacon_hdr.GetSupportedRates();

            for (const auto& mode : GetWifiPhy()->GetModeList())
            {
                uint16_t gi = ConvertGuardIntervalToNanoSeconds(mode, GetWifiPhy()->GetDevice());
                uint64_t rate = mode.GetDataRate(GetWifiPhy()->GetChannelWidth(), gi, 1);
                if (rates.IsSupportedRate(rate))
                {
                    GetWifiRemoteStationManager()->AddSupportedMode(hdr->GetAddr2(), mode);
                    if (rates.IsBasicRate(rate))
                    {
                        GetWifiRemoteStationManager()->AddBasicMode(mode);
                    }
                }
            }
        }
    }
    else
    {
        m_stats.recvBytes += packet->GetSize();
        m_stats.recvFrames++;
    }
    // Filter frame through all installed plugins
    for (PluginList::iterator i = m_plugins.begin(); i != m_plugins.end(); ++i)
    {
        bool drop = !((*i)->Receive(packet, *hdr));
        if (drop)
        {
            return; // plugin drops frame
        }
    }
    // Check if QoS tag exists and add it:
    if (hdr->IsQosData())
    {
        SocketPriorityTag priorityTag;
        priorityTag.SetPriority(hdr->GetQosTid());
        packet->ReplacePacketTag(priorityTag);
    }
    // Forward data up
    if (hdr->IsData())
    {
        ForwardUp(packet, hdr->GetAddr4(), hdr->GetAddr3());
    }

    // We don't bother invoking WifiMac::Receive() here, because
    // we've explicitly handled all the frames we care about. This is in
    // contrast to most classes which derive from WifiMac.
}

uint32_t
MeshWifiInterfaceMac::GetLinkMetric(Mac48Address peerAddress)
{
    uint32_t metric = 1;
    if (!m_linkMetricCallback.IsNull())
    {
        metric = m_linkMetricCallback(peerAddress, this);
    }
    return metric;
}

void
MeshWifiInterfaceMac::SetLinkMetricCallback(
    Callback<uint32_t, Mac48Address, Ptr<MeshWifiInterfaceMac>> cb)
{
    m_linkMetricCallback = cb;
}

void
MeshWifiInterfaceMac::SetMeshPointAddress(Mac48Address a)
{
    m_mpAddress = a;
}

Mac48Address
MeshWifiInterfaceMac::GetMeshPointAddress() const
{
    return m_mpAddress;
}

// Statistics:
MeshWifiInterfaceMac::Statistics::Statistics()
    : recvBeacons(0),
      sentFrames(0),
      sentBytes(0),
      recvFrames(0),
      recvBytes(0)
{
}

void
MeshWifiInterfaceMac::Statistics::Print(std::ostream& os) const
{
    os << "<Statistics "
          /// \todo txBeacons
          "rxBeacons=\""
       << recvBeacons
       << "\" "
          "txFrames=\""
       << sentFrames
       << "\" "
          "txBytes=\""
       << sentBytes
       << "\" "
          "rxFrames=\""
       << recvFrames
       << "\" "
          "rxBytes=\""
       << recvBytes << "\"/>" << std::endl;
}

void
MeshWifiInterfaceMac::Report(std::ostream& os) const
{
    os << "<Interface "
          "BeaconInterval=\""
       << GetBeaconInterval().GetSeconds()
       << "\" "
          "Channel=\""
       << GetFrequencyChannel()
       << "\" "
          "Address = \""
       << GetAddress() << "\">" << std::endl;
    m_stats.Print(os);
    os << "</Interface>" << std::endl;
}

void
MeshWifiInterfaceMac::ResetStats()
{
    m_stats = Statistics();
}

void
MeshWifiInterfaceMac::ConfigureStandard(enum WifiStandard standard)
{
    NS_ABORT_IF(!GetQosSupported());
    WifiMac::ConfigureStandard(standard);
    m_standard = standard;
}

void
MeshWifiInterfaceMac::ConfigureContentionWindow(uint32_t cwMin, uint32_t cwMax)
{
    WifiMac::ConfigureContentionWindow(cwMin, cwMax);
    // We use the single DCF provided by WifiMac for the purpose of
    // Beacon transmission. For this we need to reconfigure the channel
    // access parameters slightly, and do so here.
    m_txop = CreateObject<Txop>();
    m_txop->SetWifiMac(this);
    GetLink(0).channelAccessManager->Add(m_txop);
    m_txop->SetTxMiddle(m_txMiddle);
    m_txop->SetMinCw(0);
    m_txop->SetMaxCw(0);
    m_txop->SetAifsn(1);
	m_scheduler->SetWifiMac(this);
}
WifiPhyStandard
MeshWifiInterfaceMac::GetPhyStandard () const
{
  return m_standard;
}
WifiMode
MeshWifiInterfaceMac::GetDataTxWifiMode(Mac48Address peerAddress)
{
  Ptr<Packet> m_dataFrame; ///< data frame
  WifiMacHeader m_dataHeader; ///< data header

  m_dataFrame = Create<Packet> (1024 + 6 /*Mesh header*/ + 36 /*802.11 header*/);

  m_dataHeader.SetType (WIFI_MAC_DATA);
  m_dataHeader.SetDsFrom ();
  m_dataHeader.SetDsTo ();
  m_dataHeader.SetQosTid (0);

  return GetWifiRemoteStationManager ()->GetDataTxVector (peerAddress, &m_dataHeader, m_dataFrame).GetMode();
}
void
MeshWifiInterfaceMac::UpdateFailAvg (Mac48Address peerAddress, double failAvg)
{
  bool updated = false;
  for (std::vector<std::pair<Mac48Address, NeighborInfoUnit> >::iterator i = m_neighborsInfo.begin();
        i != m_neighborsInfo.end(); i++)
    {
      if (i->first == peerAddress)
      {
        // if peerAddress is already in the list, then updates the fail Avg
        i->second.failAvg = failAvg;
        updated = true;
        break;
      }
    }
    if (!updated)
    {
      // if peerAddress does not yet exist in the list, then creates a new entry
      std::pair<Mac48Address, NeighborInfoUnit> newEntry;
      newEntry.first = peerAddress;
      newEntry.second.failAvg = failAvg;
      m_neighborsInfo.push_back(newEntry);
    }
}
double
MeshWifiInterfaceMac::GetFailAvg (Mac48Address peerAddress)
{
  double retval = 0;
  for (std::vector<std::pair<Mac48Address, NeighborInfoUnit> >::const_iterator i = m_neighborsInfo.begin();
        i != m_neighborsInfo.end(); i++)
    {
      if (i->first == peerAddress)
      {
        retval = i->second.failAvg;
        break;
      }
    }
  return retval;
}
void
MeshWifiInterfaceMac::UpdatePeerGeoInfo (Mac48Address peerAddress, Vector location, Vector velocity)
{
  bool updated = false;
  for (std::vector<std::pair<Mac48Address, NeighborInfoUnit> >::iterator i = m_neighborsInfo.begin();
        i != m_neighborsInfo.end(); i++)
    {
      if (i->first == peerAddress)
      {
        // if peerAddress is already in the list, then update the GeoInformation
        i->second.lastUpdatedGeo = Simulator::Now ();
        i->second.location = location;
        i->second.velocity = velocity;
        updated = true;
        break;
      }
    }
  if (!updated)
  {
    // if peerAddress does not exist in the list, then add it
    std::pair<Mac48Address, NeighborInfoUnit> newEntry;
    newEntry.first = peerAddress;
    newEntry.second.lastUpdatedGeo = Simulator::Now ();
    newEntry.second.location = location;
    newEntry.second.velocity = velocity;
    m_neighborsInfo.push_back(newEntry);
  }
}
void
MeshWifiInterfaceMac::UpdatePeerRxPower (Mac48Address peerAddress, double rxPower)
{
  bool updated = false;
  for (std::vector<std::pair<Mac48Address, NeighborInfoUnit> >::iterator i = m_neighborsInfo.begin();
        i != m_neighborsInfo.end(); i++)
    {
      if (i->first == peerAddress)
      {
        // if peerAddress is found in the list, then updat the rx power
        i->second.lastUpdatedPower = Simulator::Now ();
        i->second.rxPowerDbm = rxPower;
        updated = true;
        break;
      }
    }
  if (!updated)
  {
    // if peerAddress is not in the list, then add it
    std::pair<Mac48Address, NeighborInfoUnit> newEntry;
    newEntry.first = peerAddress;
    newEntry.second.lastUpdatedPower = Simulator::Now();
    newEntry.second.rxPowerDbm = rxPower;
    m_neighborsInfo.push_back(newEntry);
  }
}
Vector
MeshWifiInterfaceMac::GetPeerLocation (Mac48Address peerAddress)
{
  Vector retval;
  for (std::vector<std::pair<Mac48Address, NeighborInfoUnit> >::const_iterator i = m_neighborsInfo.begin();
        i != m_neighborsInfo.end(); i++)
    {
      if (i->first == peerAddress)
      {
        retval = i->second.location;
        break;
      }
    }
  return retval;
}
Vector
MeshWifiInterfaceMac::GetPeerVelocity (Mac48Address peerAddress)
{
  Vector retval;
  for (std::vector<std::pair<Mac48Address, NeighborInfoUnit> >::const_iterator i = m_neighborsInfo.begin();
        i != m_neighborsInfo.end(); i++)
    {
      if (i->first == peerAddress)
      {
        retval = i->second.velocity;
        break;
      }
    }
  return retval;
}
double
MeshWifiInterfaceMac::GetPeerRxPower (Mac48Address peerAddress)
{
  double retval = -100;
  for (std::vector<std::pair<Mac48Address, NeighborInfoUnit> >::const_iterator i = m_neighborsInfo.begin();
        i != m_neighborsInfo.end(); i++)
    {
      if (i->first == peerAddress)
      {
        retval = i->second.rxPowerDbm;
        break;
      }
    }
  return retval;
}
Time
MeshWifiInterfaceMac::GetPeerLastTimeStampGeo (Mac48Address peerAddress)
{
  Time lastUpdateTime;
  for (std::vector<std::pair<Mac48Address, NeighborInfoUnit> >::const_iterator i = m_neighborsInfo.begin();
        i != m_neighborsInfo.end(); i++)
    {
      if (i->first == peerAddress)
      {
        lastUpdateTime = i->second.lastUpdatedGeo;
        break;
      }
    }
  return lastUpdateTime;
}
Time
MeshWifiInterfaceMac::GetPeerLastTimeStampPower (Mac48Address peerAddress)
{
  Time lastUpdateTime;
  for (std::vector<std::pair<Mac48Address, NeighborInfoUnit> >::const_iterator i = m_neighborsInfo.begin();
        i != m_neighborsInfo.end(); i++)
    {
      if (i->first == peerAddress)
      {
        lastUpdateTime = i->second.lastUpdatedPower;
        break;
      }
    }
    return lastUpdateTime;
}
    

} // namespace ns3
