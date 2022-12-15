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
 *
 * Modified by: Oscar Bautista <obaut004@fiu.edu>
 * to implement ETX and other metrics. (2019)
 */

#ifndef MESH_WIFI_INTERFACE_MAC_H
#define MESH_WIFI_INTERFACE_MAC_H

#include "ns3/callback.h"
#include "ns3/event-id.h"
#include "ns3/mac48-address.h"
#include "ns3/mesh-wifi-interface-mac-plugin.h"
#include "ns3/mgt-headers.h"
#include "ns3/nstime.h"
#include "ns3/packet.h"
#include "ns3/wifi-mac.h"

#include <map>
#include <stdint.h>

#include "ns3/vector.h"
#include "ns3/nstime.h"

namespace ns3
{

class UniformRandomVariable;

/**
 * \ingroup mesh
 *
 * \brief Basic MAC of mesh point Wi-Fi interface. Its function is extendable through plugins
 * mechanism.
 *
 * Now only three output queues are used:
 *  - beacons (PIFS and no backoff),
 *  - background traffic,
 *  - management and priority traffic.
 *
 */
class MeshWifiInterfaceMac : public WifiMac
{
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();
    /// C-tor
    MeshWifiInterfaceMac();
    /// D-tor
    ~MeshWifiInterfaceMac() override;

    // Inherited from WifiMac
    void Enqueue(Ptr<Packet> packet, Mac48Address to, Mac48Address from) override;
    void Enqueue(Ptr<Packet> packet, Mac48Address to) override;
    bool SupportsSendFrom() const override;
    void SetLinkUpCallback(Callback<void> linkUp) override;
    bool CanForwardPacketsTo(Mac48Address to) const override;

    /// \name Each mesh point interface must know the mesh point address
    ///@{
    /**
     * Set the mesh point address
     * \param addr the mesh point address
     */
    void SetMeshPointAddress(Mac48Address addr);
    /**
     * Get the mesh point address
     * \return The mesh point address
     */
    Mac48Address GetMeshPointAddress() const;
    ///@}

    /// \name Beacons
    ///@{
    /**
     * Set maximum initial random delay before first beacon
     * \param interval maximum random interval
     */
    void SetRandomStartDelay(Time interval);
    /**
     * Set interval between two successive beacons
     * \param interval beacon interval
     */
    void SetBeaconInterval(Time interval);
    /**
     * Get beacon interval.
     * \return interval between two beacons
     */
    Time GetBeaconInterval() const;
    /**
     * \brief Next beacon frame time
     * \return TBTT time
     *
     * This is supposed to be used by any entity managing beacon collision avoidance (e.g. Peer
     * management protocol in 802.11s)
     */
    Time GetTbtt() const;
    /**
     * \brief Shift TBTT.
     * \param shift Shift
     *
     * This is supposed to be used by any entity managing beacon collision avoidance (e.g. Peer
     * management protocol in 802.11s)
     *
     * \attention User of ShiftTbtt () must take care to not shift it to the past.
     */
    void ShiftTbtt(Time shift);
    ///@}

    /**
     * Install plugin.
     *
     * \param plugin Plugin
     *
     * \todo return unique ID to allow user to unregister plugins
     */
    void InstallPlugin(Ptr<MeshWifiInterfaceMacPlugin> plugin);

    /*
     * Channel center frequency = Channel starting frequency + 5 * channel_id (MHz),
     * where Starting channel frequency is standard-dependent as defined in IEEE
     * 802.11-2007 17.3.8.3.2.
     *
     * Number of channels to use must be limited elsewhere.
     */

    /**
     * Current channel Id
     * \returns the frequency channel
     */
    uint16_t GetFrequencyChannel() const;
    /**
     * Switch frequency channel.
     *
     * \param new_id New ID.
     */
    void SwitchFrequencyChannel(uint16_t new_id);

    /**
     * To be used by plugins sending management frames.
     *
     * \param frame the management frame
     * \param hdr the wifi MAC header
     */
    void SendManagementFrame(Ptr<Packet> frame, const WifiMacHeader& hdr);
    /**
     * Check supported rates.
     *
     * \param rates Rates.
     * \return true if rates are supported
     */
    bool CheckSupportedRates(SupportedRates rates) const;

    /**
     * Get supported rates.
     * \return list of supported bitrates
     */
    SupportedRates GetSupportedRates() const;

    /// \name Metric Calculation routines:
    ///@{
    /**
     * Set the link metric callback
     * \param cb the callback
     */
    void SetLinkMetricCallback(Callback<uint32_t, Mac48Address, Ptr<MeshWifiInterfaceMac>> cb);
    /**
     * Get the link metric
     * \param peerAddress the peer address
     * \return The metric
     */
    uint32_t GetLinkMetric(Mac48Address peerAddress);
    ///@}

    /**
     * \brief Report statistics
     * \param os the output stream
     */
    void Report(std::ostream& os) const;

    /**
     * Reset statistics function
     */
    void ResetStats();

    /**
     * Enable/disable beacons
     *
     * \param enable enable / disable flag
     */
    void SetBeaconGeneration(bool enable);
    /**
     * Finish configuration based on the WifiStandard being provided
     *
     * \param standard the WifiStandard being configured
     */
    void ConfigureStandard(enum WifiStandard standard) override;
    /**
     * \param cwMin the minimum contention window size
     * \param cwMax the maximum contention window size
     *
     * This method is called to set the minimum and the maximum
     * contention window size.
     */
    void ConfigureContentionWindow(uint32_t cwMin, uint32_t cwMax) override;

    /**
     * Assign a fixed random variable stream number to the random variables
     * used by this model.  Return the number of streams (possibly zero) that
     * have been assigned.
     *
     * \param stream first stream index to use
     * \return the number of stream indices assigned by this model
     */
    int64_t AssignStreams(int64_t stream);
  /**
   * Returns the WifiMode used for sending Data Packets
   * \param peerAddress, the peer Address
   */
  WifiMode GetDataTxWifiMode (Mac48Address peerAddress);
  /**
   * Update the failAvg of the link between local and Peer Interface
   * \param peerAddress, the peer Address
   * \param failAvg, the Packet failure Average
   */
  void UpdateFailAvg (Mac48Address peerAddress, double failAvg);
  /**
   * Get the failAvg of a link to a given Peer Interface Address
   * \param peerAddress, the peer Address
   * \return the packet failure average
   */
  double GetFailAvg (Mac48Address peerAddress);
  /**
   * Update the Location and Velocity of peerNode
   * \param peerAddress, the peer Address
   * \param peerLocation, the peer 3D location
   * \param peerVelocity, the peer velocity vector
   */
  void UpdatePeerGeoInfo (Mac48Address peerAddress, Vector location, Vector velocity);
  /**
   * Updates the RxPower of incoming packets from each peer within range
   * \param peerAddress, the peer Address
   * \param rxPower, the packet's RxPower including RxGain received from the peer
   */
  void UpdatePeerRxPower (Mac48Address peerAddress, double rxPower);
  /**
   * Gets the 3D location of a peer node given the Peer Interface Address
   * \param peerAddress, the peer Address
   * \return the peer's 3D location
   */
  Vector GetPeerLocation (Mac48Address peerAddress);
  /**
   * Gets the velocity vector of a peer node given the Peer Interface Address
   * \param peerAddress, the peer Address
   * \return the peer's velocity vector
   */
  Vector GetPeerVelocity (Mac48Address peerAddress);
  /**
   * Gets the last recorded RxPower received from the peer specified by it address
   * \param peerAddress, the peer Address
   * \return the packet RxPower
   */
  double GetPeerRxPower (Mac48Address peerAddress);
  /**
   * Gets the time peer information was last updated
   * \param peerAddress, the peer Address
   * \returns last Geo Information update time for that peer
   */
  Time GetPeerLastTimeStampGeo (Mac48Address peerAddress);
  /**
   * Gets the time RxPower was last updated
   * \param peerAddressm the peer address
   * \return the last RxPower update time of packets from that peer
   */
  Time GetPeerLastTimeStampPower (Mac48Address peerAddress);

  struct NeighborInfoUnit
  {
    double failAvg;
    Vector location;
    Vector velocity;
    double rxPowerDbm;
    Time lastUpdatedGeo;
    Time lastUpdatedPower;
    double pChgRate;  // Power Change Rate
    // constructor
    NeighborInfoUnit (): failAvg(0), pChgRate (0) {}
  };
  private:
    /**
     * Frame receive handler
     *
     * \param mpdu the received MPDU
     * \param linkId the ID of the link the frame was received over
     */
    void Receive(Ptr<const WifiMpdu> mpdu, uint8_t linkId) override;
    /**
     * Send frame. Frame is supposed to be tagged by routing information.
     *
     * \param packet the packet to forward
     * \param from the from address
     * \param to the to address
     */
    void ForwardDown(Ptr<Packet> packet, Mac48Address from, Mac48Address to);
    /**
     * Send beacon.
     */
    void SendBeacon();
    /**
     * Schedule next beacon.
     */
    void ScheduleNextBeacon();
    /**
     * Get current beaconing status
     *
     * \returns true if beacon active
     */
    bool GetBeaconGeneration() const;
    /**
     * Real d-tor.
     */
    void DoDispose() override;

  private:
    typedef std::vector<Ptr<MeshWifiInterfaceMacPlugin>> PluginList; ///< PluginList typedef
	typedef std::vector<std::pair<Mac48Address, NeighborInfoUnit> > NeighborInfoList;

    void DoInitialize() override;

    /// \name Mesh timing intervals
    ///@{
    /// Whether beaconing is enabled
    bool m_beaconEnable;
    /// Beaconing interval.
    Time m_beaconInterval;
    /// Maximum delay before first beacon
    Time m_randomStart;
    /// Time for the next frame
    Time m_tbtt;
    ///@}

    /// Mesh point address
    Mac48Address m_mpAddress;
  /// List of neighbor information elements (location, velocity, average frame error)
  NeighborInfoList m_neighborsInfo;

    /// "Timer" for the next beacon
    EventId m_beaconSendEvent;
    /// List of all installed plugins
    PluginList m_plugins;
    Callback<uint32_t, Mac48Address, Ptr<MeshWifiInterfaceMac>>
        m_linkMetricCallback; ///< linkMetricCallback

    /// Statistics:
    struct Statistics
    {
        uint16_t recvBeacons; ///< receive beacons
        uint32_t sentFrames;  ///< sent frames
        uint32_t sentBytes;   ///< sent bytes
        uint32_t recvFrames;  ///< receive frames
        uint32_t recvBytes;   ///< receive bytes
        /**
         * Print statistics.
         *
         * \param os Output stream
         */
        void Print(std::ostream& os) const;
        /**
         * Constructor.
         */
        Statistics();
    };

    Statistics m_stats; ///< statistics

    /// Current standard: needed to configure metric
    WifiStandard m_standard;

    /// Add randomness to beacon generation
    Ptr<UniformRandomVariable> m_coefficient;
};

} // namespace ns3

#endif /* MESH_WIFI_INTERFACE_MAC_H */
