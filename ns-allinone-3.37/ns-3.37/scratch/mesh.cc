/*
 * Copyright (c) 2008,2009 IITP RAS
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
 * Author: Kirill Andreev <andreev@iitp.ru>
 *
 *
 * By default this script creates m_xSize * m_ySize square grid topology with
 * IEEE802.11s stack installed at each node with peering management
 * and HWMP protocol.
 * The side of the square cell is defined by m_step parameter.
 * When topology is created, UDP ping is installed to opposite corners
 * by diagonals. packet size of the UDP ping and interval between two
 * successive packets is configurable.
 *
 *  m_xSize * step
 *  |<--------->|
 *   step
 *  |<--->|
 *  * --- * --- * <---Ping sink  _
 *  | \   |   / |                ^
 *  |   \ | /   |                |
 *  * --- * --- * m_ySize * step |
 *  |   / | \   |                |
 *  | /   |   \ |                |
 *  * --- * --- *                _
 *  ^ Ping source
 *
 * By varying m_xSize and m_ySize, one can configure the route that is used.
 * When the inter-nodal distance is small, the source can reach the sink
 * directly.  When the inter-nodal distance is intermediate, the route
 * selected is diagonal (two hop).  When the inter-nodal distance is a bit
 * larger, the diagonals cannot be used and a four-hop route is selected.
 * When the distance is a bit larger, the packets will fail to reach even the
 * adjacent nodes.
 *
 * As of ns-3.36 release, with default configuration (mesh uses Wi-Fi 802.11a
 * standard and the ArfWifiManager rate control by default), the maximum
 * range is roughly 50m.  The default step size in this program is set to 50m,
 * so any mesh packets in the above diagram depiction will not be received
 * successfully on the diagonal hops between two nodes but only on the
 * horizontal and vertical hops.  If the step size is reduced to 35m, then
 * the shortest path will be on the diagonal hops.  If the step size is reduced
 * to 17m or less, then the source will be able to reach the sink directly
 * without any mesh hops (for the default 3x3 mesh depicted above).
 *
 * The position allocator will lay out the nodes in the following order
 * (corresponding to Node ID and to the diagram above):
 *
 * 6 - 7 - 8
 * |   |   |
 * 3 - 4 - 5
 * |   |   |
 * 0 - 1 - 2
 *
 *  See also MeshTest::Configure to read more about configurable
 *  parameters.
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mesh-helper.h"
#include "ns3/mesh-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/yans-wifi-helper.h"

#include <fstream>
#include <iostream>
#include <sstream>

#include "ns3/flow-monitor-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("MeshExample");

/**
 * \ingroup mesh
 * \brief MeshTest class
 */
class MeshTest
{
  public:
    /// Init test
    MeshTest();
    /**
     * Configure test from command line arguments
     *
     * \param argc command line argument count
     * \param argv command line arguments
     */
    void Configure(int argc, char* argv[]);
    /**
     * Run test
     * \returns the test status
     */
    int Run();

  private:
    int m_xSize;             ///< X size
    int m_ySize;             ///< Y size
    int m_nWifis;            ///< mesh nodes number
    double m_step;           ///< step
    double m_randomStart;    ///< random start
    double m_totalTime;      ///< total time
    double m_packetInterval; ///< packet interval
    uint16_t m_packetSize;   ///< packet size
    uint32_t m_nIfaces;      ///< number interfaces
    uint32_t bytesTotal;     ///< Total received bytes.
    uint32_t packetsReceived; //!< Total received packets.
    bool m_chan;             ///< channel
    bool m_pcap;             ///< PCAP
    bool m_ascii;            ///< ASCII
    std::string m_stack;     ///< stack
    std::string m_root;      ///< root
    /// List of network nodes
    NodeContainer nodes;
    /// List of all mesh point devices
    NetDeviceContainer meshDevices;
    /// Addresses of interfaces:
    Ipv4InterfaceContainer interfaces;
    /// MeshHelper. Report is not static methods
    MeshHelper mesh;

  private:
    /// Create nodes and setup their mobility
    void CreateNodes();
    /// Install internet m_stack on nodes
    void InstallInternetStack();
    /// Install applications
    void InstallApplication();
    /// Print mesh devices diagnostics
    void Report();
    /**
     * Setup the receiving socket in a Sink Node.
     * \param addr The address of the node.
     * \param node The node pointer.
     * \return the socket.
     */
    Ptr<Socket> SetupPacketReceive(Ipv4Address addr, Ptr<Node> node);
    /**
     * Receive a packet.
     * \param socket The receiving socket.
     */
    void ReceivePacket(Ptr<Socket> socket);
    /**
     * Compute the throughput.
     */
    void CheckThroughput();
};

MeshTest::MeshTest()
    : m_xSize(3),
      m_ySize(3),
      m_nWifis(50),
      m_step(50.0),
      m_randomStart(0.1),
      m_totalTime(200.0),
      m_packetInterval(1),
      m_packetSize(1024),
      m_nIfaces(2),
      m_chan(false),
      m_pcap(false),
      m_ascii(false),
      m_stack("ns3::Dot11sStack"),
      m_root("ff:ff:ff:ff:ff:ff")
{
}

void
MeshTest::Configure(int argc, char* argv[])
{
    CommandLine cmd(__FILE__);
    cmd.AddValue("x-size", "Number of nodes in a row grid", m_xSize);
    cmd.AddValue("y-size", "Number of rows in a grid", m_ySize);
    cmd.AddValue("step", "Size of edge in our grid (meters)", m_step);
    // Avoid starting all mesh nodes at the same time (beacons may collide)
    cmd.AddValue("start", "Maximum random start delay for beacon jitter (sec)", m_randomStart);
    cmd.AddValue("time", "Simulation time (sec)", m_totalTime);
    cmd.AddValue("packet-interval", "Interval between packets in UDP ping (sec)", m_packetInterval);
    cmd.AddValue("packet-size", "Size of packets in UDP ping (bytes)", m_packetSize);
    cmd.AddValue("interfaces", "Number of radio interfaces used by each mesh point", m_nIfaces);
    cmd.AddValue("channels", "Use different frequency channels for different interfaces", m_chan);
    cmd.AddValue("pcap", "Enable PCAP traces on interfaces", m_pcap);
    cmd.AddValue("ascii", "Enable Ascii traces on interfaces", m_ascii);
    cmd.AddValue("stack", "Type of protocol stack. ns3::Dot11sStack by default", m_stack);
    cmd.AddValue("root", "Mac address of root mesh point in HWMP", m_root);

    cmd.Parse(argc, argv);
    NS_LOG_DEBUG("Grid:" << m_xSize << "*" << m_ySize);
    NS_LOG_DEBUG("Simulation time: " << m_totalTime << " s");
    if (m_ascii)
    {
        PacketMetadata::Enable();
    }
}

void
MeshTest::CreateNodes()
{
    nodes.Create(m_nWifis);
    // Configure YansWifiChannel
    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");
    wifiPhy.SetChannel(wifiChannel.Create());

    wifiPhy.Set("TxPowerStart", DoubleValue(7.5));
    wifiPhy.Set("TxPowerEnd", DoubleValue(7.5));

    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");

    /*
     * Create mesh helper and set stack installer to it
     * Stack installer creates all needed protocols and install them to
     * mesh point device
     */
    mesh = MeshHelper::Default();
    if (!Mac48Address(m_root.c_str()).IsBroadcast())
    {
        mesh.SetStackInstaller(m_stack, "Root", Mac48AddressValue(Mac48Address(m_root.c_str())));
    }
    else
    {
        // If root is not set, we do not use "Root" attribute, because it
        // is specified only for 11s
        mesh.SetStackInstaller(m_stack);
    }
    if (m_chan)
    {
        mesh.SetSpreadInterfaceChannels(MeshHelper::SPREAD_CHANNELS);
    }
    else
    {
        mesh.SetSpreadInterfaceChannels(MeshHelper::ZERO_CHANNEL);
    }
    mesh.SetMacType("RandomStart", TimeValue(Seconds(m_randomStart)));
    // Set number of interfaces - default is single-interface mesh point
    mesh.SetNumberOfInterfaces(m_nIfaces);
    // Install protocols and return container if MeshPointDevices
    meshDevices = mesh.Install(wifiPhy, nodes);
    // AssignStreams can optionally be used to control random variable streams
    mesh.AssignStreams(meshDevices, 0);
    // Setup mobility - static grid topology
        ObjectFactory pos;
    pos.SetTypeId("ns3::RandomRectanglePositionAllocator");
    pos.Set("X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));
    pos.Set("Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=1500.0]"));

    Ptr<PositionAllocator> taPositionAlloc = pos.Create()->GetObject<PositionAllocator>();
    int nodeSpeed = 20; // in m/s
    int nodePause = 0;  // in s
    MobilityHelper mobility;
    std::stringstream ssSpeed;
    ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << nodeSpeed << "]";
    std::stringstream ssPause;
    ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";
    mobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
        "Speed", StringValue(ssSpeed.str()),
        "Pause", StringValue(ssPause.str()),
        "PositionAllocator", PointerValue(taPositionAlloc)
    );
    mobility.SetPositionAllocator(taPositionAlloc);
    mobility.Install(nodes);
}

void
MeshTest::InstallInternetStack()
{
    InternetStackHelper internetStack;
    internetStack.Install(nodes);
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    interfaces = address.Assign(meshDevices);
}

void
MeshTest::InstallApplication()
{
    // std::string rate("2048bps"); // 2kbps
    std::string rate("5120bps"); // 5Kbps
    // std::string rate("10240bps"); // 10Kbps
    // std::string rate("15360bps"); // 15Kbps
    // std::string rate("20480bps"); // 20Kbps
    // std::string rate("25600bps"); // 25Kbps
    // std::string rate("30720bps"); // 30Kbps

    Config::SetDefault("ns3::OnOffApplication::DataRate", StringValue(rate));

    OnOffHelper onoff1("ns3::UdpSocketFactory", Address());
    onoff1.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
    onoff1.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));

    int nSinks = 10;
    for (int i = 0; i < nSinks; i++)
    {
        Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable>();
        uint32_t random = var->GetInteger(var->GetInteger(0, 19), var->GetInteger(20, 39));

        Ptr<Socket> sink = SetupPacketReceive(interfaces.GetAddress(random), nodes.Get(random));

        AddressValue remoteAddress(InetSocketAddress(interfaces.GetAddress(random), 9));
        onoff1.SetAttribute("Remote", remoteAddress);

        ApplicationContainer temp = onoff1.Install(nodes.Get(random + nSinks));
        temp.Start(Seconds(var->GetValue(0, 1.0)));
        temp.Stop(Seconds(m_totalTime));
    }
}

int
MeshTest::Run()
{
    CreateNodes();
    InstallInternetStack();
    InstallApplication();

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    uint32_t SentPackets = 0;
    uint32_t ReceivedPackets = 0;
    uint32_t LostPackets = 0;

    CheckThroughput();

    Simulator::Schedule(Seconds(m_totalTime), &MeshTest::Report, this);
    Simulator::Stop(Seconds(m_totalTime + 2));
    Simulator::Run();

    int j=0;
    float Throughput = 0;
    float AvgThroughput = 0;
    Time Jitter;
    Time Delay;
    Time AvgJitter;
    Time AvgDelay;

    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();

    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator iter = stats.begin (); iter != stats.end (); ++iter) {
        SentPackets = SentPackets +(iter->second.txPackets);
        ReceivedPackets = ReceivedPackets + (iter->second.rxPackets);
        LostPackets = LostPackets + (iter->second.txPackets-iter->second.rxPackets);
        Throughput = Throughput + (iter->second.rxBytes * 8.0/(iter->second.timeLastRxPacket.GetSeconds()-iter->second.timeFirstTxPacket.GetSeconds())/1024);
        Delay = Delay + (iter->second.delaySum);
        Jitter = Jitter + (iter->second.jitterSum);

        j = j + 1;
    }

    AvgThroughput = Throughput/j;
    AvgJitter = Jitter/j;
    AvgDelay = Delay/j;
    NS_LOG_UNCOND("--------Total Results of the simulation----------"<<std::endl);
    NS_LOG_UNCOND("Total sent packets  = " << SentPackets);
    NS_LOG_UNCOND("Total Received Packets = " << ReceivedPackets);
    NS_LOG_UNCOND("Total Lost Packets = " << LostPackets);
    NS_LOG_UNCOND("Packet Loss ratio = " << ((LostPackets*100)/SentPackets)<< "%");
    NS_LOG_UNCOND("Packet delivery ratio = " << ((ReceivedPackets*100)/SentPackets)<< "%");
    NS_LOG_UNCOND("Throughput = " << Throughput);
    NS_LOG_UNCOND("Average Throughput = " << AvgThroughput);
    NS_LOG_UNCOND("END to End Delay = " << Delay.GetMilliSeconds());
    NS_LOG_UNCOND("Average End to End Delay = " << AvgDelay.GetMilliSeconds());
    NS_LOG_UNCOND("End to End jitter delay = " << Jitter.GetMilliSeconds());
    NS_LOG_UNCOND("Average End to End jitter delay = " << AvgJitter.GetMilliSeconds());
    NS_LOG_UNCOND("Total Flod id " << j);
    monitor->SerializeToXmlFile("manet-routing.xml", true, true);

    Simulator::Destroy();
    return 0;
}

static inline std::string
PrintReceivedPacket(Ptr<Socket> socket, Ptr<Packet> packet, Address senderAddress)
{
    std::ostringstream oss;

    oss << Simulator::Now().GetSeconds() << " " << socket->GetNode()->GetId();

    if (InetSocketAddress::IsMatchingType(senderAddress))
    {
        InetSocketAddress addr = InetSocketAddress::ConvertFrom(senderAddress);
        oss << " received one packet from " << addr.GetIpv4();
    }
    else
    {
        oss << " received one packet!";
    }
    return oss.str();
}

void
MeshTest::Report()
{
    unsigned n(0);
    for (NetDeviceContainer::Iterator i = meshDevices.Begin(); i != meshDevices.End(); ++i, ++n)
    {
        std::ostringstream os;
        // os << "mp-report-" << n << ".xml";
        std::cerr << "Printing mesh point device #" << n << " diagnostics to " << os.str() << "\n";
        std::ofstream of;
        of.open(os.str().c_str());
        if (!of.is_open())
        {
            std::cerr << "Error: Can't open file " << os.str() << "\n";
            return;
        }
        mesh.Report(*i, of);
        of.close();
    }
}

Ptr<Socket>
MeshTest::SetupPacketReceive(Ipv4Address addr, Ptr<Node> node)
{
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    Ptr<Socket> sink = Socket::CreateSocket(node, tid);
    InetSocketAddress local = InetSocketAddress(addr, 9);
    sink->Bind(local);
    sink->SetRecvCallback(MakeCallback(&MeshTest::ReceivePacket, this));

    return sink;
}

void
MeshTest::ReceivePacket(Ptr<Socket> socket)
{
    Ptr<Packet> packet;
    Address senderAddress;
    while ((packet = socket->RecvFrom(senderAddress)))
    {
        bytesTotal += packet->GetSize();
        packetsReceived += 1;
        NS_LOG_UNCOND(PrintReceivedPacket(socket, packet, senderAddress));
    }
}

void
MeshTest::CheckThroughput()
{
    bytesTotal = 0;
    packetsReceived = 0;
    Simulator::Schedule(Seconds(1.0), &MeshTest::CheckThroughput, this);
}

int
main(int argc, char* argv[])
{
    MeshTest t;
    t.Configure(argc, argv);
    return t.Run();
}
