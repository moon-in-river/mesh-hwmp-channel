/*
 * Copyright (c) 2011 University of Kansas
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
 * Author: Justin Rohrer <rohrej@ittc.ku.edu>
 *
 * James P.G. Sterbenz <jpgs@ittc.ku.edu>, director
 * ResiliNets Research Group  https://resilinets.org/
 * Information and Telecommunication Technology Center (ITTC)
 * and Department of Electrical Engineering and Computer Science
 * The University of Kansas Lawrence, KS USA.
 *
 * Work supported in part by NSF FIND (Future Internet Design) Program
 * under grant CNS-0626918 (Postmodern Internet Architecture),
 * NSF grant CNS-1050226 (Multilayer Network Resilience Analysis and Experimentation on GENI),
 * US Department of Defense (DoD), and ITTC at The University of Kansas.
 */

/*
 * This example program allows one to run ns-3 DSDV, AODV, or OLSR under
 * a typical random waypoint mobility model.
 *
 * By default, the simulation runs for 200 simulated seconds, of which
 * the first 50 are used for start-up time.  The number of nodes is 50.
 * Nodes move according to RandomWaypointMobilityModel with a speed of
 * 20 m/s and no pause time within a 300x1500 m region.  The WiFi is
 * in ad hoc mode with a 2 Mb/s rate (802.11b) and a Friis loss model.
 * The transmit power is set to 7.5 dBm.
 *
 * It is possible to change the mobility and density of the network by
 * directly modifying the speed and the number of nodes.  It is also
 * possible to change the characteristics of the network by changing
 * the transmit power (as power increases, the impact of mobility
 * decreases and the effective density increases).
 *
 * By default, OLSR is used, but specifying a value of 2 for the protocol
 * will cause AODV to be used, and specifying a value of 3 will cause
 * DSDV to be used.
 *
 * By default, there are 10 source/sink data pairs sending UDP data
 * at an application rate of 2.048 Kb/s each.    This is typically done
 * at a rate of 4 64-byte packets per second.  Application data is
 * started at a random time between 50 and 51 seconds and continues
 * to the end of the simulation.
 *
 * The program outputs a few items:
 * - packet receptions are notified to stdout such as:
 *   <timestamp> <node-id> received one packet from <src-address>
 * - each second, the data reception statistics are tabulated and output
 *   to a comma-separated value (csv) file
 * - some tracing and flow monitor configuration that used to work is
 *   left commented inline in the program
 */

#include "ns3/aodv-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/olsr-module.h"
#include "ns3/yans-wifi-helper.h"

#include <fstream>
#include <iostream>

// add code //
#include "ns3/flow-monitor-module.h"
#include "ns3/mesh-helper.h"

//////////////

using namespace ns3;
using namespace dsr;

NS_LOG_COMPONENT_DEFINE("manet-routing-compare");

/**
 * Routing experiment class.
 *
 * It handles the creation and run of an experiment.
 */
class RoutingExperiment
{
  public:
    RoutingExperiment();
    /**
     * Run the experiment.
     * \param nSinks The number of Sink Nodes.
     * \param txp The Tx power.
     * \param CSVfileName The output CSV filename.
     */
    void Run(int nSinks, double txp, std::string CSVfileName);
    // static void SetMACParam (ns3::NetDeviceContainer & devices,
    //                                  int slotDistance);
    /**
     * Handles the command-line parmeters.
     * \param argc The argument count.
     * \param argv The argument vector.
     * \return the CSV filename.
     */
    std::string CommandSetup(int argc, char** argv);

  private:
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

    uint32_t port;            //!< Receiving port number.
    uint32_t bytesTotal;      //!< Total received bytes.
    uint32_t packetsReceived; //!< Total received packets.

    std::string m_CSVfileName;  //!< CSV filename.
    int m_nSinks;               //!< Number of sink nodes.
    std::string m_protocolName; //!< Protocol name.
    double m_txp;               //!< Tx power.
    bool m_traceMobility;       //!< Enavle mobility tracing.
    uint32_t m_protocol;        //!< Protocol type.
};

RoutingExperiment::RoutingExperiment()
    : port(9),
      bytesTotal(0),
      packetsReceived(0),
      m_CSVfileName("manet-routing.output.csv"),
      m_traceMobility(false),
      m_protocol(1) // OLSR
    //   m_protocol(2) // AODV
    //   m_protocol(5) // HWMP
{
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
RoutingExperiment::ReceivePacket(Ptr<Socket> socket)
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
RoutingExperiment::CheckThroughput()
{
    double kbs = (bytesTotal * 8.0) / 1000;
    bytesTotal = 0;

    std::ofstream out(m_CSVfileName, std::ios::app);

    out << (Simulator::Now()).GetSeconds() << "," << kbs << "," << packetsReceived << ","
        << m_nSinks << "," << m_protocolName << "," << m_txp << "" << std::endl;

    out.close();
    packetsReceived = 0;
    Simulator::Schedule(Seconds(1.0), &RoutingExperiment::CheckThroughput, this);
}

Ptr<Socket>
RoutingExperiment::SetupPacketReceive(Ipv4Address addr, Ptr<Node> node)
{
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    Ptr<Socket> sink = Socket::CreateSocket(node, tid);
    InetSocketAddress local = InetSocketAddress(addr, port);
    sink->Bind(local);
    sink->SetRecvCallback(MakeCallback(&RoutingExperiment::ReceivePacket, this));

    return sink;
}

std::string
RoutingExperiment::CommandSetup(int argc, char** argv)
{
    CommandLine cmd(__FILE__);
    cmd.AddValue("CSVfileName", "The name of the CSV output file name", m_CSVfileName);
    cmd.AddValue("traceMobility", "Enable mobility tracing", m_traceMobility);
    cmd.AddValue("protocol", "1=OLSR;2=AODV;3=DSDV;4=DSR;5=HWMP", m_protocol);
    cmd.Parse(argc, argv);
    return m_CSVfileName;
}

int
main(int argc, char* argv[])
{
    RoutingExperiment experiment;
    std::string CSVfileName = experiment.CommandSetup(argc, argv);

    // blank out the last output file and write the column headers
    std::ofstream out(CSVfileName);
    out << "SimulationSecond,"
        << "ReceiveRate,"
        << "PacketsReceived,"
        << "NumberOfSinks,"
        << "RoutingProtocol,"
        << "TransmissionPower" << std::endl;
    out.close();

    int nSinks = 10;
    double txp = 7.5;

    experiment.Run(nSinks, txp, CSVfileName);

    return 0;
}

void
RoutingExperiment::Run(int nSinks, double txp, std::string CSVfileName)
{
    Packet::EnablePrinting();
    m_nSinks = nSinks;
    m_txp = txp;
    m_CSVfileName = CSVfileName;

    int nWifis = 50;

    double TotalTime = 200.0;
    // std::string rate("2048bps"); // 2kbps
    std::string rate("5120bps"); // 5Kbps
    // std::string rate("10240bps"); // 10Kbps
    // std::string rate("15360bps"); // 15Kbps
    // std::string rate("20480bps"); // 20Kbps
    // std::string rate("25600bps"); // 25Kbps
    // std::string rate("30720bps"); // 30Kbps
    std::string phyMode("DsssRate11Mbps");
    std::string tr_name("manet-routing-compare");
    int nodeSpeed = 20; // in m/s
    int nodePause = 0;  // in s
    m_protocolName = "protocol";

    // add code //
    uint32_t SentPackets = 0;
    uint32_t ReceivedPackets = 0;
    uint32_t LostPackets = 0;
    //////////////

    // Config::SetDefault("ns3::OnOffApplication::PacketSize", StringValue("1024"));
    Config::SetDefault("ns3::OnOffApplication::DataRate", StringValue(rate));
    // Config::SetDefault("ns3::OnOffApplication::MaxBytes", StringValue("3072000"));

    // Set Non-unicastMode rate to unicast mode
    if (m_protocol <= 4) {
        Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue(phyMode));
    }

    NodeContainer adhocNodes;
    adhocNodes.Create(nWifis);

    // setting up wifi phy and channel using helpers
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211b);

    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");
    wifiPhy.SetChannel(wifiChannel.Create());

    // Add a mac and disable rate control
    WifiMacHelper wifiMac;
    if (m_protocol <= 4) {
        wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                    "DataMode",
                                    StringValue(phyMode),
                                    "ControlMode",
                                    StringValue(phyMode));
    }

    wifiPhy.Set("TxPowerStart", DoubleValue(txp));
    wifiPhy.Set("TxPowerEnd", DoubleValue(txp));

    wifiMac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer adhocDevices = wifi.Install(wifiPhy, wifiMac, adhocNodes);

    MobilityHelper mobilityAdhoc;
    int64_t streamIndex = 0; // used to get consistent mobility across scenarios

    ObjectFactory pos;
    pos.SetTypeId("ns3::RandomRectanglePositionAllocator");
    pos.Set("X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));
    pos.Set("Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=1500.0]"));

    Ptr<PositionAllocator> taPositionAlloc = pos.Create()->GetObject<PositionAllocator>();
    streamIndex += taPositionAlloc->AssignStreams(streamIndex);

    std::stringstream ssSpeed;
    ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << nodeSpeed << "]";
    std::stringstream ssPause;
    ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";
    mobilityAdhoc.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                                   "Speed",
                                   StringValue(ssSpeed.str()),
                                   "Pause",
                                   StringValue(ssPause.str()),
                                   "PositionAllocator",
                                   PointerValue(taPositionAlloc));
    mobilityAdhoc.SetPositionAllocator(taPositionAlloc);
    mobilityAdhoc.Install(adhocNodes);
    streamIndex += mobilityAdhoc.AssignStreams(adhocNodes, streamIndex);

    AodvHelper aodv;
    OlsrHelper olsr;
    DsdvHelper dsdv;
    DsrHelper dsr;
    DsrMainHelper dsrMain;
    Ipv4ListRoutingHelper list;
    InternetStackHelper internet;

    MeshHelper mesh;

    switch (m_protocol) {
        case 1:
            list.Add(olsr, 100);
            m_protocolName = "OLSR";
            break;
        case 2:
            list.Add(aodv, 100);
            m_protocolName = "AODV";
            break;
        case 3:
            list.Add(dsdv, 100);
            m_protocolName = "DSDV";
            break;
        case 4:
            m_protocolName = "DSR";
            break;
        case 5:
            m_protocolName = "HWMP";
            break;
        default:
            NS_FATAL_ERROR("No such protocol:" << m_protocol);
    }

    if (m_protocol < 4) {
        internet.SetRoutingHelper(list);
        internet.Install(adhocNodes);
    } else if (m_protocol == 4) {
        internet.Install(adhocNodes);
        dsrMain.Install(dsr, adhocNodes);
    } else if (m_protocol == 5) {
        // TODO
        std::string m_stack = "ns3::Dot11sStack";
        std::string m_root = "ff:ff:ff:ff:ff:ff";
        bool m_reactive = false;

        mesh = MeshHelper::Default();
        if (!m_reactive && !Mac48Address(m_root.c_str()).IsBroadcast()) {
            mesh.SetStackInstaller(
                m_stack, "Root",
                Mac48AddressValue(Mac48Address(m_root.c_str()))
            );
        } else {
            mesh.SetStackInstaller(m_stack);
        }

        mesh.SetSpreadInterfaceChannels(MeshHelper::ZERO_CHANNEL);
        mesh.SetMacType("RandomStart", TimeValue(Seconds(0.1)));
        mesh.SetNumberOfInterfaces(2);
        adhocDevices = mesh.Install(wifiPhy, adhocNodes);
        internet.Install(adhocNodes);
    }

    NS_LOG_INFO("assigning ip address");

    Ipv4AddressHelper addressAdhoc;
    addressAdhoc.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer adhocInterfaces;
    adhocInterfaces = addressAdhoc.Assign(adhocDevices);

    OnOffHelper onoff1("ns3::UdpSocketFactory", Address());
    onoff1.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
    onoff1.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));

    for (int i = 0; i < nSinks; i++)
    {
        Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable>();
        uint32_t random = var->GetInteger(var->GetInteger(0, 19), var->GetInteger(20, 39));

        Ptr<Socket> sink = SetupPacketReceive(adhocInterfaces.GetAddress(random), adhocNodes.Get(random));

        AddressValue remoteAddress(InetSocketAddress(adhocInterfaces.GetAddress(random), port));
        onoff1.SetAttribute("Remote", remoteAddress);

        ApplicationContainer temp = onoff1.Install(adhocNodes.Get(random + nSinks));
        temp.Start(Seconds(var->GetValue(0, 1.0)));
        temp.Stop(Seconds(TotalTime));
    }

    std::stringstream ss;
    ss << nWifis;
    std::string nodes = ss.str();

    std::stringstream ss2;
    ss2 << nodeSpeed;
    std::string sNodeSpeed = ss2.str();

    std::stringstream ss3;
    ss3 << nodePause;
    std::string sNodePause = ss3.str();

    std::stringstream ss4;
    ss4 << rate;
    std::string sRate = ss4.str();

    AsciiTraceHelper ascii;
    MobilityHelper::EnableAsciiAll(ascii.CreateFileStream(tr_name + ".mob"));

    // add code //
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    //////////////

    NS_LOG_INFO("Run Simulation.");

    CheckThroughput();

    Simulator::Stop(Seconds(TotalTime));
    Simulator::Run();

    // add code //
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
    ////////////

    Simulator::Destroy();
}
