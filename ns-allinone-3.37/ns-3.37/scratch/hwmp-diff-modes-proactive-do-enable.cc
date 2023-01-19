#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mesh-module.h"
#include "ns3/mobility-module.h"
#include "ns3/mesh-helper.h"
#include "ns3/mesh-module.h"
#include "ns3/wifi-phy.h"
#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/boolean.h"
#include "ns3/netanim-module.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
using namespace ns3;
class MeshTest {
public:
    // Init test
    MeshTest ();
    // Configure test from command line arguments
    void Configure (int argc, char ** argv);
    // Run test
    int Run ();
private:
    int m_nnodes; // number of nodes
    int m_nconn; // number of connections
    int m_nconnR; // number of connections to the root
    double m_step;
    double m_randomStart;
    double m_totalTime;
    uint16_t m_packetSize;
    uint32_t m_nIfaces;
    bool m_chan;
    bool m_pcap;
    std::string m_stack;
    int m_reactive;
    std::string m_txrate;
    //to calculate the lenght of the simulation
    float m_timeTotal, m_timeStart, m_timeEnd;
    // List of network nodes
    NodeContainer nodes;
    // List of all mesh point devices
    NetDeviceContainer meshDevices;
    //Addresses of interfaces:
    Ipv4InterfaceContainer interfaces;
    // MeshHelper. Report is not static methods
    MeshHelper mesh;
    // List of network nodes
    NodeContainer nodesC;
    // List of all mesh point devices
    NetDeviceContainer meshDevicesC;
    //Addresses of interfaces:
    Ipv4InterfaceContainer interfacesC;
    // MeshHelper. Report is not static methods
    MeshHelper meshC;
private:
    // Create nodes and setup their mobility
    void CreateNodes ();
    // Install internet m_stack on nodes
    void InstallInternetStack ();
    // Install applications randomly
    void InstallApplicationRandom ();
    // Print mesh devices diagnostics
    void Report ();
    // Create nodes and setup their mobility
    void CreateNodesC ();
    // Install internet m_stack on nodes
    void InstallInternetStackC ();
    // Setup the receiving socket in a Sink Node
    Ptr<Socket> SetupPacketReceive(Ipv4Address addr, Ptr<Node> node);
};
MeshTest::MeshTest () :
    m_nnodes (20), // 25 Proactive
    m_nconn (20), // total connections; 28 Proactive
    m_nconnR (0), // total connections to root node. increase 7, compare influnence for hwmp-r and hwmp-p under outer or innet traffic
    m_step (450), // 720 Proactive
    m_randomStart (0.5),
    m_totalTime (180), // 240s Proactive
    m_packetSize (1024),
    m_nIfaces (2),
    m_chan (false),
    m_pcap (false),
    m_stack ("ns3::Dot11sStack"),
    m_reactive (0),
    m_txrate ("150kbps") // 120kbps Proactive
{
}
void
MeshTest::Configure (int argc, char *argv[])
{
    CommandLine cmd;
    cmd.AddValue ("m_step", "Separation", m_step);
    cmd.AddValue ("m_nconn", "Number of connections", m_nconn);
    cmd.AddValue ("m_nconnR", "Number of root connections", m_nconnR);
    cmd.AddValue ("m_reactive", "Mode type", m_reactive);
    cmd.Parse (argc, argv);
}
void MeshTest::CreateNodes () {
    std::string m_root;
    double m_txpower = 18.0; // dbm
    // Calculate m_nnodes stations random topology
    nodes.Create (m_nnodes);
    // Setup mobility - static rectangle topology
    MobilityHelper mobility;
    mobility.SetPositionAllocator ("ns3::RandomRectanglePositionAllocator",
        "X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=450.0]"), // 720 Proactive
        "Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=150.0]") // 360 Proactive
    );
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (nodes);
    // Configure YansWifiChannel
    YansWifiPhyHelper WifiPhy;
    // WifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (-89.0) ); // depreacted
    WifiPhy.Set ("RxSensitivity", DoubleValue (-92.0) );
    WifiPhy.Set ("CcaEdThreshold", DoubleValue (-58.0) );
    // WifiPhy.Set ("CcaMode1Threshold", DoubleValue (-62.0) ); // depreacted
    WifiPhy.Set ("TxGain", DoubleValue (1.0) );
    WifiPhy.Set ("RxGain", DoubleValue (1.0) );
    WifiPhy.Set ("TxPowerLevels", UintegerValue (1) );
    WifiPhy.Set ("TxPowerEnd", DoubleValue (m_txpower) );
    WifiPhy.Set ("TxPowerStart", DoubleValue (m_txpower) );
    WifiPhy.Set ("RxNoiseFigure", DoubleValue (7.0) );
    WifiPhy.Set ("Antennas", UintegerValue(2)); // Default: 1
    YansWifiChannelHelper WifiChannel;
    WifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    WifiChannel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel","Exponent",StringValue ("2.7"));
    WifiPhy.SetChannel (WifiChannel.Create ());
    // Configure the parameters of the Peer Link
    Config::SetDefault ("ns3::dot11s::PeerLink::MaxBeaconLoss", UintegerValue (20));
    Config::SetDefault ("ns3::dot11s::PeerLink::MaxRetries", UintegerValue (4));
    Config::SetDefault ("ns3::dot11s::PeerLink::MaxPacketFailure", UintegerValue (5));
    // Configure the parameters of the Peer Management Protocol
    Config::SetDefault ("ns3::dot11s::PeerManagementProtocol::EnableBeaconCollisionAvoidance", BooleanValue (false));
    // Configure the parameters of the HWMP
    Config::SetDefault ("ns3::dot11s::HwmpProtocol::Dot11MeshHWMPactivePathTimeout",TimeValue (Seconds (100)));
    Config::SetDefault ("ns3::dot11s::HwmpProtocol::Dot11MeshHWMPactiveRootTimeout",TimeValue (Seconds (100)));
    Config::SetDefault ("ns3::dot11s::HwmpProtocol::Dot11MeshHWMPmaxPREQretries",UintegerValue (5));
    Config::SetDefault ("ns3::dot11s::HwmpProtocol::UnicastPreqThreshold",UintegerValue (10));
    Config::SetDefault ("ns3::dot11s::HwmpProtocol::UnicastDataThreshold",UintegerValue (5));
    Config::SetDefault ("ns3::dot11s::HwmpProtocol::DoFlag", BooleanValue (true));
    Config::SetDefault ("ns3::dot11s::HwmpProtocol::RfFlag", BooleanValue (true));
    // Create mesh helper and set stack installer to it
    // Stack installer creates all needed protocols and install them to device
    mesh = MeshHelper::Default ();
    mesh.SetStandard (WIFI_STANDARD_80211a);
    mesh.SetMacType ("RandomStart", TimeValue (Seconds(m_randomStart)));
    mesh.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
        "DataMode",StringValue ("OfdmRate6Mbps"),
        "RtsCtsThreshold", UintegerValue (2500)
    );
    // Set number of interfaces - default is single-interface mesh point
    mesh.SetNumberOfInterfaces (m_nIfaces);
    if (m_reactive == 1) {
        //If reactive mode is on, we do not use "Root" attribute
        m_root = "Reactive mode";
        mesh.SetStackInstaller (m_stack);
    } else {
        //If proactive mode is on, we define node 6 as root
        m_root = "00:00:00:00:00:06";
        mesh.SetStackInstaller (m_stack, "Root", Mac48AddressValue(Mac48Address (m_root.c_str ())));
    }
    std::cout << "\n\t root: " << m_root << "\n";
    //If multiple channels is activated
    if (m_chan) {
        mesh.SetSpreadInterfaceChannels (MeshHelper::SPREAD_CHANNELS);
    } else {
        mesh.SetSpreadInterfaceChannels (MeshHelper::ZERO_CHANNEL);
    }
    // Install protocols and return container if MeshPointDevices
    meshDevices = mesh.Install (WifiPhy, nodes);
}
void MeshTest::CreateNodesC ()
{
    // We only create the extra node
    nodesC.Create (1);
    YansWifiPhyHelper WifiPhy;
    YansWifiChannelHelper WifiChannel;
    WifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    WifiChannel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel","Exponent", StringValue ("2.7"));
    WifiPhy.SetChannel (WifiChannel.Create ());
    meshC = MeshHelper::Default ();
    meshC.SetStackInstaller(m_stack);
    meshDevicesC = meshC.Install (WifiPhy, nodesC);
    // This extra node is placed far away to not interfere
    Vector3D n1_posC (m_step*3, m_step*3, m_step*3);

    ListPositionAllocator myListPositionAllocatorC;
    myListPositionAllocatorC.Add(n1_posC);
    MobilityHelper mobilityC;
    mobilityC.SetPositionAllocator(&myListPositionAllocatorC);
    mobilityC.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobilityC.Install (nodesC);
}
void MeshTest::InstallInternetStack ()
{
    //Install the internet protocol stack on all nodes
    InternetStackHelper internetStack;
    internetStack.Install (nodes);
    //Assign IP addresses to the devices interfaces (m_nIfaces)
    Ipv4AddressHelper address;
    address.SetBase ("192.168.1.0", "255.255.255.0");
    interfaces = address.Assign (meshDevices);
}
void MeshTest::InstallInternetStackC ()
{
    //Install the internet protocol stack on all nodes
    InternetStackHelper internetStack;
    internetStack.Install (nodesC);
    //Assign IP addresses to the extra node is also from another network
    Ipv4AddressHelper address;
    address.SetBase ("192.168.2.0", "255.255.255.0");
    interfacesC = address.Assign (meshDevicesC);
}
void MeshTest::InstallApplicationRandom ()
{
    int ir=0;
    int m_source, m_dest, m_dest_port;
    char num [5];
    char onoff [10];
    char sink [9];
    double start_time, stop_time, duration;

    // Set the parameters of the onoff application
    Config::SetDefault ("ns3::OnOffApplication::PacketSize",UintegerValue (m_packetSize));
    Config::SetDefault ("ns3::OnOffApplication::DataRate", StringValue (m_txrate));
    ApplicationContainer apps [m_nconn];
    Ptr<UniformRandomVariable> rand_nodes = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> rand_port = CreateObject<UniformRandomVariable>();
    // 50 seconds for transitori are left at the beginning.
    Ptr<UniformRandomVariable> a = CreateObject<UniformRandomVariable>();
    for (int i = 0; i < m_nconn; i++){
        start_time = a->GetValue(50, m_totalTime - 15);
        Ptr<ExponentialRandomVariable> b = CreateObject<ExponentialRandomVariable>();
        duration = b->GetValue(25, 30)+1; // 30 Proactive
        // If the exponential variable gives us a value that added to the start time
        // is greater than the maximum permitted, this is changed for the maximum
        // 10 seconds are left at the end to calculate well the statistics of each flow
        if ( (start_time + duration) > (m_totalTime - 10)) {
            stop_time = m_totalTime-10;
        } else {
            stop_time = start_time + duration;
        }
        //create different names for the connections
        //(we can not use vectors for OnOffHelper)
        strcpy(onoff,"onoff");
        strcpy(sink,"sink");
        sprintf(num,"%d",i);
        strcat(onoff,num);
        strcat(sink,num);
        //set random variables of the destination (server) and destination port.
        m_dest = rand_nodes->GetInteger (0,m_nnodes-1);
        m_dest_port = rand_port->GetInteger (49000,49100);
        //change the destination to the root if more root traffic is required

        if (ir < m_nconnR){
            m_dest = 5;
            ir++;
        }

        // Set random variables of the source (client).
        // Client and server can not be the same node.
        m_source = rand_nodes->GetInteger (0,m_nnodes-1);
        while (m_source == m_dest){
            m_source = rand_nodes->GetInteger (0,m_nnodes-1);
        }
        // Plot the connection values
        std::cout << "\n Node "<< m_source << " to " << m_dest;
        std::cout << "\n Start_time: " << start_time << "s";
        std::cout << "\n Stop_time: " << stop_time << "s";
        // Define UDP traffic for the onoff application
        OnOffHelper onoff ("ns3::UdpSocketFactory",Address (InetSocketAddress(interfaces.GetAddress (m_dest), m_dest_port)));
        onoff.SetAttribute ("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
        onoff.SetAttribute ("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
        AddressValue remoteAddress(InetSocketAddress(interfaces.GetAddress(m_dest), 49001));
        onoff.SetAttribute ("Remote", remoteAddress);
        apps[i] = onoff.Install (nodes.Get(m_source));
        apps[i].Start (Seconds (start_time));
        apps[i].Stop (Seconds (stop_time));
        // Create a packet sink to receive these packets
        Ptr<Socket> sink = SetupPacketReceive(interfaces.GetAddress(m_dest), nodes.Get(m_dest));
    }
}
Ptr<Socket> MeshTest::SetupPacketReceive(Ipv4Address addr, Ptr<Node> node) {
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    Ptr<Socket> sink = Socket::CreateSocket(node, tid);
    InetSocketAddress local = InetSocketAddress(addr, 49001);
    sink->Bind(local);
    return sink;
}
int MeshTest::Run ()
{
    CreateNodes ();
    InstallInternetStack ();
    // In this mesh implementation when the proactive mode is used, when creating
    // the root node this counts as if there was another node. Thus, when using
    // the seed for the random variables it gives different values in reactive
    // and proactive mode. To solve this, in reactive mode a fake mode that does
    // not communicate neither interfere is created.
    if (m_reactive == 1) {
        CreateNodesC ();
        InstallInternetStackC ();
        std::cout << "\n Node: Installing extra node to compensate the root\n";
    }

    InstallApplicationRandom ();

    // Install FlowMonitor on all nodes
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    m_timeStart=clock();

    //NetAnim
    AnimationInterface anim ("output/xml/hwmp-diff-modes-animation.xml");
    anim.SetMobilityPollInterval (Seconds (0.5));

    Simulator::Schedule (Seconds(m_totalTime), & MeshTest::Report, this);
    Simulator::Stop (Seconds (m_totalTime));
    Simulator::Run ();

    // Define variables to calculate the metrics
    int k=0;
    int totaltxPackets = 0;
    int totalrxPackets = 0;
    double totaltxbytes = 0;
    double totalrxbytes = 0;
    double totaldelay = 0;
    double totalrxbitrate = 0;
    double difftx, diffrx;
    double pdf_value, rxbitrate_value, txbitrate_value, delay_value;
    double pdf_total, rxbitrate_total, delay_total;
    //Print per flow statistics
    monitor->CheckForLostPackets ();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>
    (flowmon.GetClassifier ());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();

    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
        difftx = i->second.timeLastTxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds();
        diffrx = i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstRxPacket.GetSeconds();
        pdf_value = (double) i->second.rxPackets / (double) i->second.txPackets * 100;
        txbitrate_value = (double) i->second.txBytes * 8 / 1024 / difftx;
        if (i->second.rxPackets != 0) {
            rxbitrate_value = (double)i->second.rxPackets * m_packetSize * 8 / 1024 / diffrx;
            delay_value = (double) i->second.delaySum.GetSeconds() /(double) i->second.rxPackets;
        } else{
            rxbitrate_value = 0;
            delay_value = 0;
        }
        // We are only interested in the metrics of the data flows
        if ((!t.destinationAddress.IsSubnetDirectedBroadcast("255.255.255.0"))) {
            k++;
            // Plot the statistics for each data flow
            std::cout << "\nFlow " << k << " (" << t.sourceAddress << " -> "
            << t.destinationAddress << ")\n";
            //std::cout << "Tx Packets: " << i->second.txPackets << "\n";
            //std::cout << "Rx Packets: " << i->second.rxPackets << "\n";
            //std::cout << "Lost Packets: " << i->second.lostPackets << "\n";
            //std::cout << "Dropped Packets: " << i->second.packetsDropped.size() << "\n";
            std::cout << "PDF: " << pdf_value << " %\n";
            std::cout << "Average delay: " << delay_value << " s\n";
            std::cout << "Rx bitrate: " << rxbitrate_value << " kbps\n";
            std::cout << "Tx bitrate: " << txbitrate_value << " kbps\n\n";
            // Acumulate for average statistics
            totaltxPackets += i->second.txPackets;
            totaltxbytes += i->second.txBytes;
            totalrxPackets += i->second.rxPackets;
            totaldelay += i->second.delaySum.GetSeconds();
            totalrxbitrate += rxbitrate_value;
            totalrxbytes += i->second.rxBytes;
        }
    }
    // Average all nodes statistics
    if (totaltxPackets != 0) {
        pdf_total = (double) totalrxPackets / (double) totaltxPackets * 100;
    } else {
        pdf_total = 0;
    }

    if (totalrxPackets != 0) {
        rxbitrate_total = totalrxbitrate;
        delay_total = (double) totaldelay / (double) totalrxPackets;
    } else {
        rxbitrate_total = 0;
        delay_total = 0;
    }
    //print all nodes statistics
    std::cout << "\nTotal PDF: " << pdf_total << " %\n";
    std::cout << "Total Rx bitrate: " << rxbitrate_total << " kbps\n";
    std::cout << "Total Delay: " << delay_total << " s\n";
    //print all nodes statistics in files
    std::ostringstream os;
    std::string prename = "output/txt/HWMP-P-vs-R-P-Do-Enable";
    os << prename << "_PDF.txt";
    std::ofstream of (os.str().c_str(), std::ios::out | std::ios::app);
    of << pdf_total << "\n";
    of.close ();
    std::ostringstream os2;
    os2 << prename << "_Delay.txt";
    std::ofstream of2 (os2.str().c_str(), std::ios::out | std::ios::app);
    of2 << delay_total << "\n";
    of2.close ();
    std::ostringstream os3;
    os3 << prename << "_Throu.txt";
    std::ofstream of3 (os3.str().c_str(), std::ios::out | std::ios::app);
    of3 << rxbitrate_total << "\n";
    of3.close ();
    Simulator::Destroy ();
    m_timeEnd=clock();
    m_timeTotal=(m_timeEnd - m_timeStart)/(double) CLOCKS_PER_SEC;
    std::cout << "\n*** Simulation time: " << m_timeTotal << "s\n\n";
    return 0;
}
void MeshTest::Report () {
    // Using this function we print detailed statistics of each mesh point device
    // These statistics are used later with an AWK files to extract routing metrics
    unsigned n (0);
    for (NetDeviceContainer::Iterator i = meshDevices.Begin ();
    i != meshDevices.End (); ++i, ++n) {
        std::ostringstream os;
        //os << "mp-report1-" << n << ".xml";
        os << "output/xml/hwmp-diff-modes-report.xml";
        std::ofstream of;
        of.open (os.str().c_str(), std::ios::out | std::ios::app);
        if (! of.is_open ())
        {
            std::cerr << "Error: Can’t open file " << os.str() << "\n";
            return;
        }
        mesh.Report (*i, of);
        of.close ();
    }
    n = 0;
}
int main (int argc, char *argv[])
{
    MeshTest t;
    t.Configure (argc, argv);
    return t.Run();
}
