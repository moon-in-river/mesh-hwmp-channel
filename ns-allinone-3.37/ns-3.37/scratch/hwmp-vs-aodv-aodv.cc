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
#include "ns3/netanim-module.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/aodv-module.h"

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
    int m_xSize; //x size of the grid
    int m_ySize; //y size of the grid
    double m_step; //separation between nodes
    double m_totalTime;
    uint16_t m_packetSize;
    bool m_pcap;
    std::string m_txrate;
    double m_txrate_dob;
    //to calculate the lenght of the simulation
    float m_timeTotal, m_timeStart, m_timeEnd;
    // List of network nodes
    NodeContainer nodes;
    // List of all wifi devices
    NetDeviceContainer wifiDevices;
    //Addresses of interfaces:
    Ipv4InterfaceContainer interfaces;
    // MeshHelper. Report is not static methods
    WifiHelper wifi_aodv;
private:
    // Create nodes and setup their mobility
    void CreateNodes ();
    // Install internet m_stack on nodes
    void InstallInternetStack ();
    // Install applications randomly
    void InstallApplicationRandom ();
    // Setup the receiving socket in a Sink Node
    Ptr<Socket> SetupPacketReceive(Ipv4Address addr, Ptr<Node> node);
};
MeshTest::MeshTest () :
    m_xSize (5),
    m_ySize (5),
    m_step (170),
    m_totalTime (240),
    m_packetSize (1024),
    m_pcap (false),
    m_txrate ("150kbps"),
    m_txrate_dob (150) //needed in kbps for the trace file
{
}
void MeshTest::Configure (int argc, char *argv[]) {
    CommandLine cmd;
    cmd.AddValue ("m_xSize", "m_xSize", m_xSize);
    cmd.AddValue ("m_ySize", "m_ySize", m_ySize);
    cmd.AddValue ("m_txrate", "m_txrate", m_txrate);
    cmd.AddValue ("m_txrate_dob", "m_txrate_dob", m_txrate_dob);
    cmd.Parse (argc, argv);
}
void MeshTest::CreateNodes () {
    int i, j;
    double m_txpower = 18.0; // dbm
    // Calculate m_ySize*m_xSize stations grid topology
    double position_x = 0;
    double position_y = 0;
    ListPositionAllocator myListPositionAllocator;
    for (i = 1; i <= m_xSize; i++){
        for (j = 1; j <= m_ySize; j++){
            std::cout << "Node at x = " << position_x << ", y = " << position_y << "\n";
            Vector3D n_pos (position_x, position_y, 0.0);
            myListPositionAllocator.Add(n_pos);
            position_y += m_step;
        }
        position_y = 0;
        position_x += m_step;
    }
    // Create the nodes
    nodes.Create (m_xSize*m_ySize);
    // Configure YansWifiChannel
    YansWifiPhyHelper WifiPhy;
    // WifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (-89.0) );
    WifiPhy.Set ("RxSensitivity", DoubleValue (-101.0) );  //Default: -101.0
    WifiPhy.Set ("CcaEdThreshold", DoubleValue (-58.0) );       //Default: -62.0
    // WifiPhy.Set ("CcaMode1Threshold", DoubleValue (-62.0) );
    WifiPhy.Set ("TxGain", DoubleValue (1.0) );
    WifiPhy.Set ("RxGain", DoubleValue (1.0) );
    WifiPhy.Set ("TxPowerLevels", UintegerValue (1) );
    WifiPhy.Set ("TxPowerEnd", DoubleValue (m_txpower) );
    WifiPhy.Set ("TxPowerStart", DoubleValue (m_txpower) );
    WifiPhy.Set ("RxNoiseFigure", DoubleValue (7.0) );
    YansWifiChannelHelper WifiChannel;
    WifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    WifiChannel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel","Exponent",StringValue ("2.7"));
    WifiPhy.SetChannel (WifiChannel.Create ());
    wifi_aodv.SetStandard (WIFI_STANDARD_80211a);
    wifi_aodv.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
        "DataMode", StringValue ("OfdmRate6Mbps"),
        "RtsCtsThreshold", UintegerValue (2500)
    );
    // Install protocols and returnf container
    // NqosWaveMacHelper wifiMac = NqosWaveMacHelper::Default();
    WifiMacHelper wifiMac;
    // wifiMac.SetType ("ns3::OcbWifiMac");
    wifiMac.SetType ("ns3::AdhocWifiMac");
    wifiDevices = wifi_aodv.Install (WifiPhy, wifiMac, nodes);
    // Place the protocols in the positions calculated before
    MobilityHelper mobility;
    mobility.SetPositionAllocator(&myListPositionAllocator);
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (nodes);
}
void MeshTest::InstallInternetStack () {
    //configure AODV
    AodvHelper aodv;
    aodv.Set ("AllowedHelloLoss", UintegerValue (20));
    aodv.Set ("HelloInterval", TimeValue (Seconds (3)));
    aodv.Set ("RreqRetries", UintegerValue (5));
    aodv.Set ("ActiveRouteTimeout", TimeValue (Seconds (100)));
    aodv.Set ("DestinationOnly", BooleanValue (true));
    //Install the internet protocol stack on all nodes
    InternetStackHelper internetStack;
    internetStack.SetRoutingHelper (aodv);
    internetStack.Install (nodes);
    //Assign IP addresses to the devices interfaces (m_nIfaces)
    Ipv4AddressHelper address;
    address.SetBase ("192.168.1.0", "255.255.255.0");
    interfaces = address.Assign (wifiDevices);
}
void MeshTest::InstallApplicationRandom () {
    // Create as many connections as nodes has the grid
    int m_nconn = m_xSize * m_ySize;
    int i=0;
    int m_source, m_dest, m_dest_port;
    char num [2];
    char onoff [7];
    char sink [6];
    double start_time, stop_time, duration;
    // Set the parameters of the onoff application
    Config::SetDefault ("ns3::OnOffApplication::PacketSize",UintegerValue (m_packetSize));
    Config::SetDefault ("ns3::OnOffApplication::DataRate", StringValue (m_txrate));
    ApplicationContainer apps [m_nconn];
    Ptr<UniformRandomVariable> rand_nodes = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> rand_port = CreateObject<UniformRandomVariable>();
    // 50 seconds for transitori are left at the beginning.
    Ptr<UniformRandomVariable> a = CreateObject<UniformRandomVariable>();
    for (i = 0; i < m_nconn; i++){
        start_time = a->GetValue(50, m_totalTime - 15);
        Ptr<ExponentialRandomVariable> b = CreateObject<ExponentialRandomVariable>();
        duration = b->GetValue(30, 50)+1;
        // If the exponential variable gives us a value that added to the start time
        // is greater than the maximum permitted, this is changed for the maximum
        // 10 seconds are left at the end to calculate well the statistics of each flow
        if ( (start_time + duration) > (m_totalTime - 10)){
            stop_time = m_totalTime-10;
        }else {
            stop_time = start_time + duration;
        }
        // Create different names for the connections
        // (we can not use vectors for OnOffHelper)
        strcpy(onoff,"onoff");
        strcpy(sink,"sink");
        sprintf(num,"%d",i);
        strcat(onoff,num);
        strcat(sink,num);
        // Set random variables of the destination (server) and destination port.
        m_dest = rand_nodes->GetInteger (0,m_ySize*m_xSize-1);
        m_dest_port = rand_port->GetInteger (49000,49100);
        // Set random variables of the source (client)
        m_source = rand_nodes->GetInteger (0,m_ySize*m_xSize-1);
        // Client and server can not be the same node.
        while (m_source == m_dest){
            m_source = rand_nodes->GetInteger (0,m_ySize*m_xSize-1);
        }
        // Plot the connection values
        std::cout << "\n Node "<< m_source << " to " << m_dest;
        std::cout << "\n Start_time: " << start_time << "s";
        std::cout << "\n Stop_time: " << stop_time << "s\n";
        // Define UDP traffic for the onoff application
        OnOffHelper onoff ("ns3::UdpSocketFactory", Address (InetSocketAddress(interfaces.GetAddress (m_dest), m_dest_port)));
        onoff.SetAttribute ("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
        onoff.SetAttribute ("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
        AddressValue remoteAddress(InetSocketAddress(interfaces.GetAddress(m_dest), 49001));
        onoff.SetAttribute ("Remote", remoteAddress);
        apps[i] = onoff.Install (nodes.Get(m_source));
        apps[i].Start (Seconds (start_time));
        apps[i].Stop (Seconds (stop_time));
        // Create a packet sink to receive the packets
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
int MeshTest::Run () {
    CreateNodes ();
    InstallInternetStack ();
    InstallApplicationRandom ();
    // Install FlowMonitor on all nodes
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    m_timeStart=clock();
    Simulator::Stop (Seconds (m_totalTime));
    Simulator::Run ();
    // Define variables to calculate the metrics
    int k=0;
    int totaltxPackets = 0;
    int totalrxPackets = 0;
    int totaltxPacketsR = 0;
    int totalrxPacketsR = 0;
    double totaltxbytes = 0;
    double totalrxbytes = 0;
    double totaltxbytesR = 0;
    double totalrxbytesR = 0;
    double totaldelay = 0;
    double totalrxbitrate = 0;
    double difftx, diffrx;
    double pdf_value, rxbitrate_value, txbitrate_value, delay_value;
    double pdf_total, rxbitrate_total, delay_total;
    double RL_rx_pack, RL_rx_bytes;
    // double RL_tx_pack, RL_tx_bytes;
    //Print per flow statistics
    monitor->CheckForLostPackets ();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier ());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
        difftx = i->second.timeLastTxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds();
        diffrx = i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstRxPacket.GetSeconds();
        pdf_value = (double) i->second.rxPackets / (double) i->second.txPackets * 100;
        txbitrate_value = (double) i->second.txBytes * 8 / 1024 / difftx;
        std::cout << "rx packets = " << i->second.rxPackets << "\n";
        if (i->second.rxPackets != 0){
            rxbitrate_value = (double) i->second.rxPackets * m_packetSize * 8 / 1024 / diffrx;
            delay_value = (double) i->second.delaySum.GetSeconds() / (double) i->second.rxPackets;
        } else{
            rxbitrate_value = 0;
            delay_value = 0;
        }
        // We are only interested in the metrics of the data flows. This AODV
        // implementation create other flows with routing information at low bitrates,
        // so a margin is defined to ensure that only our data flows are filtered.
        if (
            (!t.destinationAddress.IsSubnetDirectedBroadcast("255.255.255.0")) &&
            (txbitrate_value > m_txrate_dob/1.2) &&
            (rxbitrate_value < m_txrate_dob*1.2))
        {
            k++;
            std::cout << "\nFlow " << k << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
            //std::cout << "Tx Packets: " << i->second.txPackets << "\n";
            //std::cout << "Rx Packets: " << i->second.rxPackets << "\n";
            //std::cout << "Lost Packets: " << i->second.lostPackets << "\n";
            //std::cout << "Dropped Packets: " << i->second.packetsDropped.size() << "\n";
            std::cout << "PDF: " << pdf_value << " %\n";
            std::cout << "Average delay: " << delay_value << "s\n";
            std::cout << "Rx bitrate: " << rxbitrate_value << " kbps\n";
            std::cout << "Tx bitrate: " << txbitrate_value << " kbps\n";
            // Acumulate for average statistics
            totaltxPackets += i->second.txPackets;
            totaltxbytes += i->second.txBytes;
            totalrxPackets += i->second.rxPackets;
            totaldelay += i->second.delaySum.GetSeconds();
            totalrxbitrate += rxbitrate_value;
            totalrxbytes += i->second.rxBytes;
        } else{
            totaltxbytesR += i->second.txBytes;
            totalrxbytesR += i->second.rxBytes;
            totaltxPacketsR += i->second.txPackets;
            totalrxPacketsR += i->second.rxPackets;
        }
    }
    //Average all nodes statistics
    if (totaltxPackets != 0){
        pdf_total = (double) totalrxPackets / (double) totaltxPackets * 100;
        // RL_tx_pack = (double) totaltxPacketsR / (double) totaltxPackets;
        // RL_tx_bytes = totaltxbytesR / totaltxbytes;
    } else{
        pdf_total = 0;
        // RL_tx_pack = 0;
        // RL_tx_bytes = 0;
    }
    if (totalrxPackets != 0){
        rxbitrate_total = totalrxbitrate;
        delay_total = (double) totaldelay / (double) totalrxPackets;
        RL_rx_pack = (double) totalrxPacketsR / (double) totalrxPackets;
        RL_rx_bytes = totalrxbytesR / totalrxbytes;
    } else{
        rxbitrate_total = 0;
        delay_total = 0;
        RL_rx_pack = 0;
        RL_rx_bytes = 0;
    }
    // Print all nodes statistics
    std::cout << "\nTotal PDF: " << pdf_total << " %\n";
    std::cout << "Total Rx bitrate: " << rxbitrate_total << " kbps\n";
    std::cout << "Total Delay: " << delay_total << " s\n";
    // Print all nodes statistics in files
    std::ostringstream os;
    os << "1_AODV_PDF.txt";
    std::ofstream of (os.str().c_str(), std::ios::out | std::ios::app);
    of << pdf_total << "\n";
    std::ostringstream os2;
    os2 << "1_AODV_Delay.txt";
    std::ofstream of2 (os2.str().c_str(), std::ios::out | std::ios::app);
    of2 << delay_total << "\n";
    std::ostringstream os3;
    os3 << "1_AODV_Throu.txt";
    std::ofstream of3 (os3.str().c_str(), std::ios::out | std::ios::app);
    of3 << rxbitrate_total << "\n";
    std::ostringstream os5;
    os5 << "1_AODV_RL_RxBytes.txt";
    std::ofstream of5 (os5.str().c_str(), std::ios::out | std::ios::app);
    of5 << RL_rx_bytes << "\n";
    std::ostringstream os7;
    os7 << "1_AODV_RL_RxPack.txt";
    std::ofstream of7 (os7.str().c_str(), std::ios::out | std::ios::app);
    of7 << RL_rx_pack << "\n";
    of.close (); of2.close (); of3.close (); of5.close (); of7.close ();
    Simulator::Destroy ();
    m_timeEnd=clock();
    m_timeTotal=(m_timeEnd - m_timeStart)/(double) CLOCKS_PER_SEC;
    std::cout << "\n*** Simulation time: " << m_timeTotal << "s\n\n";
    return 0;
}
int main (int argc, char *argv[]) {
    MeshTest t;
    t.Configure (argc, argv);
    return t.Run();
}
