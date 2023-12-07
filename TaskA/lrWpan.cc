#include <fstream>
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/mobility-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/propagation-module.h"
#include "ns3/sixlowpan-module.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/csma-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv6-flow-classifier.h"
#include "ns3/flow-monitor-helper.h"
#include <ns3/lr-wpan-error-model.h>
#include "ns3/netanim-module.h"
using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WpanTaskA2");

class MyApp : public Application
{
public:
  MyApp ();
  virtual ~MyApp ();

  /**
   * Register this type.
   * \return The TypeId.
   */
  static TypeId GetTypeId (void);
  void Setup (Ptr<Socket> socket, Address address, uint32_t packetSize, DataRate dataRate);

private:
  virtual void StartApplication (void);
  virtual void StopApplication (void);

  void ScheduleTx (void);
  void SendPacket (void);

  Ptr<Socket>     m_socket;
  Address         m_peer;
  uint32_t        m_packetSize;
  DataRate        m_dataRate;
  EventId         m_sendEvent;
  bool            m_running;
  uint32_t        m_packetsSent;
};

MyApp::MyApp ()
  : m_socket (0),
    m_peer (),
    m_packetSize (0),
    m_dataRate (0),
    m_sendEvent (),
    m_running (false),
    m_packetsSent (0)
{
}

MyApp::~MyApp ()
{
  m_socket = 0;
}

/* static */
TypeId MyApp::GetTypeId (void)
{
  static TypeId tid = TypeId ("MyApp")
    .SetParent<Application> ()
    .SetGroupName ("Tutorial")
    .AddConstructor<MyApp> ()
    ;
  return tid;
}

void
MyApp::Setup (Ptr<Socket> socket, Address address, uint32_t packetSize, DataRate dataRate)
{
  m_socket = socket;
  m_peer = address;
  m_packetSize = packetSize;
  m_dataRate = dataRate;
}

void
MyApp::StartApplication (void)
{
  m_running = true;
  m_packetsSent = 0;
  if (InetSocketAddress::IsMatchingType (m_peer))
    {
      m_socket->Bind ();
    }
  else
    {
      m_socket->Bind6 ();
    }
  m_socket->Connect (m_peer);
  SendPacket ();
}

void
MyApp::StopApplication (void)
{
  m_running = false;

  if (m_sendEvent.IsRunning ())
    {
      Simulator::Cancel (m_sendEvent);
    }

  if (m_socket)
    {
      m_socket->Close ();
    }
}

void
MyApp::SendPacket (void)
{
  Ptr<Packet> packet = Create<Packet> (m_packetSize);
  m_socket->Send (packet);
  ScheduleTx();
}

void
MyApp::ScheduleTx (void)
{
  if (m_running)
    {
      Time tNext (Seconds (m_packetSize * 8 / static_cast<double> (m_dataRate.GetBitRate ())));
      m_sendEvent = Simulator::Schedule (tNext, &MyApp::SendPacket, this);
    }
}



bool tracing = true;
// uint16_t nSourceNodes=1;
uint32_t nWsnNodes;
uint16_t sinkPort=9;
uint32_t mtu_bytes = 180;
uint32_t tcp_adu_size;
uint64_t data_mbytes = 0;
double start_time = 0;
double duration = 100.0;
double stop_time;
bool sack = true;
std::string filePrefix;
std::string delay = "0.01ms";
uint32_t nodes = 40;
uint32_t coverage_area = 100;
uint32_t num_flows = 40;
uint32_t packets_sent_per_sec = 200;
std::string flow_mon_filename = "demo";
std::string dataRate = "5kbps";

Ptr<LrWpanErrorModel>  lrWpanError;

AsciiTraceHelper asciiTraceHelper;

Ptr<PacketSink> sink;                         /* Pointer to the packet sink application */
uint64_t lastTotalRx = 0;                     /* The value of the last total received bytes */


void
CalculateThroughput ()
{
  Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */
  //double cur = (sink->GetTotalRx () - lastTotalRx) * (double) 8 / 1e5;     /* Convert Application RX Packets to KBits. */
  //*throughput_stream->GetStream () << now.GetSeconds () << "\t" << cur << std::endl;
  // std::cout << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;
  lastTotalRx = sink->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput);
}


void initialize(int argc, char** argv) {
  CommandLine cmd (__FILE__);
  cmd.AddValue ("nodes", "Number of nodes", nodes);
  cmd.AddValue ("flows", "Number of flows", num_flows);
  cmd.AddValue ("packets_sent_per_sec", "Packets sent per second for each sender ", packets_sent_per_sec);
  cmd.AddValue ("tracing", "turn on log components", tracing);
  cmd.AddValue ("filename", "Flow Monitor Filename", flow_mon_filename);
  // cmd.AddValue ("nSourceNodes", "turn on log components", nSourceNodes);
  // cmd.AddValue ("tracedNode", "turn on log components", tracedNode);
  cmd.Parse (argc, argv);

  if( tracing ) {
    LogComponentEnable("PacketSink", LOG_LEVEL_INFO);
    LogComponentEnable("WpanTaskA2", LOG_LEVEL_INFO);
  }

  nWsnNodes = nodes + 1;
  // tracedNode = std::max(1, tracedNode);
  filePrefix = "wpan-sourceCount" + std::to_string(nodes);

  // 2 MB of TCP buffer
  Config::SetDefault ("ns3::TcpSocket::RcvBufSize", UintegerValue (1 << 21));
  Config::SetDefault ("ns3::TcpSocket::SndBufSize", UintegerValue (1 << 21));
  Config::SetDefault ("ns3::TcpSocketBase::Sack", BooleanValue (sack));

  Header* temp_header = new Ipv6Header ();
  uint32_t ip_header = temp_header->GetSerializedSize ();
  delete temp_header;
  temp_header = new TcpHeader ();
  uint32_t tcp_header = temp_header->GetSerializedSize ();
  delete temp_header;
  tcp_adu_size = mtu_bytes - 20 - (ip_header + tcp_header);

  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (tcp_adu_size));

  //NS_LOG_INFO("TCP ADU SIZE : " << tcp_adu_size);

  stop_time = start_time + duration;

  lrWpanError = CreateObject<LrWpanErrorModel> ();

  std::string bw_val = "";
    if( packets_sent_per_sec == 100 ){
      bw_val = "10kbps";
    }
    else if( packets_sent_per_sec == 200){
      bw_val = "20kbps";
    }
    else if( packets_sent_per_sec == 300){
      bw_val = "30kbps";
    }
    else if( packets_sent_per_sec == 400){
      bw_val = "40kbps";
    }
    else if( packets_sent_per_sec == 500){
      bw_val = "50kbps";
    }

    dataRate = bw_val;
}

int main (int argc, char** argv) {
  initialize(argc, argv);


  std::ofstream flowmonitorOutput;
  std::ofstream summaryOutput;


  //static Ptr<OutputStreamWrapper> flowStream = asciiTraceHelper.CreateFileStream("Task_A2/" + flow_mon_filename);

  Packet::EnablePrinting ();

  NodeContainer wsnNodes;
  wsnNodes.Create (nWsnNodes);

  // ----------------------------------- static mobility model ---------------------------------------
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (100),
                                 "DeltaY", DoubleValue (100),
                                 "GridWidth", UintegerValue (10),
                                 "LayoutType", StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  
  mobility.Install (wsnNodes);
  Config::SetDefault ("ns3::RangePropagationLossModel::MaxRange", DoubleValue (80));

  LrWpanHelper lrWpanHelper;
  NetDeviceContainer lrwpanDevices = lrWpanHelper.Install (wsnNodes);

  lrWpanHelper.AssociateToPan (lrwpanDevices, 0);

  InternetStackHelper internetv6;
  internetv6.Install (wsnNodes);
  
  SixLowPanHelper sixLowPanHelper;
  NetDeviceContainer sixLowPanDevices = sixLowPanHelper.Install (lrwpanDevices);

  CsmaHelper csmaHelper;
 
  Ipv6AddressHelper ipv6;
  

  ipv6.SetBase (Ipv6Address ("2001:f00d::"), Ipv6Prefix (64));
  Ipv6InterfaceContainer wsnDeviceInterfaces;
  wsnDeviceInterfaces = ipv6.Assign (sixLowPanDevices);
  wsnDeviceInterfaces.SetForwarding (0, true);
  wsnDeviceInterfaces.SetDefaultRouteInAllNodes (0);

  for (uint32_t i = 0; i < sixLowPanDevices.GetN (); i++) {
    Ptr<NetDevice> dev = sixLowPanDevices.Get (i);
    dev->SetAttribute ("UseMeshUnder", BooleanValue (true));
    dev->SetAttribute ("MeshUnderRadius", UintegerValue (10));
  }
  int  i = 1;
  for( uint32_t f = 1; f <= num_flows; f++ ) {
  
    Address sinkAddress = Inet6SocketAddress (Inet6SocketAddress (wsnDeviceInterfaces.GetAddress (0, 1), sinkPort) );
    Ptr<Socket> ns3TcpSocket = Socket::CreateSocket (wsnNodes.Get (i), TcpSocketFactory::GetTypeId ());

    Ptr<MyApp> app = CreateObject<MyApp> ();
    app->Setup (ns3TcpSocket, sinkAddress, tcp_adu_size, DataRate (dataRate));
    wsnNodes.Get (i)->AddApplication (app);
    app->SetStartTime (Seconds (start_time));
    app->SetStopTime (Seconds (stop_time));

    PacketSinkHelper sinkApp ("ns3::TcpSocketFactory",
    Inet6SocketAddress (Ipv6Address::GetAny (), sinkPort));
    sinkApp.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
    ApplicationContainer sinkApps = sinkApp.Install (wsnNodes.Get(0));
    
    sink = StaticCast<PacketSink> (sinkApps.Get (0));
    
    sinkApps.Start (Seconds (0.0));
    sinkApps.Stop (Seconds (stop_time));

    sinkPort++;
    i++;
    if( i % ( nodes + 1 ) == 0){
      i = 1;
    }
  }



  Simulator::Schedule (Seconds (1.1), &CalculateThroughput);

  // AnimationInterface anim ("Task_A2/mywpan.xml"); 
    flowmonitorOutput.open ("flowSummary.dat", std::ios::out);
    summaryOutput.open ("CoverageArea.dat", std::ios_base::app);
   // Flow monitor
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

    Simulator::Stop (Seconds (stop_time));
    Simulator::Run ();

    ///////////////////////// Calculate throughput and FlowMonitor statistics ////////////////////////////////////
    int j=0;
    float AvgThroughput = 0;
    Time Jitter;
    Time Delay;
    // variables for output measurement
    uint32_t SentPackets = 0;
    uint32_t ReceivedPackets = 0;
    uint32_t LostPackets = 0;

    Ptr<Ipv6FlowClassifier> classifier = DynamicCast<Ipv6FlowClassifier> (flowmon.GetClassifier6 ());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();
    for (auto iter = stats.begin (); iter != stats.end (); ++iter) {
        Ipv6FlowClassifier::FiveTuple t = classifier->FindFlow (iter->first); 
            // classifier returns FiveTuple in correspondance to a flowID
       flowmonitorOutput << "----Flow ID:" <<iter->first << std::endl;
    flowmonitorOutput << "Src Addr" <<t.sourceAddress << " -- Dst Addr "<< t.destinationAddress<< std::endl;
    flowmonitorOutput << "Sent Packets=" <<iter->second.txPackets<< std::endl;
    flowmonitorOutput << "Received Packets =" <<iter->second.rxPackets<< std::endl;
    flowmonitorOutput << "Lost Packets =" <<iter->second.txPackets-iter->second.rxPackets<< std::endl;
    flowmonitorOutput << "Packet delivery ratio =" <<iter->second.rxPackets*100.0/iter->second.txPackets << "%"<< std::endl;
    flowmonitorOutput << "Packet loss ratio =" << (iter->second.txPackets-iter->second.rxPackets)*100.0/iter->second.txPackets << "%"<< std::endl;
    // flowmonitorOutput << "Packet lost diff way = "<< iter->second.lostPackets;
    flowmonitorOutput << "Delay =" <<iter->second.delaySum<< std::endl;
    flowmonitorOutput << "Jitter =" <<iter->second.jitterSum<< std::endl;
    flowmonitorOutput << "Throughput =" <<iter->second.rxBytes * 8.0/(iter->second.timeLastRxPacket.GetSeconds()-iter->second.timeFirstTxPacket.GetSeconds())/1024<<"Kbps"<< std::endl;

        SentPackets = SentPackets +(iter->second.txPackets);
        ReceivedPackets = ReceivedPackets + (iter->second.rxPackets);
        LostPackets = LostPackets + (iter->second.txPackets-iter->second.rxPackets);
        if(iter->second.rxPackets != 0){
          AvgThroughput = AvgThroughput + (iter->second.rxBytes * 8.0/(iter->second.timeLastRxPacket.GetSeconds()-iter->second.timeFirstTxPacket.GetSeconds())/1024);
        }
        
        Delay = Delay + (iter->second.delaySum);
        Jitter = Jitter + (iter->second.jitterSum);

        j = j + 1;

    }
    //NS_LOG_INFO("Average Throughput : " << AvgThroughput);
    AvgThroughput = AvgThroughput/j;
    flowmonitorOutput << "--------Total Results of the simulation----------"<<std::endl;
  flowmonitorOutput << "Total sent packets  =" << SentPackets<< std::endl;
  flowmonitorOutput << "Total Received Packets =" << ReceivedPackets<< std::endl;
  flowmonitorOutput << "Total Lost Packets =" << LostPackets<< std::endl;
  flowmonitorOutput << "Packet Loss ratio =" << ((LostPackets*100.00)/SentPackets)<< "%"<< std::endl;
  flowmonitorOutput << "Packet delivery ratio =" << ((ReceivedPackets*100.00)/SentPackets)<< "%"<< std::endl;
  flowmonitorOutput << "Average Throughput =" << AvgThroughput<< "Kbps"<< std::endl;
  flowmonitorOutput << "End to End Delay =" << Delay<< std::endl;
  flowmonitorOutput << "End to End Jitter delay =" << Jitter<< std::endl;
  flowmonitorOutput << "Total Flow id " << j<< std::endl;
    //-------------------------------- end flow monitor output----------------------------------------

    summaryOutput << coverage_area<< "\t" << AvgThroughput<< "Kbps"<< "\t" << Delay << "\t" << ((ReceivedPackets*100.00)/SentPackets) << "\t" << ((LostPackets*100.00)/SentPackets) << std::endl;
     summaryOutput.close();
     flowmonitorOutput.close();

  Simulator::Destroy ();

  return 0;
}

