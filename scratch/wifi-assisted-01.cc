/*
 * Copyright (c) 2015-2019 IMDEA Networks Institute
 * Author: Hany Assasa <hany.assasa@gmail.com>
 */
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "common-functions.h"

#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-mac-header.h"

#include <math.h>       /* atan */
#include <complex.h>       /* atan */
#include <assert.h>
#include "Eigen/Dense"
#include <stdio.h>
#include <iomanip>

/**
 * Simulation Objective:
 * This script is used to evaluate the throughput achieved using a simple allocation of a static
 * service period for a communication from DMG PCP/AP to a DMG STA.
 *
 * Network Topology:
 * The scenario consists of a single DMG STA and one DMG PCP/AP as following:
 *
 *
 *                  DMG PCP/AP (0,0)          DMG STA (+1,0)
 *
 *
 * Simulation Description:
 * Once the statio have associated successfully with the DMG PCP/AP. The DMG PCP/AP allocates a signle
 * static service period for communication as following:
 *
 * SP: DMG PCP/AP -----> DMG STA (SP Length = 3.2 ms)
 *
 * Running the Simulation:
 * To run the script with the default parameters:
 * ./waf --run "evaluate_simple_service_period"
 *
 * To run the script with different duration for the service period e.g. SP1=10ms:
 * ./waf --run "evaluate_service_period_udp --spDuration=10000"
 *
 * Simulation Output:
 * The simulation generates the following traces:
 * 1. PCAP traces for each station. From the PCAP files, we can see that data transmission takes place
 * during its SP. In addition, we can notice in the announcement of the two Static Allocation Periods
 * inside each DMG Beacon.
 */

NS_LOG_COMPONENT_DEFINE ("EvaluateSimpleServicePeriod");

using namespace Eigen;
using namespace ns3;
using namespace std;

/**  Application Variables **/
Ptr<PacketSink> packetSink;

/* Network Nodes */
Ptr<WifiNetDevice> apWifiNetDevice;
Ptr<WifiNetDevice> staWifiNetDevice;
Ptr<DmgApWifiMac> apWifiMac;
Ptr<DmgStaWifiMac> staWifiMac;
NetDeviceContainer staDevices;

Ptr<WifiNetDevice> myApWifiNetDevice;
Ptr<ApWifiMac> myApWifiMac;
NetDeviceContainer myStaDevices;

/*** Service Periods ***/
const Time beaconInterval = MilliSeconds(1);
const double c = 3e8;
const double frequency = 5.18e9;
const double wavelength = c/frequency;


uint16_t spDuration = 3200;               /* The duration of the allocated service period in MicroSeconds */
SectorID apSectorId = 1;
SectorID staSectorId = 1;
uint8_t slsCounter = 0;
int64_t slsMilliSec = 0;
Time sweepTime = Time(0);



const int M = 4;
const int snapshots = 100;
const int source_no = 1;

double arrival_time[M][snapshots][source_no];
double arrival_power[M][snapshots][source_no];
vector<int> arrival_counter (M, 0);


map<Mac48Address, uint> addrMap; 

ArrayXd music_algo(double* time, double* power,int numb_ante, int snap, int sour);
vector<int> FindLocalMax(vector<double> &vec);


void
BIStarted (Ptr<DmgApWifiMac> wifiMac, Mac48Address address)
{
  sweepTime += apWifiMac->CalculateBeamformingTrainingDuration(8, 8);
  NS_LOG_UNCOND ("BIStarted, sweepTime = " << sweepTime);
}

void
SLSCompleted (Ptr<DmgWifiMac> wifiMac, Mac48Address address, ChannelAccessPeriod accessPeriod,
              BeamformingDirection beamformingDirection, bool isInitiatorTxss, bool isResponderTxss,
              SECTOR_ID sectorId, ANTENNA_ID antennaId)
{
  NS_LOG_UNCOND ("DMG STA " << wifiMac->GetAddress () << " completed SLS phase with DMG STA " << address);
  std::cout << "The best antenna configuration is SectorID=" << uint32_t (sectorId)
                << ", AntennaID=" << uint32_t (antennaId) << std::endl;

  NS_LOG_UNCOND ("slscompleted" );
}

void
ActiveTxSectorIDChanged (Ptr<DmgWifiMac> wifiMac, SectorID oldSectorID, SectorID newSectorID)
{
  if (wifiMac == apWifiMac) {
    
    NS_LOG_UNCOND ("DMG AP: " << wifiMac->GetAddress () << " , SectorID=" << uint16_t (newSectorID));
    
    if (newSectorID != ::apSectorId)
      wifiMac->GetCodebook()-> SetActiveTxSectorID(apSectorId);
    return;

  } else if (wifiMac == staWifiMac) {

    NS_LOG_UNCOND ("DMG STA: " << wifiMac->GetAddress () << " , SectorID=" << uint16_t (newSectorID)  );
    
    if (newSectorID != ::staSectorId)
      wifiMac->GetCodebook()-> SetActiveTxSectorID(staSectorId);
    return;

  } else {
    assert(false);
  }
  
  return;
}

void
SetSectors()
{
  Ptr<Codebook> apCodebook = StaticCast<DmgApWifiMac> (apWifiNetDevice->GetMac ())->GetCodebook();
  Ptr<Codebook> staCodebook = StaticCast<DmgStaWifiMac> (staWifiNetDevice->GetMac ())->GetCodebook();
  NS_LOG_UNCOND ("set sectors ");
  
  if (apCodebook->GetActiveTxSectorID() != ::apSectorId)
    apCodebook ->SetActiveTxSectorID(::apSectorId);

  if (staCodebook->GetActiveTxSectorID() != ::staSectorId)
    staCodebook ->SetActiveTxSectorID(::staSectorId);

  Simulator::Schedule (MilliSeconds(1), &SetSectors);
  
  return ;
}

void
CourseChange (std::string context, Ptr<const MobilityModel> model)
{
  ns3::Vector position = model->GetPosition ();
  NS_LOG_UNCOND (context <<  " x = " << position.x << ", y = " << position.y);
  return;
}


void
MyRxBegin (Ptr<WifiNetDevice> wifinet, Ptr<const Packet> p, double rxPowerW) { 

  WifiMacHeader head;
  p->PeekHeader (head);
  Mac48Address dst = head.GetAddr1 ();
  Mac48Address src = head.GetAddr2 ();
  Mac48Address broadcast = Mac48Address("ff:ff:ff:ff:ff:ff");
  Mac48Address rx = wifinet->GetMac() ->GetAddress();
  uint16_t seq = head.GetSequenceNumber();
  bool skipCond = !(src == myApWifiMac->GetAddress() && dst == broadcast );
  if (skipCond) return;

  if (addrMap.empty()) {
    for (uint i = 0; i < myStaDevices.GetN(); ++i){
      Mac48Address tmpaddr = StaticCast<WifiNetDevice>(myStaDevices.Get(i)) ->GetMac() ->GetAddress();
      addrMap.insert(pair<Mac48Address, uint> (tmpaddr, i));
    }    
  } else {
    for (auto it = addrMap.begin(); it != addrMap.end(); ++it){
      NS_LOG_UNCOND (" first = " << it ->first << " second = " << it->second);
    }
  }

  uint no = addrMap.at(rx);
  
  int idx = -1;
  if (arrival_counter[no] < snapshots) idx = arrival_counter[no];
  else idx = snapshots - 1;

  arrival_time[no][ idx ][0] = double(Simulator::Now().GetPicoSeconds()) / 1000.0;
  arrival_power[no][ idx ][0] = rxPowerW;
  ++arrival_counter[no];
  NS_LOG_UNCOND ("map = " << no << " idx " << idx);
  
  vector<bool> fullcheck; 
  for (auto it = arrival_counter.begin(); it != arrival_counter.end(); ++it){
    bool isfull = (*it >= snapshots);
    fullcheck.push_back(isfull);
  }
  bool flush = true;
  for (auto it = fullcheck.begin(); it != fullcheck.end(); ++it){
    if (!(*it)) flush = false;
  }


  if (flush) {

    ArrayXd analysisArr = music_algo(arrival_time[0][0], arrival_power[0][0], M, snapshots, source_no);    
    int maxDeg = std::distance(analysisArr.begin(), std::max_element(analysisArr.begin(), analysisArr.end()));
    NS_LOG_UNCOND ("max = " << maxDeg) ;

    staSectorId = maxDeg / 45 + 1 ;
    apSectorId = staSectorId + 4 ;
    NS_LOG_UNCOND ("staSectorId = " << int(staSectorId) << " apSectorId = " << int(apSectorId));
    Simulator::ScheduleNow(&SetSectors);
    

    arrival_counter = vector<int> (M, 0);
  }
  double now = double (Simulator::Now().GetPicoSeconds());
  NS_LOG_UNCOND ( setprecision(20) << now << " rx = " << rx << " seq : " << seq << " packet received from src = " << src << " with power = " << rxPowerW );
  NS_LOG_UNCOND ( setprecision(3) );
}


void
CalculateThroughput (Ptr<PacketSink> sink, uint64_t lastTotalRx, double averageThroughput)
{
  Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */
  double cur = (sink->GetTotalRx() - lastTotalRx) * (double) 8/(100000 - sweepTime.GetMilliSeconds()*1000);     /* Convert Application RX Packets to MBits. */
  NS_LOG_UNCOND ("throughput: " << now.GetSeconds () << '\t' << cur << '\t' << "sweepTime = " << sweepTime.GetMilliSeconds()) ;
  sweepTime = Time(0);
  lastTotalRx = sink->GetTotalRx ();
  averageThroughput += cur;
  
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput, sink, lastTotalRx, averageThroughput);
}

void
StationAssociated (Mac48Address address, uint16_t aid)
{
  std::cout << "DMG STA: " << staWifiMac->GetAddress () << " associated with DMG PCP/AP: " << address << std::endl;
  std::cout << "Association ID (AID) = " << aid << std::endl;
  std::cout << "Schedule Static Service Period (DMG PCP/AP ----> DMG STA)" << std::endl;
  /* Schedule Static Periods */
  apWifiMac->AllocateSingleContiguousBlock (1, SERVICE_PERIOD_ALLOCATION, true,
                                            AID_AP,
                                            aid,
                                            0, spDuration);
}

/**
 * Callback method to log the number of packets in the Wifi MAC Queue.
 */
// void
// QueueOccupancyChange (Ptr<OutputStreamWrapper> file, uint32_t oldValue, uint32_t newValue)
// {
//   std::ostream *output = file->GetStream ();
//   *output << Simulator::Now ().GetPicoSeconds () << "," << newValue << endl;
// }

int
main (int argc, char *argv[])
{
  uint32_t packetSize = 1448;                   /* Transport Layer Payload size in bytes. */
  string dataRate = "800Mbps";                  /* Application Layer Data Rate. */
  uint64_t maxPackets = 0;                      /* The maximum number of packets to transmit. */
  uint32_t msduAggregationSize = 7935;          /* The maximum aggregation size for A-MSDU in Bytes. */
  uint32_t queueSize = 10000;                   /* Wifi Mac Queue Size. */
  string phyMode = "DMG_MCS12";                 /* Type of the Physical Layer. */
  bool verbose = false;                         /* Print Logging Information. */
  double simulationTime = 10;                   /* Simulation time in seconds. */
  bool pcapTracing = false;                     /* PCAP Tracing is enabled or not. */
  
  
  Time::SetResolution(Time::Unit(Time::PS));
  /* Command line argument parser setup. */
  CommandLine cmd;
  cmd.AddValue ("packetSize", "Payload size in bytes", packetSize);
  cmd.AddValue ("dataRate", "Data rate for OnOff Application", dataRate);
  cmd.AddValue ("maxPackets", "The maximum number of packets to transmit", maxPackets);
  cmd.AddValue ("msduAggregation", "The maximum aggregation size for A-MSDU in Bytes", msduAggregationSize);
  cmd.AddValue ("queueSize", "The size of the Wifi Mac Queue", queueSize);
  cmd.AddValue ("spDuration", "The duration of service period in MicroSeconds", spDuration);
  cmd.AddValue ("phyMode", "802.11ad PHY Mode", phyMode);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("simulationTime", "Simulation time in seconds", simulationTime);
  cmd.AddValue ("pcap", "Enable PCAP Tracing", pcapTracing);
  cmd.Parse (argc, argv);

  /* Global params: no fragmentation, no RTS/CTS, fixed rate for all packets */
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("999999"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("999999"));

  /**** DmgWifiHelper is a meta-helper ****/
  DmgWifiHelper wifi;

  /* Basic setup */
  wifi.SetStandard (WIFI_PHY_STANDARD_80211ad);

  /* Turn on logging */
  if (verbose)
    {
      wifi.EnableLogComponents ();
      LogComponentEnable ("EvaluateSimpleServicePeriod", LOG_LEVEL_ALL);
    }

  /**** Set up Channel ****/
  DmgWifiChannelHelper wifiChannel ;
  /* Simple propagation delay model */
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  /* Friis model with standard-specific wavelength */
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel", "Frequency", DoubleValue (60.48e9));

  /**** Setup physical layer ****/
  DmgWifiPhyHelper wifiPhy = DmgWifiPhyHelper::Default ();
  /* Nodes will be added to the channel we set up earlier */
  wifiPhy.SetChannel (wifiChannel.Create ());
  /* All nodes transmit at 10 dBm == 10 mW, no adaptation */
  wifiPhy.Set ("TxPowerStart", DoubleValue (10.0));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (10.0));
  wifiPhy.Set ("TxPowerLevels", UintegerValue (1));
  /* Set operating channel */
  wifiPhy.Set ("ChannelNumber", UintegerValue (2));
  /* Sensitivity model includes implementation loss and noise figure */
  wifiPhy.Set ("CcaMode1Threshold", DoubleValue (-79));
  wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (-79 + 3));
  /* Set default algorithm for all nodes to be constant rate */
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "ControlMode", StringValue (phyMode),
                                                                "DataMode", StringValue (phyMode));

  /* Make four nodes and set them up with the phy and the mac */
  NodeContainer wifiNodes;
  wifiNodes.Create (5);
  Ptr<Node> apNode = wifiNodes.Get (0);
  Ptr<Node> staNode = wifiNodes.Get (1);
  Ptr<Node> staNode2 = wifiNodes.Get (2);
  Ptr<Node> staNode3 = wifiNodes.Get (3);
  Ptr<Node> staNode4 = wifiNodes.Get (4);

  NodeContainer staNodes;
  staNodes.Add(staNode);
  staNodes.Add(staNode2);
  staNodes.Add(staNode3);
  staNodes.Add(staNode4);

  /* Add a DMG upper mac */
  DmgWifiMacHelper wifiMac = DmgWifiMacHelper::Default ();

  /* Install DMG PCP/AP Node */
  Ssid ssid = Ssid ("ServicePeriod");
  wifiMac.SetType ("ns3::DmgApWifiMac",
                   "Ssid", SsidValue(ssid),
                   "BE_MaxAmpduSize", UintegerValue (0),
                   "BE_MaxAmsduSize", UintegerValue (msduAggregationSize),
                   "SSSlotsPerABFT", UintegerValue (8), "SSFramesPerSlot", UintegerValue (8),
                   "BeaconInterval", TimeValue (MicroSeconds (1024*5)),
                   "ATIPresent", BooleanValue (false));

  /* Set Analytical Codebook for the DMG Devices */
  wifi.SetCodebook ("ns3::CodebookAnalytical",
                    "CodebookType", EnumValue (SIMPLE_CODEBOOK),
                    "Antennas", UintegerValue (1),
                    "Sectors", UintegerValue (8));

  NetDeviceContainer apDevice;
  apDevice = wifi.Install (wifiPhy, wifiMac, apNode);

  /* Install DMG STA Nodes */
  wifiMac.SetType ("ns3::DmgStaWifiMac",
                   "Ssid", SsidValue (ssid), "ActiveProbing", BooleanValue (false),
                   "BE_MaxAmpduSize", UintegerValue (0),
                   "BE_MaxAmsduSize", UintegerValue (msduAggregationSize));

  staDevices = wifi.Install (wifiPhy, wifiMac, staNode);









  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  phy.SetChannel (channel.Create ());
  
  WifiHelper mywifi;
  mywifi.SetRemoteStationManager ("ns3::AarfWifiManager");

  WifiMacHelper mac;
  Ssid ssid2 = Ssid ("ns-3-ssid");
  mac.SetType ("ns3::StaWifiMac",
               "Ssid", SsidValue (ssid2),
               "ActiveProbing", BooleanValue (false));

  
  myStaDevices = mywifi.Install (phy, mac, staNodes);

  for (NetDeviceContainer::Iterator it = myStaDevices.Begin(); it != myStaDevices.End(); ++it){
    Ptr<WifiNetDevice> wifinetdev = StaticCast<WifiNetDevice> (*it);
    wifinetdev ->GetPhy() ->TraceConnectWithoutContext ("PhyRxBegin2", MakeBoundCallback (&MyRxBegin, wifinetdev));
  }

  mac.SetType ("ns3::ApWifiMac",
               "Ssid", SsidValue (ssid2),
               "BeaconInterval", TimeValue (beaconInterval)
               );

  NetDeviceContainer apWifiDev;
  apWifiDev = mywifi.Install (phy, mac, apNode);


  myApWifiNetDevice = StaticCast<WifiNetDevice> (apWifiDev.Get (0));
  myApWifiMac = StaticCast<ApWifiMac> (myApWifiNetDevice->GetMac ());






  /* Setting mobility model */
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  
  for (double i = 0 ; i < staNodes.GetN(); ++i){
    positionAlloc ->Add (ns3::Vector(i * 0.5 * wavelength, 0.0, 0.0));
  }

  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (staNodes);

  positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (ns3::Vector (-2.0, 2.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);

  // mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (-5, 5, 0.5, 5))
                            );
  mobility.Install (apNode);




  std::ostringstream oss;
  oss <<
    "/NodeList/" << apNode->GetId() <<
    "/$ns3::MobilityModel/CourseChange";
  
  Config::Connect (oss.str (), MakeCallback (&CourseChange));





  /* Internet stack*/
  InternetStackHelper stack;
  stack.Install (wifiNodes);

  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.255.255.0");
  Ipv4InterfaceContainer apInterface;
  apInterface = address.Assign (apDevice);
  Ipv4InterfaceContainer staInterfaces;
  staInterfaces = address.Assign (staDevices);

  /* Populate routing table */
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  /* We do not want any ARP packets */
  PopulateArpCache ();
  // ArpCache::SetAliveTimeout()
  // ArpCache::Entry *entry = 
              //   ArpCache::Entry * entry = arp->Add(ipAddr);
              // entry->MarkWaitReply(0);
              // entry->MarkAlive(addr);

  /* Install Simple UDP Server on the STA */
  PacketSinkHelper sinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), 9999));
  ApplicationContainer sinks = sinkHelper.Install (staNode);
  packetSink = StaticCast<PacketSink> (sinks.Get (0));

  /** STA Node Variables **/
  uint64_t staNodeLastTotalRx = 0;
  double staNodeAverageThroughput = 0;

  /* Install Simple UDP Transmiter on the DMG PCP/AP */
  Ptr<OnOffApplication> onoff;
  ApplicationContainer srcApp;
  OnOffHelper src ("ns3::UdpSocketFactory", InetSocketAddress (staInterfaces.GetAddress (0), 9999));
  src.SetAttribute ("MaxPackets", UintegerValue (maxPackets));
  src.SetAttribute ("PacketSize", UintegerValue (packetSize));
  src.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1e6]"));
  src.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
  src.SetAttribute ("DataRate", DataRateValue (DataRate (dataRate)));
  srcApp = src.Install (apNode);
  onoff = StaticCast<OnOffApplication> (srcApp.Get (0));

  /* Schedule Applications */
  srcApp.Start (Seconds (1.0));
  srcApp.Stop (Seconds (simulationTime));
  sinks.Start (Seconds (1.0));

  /* Schedule Throughput Calulcations */
  Simulator::Schedule (Seconds (1.1), &CalculateThroughput, packetSink, staNodeLastTotalRx, staNodeAverageThroughput);
  Simulator::Schedule (Seconds (1.1), &SetSectors);

  /* Set Maximum number of packets in WifiMacQueue */
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/BE_EdcaTxopN/Queue/MaxPackets", UintegerValue (queueSize));

  /* Connect Wifi MAC Queue Occupancy */
  // AsciiTraceHelper asciiTraceHelper;
  // Ptr<OutputStreamWrapper> queueOccupanyStream;
  /* Trace DMG PCP/AP MAC Queue Changes */
  // queueOccupanyStream = asciiTraceHelper.CreateFileStream ("Traces/AccessPointMacQueueOccupany.txt");
  // Config::ConnectWithoutContext ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/BE_EdcaTxopN/Queue/OccupancyChanged",
  //                                MakeBoundCallback (&QueueOccupancyChange, queueOccupanyStream));

  /* Enable Traces */
  if (pcapTracing)
    {
      wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
      wifiPhy.EnablePcap ("Traces/AccessPoint", apDevice, false);
      wifiPhy.EnablePcap ("Traces/STA", staDevices.Get (0), false);
    }

  /* Install FlowMonitor on all nodes */
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

  /** Connect Traces **/
  apWifiNetDevice = StaticCast<WifiNetDevice> (apDevice.Get (0));
  staWifiNetDevice = StaticCast<WifiNetDevice> (staDevices.Get (0));
  apWifiMac = StaticCast<DmgApWifiMac> (apWifiNetDevice->GetMac ());
  staWifiMac = StaticCast<DmgStaWifiMac> (staWifiNetDevice->GetMac ());
  apWifiMac->TraceConnectWithoutContext ("StationAssociated", MakeCallback (&StationAssociated));
  
  apWifiMac->TraceConnectWithoutContext ("SLSCompleted", MakeBoundCallback (&SLSCompleted, apWifiMac));
  staWifiMac->TraceConnectWithoutContext ("SLSCompleted", MakeBoundCallback (&SLSCompleted, staWifiMac));

  apWifiMac->TraceConnectWithoutContext ("BIStarted", MakeBoundCallback (&BIStarted, apWifiMac));

  
  apWifiMac->GetCodebook ()->TraceConnectWithoutContext ("ActiveTxSectorID", MakeBoundCallback (&ActiveTxSectorIDChanged, apWifiMac));
  staWifiMac->GetCodebook ()->TraceConnectWithoutContext ("ActiveTxSectorID", MakeBoundCallback (&ActiveTxSectorIDChanged, staWifiMac));




//   for (double i = 1.2 ; i < simulationTime; i += 0.1){
//     Simulator::Schedule (Seconds (i), &DmgWifiMac::InitiateTxssCbap, staWifiMac, apWifiMac->GetAddress ());
//   }




  Simulator::Stop (Seconds (simulationTime + 0.101));
  Simulator::Run ();
  Simulator::Destroy ();

  /* Print per flow statistics */
  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
      std::cout << "Flow " << i->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")" << std::endl;;
      std::cout << "  Tx Packets: " << i->second.txPackets << std::endl;
      std::cout << "  Tx Bytes:   " << i->second.txBytes << std::endl;
      std::cout << "  TxOffered:  " << i->second.txBytes * 8.0 / ((simulationTime - 1) * 1e6)  << " Mbps" << std::endl;;
      std::cout << "  Rx Packets: " << i->second.rxPackets << std::endl;;
      std::cout << "  Rx Bytes:   " << i->second.rxBytes << std::endl;
      std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / ((simulationTime - 1) * 1e6)  << " Mbps" << std::endl;;
    }

  /* Print Application Layer Results Summary */
  std::cout << "\nApplication Layer Statistics:" << std::endl;;
  std::cout << "  Tx Packets: " << onoff->GetTotalTxPackets () << std::endl;
  std::cout << "  Tx Bytes:   " << onoff->GetTotalTxBytes () << std::endl;
  std::cout << "  Rx Packets: " << packetSink->GetTotalReceivedPackets () << std::endl;
  std::cout << "  Rx Bytes:   " << packetSink->GetTotalRx () << std::endl;
  std::cout << "  Throughput: " << packetSink->GetTotalRx () * 8.0 / ((simulationTime - 1) * 1e6) << " Mbps" << std::endl;

  return 0;
}



ArrayXd music_algo(double *time, double *power,int numb_ante, int snap, int sour){

    const std::complex<double> z(0, 1);
    int M = numb_ante; /* Number of antenna elements */
    double c = 3e8; 
    double wavelength = c/5.18e9; /* In units of meters */
    double frequency = c/wavelength;
    double seperation = wavelength*0.5;
    int angular_profile = 180; /* in number of intervals from 0 to 180*/

    ArrayXd angle(angular_profile);
    for (int i = 0; i<angular_profile; i++){
        angle(i) = i*M_PI/(angular_profile-1);
    }

    MatrixXcd sensor_matrix(numb_ante,snap);
    sensor_matrix = MatrixXcd::Zero(numb_ante,snap);
    ArrayXcd calibration(snap);

    MatrixXcd sensor_covariance(M,M);
    sensor_covariance = MatrixXcd::Zero(M,M);

    MatrixXcd E_matrix(M,M);
     E_matrix = MatrixXcd::Zero(M,M);

    MatrixXd posantenna(2,M);
    for (int i = 0; i<M; i++){
        posantenna(0,i) = i*seperation;
        posantenna(1,i) = 0;
    }
    
    MatrixXd anti_sym(M,M);
    anti_sym = MatrixXd::Zero(M,M);
    for(int i = 0; i<M;i++){
        for(int j = 0; j<M;j++){
            if (j+i == M-1){
                anti_sym(i,j) = 1;
            }
        }
    }

    MatrixXcd atheta(M,angular_profile);
    for (int i = 0; i<M;i++){
        for (int j = 0; j<angular_profile;j++){
            Vector2d vec(cos(angle(j)),sin(angle(j)));
            double phase = -vec.dot(posantenna.col(i))/wavelength*2.0*M_PI;
            atheta(i,j) = exp(z*phase);
        }
    }
    
    
    for (int i  = 0; i<numb_ante; i++){
        for (int j = 0; j<snap;j++){
            for (int k = 0; k<sour;k++){
                double t = *(time+(i*snap + j)*sour+k);
                double p = *(power+(i*snap + j)*sour+k);
                sensor_matrix(i,j) = sensor_matrix(i,j) + exp(z*2.0*M_PI*t*frequency*1e-9)*p;
            }
        }
    }
    for (int j = 0; j<snap; j++){
        calibration(j) = exp(-z*arg(sensor_matrix(0,j)));
    }
    for (int i = 0; i<numb_ante;i++){
        sensor_matrix.row(i) << sensor_matrix.row(i).array() * calibration.transpose();
    }

    for (int j = 0; j<snap; j++){
    sensor_covariance = sensor_covariance + (anti_sym * sensor_matrix.col(j).adjoint().transpose())*(anti_sym * sensor_matrix.col(j).adjoint().transpose()).adjoint()/snap + sensor_matrix.col(j) * (sensor_matrix.col(j)).adjoint()/snap;
    }

    SelfAdjointEigenSolver<MatrixXcd> eigensolver(sensor_covariance);
    if (eigensolver.info() != Success) abort();    

    int number_of_sig = M;
    for (int i = 0; i<M; i++){
        if(eigensolver.eigenvalues()(i) < eigensolver.eigenvalues().maxCoeff()/100){
            E_matrix.col(i) = eigensolver.eigenvectors().col(i);
            number_of_sig--;            

        }       
    }

    ArrayXd spatial_metric(angular_profile); 

    for(int i = 0;i<angular_profile;i++){
        spatial_metric(i) = 1/(atheta.col(i).adjoint()*E_matrix*E_matrix.adjoint()*atheta.col(i))(0,0).real();
    }
    spatial_metric = spatial_metric/spatial_metric.maxCoeff();


    return spatial_metric; 
}

vector<int> FindLocalMax(vector<double> &vec){
  vector<int> ret;

  for (auto it = vec.begin(); it != vec.end(); ++it){
    if (it == vec.begin() || it+1 == vec.end()) continue;
    if (*(it -1) < *it && *(it + 1) < *it) 
      ret.push_back(std::distance(vec.begin(), it));
  }

  return ret;
}