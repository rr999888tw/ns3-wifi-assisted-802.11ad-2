/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 */

#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-mac-header.h"
#include "Eigen/Dense"
#include <complex>       
#include <map> 

// Default Network Topology
//
//   Wifi 10.1.3.0
//                 AP
//  *    *    *    *
//  |    |    |    |    10.1.1.0
// n5   n6   n7   n0 -------------- n1   n2   n3   n4
//                   point-to-point  |    |    |    |
//                                   ================
//                                     LAN 10.1.2.0

using namespace Eigen;
using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("ThirdScriptExample");


ArrayXd music_algo(double* time, double* power,int numb_ante, int snap, int sour);

const uint M = 3;
const uint snapshots = 10;
const uint source_no = 1;

double arrival_time[M][snapshots][source_no];
double arrival_power[M][snapshots][source_no];

map<Address, uint> addrMap; 


void
CourseChange (std::string context, Ptr<const MobilityModel> model)
{
  ns3::Vector position = model->GetPosition ();
  NS_LOG_UNCOND (context <<
    " x = " << position.x << ", y = " << position.y);
}

void
MyRxBegin (Ptr<WifiNetDevice> net, Ptr<const Packet> p, double rxPowerW) { 
  
  WifiMacHeader head;
  p->PeekHeader (head);
  Address dest = net->GetAddress();  
  uint16_t seq = head.GetSequenceNumber();

  if (addrMap.find(dest) == addrMap.end()){
    addrMap.insert(pair<Address, uint> (dest, addrMap.size()));
  }
  
  NS_LOG_UNCOND (Simulator::Now() << " seq : " << seq << " packet received at " << dest << " with power = " << rxPowerW );
  NS_LOG_UNCOND ( addrMap.at(dest) );

}

int 
main (int argc, char *argv[])
{


  Eigen::MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << m << std::endl;




  bool verbose = true;
  uint32_t nCsma = 3;
  uint32_t nWifi = 3;
  bool tracing = false;

  CommandLine cmd;
  cmd.AddValue ("nCsma", "Number of \"extra\" CSMA nodes/devices", nCsma);
  cmd.AddValue ("nWifi", "Number of wifi STA devices", nWifi);
  cmd.AddValue ("verbose", "Tell echo applications to log if true", verbose);
  cmd.AddValue ("tracing", "Enable pcap tracing", tracing);

  cmd.Parse (argc,argv);

  // The underlying restriction of 18 is due to the grid position
  // allocator's configuration; the grid layout will exceed the
  // bounding box if more than 18 nodes are provided.
  if (nWifi > 18)
    {
      std::cout << "nWifi should be 18 or less; otherwise grid layout exceeds the bounding box" << std::endl;
      return 1;
    }

  if (verbose)
    {
      LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
      LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);
    }

  NodeContainer p2pNodes;
  p2pNodes.Create (2);

  PointToPointHelper pointToPoint;
  pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
  pointToPoint.SetChannelAttribute ("Delay", StringValue ("2ms"));

  NetDeviceContainer p2pDevices;
  p2pDevices = pointToPoint.Install (p2pNodes);

  NodeContainer csmaNodes;
  csmaNodes.Add (p2pNodes.Get (1));
  csmaNodes.Create (nCsma);

  CsmaHelper csma;
  csma.SetChannelAttribute ("DataRate", StringValue ("100Mbps"));
  csma.SetChannelAttribute ("Delay", TimeValue (NanoSeconds (6560)));

  NetDeviceContainer csmaDevices;
  csmaDevices = csma.Install (csmaNodes);

  NodeContainer wifiStaNodes;
  wifiStaNodes.Create (nWifi);
  NodeContainer wifiApNode = p2pNodes.Get (0);

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  phy.SetChannel (channel.Create ());

  WifiHelper wifi;
  wifi.SetRemoteStationManager ("ns3::AarfWifiManager");

  WifiMacHelper mac;
  Ssid ssid = Ssid ("ns-3-ssid");
  mac.SetType ("ns3::StaWifiMac",
               "Ssid", SsidValue (ssid),
               "ActiveProbing", BooleanValue (false));

  NetDeviceContainer staDevices;
  staDevices = wifi.Install (phy, mac, wifiStaNodes);

  for (NetDeviceContainer::Iterator it = staDevices.Begin(); it != staDevices.End(); ++it){

    Ptr<WifiNetDevice> wifinetdev = StaticCast<WifiNetDevice> (*it);
    wifinetdev ->GetPhy() ->TraceConnectWithoutContext ("PhyRxBegin2", MakeBoundCallback (&MyRxBegin, wifinetdev));
  }





  mac.SetType ("ns3::ApWifiMac",
               "Ssid", SsidValue (ssid));

  NetDeviceContainer apDevices;
  apDevices = wifi.Install (phy, mac, wifiApNode);
  

  MobilityHelper mobility;

  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (5.0),
                                 "DeltaY", DoubleValue (10.0),
                                 "GridWidth", UintegerValue (3),
                                 "LayoutType", StringValue ("RowFirst"));

  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (-50, 50, -50, 50)));
  mobility.Install (wifiStaNodes);

  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiApNode);

  InternetStackHelper stack;
  stack.Install (csmaNodes);
  stack.Install (wifiApNode);
  stack.Install (wifiStaNodes);

  Ipv4AddressHelper address;

  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer p2pInterfaces;
  p2pInterfaces = address.Assign (p2pDevices);

  address.SetBase ("10.1.2.0", "255.255.255.0");
  Ipv4InterfaceContainer csmaInterfaces;
  csmaInterfaces = address.Assign (csmaDevices);

  address.SetBase ("10.1.3.0", "255.255.255.0");
  address.Assign (staDevices);
  address.Assign (apDevices);

  UdpEchoServerHelper echoServer (9);

  ApplicationContainer serverApps = echoServer.Install (csmaNodes.Get (nCsma));
  serverApps.Start (Seconds (1.0));
  serverApps.Stop (Seconds (10.0));

  UdpEchoClientHelper echoClient (csmaInterfaces.GetAddress (nCsma), 9);
  echoClient.SetAttribute ("MaxPackets", UintegerValue (1));
  echoClient.SetAttribute ("Interval", TimeValue (Seconds (1.0)));
  echoClient.SetAttribute ("PacketSize", UintegerValue (1024));

  ApplicationContainer clientApps = 
    echoClient.Install (wifiStaNodes.Get (nWifi - 1));
  clientApps.Start (Seconds (2.0));
  clientApps.Stop (Seconds (10.0));

  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  Simulator::Stop (Seconds (10.0));

  if (tracing == true)
    {
      pointToPoint.EnablePcapAll ("third");
      phy.EnablePcap ("third", apDevices.Get (0));
      csma.EnablePcap ("third", csmaDevices.Get (0), true);
    }


  std::ostringstream oss;
  oss <<
    "/NodeList/" << wifiStaNodes.Get (nWifi - 1)->GetId () <<
    "/$ns3::MobilityModel/CourseChange";
  
  Config::Connect (oss.str (), MakeCallback (&CourseChange));

  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}







ArrayXd music_algo(double *time, double *power,int numb_ante, int snap, int sour){
    const std::complex<double> z(0, 1);
    int M = numb_ante; /* Number of antenna elements */
    double frequency = 2.4e9;
    double c = 3e8;
    double wavelength = c/frequency; /* In units of meters */
    double seperation = wavelength*0.5;
    int angular_profile = 180; /* in number of intervals from 0 to 180*/

    ArrayXd angle(angular_profile);
    for (int i = 0; i<angular_profile; i++){
        angle(i) = i*180/angular_profile;
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
            double phase = -vec.dot(posantenna.col(i))/wavelength*2*M_PI;
            atheta(i,j) = exp(z*phase);
        }
    }
    
    
    for (int i  = 0; i<numb_ante; i++){
        for (int j = 0; j<snap;j++){
            for (int k = 0; k<sour;k++){
                double t = *(time+(i*snap + j)*sour+k);
                double p = *(power+(i*snap + j)*sour+k);
                sensor_matrix(i,j) = sensor_matrix(i,j) + exp(z*2.0*M_PI*t)*p;
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

    cout << sensor_covariance << endl << endl;
    SelfAdjointEigenSolver<MatrixXcd> eigensolver(sensor_covariance);
    if (eigensolver.info() != Success) abort();    

    int number_of_sig = 3;
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


    cout << "The eigenvalues are:\n" << eigensolver.eigenvalues()<< endl;
    cout << "Here's a matrix whose columns are eigenvectors \n"
        << "corresponding to these eigenvalues:\n"
        << eigensolver.eigenvectors() << endl;
    cout << "There are " << number_of_sig << " signals arriving" << endl<<endl;
    // cout << atheta.block(0,174,4,3) << endl;

    return spatial_metric; 
}
