/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/* *
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
 * Authors: Andrea Lacava <thecave003@gmail.com>
 *          Michele Polese <michele.polese@gmail.com>
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include <ns3/lte-ue-net-device.h>
#include "ns3/mmwave-helper.h"
#include "ns3/epc-helper.h"
#include <ns3/epc-enb-s1-sap.h>

#include "ns3/mmwave-point-to-point-epc-helper.h"
#include "ns3/lte-helper.h"
#include <chrono>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <boost/algorithm/string/classification.hpp> // Include boost::for is_any_of
#include <boost/algorithm/string/split.hpp> // Include for boost::split
#include "ns3/netanim-module.h"
#include <random>
#include <iostream>
using namespace ns3;
using namespace mmwave;

/**
 * Scenario Zero
 * 
 */


#define PORT 1488
using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("5GX2HandoverExample");



class MySocket
{
public:
  MySocket ()
  {
    my_socket = socket (AF_INET, SOCK_STREAM, 0);

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons (PORT);
    if (inet_pton (AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0)
      {
        std::cout << "Invalid address/ Address not supported \n";
      }
    Connect ();
  }
  void
  SendData (std::string msg)
  {
    if (Connect ())
      {

        send (my_socket, msg.data (), strlen (msg.data ()), 0);
      }
  }

  void
  GetData (void *buffer)
  {

    if (Connect ())
      {
        read (my_socket, buffer, 1024);
      }
  }
  ~MySocket(){
    close(my_socket);
  }

private:
  MySocket (const MySocket &) = delete;
  bool
  Connect ()
  {
    std::cout << "status: " << status << std::endl;
    if (status >= 0)
      {
        return true;
      }
    if ((status = connect (my_socket, (struct sockaddr *) &serv_addr, sizeof (serv_addr))) < 0)
      {
        std::cout << "\nConnection Failed \n";
        return false;
      }
    else
      {
        return true;
      }
  }

  int my_socket;
  int status = -1;
  struct sockaddr_in serv_addr;
};

MySocket my_socket;
Ptr<MmWaveHelper> mmwave_helper_ptr;
Ptr<LteHelper> lte_helper_ptr;
NetDeviceContainer* ue_ptr = nullptr;
NetDeviceContainer* enb_ptr = nullptr;
NodeContainer* lte_ptr = nullptr;
void
NotifyConnectionEstablishedUe (std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
  std::cout << Simulator::Now ().GetSeconds () << " " << context << " UE IMSI " << imsi
            << ": connected to CellId " << cellid << " with RNTI " << rnti << std::endl;
}

void
NotifyHandoverStartUe (std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti,
                       uint16_t targetCellId)
{
  std::cout << Simulator::Now ().GetSeconds () << " " << context << " UE IMSI " << imsi
            << ": previously connected to CellId " << cellid << " with RNTI " << rnti
            << ", doing handover to CellId " << targetCellId << std::endl;
}

void
NotifyHandoverEndOkUe (std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
  std::cout << Simulator::Now ().GetSeconds () << " " << context << " UE IMSI " << imsi
            << ": successful handover to CellId " << cellid << " with RNTI " << rnti << std::endl;
}

void
NotifyConnectionEstablishedEnb (std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
  std::cout << Simulator::Now ().GetSeconds () << " " << context << " eNB CellId " << cellid
            << ": successful connection of UE with IMSI " << imsi << " RNTI " << rnti << std::endl;
}

void
NotifyHandoverStartEnb (std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti,
                        uint16_t targetCellId)
{
  std::cout << Simulator::Now ().GetSeconds () << " " << context << " eNB CellId " << cellid
            << ": start handover of UE with IMSI " << imsi << " RNTI " << rnti << " to CellId "
            << targetCellId << std::endl;
}

void
NotifyHandoverEndOkEnb (std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
  std::cout << Simulator::Now ().GetSeconds () << " " << context << " eNB CellId " << cellid
            << ": completed handover of UE with IMSI " << imsi << " RNTI " << rnti << std::endl;
}
void ParseReportToVector(const std::string& msg, std::vector<std::string>& parsed_msg){
  boost::split(parsed_msg, msg, boost::is_any_of(", "), boost::token_compress_on);
}
void
SendRequest (Ptr<NetDevice> ue_node)
{
  std::ifstream inFile ("control.txt");
  std::stringstream test_msg;
  std::cout << "Check at the " << Simulator::Now ().GetSeconds () << std::endl;
  uint64_t imsi = ue_node->GetObject<McUeNetDevice> ()->GetImsi ();
  test_msg << imsi;
  //std::cout << "imsi: " << test_msg.str() << std::endl;
  my_socket.SendData (test_msg.str ());
  std::string message_str;
  int count = 0;
  char buffer[1024] = {'0'};
  while (message_str.size() == 0 && count < 2){
    count++;

    my_socket.GetData(&buffer);
    message_str = std::string(buffer);
  }
  std::vector<std::string> parsed_msg;
  ParseReportToVector(message_str, parsed_msg);
  if ( parsed_msg.size() > 2 )
    {
      //uint64_t imsi = ue_ptr->Get(0)->GetObject<McUeNetDevice> ()->GetImsi ();
      
      uint16_t oldCellId = stoi(parsed_msg[0]);
      uint64_t imsi = stoi(parsed_msg[1]);
      uint16_t targetCellId = stoi(parsed_msg[2]);
      
      
      if (oldCellId != targetCellId){
        mmwave_helper_ptr->SendSecondaryHandoverRequest(lte_ptr->Get(0), imsi, targetCellId, oldCellId);
      }

    }
}

std::string uePositionsFilename = "UePositions.txt";

void
PrintGnuplottableUeListToFile (std::string filename)
{
  std::ofstream outFile;
  outFile.open (filename.c_str (), std::ios_base::out | std::ios_base::trunc);
  if (!outFile.is_open ())
    {
      NS_LOG_ERROR ("Can't open file " << filename);
      return;
    }
  for (NodeList::Iterator it = NodeList::Begin (); it != NodeList::End (); ++it)
    {
      Ptr<Node> node = *it;
      int nDevs = node->GetNDevices ();
      for (int j = 0; j < nDevs; j++)
        {
          Ptr<LteUeNetDevice> uedev = node->GetDevice (j)->GetObject<LteUeNetDevice> ();
          Ptr<MmWaveUeNetDevice> mmuedev = node->GetDevice (j)->GetObject<MmWaveUeNetDevice> ();
          Ptr<McUeNetDevice> mcuedev = node->GetDevice (j)->GetObject<McUeNetDevice> ();
          if (uedev)
            {
              Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
              outFile << "set label \"" << uedev->GetImsi () << "\" at " << pos.x << "," << pos.y
                      << " left font \"Helvetica,8\" textcolor rgb \"black\" front point pt 1 ps "
                         "0.3 lc rgb \"black\" offset 0,0"
                      << std::endl;
            }
          else if (mmuedev)
            {
              Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
              outFile << "set label \"" << mmuedev->GetImsi () << "\" at " << pos.x << "," << pos.y
                      << " left font \"Helvetica,8\" textcolor rgb \"black\" front point pt 1 ps "
                         "0.3 lc rgb \"black\" offset 0,0"
                      << std::endl;
            }
          else if (mcuedev)
            {
              Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
              outFile << "set label \"" << mcuedev->GetImsi () << "\" at " << pos.x << "," << pos.y
                      << " left font \"Helvetica,8\" textcolor rgb \"black\" front point pt 1 ps "
                         "0.3 lc rgb \"black\" offset 0,0"
                      << std::endl;
            }
        }
    }
}

void
PrintGnuplottableEnbListToFile (std::string filename)
{
  std::ofstream outFile;
  outFile.open (filename.c_str (), std::ios_base::out | std::ios_base::trunc);
  if (!outFile.is_open ())
    {
      NS_LOG_ERROR ("Can't open file " << filename);
      return;
    }
  for (NodeList::Iterator it = NodeList::Begin (); it != NodeList::End (); ++it)
    {
      Ptr<Node> node = *it;
      int nDevs = node->GetNDevices ();
      for (int j = 0; j < nDevs; j++)
        {
          Ptr<LteEnbNetDevice> enbdev = node->GetDevice (j)->GetObject<LteEnbNetDevice> ();
          Ptr<MmWaveEnbNetDevice> mmdev = node->GetDevice (j)->GetObject<MmWaveEnbNetDevice> ();
          if (enbdev)
            {
              Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
              outFile << "set label \"" << enbdev->GetCellId () << "\" at " << pos.x << "," << pos.y
                      << " left font \"Helvetica,8\" textcolor rgb \"blue\" front  point pt 4 ps "
                         "0.3 lc rgb \"blue\" offset 0,0"
                      << std::endl;
            }
          else if (mmdev)
            {
              Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
              outFile << "set label \"" << mmdev->GetCellId () << "\" at " << pos.x << "," << pos.y
                      << " left font \"Helvetica,8\" textcolor rgb \"red\" front  point pt 4 ps "
                         "0.3 lc rgb \"red\" offset 0,0"
                      << std::endl;
            }
        }
    }
}

void
PrintPosition (Ptr<Node> node)
{
  std::ofstream outFile;
  outFile.open (uePositionsFilename, std::ios_base::out | std::ios_base::app);
  Ptr<MmWaveUeNetDevice> mmuedev = node->GetDevice (0)->GetObject<MmWaveUeNetDevice>();
  Ptr<McUeNetDevice> mcuedev = node->GetDevice (0)->GetObject<McUeNetDevice> ();
        if (mmuedev)
        {
        Ptr<MobilityModel> model = node->GetObject<MobilityModel> ();
        Vector pos = model->GetPosition ();
        outFile << mmuedev->GetImsi () << " " << pos.x << " " << pos.y<< " " << Simulator::Now ().GetSeconds ()
          << std::endl;
        }
        else if (mcuedev)
        {
        Ptr<MobilityModel> model = node->GetObject<MobilityModel> ();
        Vector pos = model->GetPosition ();
        outFile << mcuedev->GetImsi () << " " << pos.x << " " << pos.y<< " " << Simulator::Now ().GetSeconds ()
          << std::endl;
        }  
}

static ns3::GlobalValue g_bufferSize ("bufferSize", "RLC tx buffer size (MB)",
                                      ns3::UintegerValue (10),
                                      ns3::MakeUintegerChecker<uint32_t> ());

static ns3::GlobalValue g_enableTraces ("enableTraces", "If true, generate ns-3 traces",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_e2lteEnabled ("e2lteEnabled", "If true, send LTE E2 reports",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_e2nrEnabled ("e2nrEnabled", "If true, send NR E2 reports",
                                       ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_e2du ("e2du", "If true, send DU reports", ns3::BooleanValue (true),
                                ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_e2cuUp ("e2cuUp", "If true, send CU-UP reports", ns3::BooleanValue (true),
                                  ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_e2cuCp ("e2cuCp", "If true, send CU-CP reports", ns3::BooleanValue (true),
                                  ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_reducedPmValues ("reducedPmValues", "If true, use a subset of the the pm containers",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue
    g_hoSinrDifference ("hoSinrDifference",
                        "The value for which an handover between MmWave eNB is triggered",
                        ns3::DoubleValue (5), ns3::MakeDoubleChecker<double> ());

static ns3::GlobalValue
    g_indicationPeriodicity ("indicationPeriodicity",
                             "E2 Indication Periodicity reports (value in seconds)",
                             ns3::DoubleValue (0.5), ns3::MakeDoubleChecker<double> (0.01, 2.0));

static ns3::GlobalValue g_simTime ("simTime", "Simulation time in seconds", ns3::DoubleValue (20),
                                   ns3::MakeDoubleChecker<double> (0.1, 150.0));

static ns3::GlobalValue g_outageThreshold ("outageThreshold",
                                           "SNR threshold for outage events [dB]", // use -1000.0 with NoAuto
                                           ns3::DoubleValue (-5.0),
                                           ns3::MakeDoubleChecker<double> ());

static ns3::GlobalValue g_numberOfRaPreambles (
    "numberOfRaPreambles",
    "how many random access preambles are available for the contention based RACH process",
    ns3::UintegerValue (40), // Indicated for TS use case, 52 is default
    ns3::MakeUintegerChecker<uint8_t> ());

static ns3::GlobalValue
    g_handoverMode ("handoverMode",
                    "HO euristic to be used,"
                    "can be only \"NoAuto\", \"FixedTtt\", \"DynamicTtt\",   \"Threshold\"",
                    ns3::StringValue ("NoAuto"), ns3::MakeStringChecker ());


static ns3::GlobalValue g_e2TermIp ("e2TermIp", "The IP address of the RIC E2 termination",
                                    ns3::StringValue ("10.0.2.10"), ns3::MakeStringChecker ());

static ns3::GlobalValue
    g_enableE2FileLogging ("enableE2FileLogging",
                           "If true, generate offline file logging instead of connecting to RIC",
                           ns3::BooleanValue (true), ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_controlFileName ("controlFileName",
                                           "The path to the control file (can be absolute)",
                                           ns3::StringValue (""),
                                           ns3::MakeStringChecker ());

int
main (int argc, char *argv[])
{
  std::random_device dev;
  std::mt19937 rng(dev());
  std::uniform_int_distribution<std::mt19937::result_type> dist6(1,1000000); 
  RngSeedManager::SetRun(dist6(rng));
  //RngSeedManager::SetRun(42);
  //LogComponentEnableAll (LOG_PREFIX_ALL);
  //LogComponentEnable("EpcX2", LOG_LEVEL_ALL);
  // LogComponentEnable ("RicControlMessage", LOG_LEVEL_ALL);
  // LogComponentEnable ("Asn1Types", LOG_LEVEL_LOGIC);
  // LogComponentEnable ("E2Termination", LOG_LEVEL_LOGIC);

  // LogComponentEnable ("LteEnbNetDevice", LOG_LEVEL_ALL);
  LogComponentEnable ("MmWaveEnbNetDevice", LOG_LEVEL_DEBUG);

  // The maximum X coordinate of the scenario
  // The maximum Y coordinate of the scenario
  double enbTxPowerDbm = -100;
  // Command line arguments
  CommandLine cmd;
  cmd.Parse (argc, argv);

  bool harqEnabled = true;

  UintegerValue uintegerValue;
  BooleanValue booleanValue;
  StringValue stringValue;
  DoubleValue doubleValue;

  GlobalValue::GetValueByName ("hoSinrDifference", doubleValue);
  double hoSinrDifference = doubleValue.Get ();
  GlobalValue::GetValueByName ("bufferSize", uintegerValue);
  uint32_t bufferSize = uintegerValue.Get ();
  GlobalValue::GetValueByName ("enableTraces", booleanValue);
  //bool enableTraces = booleanValue.Get ();
  GlobalValue::GetValueByName ("outageThreshold", doubleValue);
  double outageThreshold = doubleValue.Get ();
  GlobalValue::GetValueByName ("handoverMode", stringValue);
  std::string handoverMode = stringValue.Get ();
  GlobalValue::GetValueByName ("e2TermIp", stringValue);
  std::string e2TermIp = stringValue.Get ();
  GlobalValue::GetValueByName ("enableE2FileLogging", booleanValue);
  bool enableE2FileLogging = booleanValue.Get ();
  GlobalValue::GetValueByName ("numberOfRaPreambles", uintegerValue);
  uint8_t numberOfRaPreambles = uintegerValue.Get ();

  NS_LOG_UNCOND ("bufferSize " << bufferSize << " OutageThreshold " << outageThreshold
                               << " HandoverMode " << handoverMode << " e2TermIp " << e2TermIp
                               << " enableE2FileLogging " << enableE2FileLogging);

  GlobalValue::GetValueByName ("e2lteEnabled", booleanValue);
  bool e2lteEnabled = booleanValue.Get ();
  GlobalValue::GetValueByName ("e2nrEnabled", booleanValue);
  bool e2nrEnabled = booleanValue.Get ();
  GlobalValue::GetValueByName ("e2du", booleanValue);
  bool e2du = booleanValue.Get ();
  GlobalValue::GetValueByName ("e2cuUp", booleanValue);
  bool e2cuUp = booleanValue.Get ();
  GlobalValue::GetValueByName ("e2cuCp", booleanValue);
  bool e2cuCp = booleanValue.Get ();

  GlobalValue::GetValueByName ("reducedPmValues", booleanValue);
  bool reducedPmValues = booleanValue.Get ();

  GlobalValue::GetValueByName ("indicationPeriodicity", doubleValue);
  double indicationPeriodicity = doubleValue.Get ();
  GlobalValue::GetValueByName ("controlFileName", stringValue);
  std::string controlFilename = stringValue.Get ();

  NS_LOG_UNCOND ("e2lteEnabled " << e2lteEnabled << " e2nrEnabled " << e2nrEnabled << " e2du "
                                 << e2du << " e2cuCp " << e2cuCp << " e2cuUp " << e2cuUp
                                 << " controlFilename " << controlFilename
                                 << " indicationPeriodicity " << indicationPeriodicity);

  Config::SetDefault ("ns3::LteEnbNetDevice::ControlFileName", StringValue (controlFilename));
  Config::SetDefault ("ns3::LteEnbNetDevice::E2Periodicity", DoubleValue (indicationPeriodicity));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::E2Periodicity",
                      DoubleValue (indicationPeriodicity));

  Config::SetDefault ("ns3::MmWaveHelper::E2ModeLte", BooleanValue (e2lteEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::E2ModeNr", BooleanValue (e2nrEnabled));

  // The DU PM reports should come from both NR gNB as well as LTE eNB,
  // since in the RLC/MAC/PHY entities are present in BOTH NR gNB as well as LTE eNB.
  // DU reports from LTE eNB are not implemented in this release
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableDuReport", BooleanValue (e2du));

  // The CU-UP PM reports should only come from LTE eNB, since in the NS3 “EN-DC
  // simulation (Option 3A)”, the PDCP is only in the LTE eNB and NOT in the NR gNB
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableCuUpReport", BooleanValue (e2cuUp));
  Config::SetDefault ("ns3::LteEnbNetDevice::EnableCuUpReport", BooleanValue (e2cuUp));

  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableCuCpReport", BooleanValue (e2cuCp));
  Config::SetDefault ("ns3::LteEnbNetDevice::EnableCuCpReport", BooleanValue (e2cuCp));

  Config::SetDefault ("ns3::MmWaveEnbNetDevice::ReducedPmValues", BooleanValue (reducedPmValues));
  Config::SetDefault ("ns3::LteEnbNetDevice::ReducedPmValues", BooleanValue (reducedPmValues));

  Config::SetDefault ("ns3::LteEnbNetDevice::EnableE2FileLogging",
                      BooleanValue (enableE2FileLogging));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableE2FileLogging",
                      BooleanValue (enableE2FileLogging));

  Config::SetDefault ("ns3::MmWaveEnbMac::NumberOfRaPreambles",
                      UintegerValue (numberOfRaPreambles));

  Config::SetDefault ("ns3::MmWaveHelper::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::UseIdealRrc", BooleanValue (true));
  Config::SetDefault ("ns3::MmWaveHelper::E2TermIp", StringValue (e2TermIp));

  Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::NumHarqProcess", UintegerValue (100));
  //Config::SetDefault ("ns3::MmWaveBearerStatsCalculator::EpochDuration", TimeValue (MilliSeconds (10.0)));

  // set to false to use the 3GPP radiation pattern (proper configuration of the bearing and downtilt angles is needed)
  Config::SetDefault ("ns3::ThreeGppAntennaArrayModel::IsotropicElements", BooleanValue (true));
  Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (100.0)));
  Config::SetDefault ("ns3::ThreeGppChannelConditionModel::UpdatePeriod",
                      TimeValue (MilliSeconds (100)));

  Config::SetDefault ("ns3::LteRlcAm::ReportBufferStatusTimer", TimeValue (MilliSeconds (10.0)));
  Config::SetDefault ("ns3::LteRlcUmLowLat::ReportBufferStatusTimer",
                      TimeValue (MilliSeconds (10.0)));
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (bufferSize * 1024 * 1024));
  Config::SetDefault ("ns3::LteRlcUmLowLat::MaxTxBufferSize",
                      UintegerValue (bufferSize * 1024 * 1024));
  Config::SetDefault ("ns3::LteRlcAm::MaxTxBufferSize", UintegerValue (bufferSize * 1024 * 1024));

  Config::SetDefault ("ns3::LteEnbRrc::OutageThreshold", DoubleValue (outageThreshold));
  Config::SetDefault ("ns3::LteEnbRrc::SecondaryCellHandoverMode", StringValue (handoverMode));
  Config::SetDefault ("ns3::LteEnbRrc::HoSinrDifference", DoubleValue (hoSinrDifference));
  Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (enbTxPowerDbm));

  std::ofstream outFile;
  outFile.open (uePositionsFilename, std::ios_base::out);
  outFile  << "imsi" << " x" <<" y" << " time" <<std::endl;


  // Carrier bandwidth in Hz
  double bandwidth = 20e6;
  // Center frequency in Hz
  double centerFrequency = 3.5e9;
  // Distance between the mmWave BSs and the two co-located LTE and mmWave BSs in meters
  double isd = 1000; // (interside distance)
  // Number of antennas in each UE
  int numAntennasMcUe = 1;
  // Number of antennas in each mmWave BS
  int numAntennasMmWave = 1;

  NS_LOG_INFO ("Bandwidth " << bandwidth << " centerFrequency " << double (centerFrequency)
                            << " isd " << isd << " numAntennasMcUe " << numAntennasMcUe
                            << " numAntennasMmWave " << numAntennasMmWave);

  // Set the number of antennas in the devices
  Config::SetDefault ("ns3::McUeNetDevice::AntennaNum", UintegerValue (numAntennasMcUe));
  Config::SetDefault ("ns3::MmWaveNetDevice::AntennaNum", UintegerValue (numAntennasMmWave));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::Bandwidth", DoubleValue (bandwidth));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue (centerFrequency));

  Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper> ();
  mmwave_helper_ptr = mmwaveHelper;
  mmwaveHelper->SetPathlossModelType ("ns3::ThreeGppUmaPropagationLossModel");
  mmwaveHelper->SetChannelConditionModelType ("ns3::AlwaysLosChannelConditionModel");

  Ptr<MmWavePointToPointEpcHelper> epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
  mmwaveHelper->SetEpcHelper (epcHelper);
  mmwaveHelper->SetLteHandoverAlgorithmType("ns3::NoOpHandoverAlgorithm");
  uint8_t nMmWaveEnbNodes = 3;
  uint8_t nLteEnbNodes = 1;
  uint8_t nUeNodes = 1;
  //uint16_t numBearersPerUe = 2;
  
  double distance = 100.0;

  //double speed = 25;  
  Time simTime = Seconds (distance*2*(nMmWaveEnbNodes-1)/20);
  NS_LOG_INFO (" Bandwidth " << bandwidth << " centerFrequency " << double (centerFrequency)
                             << " isd " << isd << " numAntennasMcUe " << numAntennasMcUe
                             << " numAntennasMmWave " << numAntennasMmWave << " nMmWaveEnbNodes "
                             << unsigned (nMmWaveEnbNodes));

  // Get SGW/PGW and create a single RemoteHost
  Ptr<Node> pgw = epcHelper->GetPgwNode ();
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  // Create the Internet by connecting remoteHost to pgw. Setup routing too
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (2500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  // interface 0 is localhost, 1 is the p2p device
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  // create LTE, mmWave eNB nodes and UE node
  NodeContainer ueNodes;
  NodeContainer mmWaveEnbNodes;
  NodeContainer lteEnbNodes;
  NodeContainer allEnbNodes;
  mmWaveEnbNodes.Create (nMmWaveEnbNodes);
  lteEnbNodes.Create (nLteEnbNodes);

  uint nGroups = 4;
  std::vector<NodeContainer> ueNodesGroupList;
  for (uint i = 0; i < nGroups; ++i){
    NodeContainer ueNodesGroup;
    ueNodesGroup.Create(nUeNodes);
    ueNodesGroupList.push_back(ueNodesGroup);
    ueNodes.Add(ueNodesGroupList.back());

  }

  //ueNodes.Create (nUeNodes);

  allEnbNodes.Add (lteEnbNodes);
  allEnbNodes.Add (mmWaveEnbNodes);

    // Install Mobility Model
  Ptr<ListPositionAllocator> positionAlloc5G = CreateObject<ListPositionAllocator> ();
  Ptr<ListPositionAllocator> positionAllocLTE = CreateObject<ListPositionAllocator> ();
  //for (uint16_t i = 0; i < nMmWaveEnbNodes; i++)
  //  {
  //    positionAlloc5G->Add (Vector (distance * 2 * i + distance, 0, 0));
  //  }
  double xpose, ypose;
  double maxXAxis = 500.0;
  double maxYAxis = 500.0;
  Vector centerPosition = Vector (maxXAxis / 2, maxYAxis / 2, 0);
  for (uint16_t i = 0; i < nMmWaveEnbNodes; i++) {
      xpose = distance * cos ((2 * M_PI * i) / (nMmWaveEnbNodes));
      ypose = distance * sin ((2 * M_PI * i) / (nMmWaveEnbNodes));
      positionAlloc5G->Add (Vector (centerPosition.x + xpose, centerPosition.y + ypose, 0));
  }
  positionAllocLTE->Add(Vector(centerPosition.x,centerPosition.y,0));
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator (positionAlloc5G);
  mobility.Install (mmWaveEnbNodes);
  mobility.SetPositionAllocator (positionAllocLTE);
  mobility.Install (lteEnbNodes);


  std::vector<std::vector<uint64_t>> x_vec = {{125, 280}, // 1st Group
                                              {375,1700}, // 2
                                              {125, 240, 385}, // 3
                                              {93, 250, 135}, //4
                                              };
  std::vector<std::vector<uint64_t>> y_vec = {{120, 370},// 1st Group
                                              {125, 93},// 2nd Group
                                              {120, 280, 155}, // 3
                                              {340, 245, 93}, // 4
                                              };
  // GROUP 1
  uint64_t var = 2;
  std::vector<std::pair<Ptr<UniformRandomVariable>, Ptr<UniformRandomVariable>>> waypoints;
  MobilityHelper ueMobility;
  ueMobility.SetMobilityModel ("ns3::WaypointMobilityModel");
  ueMobility.Install(ueNodes);
  for (uint64_t j = 0; j < x_vec.size(); ++j){
    for (uint64_t i = 0; i < x_vec[j].size(); i++){
      Ptr<UniformRandomVariable> waypoint_x = CreateObject<UniformRandomVariable> ();
      waypoint_x->SetAttribute ("Min", DoubleValue (x_vec[j][i] - var));
      waypoint_x->SetAttribute ("Max", DoubleValue (x_vec[j][i] + var));
      Ptr<UniformRandomVariable> waypoint_y = CreateObject<UniformRandomVariable> ();
      waypoint_y->SetAttribute ("Min", DoubleValue (y_vec[j][i] - var));
      waypoint_y->SetAttribute ("Max", DoubleValue (y_vec[j][i] + var));
      waypoints.push_back({waypoint_x, waypoint_y});
    }
    
    

    NodeContainer ueNodesGroup = ueNodesGroupList[j];
    //std::cout<<time_delta<<std::endl;

    double speed = 10; // m/s
    for (uint64_t i = 0; i < ueNodesGroup.GetN(); ++i){
      Time nextWaypoint = Seconds (0.01);
      double waypoint_x = waypoints[0].first->GetValue();
      double prev_waypoint_x = 0;
      double waypoint_y = waypoints[0].second->GetValue();
      double prev_waypoint_y = 0;
      std::cout << waypoints.size() <<std::endl;
      for (uint64_t k = 0; k < waypoints.size(); k++){
        prev_waypoint_x = waypoint_x;
        prev_waypoint_y = waypoint_y;
        std::cout << k <<std::endl;
        waypoint_x = waypoints[k].first->GetValue();
        waypoint_y = waypoints[k].second->GetValue();
        std::cout<<"NW b"<<std::endl;
        std::cout<<prev_waypoint_x <<" "<<waypoint_x <<std::endl;
        std::cout<<prev_waypoint_y <<" "<<waypoint_y <<std::endl;
        std::cout<<std::pow(std::pow((prev_waypoint_x-waypoint_x), 2) + std::pow((prev_waypoint_y-waypoint_y), 2), 0.5)/speed<<std::endl;
        nextWaypoint += Seconds(std::sqrt(std::pow((prev_waypoint_x-waypoint_x), 2) + std::pow((prev_waypoint_y-waypoint_y), 2))/speed);
        ueNodesGroup.Get(i)->GetObject<WaypointMobilityModel> ()->AddWaypoint (Waypoint (nextWaypoint, Vector (waypoint_x, waypoint_y, 1.5)));
        std::cout<<"NW end"<<std::endl;
      }
      std::cout<<nextWaypoint.GetSeconds()<<std::endl;
      std::cout<<"NW end"<<std::endl;
    }

    waypoints.clear();
  }
  //ueMobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");


  //uePositionAlloc->SetX (centerPosition.x); old code
  //uePositionAlloc->SetY (centerPosition.y); old code
  //uePositionAlloc->SetRho (distance); old code

  /*Ptr<UniformRandomVariable> speed = CreateObject<UniformRandomVariable> (); code for 6 users
  speed->SetAttribute ("Min", DoubleValue (10.0));
  speed->SetAttribute ("Max", DoubleValue (20.0));

  ueMobility.SetMobilityModel ("ns3::RandomWalk2dOutdoorMobilityModel", "Speed",
                               PointerValue (speed), "Bounds",
                               RectangleValue (Rectangle (0, 500, 0, 500)));
  
  */
  //ueMobility.SetPositionAllocator (uePositionAlloc); old code
  //ueMobility.Install (ueNodes); old code
  //ueMobility.Install (ueNodes); code 6 users
  //ueNodes.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0, 0, 0)); old code 
  //ueNodes.Get (0)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (speed, 0, 0)); old code
  //ueNodes.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (500, -15, 0)); old code
  //ueNodes.Get (1)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (-1*(speed), 0, 0)); old code
  
  
  //Ptr<UniformDiscPositionAllocator> uePositionAlloc = CreateObject<UniformDiscPositionAllocator> (); old code
  /*ueNodes.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (240, 240, 0));
  ueNodes.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (245, 245, 0)); code for 6 users
  ueNodes.Get (2)->GetObject<MobilityModel> ()->SetPosition (Vector (250, 250, 0));
  ueNodes.Get (3)->GetObject<MobilityModel> ()->SetPosition (Vector (255, 255, 0));
  ueNodes.Get (4)->GetObject<MobilityModel> ()->SetPosition (Vector (260, 260, 0));
  ueNodes.Get (5)->GetObject<MobilityModel> ()->SetPosition (Vector (265, 265, 0));*/
  
  
  
  
  // Install mmWave, lte, mc Devices to the nodes
  NetDeviceContainer lteEnbDevs = mmwaveHelper->InstallLteEnbDevice (lteEnbNodes);
  NetDeviceContainer mmWaveEnbDevs = mmwaveHelper->InstallEnbDevice (mmWaveEnbNodes);
  NetDeviceContainer mcUeDevs = mmwaveHelper->InstallMcUeDevice (ueNodes);
  ue_ptr = &mcUeDevs;
  enb_ptr = &mmWaveEnbDevs;
  lte_ptr = &lteEnbNodes;
  // Install the IP stack on the UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (mcUeDevs));
  // Assign IP address to UEs, and install applications
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting =
          ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

  // Add X2 interfaces
  mmwaveHelper->AddX2Interface (lteEnbNodes, mmWaveEnbNodes);


  // Manual attachment
  mmwaveHelper->AttachToClosestEnb (mcUeDevs, mmWaveEnbDevs, lteEnbDevs);

  // Install and start applications
  // On the remoteHost there is UDP OnOff Application
  // /*
  uint16_t portUdp = 60000;
  Address sinkLocalAddressUdp (InetSocketAddress (Ipv4Address::GetAny (), portUdp));
  PacketSinkHelper sinkHelperUdp ("ns3::UdpSocketFactory", sinkLocalAddressUdp);
  AddressValue serverAddressUdp (InetSocketAddress (remoteHostAddr, portUdp));

  ApplicationContainer sinkApp;
  sinkApp.Add (sinkHelperUdp.Install (remoteHost));

  ApplicationContainer clientApp;

  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      // Full traffic
      PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory",
                                           InetSocketAddress (Ipv4Address::GetAny (), 1234));
      sinkApp.Add (dlPacketSinkHelper.Install (ueNodes.Get (u)));
      UdpClientHelper dlClient (ueIpIface.GetAddress (u), 1234);
      dlClient.SetAttribute ("Interval", TimeValue (MicroSeconds (500)));
      dlClient.SetAttribute ("MaxPackets", UintegerValue (UINT32_MAX));
      dlClient.SetAttribute ("PacketSize", UintegerValue (1280));
      clientApp.Add (dlClient.Install (remoteHost));
    }

  // Start applications
  //GlobalValue::GetValueByName ("simTime", doubleValue);
  //double simTime = doubleValue.Get ();
  sinkApp.Start (Seconds (0));

  clientApp.Start (MilliSeconds (100));
  clientApp.Stop (Seconds (20 - 0.1));
  //*/
  
   int numPrints = 1000;
   for (int i = 0; i < numPrints; i++)
     {
       for (uint32_t j = 0; j < ueNodes.GetN (); j++)
         {
          //std::cout<<simTime.GetDouble()<<std::endl;
           Simulator::Schedule (i * simTime / numPrints, &PrintPosition, ueNodes.Get (j));
         }
     }

  int numSends = 10;
  for (int i = 1; i <= numSends; i++)
    {
        //std::cout<<simTime.GetDouble()<<std::endl;
        for (uint32_t ue = 0; ue < ueNodes.GetN(); ue++){
          Simulator::Schedule ((i) * simTime / (numSends), &SendRequest, mcUeDevs.Get (ue));
        }
        
    }

  //Simulator::Schedule (Seconds (0.2), &SendRequest);
  //Simulator::Schedule (Seconds (0.5), &SendRequest);
  //Simulator::Schedule (Seconds (1), &SendRequest);
  /*
  if (enableTraces)
    {
      mmwaveHelper->EnableTraces ();
    }
    */
   
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
                   MakeCallback (&NotifyConnectionEstablishedEnb));
  Config::Connect ("/NodeList/*/DeviceList/*/MmWaveUeRrc/ConnectionReconfiguration",
                   MakeCallback (&NotifyConnectionEstablishedUe));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                   MakeCallback (&NotifyHandoverStartEnb));
  Config::Connect ("/NodeList/*/DeviceList/*/MmWaveUeRrc/HandoverStart",
                   MakeCallback (&NotifyHandoverStartUe));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                   MakeCallback (&NotifyHandoverEndOkEnb));
  Config::Connect ("/NodeList/*/DeviceList/*/MmWaveUeRrc/HandoverEndOk",
                   MakeCallback (&NotifyHandoverEndOkUe));

  // trick to enable PHY traces for the LTE stack
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  lteHelper->SetEpcHelper (epcHelper);
  lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler");
  lteHelper->SetHandoverAlgorithmType ("ns3::NoOpHandoverAlgorithm"); // disable automatic handover
  mmwaveHelper->EnableTraces ();
  //lte_helper_ptr = lteHelper;
  //lteHelper->Initialize ();
  //lteHelper->EnablePhyTraces ();
  //lteHelper->EnableMacTraces ();

  // Since nodes are randomly allocated during each run we always need to print their positions
  PrintGnuplottableUeListToFile ("ues.txt");
  PrintGnuplottableEnbListToFile ("enbs.txt");
  AnimationInterface anim("myfirst_animation.xml");
  bool run = true;
  if (run)
    {
      NS_LOG_UNCOND ("Simulation time is " << simTime << " seconds ");
      Simulator::Stop (simTime);
      NS_LOG_INFO ("Run Simulation.");
      Simulator::Run ();
    }

  NS_LOG_INFO (lteHelper);
  Simulator::Destroy ();
  NS_LOG_INFO ("Done.");
  return 0;
}
