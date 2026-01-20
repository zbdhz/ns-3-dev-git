/*
 * Copyright (c) 2005,2006 INRIA
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */

#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"

#include "ns3/flow-id-tag.h"
#include "ns3/nist-error-rate-model.h"
#include "ns3/packet.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/simulator.h"
#include "ns3/wifi-psdu.h"
#include "ns3/yans-error-rate-model.h"
#include "ns3/table-based-error-rate-model.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-phy.h"
#include "ns3/wifi-types.h"
#include "ns3/wifi-utils.h"

#include <fstream>
#include <string>
#include <cstdlib>

using namespace ns3;

/// PsrExperiment
class PsrExperiment
{
  public:
    /// Input structure
    struct Input
    {
        Input();
        meter_u distance;     ///< distance
        std::string txMode;   ///< transmit mode
        uint8_t txPowerLevel; ///< transmit power level
        uint32_t packetSize;  ///< packet size
        uint32_t nPackets;    ///< number of packets
    };

    /// Output structure
    struct Output
    {
        uint32_t received; ///< received
    };

    PsrExperiment();

    /**
     * Run function
     * @param input the PSR experiment
     * @returns the PSR experiment output
     */
    PsrExperiment::Output Run(PsrExperiment::Input input);

  private:
    /// Send function
    void Send();
    /**
     * Send receive function
     * @param psdu the PSDU
     * @param rxSignalInfo the info on the received signal (\see RxSignalInfo)
     * @param txVector the wifi transmit vector
     * @param statusPerMpdu reception status per MPDU
     */
    void Receive(Ptr<const WifiPsdu> psdu,
                 RxSignalInfo rxSignalInfo,
                 const WifiTxVector& txVector,
                 const std::vector<bool>& statusPerMpdu);
    Ptr<WifiPhy> m_tx; ///< transmit
    Input m_input;     ///< input
    Output m_output;   ///< output
};

void
PsrExperiment::Send()
{
    Ptr<WifiPsdu> psdu = Create<WifiPsdu>(Create<Packet>(m_input.packetSize), WifiMacHeader());
    WifiMode mode = WifiMode(m_input.txMode);
    WifiTxVector txVector;
    txVector.SetTxPowerLevel(m_input.txPowerLevel);
    txVector.SetMode(mode);
    txVector.SetPreambleType(WIFI_PREAMBLE_LONG);
    m_tx->Send(psdu, txVector);
}

void
PsrExperiment::Receive(Ptr<const WifiPsdu> psdu,
                       RxSignalInfo rxSignalInfo,
                       const WifiTxVector& txVector,
                       const std::vector<bool>& statusPerMpdu)
{
    m_output.received++;
}

PsrExperiment::PsrExperiment()
{
}

PsrExperiment::Input::Input()
    : distance(5.0),
      txMode("OfdmRate6Mbps"),
      txPowerLevel(1),
      packetSize(2304),
      nPackets(400)
{
}

PsrExperiment::Output
PsrExperiment::Run(PsrExperiment::Input input)
{
    m_output.received = 0;
    m_input = input;

    Ptr<MobilityModel> posTx = CreateObject<ConstantPositionMobilityModel>();
    posTx->SetPosition(Vector(0.0, 0.0, 0.0));
    Ptr<MobilityModel> posRx = CreateObject<ConstantPositionMobilityModel>();
    posRx->SetPosition(Vector(m_input.distance, 0.0, 0.0));

    Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel>();
    channel->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
    Ptr<LogDistancePropagationLossModel> log = CreateObject<LogDistancePropagationLossModel>();
    channel->SetPropagationLossModel(log);

    Ptr<YansWifiPhy> tx = CreateObject<YansWifiPhy>();
    Ptr<YansWifiPhy> rx = CreateObject<YansWifiPhy>();
    
    // Create and set InterferenceHelper for both PHY objects
    Ptr<InterferenceHelper> txInterferenceHelper = CreateObject<InterferenceHelper>();
    tx->SetInterferenceHelper(txInterferenceHelper);
    
    Ptr<InterferenceHelper> rxInterferenceHelper = CreateObject<InterferenceHelper>();
    rx->SetInterferenceHelper(rxInterferenceHelper);
    
    // Ptr<ErrorRateModel> error = CreateObject<NistErrorRateModel>();
    // Ptr<YansErrorRateModel> error = CreateObject<YansErrorRateModel>();
    Ptr<TableBasedErrorRateModel> error = CreateObject<TableBasedErrorRateModel>();
    tx->SetErrorRateModel(error);
    rx->SetErrorRateModel(error);
    tx->SetChannel(channel);
    rx->SetChannel(channel);
    tx->SetMobility(posTx);
    rx->SetMobility(posRx);

    tx->ConfigureStandard(WIFI_STANDARD_80211a);
    rx->ConfigureStandard(WIFI_STANDARD_80211a);

    rx->SetReceiveOkCallback(MakeCallback(&PsrExperiment::Receive, this));

    for (uint32_t i = 0; i < m_input.nPackets; ++i)
    {
        Simulator::Schedule(Seconds(i), &PsrExperiment::Send, this);
    }
    m_tx = tx;
    Simulator::Run();
    Simulator::Destroy();
    return m_output;
}

/// CollisionExperiment
class CollisionExperiment
{
  public:
    /// Input structure
    struct Input
    {
        Input();
        Time interval;         ///< interval
        double xA;             ///< x A
        double xB;             ///< x B
        std::string txModeA;   ///< transmit mode A
        std::string txModeB;   ///< transmit mode B
        uint8_t txPowerLevelA; ///< transmit power level A
        uint8_t txPowerLevelB; ///< transmit power level B
        uint32_t packetSizeA;  ///< packet size A
        uint32_t packetSizeB;  ///< packet size B
        uint32_t nPackets;     ///< number of packets
    };

    /// Output structure
    struct Output
    {
        uint32_t receivedA; ///< received A
        uint32_t receivedB; ///< received B
    };

    CollisionExperiment();

    /**
     * Run function
     * @param input the collision experiment data
     * @returns the experiment output
     */
    CollisionExperiment::Output Run(CollisionExperiment::Input input);

  private:
    /// Send A function
    void SendA() const;
    /// Send B function
    void SendB() const;
    /**
     * Receive function
     * @param psdu the PSDU
     * @param rxSignalInfo the info on the received signal (\see RxSignalInfo)
     * @param txVector the wifi transmit vector
     * @param statusPerMpdu reception status per MPDU
     */
    void Receive(Ptr<const WifiPsdu> psdu,
                 RxSignalInfo rxSignalInfo,
                 const WifiTxVector& txVector,
                 const std::vector<bool>& statusPerMpdu);
    Ptr<WifiPhy> m_txA; ///< transmit A
    Ptr<WifiPhy> m_txB; ///< transmit B
    uint32_t m_flowIdA; ///< flow ID A
    uint32_t m_flowIdB; ///< flow ID B
    Input m_input;      ///< input
    Output m_output;    ///< output
};

void
CollisionExperiment::SendA() const
{
    Ptr<WifiPsdu> psdu = Create<WifiPsdu>(Create<Packet>(m_input.packetSizeA), WifiMacHeader());
    (*psdu->begin())->GetPacket()->AddByteTag(FlowIdTag(m_flowIdA));
    WifiTxVector txVector;
    txVector.SetTxPowerLevel(m_input.txPowerLevelA);
    txVector.SetMode(WifiMode(m_input.txModeA));
    txVector.SetPreambleType(WIFI_PREAMBLE_LONG);
    m_txA->Send(psdu, txVector);
}

void
CollisionExperiment::SendB() const
{
    Ptr<WifiPsdu> psdu = Create<WifiPsdu>(Create<Packet>(m_input.packetSizeB), WifiMacHeader());
    (*psdu->begin())->GetPacket()->AddByteTag(FlowIdTag(m_flowIdB));
    WifiTxVector txVector;
    txVector.SetTxPowerLevel(m_input.txPowerLevelB);
    txVector.SetMode(WifiMode(m_input.txModeB));
    txVector.SetPreambleType(WIFI_PREAMBLE_LONG);
    m_txB->Send(psdu, txVector);
}

void
CollisionExperiment::Receive(Ptr<const WifiPsdu> psdu,
                             RxSignalInfo rxSignalInfo,
                             const WifiTxVector& txVector,
                             const std::vector<bool>& statusPerMpdu)
{
    FlowIdTag tag;
    if ((*psdu->begin())->GetPacket()->FindFirstMatchingByteTag(tag))
    {
        if (tag.GetFlowId() == m_flowIdA)
        {
            m_output.receivedA++;
        }
        else if (tag.GetFlowId() == m_flowIdB)
        {
            m_output.receivedB++;
        }
    }
}

CollisionExperiment::CollisionExperiment()
{
}

CollisionExperiment::Input::Input()
    : interval(),
      xA(-5),
      xB(5),
      txModeA("OfdmRate6Mbps"),
      txModeB("OfdmRate6Mbps"),
      txPowerLevelA(1),
      txPowerLevelB(1),
      packetSizeA(2304),
      packetSizeB(2304),
      nPackets(400)
{
}

CollisionExperiment::Output
CollisionExperiment::Run(CollisionExperiment::Input input)
{
    m_output.receivedA = 0;
    m_output.receivedB = 0;
    m_input = input;

    m_flowIdA = FlowIdTag::AllocateFlowId();
    m_flowIdB = FlowIdTag::AllocateFlowId();

    Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel>();
    channel->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
    Ptr<LogDistancePropagationLossModel> log = CreateObject<LogDistancePropagationLossModel>();
    channel->SetPropagationLossModel(log);

    Ptr<MobilityModel> posTxA = CreateObject<ConstantPositionMobilityModel>();
    posTxA->SetPosition(Vector(input.xA, 0.0, 0.0));
    Ptr<MobilityModel> posTxB = CreateObject<ConstantPositionMobilityModel>();
    posTxB->SetPosition(Vector(input.xB, 0.0, 0.0));
    Ptr<MobilityModel> posRx = CreateObject<ConstantPositionMobilityModel>();
    posRx->SetPosition(Vector(0, 0.0, 0.0));

    Ptr<YansWifiPhy> txA = CreateObject<YansWifiPhy>();
    Ptr<YansWifiPhy> txB = CreateObject<YansWifiPhy>();
    Ptr<YansWifiPhy> rx = CreateObject<YansWifiPhy>();

    // Create and set InterferenceHelper for all PHY objects
    Ptr<InterferenceHelper> txAInterferenceHelper = CreateObject<InterferenceHelper>();
    txA->SetInterferenceHelper(txAInterferenceHelper);
    
    Ptr<InterferenceHelper> txBInterferenceHelper = CreateObject<InterferenceHelper>();
    txB->SetInterferenceHelper(txBInterferenceHelper);
    
    Ptr<InterferenceHelper> rxInterferenceHelper = CreateObject<InterferenceHelper>();
    rx->SetInterferenceHelper(rxInterferenceHelper);

    Ptr<ErrorRateModel> error = CreateObject<NistErrorRateModel>();
    txA->SetErrorRateModel(error);
    txB->SetErrorRateModel(error);
    rx->SetErrorRateModel(error);
    txA->SetChannel(channel);
    txB->SetChannel(channel);
    rx->SetChannel(channel);
    txA->SetMobility(posTxA);
    txB->SetMobility(posTxB);
    rx->SetMobility(posRx);

    txA->ConfigureStandard(WIFI_STANDARD_80211a);
    txB->ConfigureStandard(WIFI_STANDARD_80211a);
    rx->ConfigureStandard(WIFI_STANDARD_80211a);

    rx->SetReceiveOkCallback(MakeCallback(&CollisionExperiment::Receive, this));

    for (uint32_t i = 0; i < m_input.nPackets; ++i)
    {
        Simulator::Schedule(Seconds(i), &CollisionExperiment::SendA, this);
    }
    for (uint32_t i = 0; i < m_input.nPackets; ++i)
    {
        Simulator::Schedule(Seconds(i) + m_input.interval, &CollisionExperiment::SendB, this);
    }
    m_txA = txA;
    m_txB = txB;
    Simulator::Run();
    Simulator::Destroy();
    return m_output;
}

/// SnrVsPsrExperiment
class SnrVsPsrExperiment
{
  public:
    /// Input structure
    struct Input
    {
        Input();
        std::string txMode;   ///< transmit mode (MCS)
        uint8_t txPowerLevel; ///< transmit power level
        uint32_t packetSize;  ///< packet size
        uint32_t nPackets;    ///< number of packets
        double targetSnr;     ///< target SNR in dB
    };

    /// Output structure
    struct Output
    {
        uint32_t received;     ///< received packets count
        double avgSnr;         ///< average SNR in linear scale
        double avgRssi;        ///< average RSSI in dBm
        double packetErrorRate; ///< packet error rate
    };

    SnrVsPsrExperiment();

    /**
     * Run function
     * @param input the SNR vs PSR experiment input
     * @returns the SNR vs PSR experiment output
     */
    SnrVsPsrExperiment::Output Run(SnrVsPsrExperiment::Input input);

  private:
    /// Send function
    void Send();
    /**
     * Receive function
     * @param psdu the PSDU
     * @param rxSignalInfo the info on the received signal (ee RxSignalInfo)
     * @param txVector the wifi transmit vector
     * @param statusPerMpdu reception status per MPDU
     */
    void Receive(Ptr<const WifiPsdu> psdu,
                 RxSignalInfo rxSignalInfo,
                 const WifiTxVector& txVector,
                 const std::vector<bool>& statusPerMpdu);
    Ptr<WifiPhy> m_tx; ///< transmit PHY
    Input m_input;     ///< input parameters
    Output m_output;   ///< output results
    double m_totalSnr; ///< total SNR for average calculation
    double m_totalRssi;///< total RSSI for average calculation
};

void
SnrVsPsrExperiment::Send()
{
    Ptr<WifiPsdu> psdu = Create<WifiPsdu>(Create<Packet>(m_input.packetSize), WifiMacHeader());
    WifiMode mode = WifiMode(m_input.txMode);
    WifiTxVector txVector;
    txVector.SetTxPowerLevel(m_input.txPowerLevel);
    txVector.SetMode(mode);
    txVector.SetPreambleType(WIFI_PREAMBLE_LONG);
    m_tx->Send(psdu, txVector);
}

void
SnrVsPsrExperiment::Receive(Ptr<const WifiPsdu> psdu,
                           RxSignalInfo rxSignalInfo,
                           const WifiTxVector& txVector,
                           const std::vector<bool>& statusPerMpdu)
{
    m_output.received++;
    m_totalSnr += rxSignalInfo.snr;
    m_totalRssi += rxSignalInfo.rssi;
}

SnrVsPsrExperiment::SnrVsPsrExperiment()
{
}

SnrVsPsrExperiment::Input::Input()
    : txMode("OfdmRate6Mbps"),
      txPowerLevel(1),
      packetSize(2304),
      nPackets(400),
      targetSnr(10.0) // Default target SNR in dB
{
}

SnrVsPsrExperiment::Output
SnrVsPsrExperiment::Run(SnrVsPsrExperiment::Input input)
{
    m_output.received = 0;
    m_totalSnr = 0.0;
    m_totalRssi = 0.0;
    m_input = input;

    // Default noise power in dBm (typical value)
    const double defaultNoisePowerDbm = -90.0;
    
    // Adjust target SNR by a fixed offset to match actual results
    // Actual SNR is ~4dB higher than target, so subtract 4dB from target
    const double SNR_OFFSET = 4.0;
    double adjustedSnr = m_input.targetSnr - SNR_OFFSET;
    
    // Calculate required signal power in dBm for adjusted SNR
    double signalPowerDbm = defaultNoisePowerDbm + adjustedSnr;

    Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel>();
    channel->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
    
    // Use FixedRssLossModel to set the calculated signal power
    Ptr<FixedRssLossModel> fixedLoss = CreateObject<FixedRssLossModel>();
    fixedLoss->SetRss(dBm_u(signalPowerDbm));
    channel->SetPropagationLossModel(fixedLoss);

    Ptr<MobilityModel> posTx = CreateObject<ConstantPositionMobilityModel>();
    posTx->SetPosition(Vector(0.0, 0.0, 0.0));
    Ptr<MobilityModel> posRx = CreateObject<ConstantPositionMobilityModel>();
    posRx->SetPosition(Vector(1.0, 0.0, 0.0)); // Fixed position, distance irrelevant

    Ptr<YansWifiPhy> tx = CreateObject<YansWifiPhy>();
    Ptr<YansWifiPhy> rx = CreateObject<YansWifiPhy>();
    
    // Create and set InterferenceHelper for both PHY objects
    Ptr<InterferenceHelper> txInterferenceHelper = CreateObject<InterferenceHelper>();
    tx->SetInterferenceHelper(txInterferenceHelper);
    
    Ptr<InterferenceHelper> rxInterferenceHelper = CreateObject<InterferenceHelper>();
    rx->SetInterferenceHelper(rxInterferenceHelper);
    
    // Ptr<ErrorRateModel> error = CreateObject<NistErrorRateModel>();
    // Ptr<YansErrorRateModel> error = CreateObject<YansErrorRateModel>();
    Ptr<TableBasedErrorRateModel> error = CreateObject<TableBasedErrorRateModel>();
    tx->SetErrorRateModel(error);
    rx->SetErrorRateModel(error);
    tx->SetChannel(channel);
    rx->SetChannel(channel);
    tx->SetMobility(posTx);
    rx->SetMobility(posRx);

    tx->ConfigureStandard(WIFI_STANDARD_80211a);
    rx->ConfigureStandard(WIFI_STANDARD_80211a);

    rx->SetReceiveOkCallback(MakeCallback(&SnrVsPsrExperiment::Receive, this));

    // Schedule packet transmissions
    for (uint32_t i = 0; i < m_input.nPackets; ++i)
    {
        Simulator::Schedule(Seconds(i), &SnrVsPsrExperiment::Send, this);
    }
    m_tx = tx;
    Simulator::Run();
    Simulator::Destroy();
    
    // Calculate average SNR and RSSI
    if (m_output.received > 0)
    {
        m_output.avgSnr = m_totalSnr / m_output.received;
        m_output.avgRssi = m_totalRssi / m_output.received;
    }
    else
    {
        m_output.avgSnr = 0.0;
        m_output.avgRssi = 0.0;
    }
    
    // Calculate packet error rate
    m_output.packetErrorRate = 1.0 - (static_cast<double>(m_output.received) / m_input.nPackets);
    
    return m_output;
}

static void
PrintPsr(int argc, char* argv[])
{
    PsrExperiment experiment;
    PsrExperiment::Input input;

    CommandLine cmd(__FILE__);
    cmd.AddValue("Distance", "The distance between two phys", input.distance);
    cmd.AddValue("PacketSize", "The size of each packet sent", input.packetSize);
    cmd.AddValue("TxMode", "The mode to use to send each packet", input.txMode);
    cmd.AddValue("NPackets", "The number of packets to send", input.nPackets);
    cmd.AddValue("TxPowerLevel",
                 "The power level index to use to send each packet",
                 input.txPowerLevel);
    cmd.Parse(argc, argv);

    PsrExperiment::Output output;
    output = experiment.Run(input);

    double psr = output.received;
    psr /= input.nPackets;

    std::cout << psr << std::endl;
}

double
CalcPsr(PsrExperiment::Output output, PsrExperiment::Input input)
{
    double psr = output.received;
    psr /= input.nPackets;
    return psr;
}

static void
PrintPsrVsDistance(int argc, char* argv[])
{
    PsrExperiment::Input input;
    CommandLine cmd(__FILE__);
    cmd.AddValue("TxPowerLevel",
                 "The power level index to use to send each packet",
                 input.txPowerLevel);
    cmd.AddValue("TxMode", "The mode to use to send each packet", input.txMode);
    cmd.AddValue("NPackets", "The number of packets to send", input.nPackets);
    cmd.AddValue("PacketSize", "The size of each packet sent", input.packetSize);
    cmd.Parse(argc, argv);

    for (input.distance = 1.0; input.distance < 165; input.distance += 2.0)
    {
        std::cout << input.distance;
        PsrExperiment experiment;
        PsrExperiment::Output output;

        input.txMode = "OfdmRate6Mbps";
        output = experiment.Run(input);
        std::cout << " " << CalcPsr(output, input);

        input.txMode = "OfdmRate9Mbps";
        output = experiment.Run(input);
        std::cout << " " << CalcPsr(output, input);

        input.txMode = "OfdmRate12Mbps";
        output = experiment.Run(input);
        std::cout << " " << CalcPsr(output, input);

        input.txMode = "OfdmRate18Mbps";
        output = experiment.Run(input);
        std::cout << " " << CalcPsr(output, input);

        input.txMode = "OfdmRate24Mbps";
        output = experiment.Run(input);
        std::cout << " " << CalcPsr(output, input);

        input.txMode = "OfdmRate36Mbps";
        output = experiment.Run(input);
        std::cout << " " << CalcPsr(output, input);

        input.txMode = "OfdmRate48Mbps";
        output = experiment.Run(input);
        std::cout << " " << CalcPsr(output, input);

        input.txMode = "OfdmRate54Mbps";
        output = experiment.Run(input);
        std::cout << " " << CalcPsr(output, input);

        std::cout << std::endl;
    }
}

static void
PrintSizeVsRange(int argc, char* argv[])
{
    double targetPsr = 0.05;
    PsrExperiment::Input input;
    CommandLine cmd(__FILE__);
    cmd.AddValue("TxPowerLevel",
                 "The power level index to use to send each packet",
                 input.txPowerLevel);
    cmd.AddValue("TxMode", "The mode to use to send each packet", input.txMode);
    cmd.AddValue("NPackets", "The number of packets to send", input.nPackets);
    cmd.AddValue("TargetPsr", "The psr needed to assume that we are within range", targetPsr);
    cmd.Parse(argc, argv);

    for (input.packetSize = 10; input.packetSize < 3000; input.packetSize += 40)
    {
        double precision = 0.1;
        double low = 1.0;
        double high = 200.0;
        while (high - low > precision)
        {
            double middle = low + (high - low) / 2;
            PsrExperiment::Output output;
            PsrExperiment experiment;
            input.distance = middle;
            output = experiment.Run(input);
            double psr = CalcPsr(output, input);
            if (psr >= targetPsr)
            {
                low = middle;
            }
            else
            {
                high = middle;
            }
        }
        std::cout << input.packetSize << " " << input.distance << std::endl;
    }
}

static void
PrintPsrVsCollisionInterval(int argc, char* argv[])
{
    CollisionExperiment::Input input;
    input.nPackets = 100;
    CommandLine cmd(__FILE__);
    cmd.AddValue("NPackets", "The number of packets to send for each transmitter", input.nPackets);
    cmd.AddValue("xA", "the position of transmitter A", input.xA);
    cmd.AddValue("xB", "the position of transmitter B", input.xB);
    cmd.Parse(argc, argv);

    for (uint32_t i = 0; i < 100; i += 1)
    {
        CollisionExperiment experiment;
        CollisionExperiment::Output output;
        input.interval = MicroSeconds(i);
        output = experiment.Run(input);
        double perA = (output.receivedA + 0.0) / (input.nPackets + 0.0);
        double perB = (output.receivedB + 0.0) / (input.nPackets + 0.0);
        std::cout << i << " " << perA << " " << perB << std::endl;
    }
    for (uint32_t i = 100; i < 4000; i += 50)
    {
        CollisionExperiment experiment;
        CollisionExperiment::Output output;
        input.interval = MicroSeconds(i);
        output = experiment.Run(input);
        double perA = (output.receivedA + 0.0) / (input.nPackets + 0.0);
        double perB = (output.receivedB + 0.0) / (input.nPackets + 0.0);
        std::cout << i << " " << perA << " " << perB << std::endl;
    }
}

static void
PrintSnrVsPer(int argc, char* argv[])
{
    SnrVsPsrExperiment::Input input;
    CommandLine cmd(__FILE__);
    
    // SNR test range parameters
    double startSnr = 0.0;   // Start SNR in dB
    double snrStep = 2.0;    // SNR step in dB
    double endSnr = 20.0;    // End SNR in dB
    
    // Output directory
    std::string outputDir = "/home/emu/dev/RealEmu-test/mcs-ns3";
    
    cmd.AddValue("PacketSize", "The size of each packet sent", input.packetSize);
    cmd.AddValue("TxMode", "The mode to use to send each packet", input.txMode);
    cmd.AddValue("NPackets", "The number of packets to send", input.nPackets);
    cmd.AddValue("TxPowerLevel",
                 "The power level index to use to send each packet",
                 input.txPowerLevel);
    cmd.AddValue("StartSnr", "Start SNR in dB", startSnr);
    cmd.AddValue("SnrStep", "SNR step between test points in dB", snrStep);
    cmd.AddValue("EndSnr", "End SNR in dB", endSnr);
    cmd.AddValue("OutputDir", "Directory to save output CSV files", outputDir);
    cmd.Parse(argc, argv);

    // Define different MCS modes to test
    std::vector<std::string> mcsModes = {
        "OfdmRate6Mbps",  // MCS 0
        "OfdmRate9Mbps",  // MCS 1
        "OfdmRate12Mbps", // MCS 2
        "OfdmRate18Mbps", // MCS 3
        "OfdmRate24Mbps", // MCS 4
        "OfdmRate36Mbps", // MCS 5
        "OfdmRate48Mbps", // MCS 6
        "OfdmRate54Mbps"  // MCS 7
    };

    // Print header
    std::cout << "TargetSNR(dB)\tActualSNR(dB)\tMCS\tReceived\tPER\tAvgRssi(dBm)" << std::endl;

    // Ensure output directory exists
    int mkdirResult = system(("mkdir -p " + outputDir).c_str());
    if (mkdirResult != 0)
    {
        std::cerr << "Warning: Failed to create output directory " << outputDir << std::endl;
    }

    // Test each MCS mode at different SNR levels
    for (size_t mcsIndex = 0; mcsIndex < mcsModes.size(); ++mcsIndex)
    {
        const auto& mode = mcsModes[mcsIndex];
        
        // Create CSV file for this MCS
        std::string csvFilename = outputDir + "/sinr_success_rate_scaled_mcs" + std::to_string(mcsIndex) + ".csv";
        std::ofstream csvFile(csvFilename);
        
        // Write CSV header
        csvFile << "SINR(dB),SuccessRate" << std::endl;
        
        for (double snr = startSnr; snr <= endSnr; snr += snrStep)
        {
            input.txMode = mode;
            input.targetSnr = snr;
            
            SnrVsPsrExperiment experiment;
            SnrVsPsrExperiment::Output output;
            output = experiment.Run(input);
            
            // Convert actual SNR from linear to dB for display
            double actualSnrDb = 10 * log10(output.avgSnr);
            
            // Calculate success rate (1 - PER)
            double successRate = 1.0 - output.packetErrorRate;
            
            // Write to console
            std::cout << snr << "\t" << actualSnrDb << "\t" << mode << "\t" << output.received << "\t";
            std::cout << output.packetErrorRate << "\t" << output.avgRssi << std::endl;
            
            // Write to CSV file
            csvFile << snr << "," << successRate << std::endl;
        }
        
        // Close CSV file
        csvFile.close();
        std::cout << "CSV file saved: " << csvFilename << std::endl;
    }
}

int
main(int argc, char* argv[])
{
    if (argc <= 1)
    {
        std::cout << "Available experiments: "
                  << "Psr "
                  << "SizeVsRange "
                  << "PsrVsDistance "
                  << "PsrVsCollisionInterval "
                  << "SnrVsPer " << std::endl;
        return 0;
    }
    std::string type = argv[1];
    argc--;
    argv[1] = argv[0];
    argv++;
    if (type == "Psr")
    {
        PrintPsr(argc, argv);
    }
    else if (type == "SizeVsRange")
    {
        PrintSizeVsRange(argc, argv);
    }
    else if (type == "PsrVsDistance")
    {
        PrintPsrVsDistance(argc, argv);
    }
    else if (type == "PsrVsCollisionInterval")
    {
        PrintPsrVsCollisionInterval(argc, argv);
    }
    else if (type == "SnrVsPer")
    {
        PrintSnrVsPer(argc, argv);
    }
    else
    {
        std::cout << "Wrong arguments!" << std::endl;
    }

    return 0;
}
