// Copyright (c) 2021, BEA
// Copyright (c) 2021, BEA
// All rights reserved.

#ifndef PACKET_STRUCTURE_H
#define PACKET_STRUCTURE_H
#include <cstdint>
#include <vector>

namespace bea_power {

#pragma pack(1)
//@ struct PacketHeader
//@ Header of a  UDP data packet from the scanner
struct PacketHeader
{
    //  BE A0 12 34 (hex)
    std::uint32_t SYNC;
    std::uint8_t PacketType;
    std::uint16_t PacketSize;
    std::uint16_t ReservedA;
    std::uint16_t ReservedB;
    std::uint16_t ReservedC;
    std::uint16_t PacketNO;
    std::uint8_t TotalNO;
    std::uint8_t SubNO;
    std::uint16_t ScanFreq;
    std::uint16_t ScanPoints;
    std::int32_t FirstAngle;
    std::int32_t DeltaAngle;
    std::uint16_t Timestamp;

};

//@ \brief Structure of a UDP or TCP data packet from the laserscanner
struct PacketTypeC
{
    PacketHeader header;
    //std::uint32_t distance_amplitude_payload; // distance 20 bit, amplitude 12 bit
};
#pragma pack()

//@ \struct ScanData
//@ \brief Normally contains one complete laserscan (a full rotation of the scanner head)
struct ScanData
{
    //@ Distance data in polar form in millimeter
    std::vector<std::uint16_t> distance_data;

    //@ Amplitude data in the range 32-4095, values lower than 32 indicate an error or undefined values
    std::vector<std::uint16_t> amplitude_data;

    //@ Header received with the distance and amplitude data
    std::vector<PacketHeader> headers;
};

}

#endif // PACKET_STRUCTURE_H
