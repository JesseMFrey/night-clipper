
#include "MSP_GPS.h"
#include <AlfredoCRSF.h>
#include <math.h>

static uint8_t crc8_calc(uint8_t crc, unsigned char a, uint8_t poly);
static uint8_t crc8_update(uint8_t crc, const void *data, uint32_t length, uint8_t poly);
#define crc8_dvb_s2(crc, a)                         crc8_calc(crc, a, 0xD5)
#define crc8_dvb_s2_update(crc, data, length)       crc8_update(crc, data, length, 0xD5)

mspPacket_t gps_stat = {.packetState=MSP_IDLE};


// This is a stripped down version of the beta flight MSP processing code
void mspGPS_ProcessReceivedPacketData(mspPacket_t *mspPacket, uint8_t c)
{
    switch (mspPacket->packetState) {
        default:
        case MSP_IDLE:
        case MSP_HEADER_START:  // Waiting for 'X' (MSPv2 native)
            mspPacket->offset = 0;
            mspPacket->checksum1 = 0;
            mspPacket->checksum2 = 0;
            switch (c) {
                case 'X':
                    mspPacket->packetState = MSP_HEADER_X;
                    break;
                default:
                    mspPacket->packetState = MSP_IDLE;
                    break;
            }
            break;

        case MSP_HEADER_X:
            mspPacket->packetState = MSP_HEADER_V2_NATIVE;
            switch (c) {
                case '<':
                    mspPacket->packetType = MSP_PACKET_COMMAND;
                    break;
                case '>':
                    mspPacket->packetType = MSP_PACKET_REPLY;
                    break;
                default:
                    mspPacket->packetState = MSP_IDLE;
                    break;
            }
            break;

        case MSP_HEADER_V2_NATIVE:
            mspPacket->inBuf[mspPacket->offset++] = c;
            mspPacket->checksum2 = crc8_dvb_s2(mspPacket->checksum2, c);
            if (mspPacket->offset == sizeof(mspHeaderV2_t)) {
                mspHeaderV2_t * hdrv2 = (mspHeaderV2_t *)&mspPacket->inBuf[0];
                mspPacket->dataSize = hdrv2->size;
                mspPacket->cmdMSP = hdrv2->cmd;
                mspPacket->cmdFlags = hdrv2->flags;
                mspPacket->offset = 0;                // re-use buffer
                mspPacket->packetState = mspPacket->dataSize > 0 ? MSP_PAYLOAD_V2_NATIVE : MSP_CHECKSUM_V2_NATIVE;
            }
            break;

        case MSP_PAYLOAD_V2_NATIVE:
            mspPacket->checksum2 = crc8_dvb_s2(mspPacket->checksum2, c);
            mspPacket->inBuf[mspPacket->offset++] = c;

            if (mspPacket->offset == mspPacket->dataSize) {
                mspPacket->packetState = MSP_CHECKSUM_V2_NATIVE;
            }
            break;

        case MSP_CHECKSUM_V2_NATIVE:
            if (mspPacket->checksum2 == c) {
                mspPacket->packetState = MSP_COMMAND_RECEIVED;
            } else {
                mspPacket->packetState = MSP_IDLE;
            }
            break;
    }
}


static uint8_t crc8_calc(uint8_t crc, unsigned char a, uint8_t poly)
{
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ poly;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

static uint8_t crc8_update(uint8_t crc, const void *data, uint32_t length, uint8_t poly)
{
    const uint8_t *p = (const uint8_t *)data;
    const uint8_t *pend = p + length;

    for (; p != pend; p++) {
        crc = crc8_calc(crc, *p, poly);
    }
    return crc;
}

uint16_t get_uint16(const void *ptr)
{
  return (*(uint8_t*)ptr) | ((*(uint8_t*)ptr+1)<<8);
}

uint32_t get_uint32(const void *ptr)
{
  uint8_t *bptr=(uint8_t*)ptr;
  return (bptr[0]) | (bptr[1]<<8) | (bptr[2]<<16) | (bptr[3]<<24);
}

void populate_gps_packet(crsf_sensor_gps_t *crsf_packet, const mspGPSdat_t *msp_packet)
{
  int32_t lat, lon, alt;
  uint16_t groundspeed;
  int32_t ned_v_north, ned_v_east;
  int16_t heading;

  // get lat and lon in 1/1e7 ths of a degree, no change needed
  lat = get_uint32(&msp_packet->lat);
  lon = get_uint32(&msp_packet->lon);
  //get velocities for groundspeed
  ned_v_north = get_uint32(&msp_packet->ned_vel_north);
  ned_v_east = get_uint32(&msp_packet->ned_vel_east);
  groundspeed = sqrt(ned_v_north*ned_v_north + ned_v_east*ned_v_east)/10;
  //heading value in 100ths of a degree, convert to 100ths of a degree
  heading = (msp_packet->ground_course % 36000);
  alt = get_uint32(&msp_packet->altCm)/100 + 1000;

  // Values are MSB first (BigEndian)
  crsf_packet->latitude = htobe32(lat);
  crsf_packet->longitude = htobe32(lon);
  crsf_packet->groundspeed = htobe16(groundspeed);
  crsf_packet->heading = htobe16(heading); //TODO: test heading
  crsf_packet->altitude = htobe16(alt);
  crsf_packet->satellites = msp_packet->satellites_in_view;
}
