
//#define MSP_PORT_INBUF_SIZE 192
//GPS packets are smaller, we don't need so much space
#define MSP_PORT_INBUF_SIZE 80

#define MSP2_SENSOR_GPS                 0x1F03

typedef struct __attribute__((packed)) {
    uint8_t  flags;
    uint16_t cmd;
    uint16_t size;
} mspHeaderV2_t;

typedef enum {
    MSP_IDLE,
    MSP_HEADER_START,
    MSP_HEADER_M,
    MSP_HEADER_X,

    MSP_HEADER_V1,
    MSP_PAYLOAD_V1,
    MSP_CHECKSUM_V1,

    MSP_HEADER_V2_OVER_V1,
    MSP_PAYLOAD_V2_OVER_V1,
    MSP_CHECKSUM_V2_OVER_V1,

    MSP_HEADER_V2_NATIVE,
    MSP_PAYLOAD_V2_NATIVE,
    MSP_CHECKSUM_V2_NATIVE,

    MSP_COMMAND_RECEIVED
} mspPacketState_e;

typedef enum {
    MSP_PACKET_COMMAND,
    MSP_PACKET_REPLY
} mspPacketType_e;

typedef struct mspPort_s {
    //timeMs_t lastActivityMs;
    //mspPendingSystemRequest_e pendingRequest;
    //mspPortState_e portState;
    mspPacketState_e packetState;
    mspPacketType_e packetType;
    uint8_t inBuf[MSP_PORT_INBUF_SIZE];
    uint16_t cmdMSP;
    uint8_t cmdFlags;
    //mspVersion_e mspVersion;
    uint_fast16_t offset;
    uint_fast16_t dataSize;
    uint8_t checksum1;
    uint8_t checksum2;
    //bool sharedWithTelemetry;
    //mspDescriptor_t descriptor;
} mspPacket_t;

typedef struct __attribute__((packed)){
    uint8_t instance;
    uint16_t gps_week;
    uint32_t ms_tow;
    uint8_t fix_type;
    uint8_t satellites_in_view;
    uint16_t horizontal_pos_accuracy;
    uint16_t vertical_pos_accuracy;
    uint16_t horizontal_vel_accuracy;
    uint16_t pdop;
    int32_t lon;
    int32_t lat;
    int32_t altCm;
    int32_t ned_vel_north;
    int32_t ned_vel_east;
    int32_t ned_vel_down;
    int16_t ground_course;
    int16_t true_yaw;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
} mspGPSdat_t;


uint8_t crc8_calc(uint8_t crc, unsigned char a, uint8_t poly);
uint8_t crc8_update(uint8_t crc, const void *data, uint32_t length, uint8_t poly);
#define crc8_dvb_s2(crc, a)                         crc8_calc(crc, a, 0xD5)
#define crc8_dvb_s2_update(crc, data, length)       crc8_update(crc, data, length, 0xD5)
