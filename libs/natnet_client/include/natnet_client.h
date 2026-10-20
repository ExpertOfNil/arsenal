#ifndef NNCLIENT_H
#define NNCLIENT_H

#include <arpa/inet.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Definitions based on NatNetTypes.h
#define NNCLIENT_PORT_COMMAND 1510
#define NNCLIENT_PORT_DATA 1511
#define NNCLIENT_MAX_STRING_SIZE 256u
#define NNCLIENT_MAX_STRING_LENGTH (NNCLIENT_MAX_STRING_SIZE - 1u)
#define NNCLIENT_PACKET_HEADER_SIZE 4u
#define NNCLIENT_MAX_PAYLOAD_SIZE 65503u
#define NNCLIENT_MAX_PACKET_SIZE \
    (NNCLIENT_MAX_PAYLOAD_SIZE + NNCLIENT_PACKET_HEADER_SIZE)

#ifndef NNCLIENT_KEEPALIVE_REQUEST_INTERVAL_SEC
#define NNCLIENT_KEEPALIVE_REQUEST_INTERVAL_SEC 1
#endif

typedef enum {
    NNCLIENT_INITIALIZED = 0,
    NNCLIENT_RUNNING,
    NNCLIENT_STOP_REQUESTED,
    NNCLIENT_STOP_FAILURE,
} NnClientRunState;

typedef enum {
    NNCLIENT_CONNECT = 0,
    NNCLIENT_SERVERINFO,
    NNCLIENT_REQUEST,
    NNCLIENT_RESPONSE,
    NNCLIENT_REQUEST_MODELDEF,
    NNCLIENT_MODELDEF,
    NNCLIENT_REQUEST_FRAMEOFDATA,
    NNCLIENT_FRAMEOFDATA,
    NNCLIENT_MESSAGESTRING,
    NNCLIENT_DISCONNECT,
    NNCLIENT_KEEPALIVE,
    NNCLIENT_DISCONNECTBYTIMEOUT,
    NNCLIENT_ECHOREQUEST,
    NNCLIENT_ECHORESPONSE,
    NNCLIENT_DISCOVERY,
    NNCLIENT_UNRECOGNIZED_REQUEST = 100,
} NnMessageId;

typedef struct NnPacketView {
    NnMessageId message_id;
    size_t payload_size;
    const uint8_t* payload;
    const uint8_t* wire_data;
    size_t wire_size;
} NnPacketView;

typedef struct Version {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    uint8_t build;
} Version;

typedef void (*NnPacketHandler)(const NnPacketView* packet, void* userdata);

typedef enum {
    NNCLIENT_LOG_TRACE = 0,
    NNCLIENT_LOG_DEBUG,
    NNCLIENT_LOG_INFO,
    NNCLIENT_LOG_WARNING,
    NNCLIENT_LOG_ERROR,
} NnClientLogLevel;

typedef void (*NnClientLogHandler)(
    NnClientLogLevel level, int error, const char* message, void* userdata
);

typedef struct NnClientConfig {
    struct in_addr motive_addr;
    struct in_addr host_addr;
    Version version;
    void* handle_command_packet_data;
    NnPacketHandler handle_command_packet;
    void* handle_data_packet_data;
    NnPacketHandler handle_data_packet;
    void* handle_log_data;
    NnClientLogHandler handle_log;
} NnClientConfig;

typedef struct NnClient NnClient;

int NnClient_create(const NnClientConfig* cfg, NnClient** out);
int NnClient_start(NnClient* client);
NnClientRunState NnClient_get_run_state(const NnClient* client);
bool NnClient_is_connected(const NnClient* client);
int NnClient_send_connect(const NnClient* client);
int NnClient_send_modeldef_request(const NnClient* client);
bool NnClient_request_stop(NnClient* client);
int NnClient_destroy(NnClient* client);

// TODO(mmckenna): Make private?
int NnPacket_compose_empty_request(
    uint8_t* dst,
    size_t dst_capacity,
    NnMessageId message_id,
    size_t* packet_size
);
int NnPacket_compose_modeldef_request(
    uint8_t* dst, size_t dst_capacity, size_t* packet_size
);
int NnPacket_compose_connect(
    uint8_t* dst, size_t dst_capacity, Version version, size_t* packet_size
);

#endif /* NNCLIENT_H */
