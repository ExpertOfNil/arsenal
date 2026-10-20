#ifndef HMS_WEBSOCKET_CLIENT
#define HMS_WEBSOCKET_CLIENT

#include <libwebsockets.h>
#include <stdbool.h>
#include <stdint.h>

#define container_of(ptr, type, member) \
    ((type*)((char*)(ptr) - offsetof(type, member)))

typedef struct WsClient WsClient;

/* COMPATIBILITY LAYER */
#ifndef LWS_PROTOCOL_LIST_TERM
#define LWS_PROTOCOL_LIST_TERM {NULL, NULL, 0, 0, 0, NULL, 0}
#endif

#ifndef lws_wsi_close
#define lws_wsi_close(w, to_kill) lws_set_timeout(w, 1, to_kill)
#endif
/**/

// TODO (mmckenna) : Create note about 0 for success, -errno on error

/**
 * @struct WsEndpoint
 * @brief
 * @var address - required, address to connect to
 * @var port - required, endpoint port
 * @var path - required, endpoint path, e.g. "/ws"
 * @var host - optional, content of host header, defaults to `address`
 * @var origin - optional, content of origin header, defaults to `address`
 * @var protocol - optional, protocol name, e.g. "my-protocol"
 * @var ssl_flags - required, LCCSCF_* flags, 0 for ws://
 * @var iface - optional, bind interface
 */
typedef struct WsEndpoint {
    const char* address;
    int port;
    const char* path;
    const char* host;
    const char* origin;
    const char* protocol;
    int ssl_flags;
    const char* iface;
} WsEndpoint;

typedef enum {
    WS_RETRY_DEFAULT = 0,
    WS_RETRY_DISABLED,
    WS_RETRY_LIMITED,
    WS_RETRY_UNLIMITED,
} WsRetryMode;

// Zero size and delay values will signal to use defaults
// WS_RETRY_DEFAULT resolves to the library default
// max_retries only used with WS_RETRY_LIMITED and may be zero
typedef struct WsClientOptions {
    size_t tx_max_payload;
    size_t rx_buffer_size;
    WsRetryMode retry_mode;
    // Number of retries after the initial attempt.  0 disables retries.
    uint32_t max_retries;
    lws_usec_t retry_delay_us;
    bool ws_compression_enabled;
} WsClientOptions;

// NOTE: Do not call `WsClient_destroy` from callbacks
typedef struct WsClientHandlers {
    void (*on_connected)(WsClient* c, void* user);
    void (*on_message)(
        WsClient* c, const uint8_t* data, size_t len, void* user
    );
    void (*on_closed)(WsClient* c, void* user);
    void (*on_error)(WsClient* c, int err_code, const char* msg, void* user);
    void (*on_raw_readable)(WsClient* c, int fd, void* user);
    void* user;
} WsClientHandlers;

typedef enum {
    WS_ADOPT_FD_GENERIC,
    WS_ADOPT_FD_EVENTFD,
} WsAdoptFdType;

typedef struct WsAdoptFdInfo {
    int fd;
    WsAdoptFdType type;
} WsAdoptFdInfo;

int WsClient_create(
    WsClient** out,
    const WsEndpoint* endpoint,
    const WsClientOptions* opts,
    const WsClientHandlers* handlers
);

void WsClient_destroy(WsClient* client);
int WsClient_connect(WsClient* client);
int WsClient_disconnect(WsClient* client, size_t timeout_ms);
int WsClient_service(WsClient* client);
int WsClient_send_binary(WsClient* client, const void* data, size_t len);
int WsClient_send_text(WsClient* client, const void* data, size_t len);
bool WsClient_is_connected(const WsClient* client);
void WsClient_interrupt(WsClient* client);
int WsClient_adopt_fd(WsClient* client, WsAdoptFdInfo info);
int WsClient_reconnect(WsClient* client);
void WsClient_wake(WsClient* client);

#endif /* HMS_WEBSOCKET_CLIENT */
