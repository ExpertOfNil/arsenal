#include "ws_client.h"

#include <assert.h>
#include <errno.h>
#include <libwebsockets.h>
#include <lws_config.h>
#include <pthread.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "logging.h"

#define WS_DEFAULT_TX_MAX_PAYLOAD 65536u
#define WS_DEFAULT_RX_BUFFER_SIZE 65536u
#define WS_DEFAULT_RETRY_MODE WS_RETRY_UNLIMITED
#define WS_DEFAULT_RETRY_DELAY_US (2 * LWS_US_PER_SEC)
#define WS_DEFAULT_PROTOCOL_NAME "default-proto"
#define WS_DEFAULT_VHOST_NAME "default"

static const struct lws_extension extensions[] = {
    {
        "permessage-deflate",
        lws_extension_callback_pm_deflate,
        "permessage-deflate; client_max_window_bits",
    },
    {NULL, NULL, NULL},
};

typedef enum {
    WS_STATE_DISCONNECTED = 0,
    WS_STATE_CONNECTING,
    WS_STATE_CONNECTED,
    WS_STATE_RETRY_WAIT,
    WS_STATE_CLOSING
} WsClientState;

/**
 * @struct WsClient
 * @brief
 * @var ctx - lws context handle
 * @var wsi - lws instance handle
 * @var sul - lws sorted usec list (service requests)
 * @var ccinfo - client connect info, retained for reconnects
 * @var opt - client behavior
 * @var cb - callback function pointers and user pointer
 * @var state - connection state
 * @var retries_left - number of connection retries before giving up
 * @var interrupted - flag tracking if the stream is interrupted
 * @var owns_ctx - flag tracking if the context is currently owned
 * @var tx_buf - message buffer, allocated to tx_capacity + LWS_PRE
 * @var tx_capacity - capacity of message buffer (excluding LWS_PRE)
 * @var tx_len - pending payload length
 * @var tx_type - pending payload type
 * @var tx_pending - flag tracking if a outgoing message is pending
 */
struct WsClient {
    // lws handles / runtime
    struct lws_context* ctx;
    struct lws* wsi;
    struct lws_sorted_usec_list sul;
    struct lws_client_connect_info ccinfo;
    struct lws_protocols protocols[3];
    // configuration
    WsClientOptions opt;
    WsClientHandlers cb;
    // connection state
    WsClientState state;
    uint32_t retries_left;
    bool interrupted;
    bool owns_ctx;
    // tx state
    pthread_mutex_t tx_lock;
    uint8_t* tx_buf;
    size_t tx_capacity;
    size_t tx_len;
    enum lws_write_protocol tx_type;
    bool tx_pending;
    char* ep_storage;
    int raw_fd;
    WsAdoptFdType raw_fd_type;
    struct lws* raw_wsi;
};

#if LWS_LIBRARY_VERSION_MAJOR < 4
#define LWS_PRE_V4_ADOPT 1
static inline void lws_sul_cancel_compat(lws_sorted_usec_list_t* sul) {
    WsClient* c = container_of(sul, WsClient, sul);
    lws_sul_schedule(c->ctx, 0, sul, sul->cb, LWS_SET_TIMER_USEC_CANCEL);
}
#define lws_sul_cancel lws_sul_cancel_compat
#else
#define LWS_PRE_V4_ADOPT 0
#endif

// Forward declarations for internal functions
static int ws_options_resolve(
    const WsClientOptions* supplied, WsClientOptions* resolved
);
static int ws_callback(
    struct lws* wsi,
    enum lws_callback_reasons reason,
    void* user,
    void* in,
    size_t len
);
static int ws_connect(WsClient* client);
static void ws_retry_cb(struct lws_sorted_usec_list* sul_ptr);
static void ws_schedule_retry(WsClient* client);
static int ws_raw_file_callback(
    struct lws* wsi,
    enum lws_callback_reasons reason,
    void* user,
    void* in,
    size_t len
);
static int ws_endpoint_clone(WsClient* client, const WsEndpoint* ep);
static int ws_send_message(
    WsClient* client, enum lws_write_protocol type, const void* data, size_t len
);

// Internal function definitions

static int ws_options_resolve(
    const WsClientOptions* supplied, WsClientOptions* resolved
) {
    *resolved = supplied ? *supplied : (WsClientOptions){0};
    switch (resolved->retry_mode) {
        case WS_RETRY_DEFAULT: {
            resolved->retry_mode = WS_DEFAULT_RETRY_MODE;
        } break;
        case WS_RETRY_DISABLED:
        case WS_RETRY_LIMITED:
        case WS_RETRY_UNLIMITED:
            break;
        default:
            return -EINVAL;
    }

    if (resolved->retry_mode != WS_RETRY_LIMITED) {
        resolved->max_retries = 0;
    }

    // Zero values means use default
    if (resolved->retry_delay_us == 0) {
        resolved->retry_delay_us = WS_DEFAULT_RETRY_DELAY_US;
    }
    if (resolved->tx_max_payload == 0) {
        resolved->tx_max_payload = WS_DEFAULT_TX_MAX_PAYLOAD;
    }
    if (resolved->rx_buffer_size == 0) {
        resolved->rx_buffer_size = WS_DEFAULT_RX_BUFFER_SIZE;
    }
    return 0;
}

static int ws_connect(WsClient* client) {
    /* Context */
    client->ccinfo.local_protocol_name = client->protocols[0].name;
    client->ccinfo.opaque_user_data = client;
    client->ccinfo.pwsi = &client->wsi;
    client->ccinfo.context = client->ctx;

    client->state = WS_STATE_CONNECTING;
    client->wsi = lws_client_connect_via_info(&client->ccinfo);
    if (client->wsi == NULL) {
        if (client->state == WS_STATE_CONNECTING) {
            client->state = WS_STATE_DISCONNECTED;
            ws_schedule_retry(client);
        }
        return -ECONNREFUSED;
    }
    return 0;
}

static void ws_retry_cb(struct lws_sorted_usec_list* sul_ptr) {
    WsClient* c = container_of(sul_ptr, WsClient, sul);
    if (c->interrupted || c->wsi) return;
    (void)ws_connect(c);
}

static void ws_schedule_retry(WsClient* client) {
    if (client->interrupted || client->state == WS_STATE_RETRY_WAIT) {
        return;
    }

    switch (client->opt.retry_mode) {
        case WS_RETRY_DISABLED:
            return;

        case WS_RETRY_LIMITED: {
            if (client->retries_left == 0) {
                log_error("WebSocket reconnect limit reached\n");
                return;
            }
            --client->retries_left;
        } break;

        case WS_RETRY_UNLIMITED:
            break;

        case WS_RETRY_DEFAULT:
        default:
            assert(false && "Found unnormalized retry_mode");
            return;
    }

    client->state = WS_STATE_RETRY_WAIT;
    if (client->opt.retry_mode == WS_RETRY_LIMITED) {
        log_info(
            "WebSocket client scheduling reconnect.  Retries remaining=%u\n",
            client->retries_left
        );
    } else {
        log_info("WebSocket client scheduling reconnect\n");
    }
    lws_sul_schedule(
        client->ctx, 0, &client->sul, ws_retry_cb, client->opt.retry_delay_us
    );
}

static int ws_callback(
    struct lws* wsi,
    enum lws_callback_reasons reason,
    void* user,
    void* in,
    size_t len
) {
    (void)user;
    WsClient* client = (WsClient*)lws_get_opaque_user_data(wsi);
    if (client == NULL) return 0;
    switch (reason) {
        case LWS_CALLBACK_EVENT_WAIT_CANCELLED: {
            pthread_mutex_lock(&client->tx_lock);
            bool pending = client->tx_pending;
            pthread_mutex_unlock(&client->tx_lock);
            log_debug(
                "EVENT_WAIT_CANCELLED: pending=%s\n", pending ? "true" : "false"
            );
            if (client->wsi && pending) {
                lws_callback_on_writable(client->wsi);
            }
        } break;
        case LWS_CALLBACK_CLIENT_ESTABLISHED: {
            client->state = WS_STATE_CONNECTED;
            client->retries_left = client->opt.max_retries;
            lws_sul_cancel(&client->sul);
            log_info("WebSocket client connected\n");
            if (client->cb.on_connected) {
                client->cb.on_connected(client, client->cb.user);
            }
            pthread_mutex_lock(&client->tx_lock);
            bool pending = client->tx_pending;
            pthread_mutex_unlock(&client->tx_lock);
            if (client->wsi && pending) lws_callback_on_writable(wsi);
        } break;

        case LWS_CALLBACK_CLIENT_WRITEABLE: {
            pthread_mutex_lock(&client->tx_lock);
            log_trace(
                "WRITEABLE: pending=%d len=%zu\n",
                client->tx_pending,
                client->tx_len
            );

            // No message pending, nothing to do
            if (!client->tx_pending) {
                pthread_mutex_unlock(&client->tx_lock);
                break;
            }

            int result = lws_write(
                wsi,
                client->tx_buf + LWS_PRE,
                client->tx_len,
                (enum lws_write_protocol)client->tx_type
            );
            if (result < 0 || (size_t)result != client->tx_len) {
                client->tx_pending = false;
                client->tx_len = 0;
                pthread_mutex_unlock(&client->tx_lock);
                if (client->cb.on_error) {
                    client->cb.on_error(
                        client, result, "lws_write failed", client->cb.user
                    );
                }
                return -EIO;
            }
            client->tx_pending = false;
            client->tx_len = 0;
            pthread_mutex_unlock(&client->tx_lock);

        } break;

        case LWS_CALLBACK_CLIENT_RECEIVE: {
            log_debug("WebSocket client received %zu bytes\n", len);
            if (client->cb.on_message) {
                client->cb.on_message(
                    client, (const uint8_t*)in, len, client->cb.user
                );
            }
        } break;

        case LWS_CALLBACK_CLIENT_CONNECTION_ERROR: {
            log_error("Connection error: %s\n", in ? (char*)in : "unknown");
            if (client->cb.on_error) {
                client->cb.on_error(
                    client,
                    -ECONNREFUSED,
                    in ? (const char*)in : "WebSocket connection error",
                    client->cb.user
                );
            }
        }
            // fallthrough and close

        case LWS_CALLBACK_CLIENT_CLOSED: {
            log_info("WebSocket client connection closed\n");
            if (wsi == client->wsi) {
                pthread_mutex_lock(&client->tx_lock);
                client->tx_pending = false;
                client->tx_len = 0;
                pthread_mutex_unlock(&client->tx_lock);
                client->state = WS_STATE_DISCONNECTED;
                client->wsi = NULL;
                if (reason == LWS_CALLBACK_CLIENT_CLOSED &&
                    client->cb.on_closed) {
                    client->cb.on_closed(client, client->cb.user);
                }
                ws_schedule_retry(client);
            }
        } break;

        default:
            break;
    }
    return 0;
}

static int ws_raw_file_callback(
    struct lws* wsi,
    enum lws_callback_reasons reason,
    void* user,
    void* in,
    size_t len
) {
    (void)user;
    (void)in;
    (void)len;

    WsClient* client = (WsClient*)lws_get_opaque_user_data(wsi);
    if (!client) return 0;

    switch (reason) {
        case LWS_CALLBACK_RAW_ADOPT_FILE: {
            log_debug("Raw fd adopted\n");
        } break;
        case LWS_CALLBACK_RAW_RX_FILE: {
            if (client->raw_fd_type == WS_ADOPT_FD_EVENTFD) {
                // Drain eventfd so poll() doesn't spin
                uint64_t val;
                if (read(client->raw_fd, &val, sizeof(val)) != sizeof(val)) {
                    if (errno != EAGAIN && errno != EWOULDBLOCK) {
                        log_debug("eventfd read error: %s\n", strerror(errno));
                    }
                }
            }
            if (client->cb.on_raw_readable) {
                client->cb.on_raw_readable(
                    client, client->raw_fd, client->cb.user
                );
            }
        } break;
        case LWS_CALLBACK_RAW_CLOSE_FILE: {
            log_info("Raw fd closed\n");
            client->raw_fd = -1;
            client->raw_wsi = NULL;
        } break;
        default:
            break;
    }

    return 0;
}

static int ws_endpoint_clone(WsClient* client, const WsEndpoint* ep) {
    // Check required members
    if (!ep || !ep->address || !ep->path || ep->port <= 0 ||
        ep->port > UINT16_MAX) {
        return -EINVAL;
    }

    // Deep-copy + normalize defaults once
    // TODO (mmckenna) : Need to ensure no malformed values
    size_t address_len = strlen(ep->address) + 1;
    size_t total_len = address_len;

    size_t path_len = strlen(ep->path) + 1;
    total_len += path_len;

    size_t host_len = 0;
    if (ep->host) {
        host_len = strlen(ep->host) + 1;
    } else {
        host_len = strlen(ep->address) + 1;
    }
    total_len += host_len;

    size_t origin_len = 0;
    if (ep->origin) {
        origin_len = strlen(ep->origin) + 1;
    } else {
        origin_len = strlen(ep->address) + 1;
    }
    total_len += origin_len;

    size_t protocol_len = 0;
    if (ep->protocol) {
        protocol_len = strlen(ep->protocol) + 1;
    }
    total_len += protocol_len;

    size_t iface_len = 0;
    if (ep->iface) {
        iface_len = strlen(ep->iface) + 1;
        total_len += iface_len;
    }

    // Reallocate if already allocated
    char* new_storage = NULL;
    if (client->ep_storage) {
        new_storage = realloc(client->ep_storage, total_len);
    } else {
        new_storage = malloc(total_len);
    }

    if (!new_storage) {
        return -ENOMEM;
    }
    client->ep_storage = new_storage;

    // Copy integral members
    memset(&client->ccinfo, 0, sizeof(client->ccinfo));
    client->ccinfo.port = ep->port;
    client->ccinfo.ssl_connection = ep->ssl_flags;

    char* cursor = client->ep_storage;
    memcpy(cursor, ep->address, address_len);
    client->ccinfo.address = cursor;
    cursor += address_len;

    memcpy(cursor, ep->path, path_len);
    client->ccinfo.path = cursor;
    cursor += path_len;

    memcpy(cursor, ep->host ? ep->host : ep->address, host_len);
    client->ccinfo.host = cursor;
    cursor += host_len;

    memcpy(cursor, ep->origin ? ep->origin : ep->address, origin_len);
    client->ccinfo.origin = cursor;
    cursor += origin_len;

    if (ep->protocol) {
        memcpy(cursor, ep->protocol, protocol_len);
        client->ccinfo.protocol = cursor;
        cursor += protocol_len;
    } else {
        client->ccinfo.protocol = NULL;
    }

    if (ep->iface) {
        memcpy(cursor, ep->iface, iface_len);
        client->ccinfo.iface = cursor;
        cursor += iface_len;
    } else {
        client->ccinfo.iface = NULL;
    }

    return 0;
}

static int ws_send_message(
    WsClient* client, enum lws_write_protocol type, const void* data, size_t len
) {
    if (!client || !data) return -EINVAL;
    if (len == 0) return -EINVAL;
    if (!client->wsi || client->state != WS_STATE_CONNECTED) return -ENOTCONN;
    if (len > client->tx_capacity) return -EMSGSIZE;

    pthread_mutex_lock(&client->tx_lock);
    if (client->tx_pending) {
        pthread_mutex_unlock(&client->tx_lock);
        return -EAGAIN;
    }
    memcpy(client->tx_buf + LWS_PRE, data, len);
    client->tx_len = len;
    client->tx_pending = true;
    client->tx_type = type;
    pthread_mutex_unlock(&client->tx_lock);

    log_trace(
        "Message type=%d, len=%zu. Calling lws_cancel_service\n", type, len
    );
    if (client->wsi) lws_callback_on_writable(client->wsi);
    lws_cancel_service(client->ctx);
    return 0;
}

/* PUBLIC API */

int WsClient_create(
    WsClient** out,
    const WsEndpoint* ep,
    const WsClientOptions* opt,
    const WsClientHandlers* handlers
) {
    if (!out) return -EINVAL;
    *out = NULL;

    if (!ep || !ep->address || !ep->path || ep->port <= 0) {
        return -EINVAL;
    }

    WsClientOptions resolved;
    int result = ws_options_resolve(opt, &resolved);
    if (result < 0) return result;

    WsClient* client = calloc(1, sizeof(*client));
    if (!client) return -ENOMEM;

    client->opt = resolved;
    client->raw_fd = -1;
    client->raw_fd_type = WS_ADOPT_FD_EVENTFD;
    client->raw_wsi = NULL;

    result = ws_endpoint_clone(client, ep);
    if (result < 0) {
        free(client);
        return result;
    }

    client->cb = (handlers ? *handlers : (WsClientHandlers){0});

    // Create protocols table
    client->protocols[0] = (struct lws_protocols){
        .name = client->ccinfo.protocol ? client->ccinfo.protocol
                                        : WS_DEFAULT_PROTOCOL_NAME,
        .callback = ws_callback,
        .per_session_data_size = 0,
        .rx_buffer_size = client->opt.rx_buffer_size
    };
    client->protocols[1] = (struct lws_protocols){
        .name = "raw-file-protocol",
        .callback = ws_raw_file_callback,
        .per_session_data_size = 0,
        .rx_buffer_size = 0,
    };
    client->protocols[2] = (struct lws_protocols)LWS_PROTOCOL_LIST_TERM;

    // Create message buffer
    client->tx_capacity = client->opt.tx_max_payload;
    client->tx_buf = malloc(LWS_PRE + client->tx_capacity);
    if (!client->tx_buf) {
        free(client->ep_storage);
        free(client);
        log_error("Failed to allocate WebSocket message buffer\n");
        return -ENOMEM;
    }
    if (pthread_mutex_init(&client->tx_lock, NULL)) {
        free(client->ep_storage);
        free(client->tx_buf);
        free(client);
        return -ENOMEM;
    }

    // Create context
    struct lws_context_creation_info info;
    memset(&info, 0, sizeof(info));
    info.port = CONTEXT_PORT_NO_LISTEN;
    info.protocols = client->protocols;
    info.vhost_name = WS_DEFAULT_VHOST_NAME;

    if (ep->ssl_flags & LCCSCF_USE_SSL) {
        info.options |= LWS_SERVER_OPTION_DO_SSL_GLOBAL_INIT;
    }
    info.options |= LWS_SERVER_OPTION_DISABLE_IPV6;
    info.extensions = client->opt.ws_compression_enabled ? extensions : NULL;

    client->ctx = lws_create_context(&info);
    if (!client->ctx) {
        pthread_mutex_destroy(&client->tx_lock);
        free(client->ep_storage);
        free(client->tx_buf);
        free(client);
        return -EIO;
    }

    client->owns_ctx = true;
    client->retries_left = client->opt.max_retries;
    client->state = WS_STATE_DISCONNECTED;
    *out = client;
    return 0;
}

int WsClient_connect(WsClient* client) {
    if (!client) return -EINVAL;
    if (client->state == WS_STATE_CONNECTED ||
        client->state == WS_STATE_CONNECTING ||
        client->state == WS_STATE_CLOSING)
        return 0;
    client->interrupted = false;
    return ws_connect(client);
}

/* Ensure clean disconnect within provided timeout duration
 *
 * NOTE: If caller provides timeout_ms=0, they must pump the event loop with
 * WsClient_service
 *
 * @param client - targeted WebSocket client
 * @param timeout_ms - duration to pump websocket event loop until giving up
 * @returns - 0 on success, -EIO if `lws_service` fails, or -ETIMEDOUT if
 * `timeout_ms` is exceeded
 *
 */
int WsClient_disconnect(WsClient* client, size_t timeout_ms) {
    if (!client) return -EINVAL;
    if (client->state == WS_STATE_DISCONNECTED) return 0;
    client->retries_left = 0;
    lws_sul_cancel(&client->sul);
    client->interrupted = true;
    if (!client->wsi) {
        client->state = WS_STATE_DISCONNECTED;
        return 0;
    }
    lws_wsi_close(client->wsi, LWS_TO_KILL_ASYNC);
    if (timeout_ms == 0) {
        return 0;
    }

    struct timespec deadline;
    clock_gettime(CLOCK_MONOTONIC, &deadline);
    deadline.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    deadline.tv_sec += timeout_ms / 1000 + deadline.tv_nsec / 1000000000L;
    deadline.tv_nsec %= 1000000000L;

    while (client->state != WS_STATE_DISCONNECTED) {
        if (lws_service(client->ctx, 0) < 0) {
            client->state = WS_STATE_DISCONNECTED;
            client->wsi = NULL;
            return -EIO;
        }

        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (now.tv_sec > deadline.tv_sec || (now.tv_sec == deadline.tv_sec &&
                                             now.tv_nsec >= deadline.tv_nsec)) {
            client->state = WS_STATE_DISCONNECTED;
            client->wsi = NULL;
            return -ETIMEDOUT;
        }
    }
    return 0;
}

int WsClient_service(WsClient* client) {
    if (!client || !client->ctx) return -EINVAL;
    return lws_service(client->ctx, 0);
}

int WsClient_send_binary(WsClient* client, const void* data, size_t len) {
    return ws_send_message(client, LWS_WRITE_BINARY, data, len);
}

int WsClient_send_text(WsClient* client, const void* data, size_t len) {
    return ws_send_message(client, LWS_WRITE_TEXT, data, len);
}

bool WsClient_is_connected(const WsClient* client) {
    return client && client->state == WS_STATE_CONNECTED;
}

void WsClient_interrupt(WsClient* client) {
    if (!client) return;
    client->interrupted = true;
    WsClient_wake(client);
}

/**
 * Closes websocket and releases allocated resources
 * NOTE: Ensure all threads using WsClient have been joined before calling
 * NOTE: Do not call from callbacks
 */
void WsClient_destroy(WsClient* client) {
    if (!client) return;
    client->interrupted = true;
    lws_sul_cancel(&client->sul);

    if (client->owns_ctx && client->ctx) {
        lws_context_destroy(client->ctx);
        client->ctx = NULL;
    }

    client->raw_fd = -1;
    client->raw_wsi = NULL;

    free(client->tx_buf);
    pthread_mutex_destroy(&client->tx_lock);
    free(client->ep_storage);
    free(client);
}

int WsClient_adopt_fd(WsClient* client, WsAdoptFdInfo info) {
    if (!client || !client->ctx || info.fd < 0) return -EINVAL;
    if (client->raw_fd >= 0) return -EALREADY;

    struct lws_vhost* vh =
        lws_get_vhost_by_name(client->ctx, WS_DEFAULT_VHOST_NAME);
    if (!vh) {
        log_error("vhost '%s' not found\n", WS_DEFAULT_VHOST_NAME);
        return -ENOENT;
    }
    log_debug(
        "vhost '%s' found, adopting fd %d\n", WS_DEFAULT_VHOST_NAME, info.fd
    );

#if LWS_PRE_V4_ADOPT
    lws_sock_file_fd_type fd_union;
    fd_union.filefd = info.fd;
    struct lws* raw_wsi = lws_adopt_descriptor_vhost(
        vh, LWS_ADOPT_RAW_FILE_DESC, fd_union, "raw-file-protocol", NULL
    );
    if (!raw_wsi) return -EIO;
    lws_set_opaque_user_data(raw_wsi, client);
#else
    lws_adopt_desc_t adopt_desc = {
        .vh = vh,
        .type = LWS_ADOPT_RAW_FILE_DESC,
        .fd = {.filefd = info.fd},
        .vh_prot_name = "raw-file-protocol",
        .parent = NULL,
        .opaque = client,
        .fi_wsi_name = NULL,
    };
    struct lws* raw_wsi = lws_adopt_descriptor_vhost_via_info(&adopt_desc);
    // LWS should close the fd on adoption failure (see lws-adopt.h)
    if (!raw_wsi) return -EIO;
#endif

    client->raw_fd = info.fd;
    client->raw_fd_type = info.type;
    client->raw_wsi = raw_wsi;
    return 0;
}

/**
 * Asynchronously closes the current connection and allows the configured
 * retry policy to establish a new connection.
 *
 * Safe from callbacks on a WebSocket service thread.
 */
int WsClient_reconnect(WsClient* client) {
    if (!client) return -EINVAL;
    if (client->state == WS_STATE_CLOSING) return -EALREADY;
    if (!client->wsi || client->state != WS_STATE_CONNECTED) return -ENOTCONN;

    /*
     * Do not set interrupted or clear retries. The CLIENT_CLOSED callback
     * will clear pending transmission state and schedule the retry.
     */
    client->state = WS_STATE_CLOSING;
    lws_wsi_close(client->wsi, LWS_TO_KILL_ASYNC);
    lws_cancel_service(client->ctx);
    return 0;
}

/**
 * Wake a thread blocked in WsClient_service().
 *
 * May be called from another thread while the WsClient and its context exist.
 * Does not change connection or retry state.
 */
void WsClient_wake(WsClient* client) {
    if (client && client->ctx) {
        lws_cancel_service(client->ctx);
    }
}
