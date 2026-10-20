#include "natnet_client.h"

#include <arpa/inet.h>
#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define NNCLIENT_LOG_MESSAGE_SIZE 512u

#define NNCLIENT_MAX_REQUEST_PACKET_SIZE 273u

#define STREAM_TIMEOUT_SEC 10
#define RECONNECT_INTERVAL_SEC 1
#define POLL_TIMEOUT_MS 1000

#if defined(__GNUC__) || defined(__clang__)
#define NNCLIENT_PRINTF(format_index, args_index) \
    __attribute__((format(printf, format_index, args_index)))
#else
#define NNCLIENT_PRINTF(format_index, args_index)
#endif

enum {
    SENDER_SIZE = NNCLIENT_MAX_STRING_SIZE + sizeof(Version) * 2,
    CONNECTION_OPTION_SIZE = 5,
    CONNECT_PAYLOAD_SIZE = SENDER_SIZE + CONNECTION_OPTION_SIZE,
    CONNECT_PACKET_SIZE = NNCLIENT_PACKET_HEADER_SIZE + CONNECT_PAYLOAD_SIZE,
};

typedef struct NnClient {
    NnClientConfig config;
    struct sockaddr_in motive_addr;
    _Atomic bool connected;
    pthread_t cmd_thread;
    bool cmd_thread_started;
    pthread_t data_thread;
    bool data_thread_started;
    _Atomic NnClientRunState run_state;
    int cmd_socket;
    int data_socket;
    Version connect_version;
} NnClient;

static const char* packet_type_from_id(uint16_t id);
static uint8_t* cursor_write_u16_le(uint8_t* cursor, uint16_t value);
static const uint8_t* cursor_read_u16_le(const uint8_t* cursor, uint16_t* out);
static int packet_begin(
    uint8_t* dst,
    size_t dst_capacity,
    NnMessageId message_id,
    size_t payload_size,
    uint8_t** payload_out,
    size_t* packet_size
);
static int create_cmd_socket(
    const NnClientConfig* config, struct in_addr addr, uint16_t port
);
static void* command_listen_thread(void* userdata);
static int create_data_socket(
    const NnClientConfig* config, struct in_addr addr, uint16_t port
);
static void* data_listen_thread(void* userdata);
static void nnclient_log(
    const NnClientConfig* config,
    NnClientLogLevel level,
    int error,
    const char* format,
    ...
) NNCLIENT_PRINTF(4, 5);
static bool nnclient_send_error_is_fatal(int error);
static int nnclient_send_command_packet(
    const NnClient* client, const uint8_t* packet, size_t packet_size
);
static bool nnclient_request_terminal_state(
    NnClient* client, NnClientRunState requested_state
);

/* PRIVATE DEFINITIONS */

static const char* packet_type_from_id(uint16_t id) {
    switch (id) {
        case NNCLIENT_CONNECT:
            return "CONNECT";
        case NNCLIENT_SERVERINFO:
            return "SERVERINFO";
        case NNCLIENT_RESPONSE:
            return "RESPONSE";
        case NNCLIENT_MODELDEF:
            return "MODELDEF";
        case NNCLIENT_FRAMEOFDATA:
            return "FRAMEOFDATA";
        case NNCLIENT_DISCONNECT:
            return "DISCONNECT";
        case NNCLIENT_KEEPALIVE:
            return "KEEPALIVE";
        case NNCLIENT_UNRECOGNIZED_REQUEST:
            return "UNRECOGNIZED_REQUEST";
        case NNCLIENT_MESSAGESTRING:
            return "MESSAGESTRING";
        default:
            return "UNRECOGNIZED";
    }
}

static uint8_t* cursor_write_u16_le(uint8_t* cursor, uint16_t value) {
    *cursor++ = (uint8_t)(value & UINT16_C(0x00ff));
    *cursor++ = (uint8_t)((value >> 8) & UINT16_C(0x00ff));
    return cursor;
}

static const uint8_t* cursor_read_u16_le(const uint8_t* cursor, uint16_t* out) {
    *out = ((uint16_t)cursor[0]) | ((uint16_t)cursor[1] << 8);
    return cursor + 2;
}

static int packet_decode_view(
    const uint8_t* wire_data, size_t wire_size, NnPacketView* view
) {
    if (wire_data == NULL || view == NULL ||
        wire_size < NNCLIENT_PACKET_HEADER_SIZE) {
        return -1;
    }

    const uint8_t* cursor = wire_data;

    uint16_t message_id;
    cursor = cursor_read_u16_le(cursor, &message_id);
    uint16_t payload_size;
    cursor = cursor_read_u16_le(cursor, &payload_size);

    if ((size_t)payload_size != wire_size - NNCLIENT_PACKET_HEADER_SIZE) {
        return -1;
    }

    *view = (NnPacketView){
        .message_id = message_id,
        .payload_size = payload_size,
        .payload = cursor,
        .wire_data = wire_data,
        .wire_size = wire_size,
    };
    return 0;
}

static int packet_begin(
    uint8_t* dst,
    size_t dst_capacity,
    NnMessageId message_id,
    size_t payload_size,
    uint8_t** payload_out,
    size_t* packet_size
) {
    if (dst == NULL || packet_size == NULL) return -1;
    if (payload_size > UINT16_MAX) return -1;
    if (dst_capacity < NNCLIENT_PACKET_HEADER_SIZE ||
        payload_size > dst_capacity - NNCLIENT_PACKET_HEADER_SIZE) {
        return -1;
    }

    uint8_t* cursor = dst;
    cursor = cursor_write_u16_le(cursor, message_id);
    cursor = cursor_write_u16_le(cursor, (uint16_t)payload_size);

    if (payload_out != NULL) {
        *payload_out = cursor;
    }

    *packet_size = NNCLIENT_PACKET_HEADER_SIZE + payload_size;
    return 0;
}

static void nnclient_log(
    const NnClientConfig* config,
    NnClientLogLevel level,
    int error,
    const char* format,
    ...
) {
    if (config == NULL || config->handle_log == NULL || format == NULL) {
        return;
    }

    int saved_errno = errno;
    char message[NNCLIENT_LOG_MESSAGE_SIZE];

    va_list args;
    va_start(args, format);
    int result = vsnprintf(message, sizeof(message), format, args);
    va_end(args);

    if (result < 0) {
        config->handle_log(
            NNCLIENT_LOG_ERROR,
            0,
            "Failed to format natnet_client diagnostic",
            config->handle_log_data
        );
    } else {
        config->handle_log(level, error, message, config->handle_log_data);
    }

    errno = saved_errno;
}

static bool nnclient_send_error_is_fatal(int error) {
    switch (error) {
        case EBADF:
        case ENOTSOCK:
        case EINVAL:
        case EDESTADDRREQ:
        case EAFNOSUPPORT:
        case EMSGSIZE:
        case EACCES:
        case EPERM:
        case EIO:
            return true;
        default:
            return false;
    }
}

static int nnclient_send_command_packet(
    const NnClient* client, const uint8_t* packet, size_t packet_size
) {
    if (client == NULL || packet == NULL || packet_size == 0) {
        return EINVAL;
    }

    if (client->cmd_socket < 0) {
        return EBADF;
    }

    if (packet_size > NNCLIENT_MAX_PACKET_SIZE) {
        return EMSGSIZE;
    }

    for (;;) {
        ssize_t bytes_sent = sendto(
            client->cmd_socket,
            packet,
            packet_size,
            0,
            (const struct sockaddr*)&client->motive_addr,
            sizeof(client->motive_addr)
        );
        if (bytes_sent == (ssize_t)packet_size) {
            return 0;
        }
        if (bytes_sent >= 0) {
            return EIO;
        }

        int error = errno;
        if (error != EINTR) return error;
        if (atomic_load_explicit(&client->run_state, memory_order_acquire) !=
            NNCLIENT_RUNNING) {
            return ECANCELED;
        }
        // otherwise, retry
    }
}

/**
 * Creates the command communication socket
 *
 * Currently only handles unicast.
 */
static int create_cmd_socket(
    const NnClientConfig* config, struct in_addr addr, uint16_t port
) {
    int cmd_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (cmd_sock == -1) {
        int error = errno;
        nnclient_log(
            config, NNCLIENT_LOG_ERROR, error, "Failed to create command socket"
        );
        errno = error;
        return -1;
    }

    struct sockaddr_in cmd_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
        .sin_addr = addr,
    };
    socklen_t cmd_addr_len = sizeof(cmd_addr);

    int reuse_addr = 1;
    int retval = setsockopt(
        cmd_sock, SOL_SOCKET, SO_REUSEADDR, &reuse_addr, sizeof(reuse_addr)
    );
    if (retval == -1) {
        int error = errno;
        nnclient_log(
            config,
            NNCLIENT_LOG_ERROR,
            error,
            "Failed setsockopt SO_REUSEADDR on command socket"
        );
        close(cmd_sock);
        errno = error;
        return retval;
    }

    retval = bind(cmd_sock, (struct sockaddr*)&cmd_addr, cmd_addr_len);
    if (retval < 0) {
        int error = errno;
        nnclient_log(
            config, NNCLIENT_LOG_ERROR, error, "Failed to bind command socket"
        );
        close(cmd_sock);
        errno = error;
        return retval;
    }

    int request_buf_size = NNCLIENT_MAX_PACKET_SIZE;
    retval = setsockopt(
        cmd_sock,
        SOL_SOCKET,
        SO_RCVBUF,
        &request_buf_size,
        sizeof(request_buf_size)
    );
    if (retval == -1) {
        int error = errno;
        nnclient_log(
            config,
            NNCLIENT_LOG_ERROR,
            error,
            "Failed to set command socket receive buffer size"
        );
        close(cmd_sock);
        errno = error;
        return retval;
    }

    // Ensure buffer was actually changed
    int confirm_buf_size = 0;
    socklen_t opt_size = sizeof(confirm_buf_size);
    retval = getsockopt(
        cmd_sock, SOL_SOCKET, SO_RCVBUF, &confirm_buf_size, &opt_size
    );
    if (retval == -1) {
        int error = errno;
        nnclient_log(
            config,
            NNCLIENT_LOG_ERROR,
            error,
            "Failed to read command socket receive buffer size"
        );
        close(cmd_sock);
        errno = error;
        return -1;
    }
    if (confirm_buf_size != 2 * request_buf_size) {
        nnclient_log(
            config,
            NNCLIENT_LOG_WARNING,
            0,
            "Command socket receive buffer differs from expected: %d/%d",
            confirm_buf_size,
            2 * request_buf_size
        );
    }

    // Packet fragmentation setting
    int dont_fragment = IP_PMTUDISC_DO;
    opt_size = sizeof(dont_fragment);
    retval = setsockopt(
        cmd_sock, IPPROTO_IP, IP_MTU_DISCOVER, &dont_fragment, opt_size
    );
    if (retval == -1) {
        int error = errno;
        nnclient_log(
            config,
            NNCLIENT_LOG_ERROR,
            error,
            "Failed to enable path MTU discovery on command socket"
        );
        close(cmd_sock);
        errno = error;
        return -1;
    }

    retval = getsockname(cmd_sock, (struct sockaddr*)&cmd_addr, &cmd_addr_len);
    if (retval == -1) {
        int error = errno;
        nnclient_log(
            config,
            NNCLIENT_LOG_WARNING,
            error,
            "Command socket created, but its local endpoint could not be read"
        );
        return cmd_sock;
    }

    char ip_addr_str[INET_ADDRSTRLEN];
    const char* str_ptr = inet_ntop(
        AF_INET, &cmd_addr.sin_addr.s_addr, ip_addr_str, INET_ADDRSTRLEN
    );
    if (str_ptr == NULL) {
        int error = errno;
        nnclient_log(
            config,
            NNCLIENT_LOG_WARNING,
            error,
            "Command socket created, but its local address could not be "
            "formatted"
        );
        return cmd_sock;
    }
    nnclient_log(
        config,
        NNCLIENT_LOG_INFO,
        0,
        "Command socket created at %s:%u",
        ip_addr_str,
        (unsigned)ntohs(cmd_addr.sin_port)
    );
    return cmd_sock;
}

static void* command_listen_thread(void* userdata) {
    NnClient* client = (NnClient*)userdata;
    const NnClientConfig* config = &client->config;
    uint8_t* packet_in = calloc(1, NNCLIENT_MAX_PACKET_SIZE);
    if (packet_in == NULL) {
        nnclient_log(
            config,
            NNCLIENT_LOG_ERROR,
            ENOMEM,
            "Failed to allocate command packet buffer"
        );
        nnclient_request_terminal_state(client, NNCLIENT_STOP_FAILURE);
        return NULL;
    }
    uint8_t packet_out[NNCLIENT_MAX_REQUEST_PACKET_SIZE] = {0};

    struct timespec last_keepalive = {0};
    struct timespec last_connect_attempt = {0};
    struct timespec last_recv_success = {0};
    struct timespec now;
    struct sockaddr_in recv_addr;
    struct pollfd pfd = {
        .fd = client->cmd_socket,
        .events = POLLIN,
    };
    socklen_t recv_addr_len = sizeof(recv_addr);
    nnclient_log(
        config, NNCLIENT_LOG_INFO, 0, "Command listener thread started"
    );
    // NOTE (mmckenna): only supports unicast
    while (atomic_load_explicit(&client->run_state, memory_order_acquire) ==
           NNCLIENT_RUNNING) {
        bool connected =
            atomic_load_explicit(&client->connected, memory_order_acquire);
        int retval = 0;
        // Rate limit KEEPALIVE request
        if (clock_gettime(CLOCK_MONOTONIC, &now) == -1) {
            int error = errno;
            nnclient_log(
                config,
                NNCLIENT_LOG_ERROR,
                error,
                "Failed to read monotonic clock"
            );
            nnclient_request_terminal_state(client, NNCLIENT_STOP_FAILURE);
            break;
        }
        time_t elapsed_time = now.tv_sec - last_keepalive.tv_sec;
        // Motive keepalive rate limiting
        if (connected &&
            elapsed_time >= NNCLIENT_KEEPALIVE_REQUEST_INTERVAL_SEC) {
            size_t packet_size;
            int result = NnPacket_compose_empty_request(
                packet_out, sizeof(packet_out), NNCLIENT_KEEPALIVE, &packet_size
            );
            if (result != 0) {
                nnclient_log(
                    config,
                    NNCLIENT_LOG_ERROR,
                    0,
                    "Failed to compose KEEPALIVE packet"
                );
                nnclient_request_terminal_state(client, NNCLIENT_STOP_FAILURE);
                break;
            }

            int error =
                nnclient_send_command_packet(client, packet_out, packet_size);
            if (error != 0) {
                if (error == ECANCELED) break;

                if (nnclient_send_error_is_fatal(error)) {
                    nnclient_log(
                        config,
                        NNCLIENT_LOG_ERROR,
                        error,
                        "Failed to send KEEPALIVE"
                    );
                    nnclient_request_terminal_state(
                        client, NNCLIENT_STOP_FAILURE
                    );
                    break;
                }

                // Log non-fatal error
                nnclient_log(
                    config,
                    NNCLIENT_LOG_WARNING,
                    error,
                    "Failed to send KEEPALIVE"
                );
            }

            if (atomic_load_explicit(
                    &client->run_state, memory_order_acquire
                ) != NNCLIENT_RUNNING) {
                break;
            }

            last_keepalive = now;
        }

        // Motive stream timout
        elapsed_time = now.tv_sec - last_recv_success.tv_sec;
        if (connected && elapsed_time >= STREAM_TIMEOUT_SEC) {
            nnclient_log(
                config,
                NNCLIENT_LOG_WARNING,
                0,
                "Motive stream timed out.  Reconnecting..."
            );
            atomic_store_explicit(
                &client->connected, false, memory_order_release
            );
            connected = false;
        }

        // Try to reconnect with Motive
        elapsed_time = now.tv_sec - last_connect_attempt.tv_sec;
        if (!connected && elapsed_time >= RECONNECT_INTERVAL_SEC) {
            int error = NnClient_send_connect(client);
            last_connect_attempt = now;

            // Failed to send connect packet to Motive
            if (error != 0) {
                if (error == ECANCELED) break;

                if (nnclient_send_error_is_fatal(error)) {
                    nnclient_log(
                        config,
                        NNCLIENT_LOG_ERROR,
                        error,
                        "Failed to send CONNECT"
                    );
                    nnclient_request_terminal_state(
                        client, NNCLIENT_STOP_FAILURE
                    );
                    break;
                }

                // Log non-fatal error
                nnclient_log(
                    config,
                    NNCLIENT_LOG_WARNING,
                    error,
                    "Failed to send CONNECT"
                );
            }
        }

        retval = poll(&pfd, 1, POLL_TIMEOUT_MS);

        // Poll error
        if (retval < 0) {
            int error = errno;
            if (error == EINTR) continue;
            if (atomic_load_explicit(
                    &client->run_state, memory_order_acquire
                ) != NNCLIENT_RUNNING) {
                break;
            }
            nnclient_log(
                config,
                NNCLIENT_LOG_WARNING,
                error,
                "Failed to poll command socket"
            );
            atomic_store_explicit(
                &client->connected, false, memory_order_release
            );
            continue;
        }

        // Poll timeout
        if (retval == 0) continue;

        // Invalid fd is critical
        if (pfd.revents & POLLNVAL) {
            nnclient_log(
                config,
                NNCLIENT_LOG_ERROR,
                EBADF,
                "Command socket fd is invalid"
            );
            nnclient_request_terminal_state(client, NNCLIENT_STOP_FAILURE);
            break;
        }

        // Socket is in error/hangup state.  Try to reconnect.
        if (pfd.revents & (POLLERR | POLLHUP)) {
            atomic_store_explicit(
                &client->connected, false, memory_order_release
            );
            continue;
        }

        // Poll success
        ssize_t recv_bytes = 0;
        if (retval > 0 && (pfd.revents & POLLIN)) {
            recv_addr_len = sizeof(recv_addr);
            recv_bytes = recvfrom(
                client->cmd_socket,
                packet_in,
                NNCLIENT_MAX_PACKET_SIZE,
                0,
                (struct sockaddr*)&recv_addr,
                &recv_addr_len
            );

            if (recv_bytes == 0) continue;
            if (recv_bytes < 0) {
                int error = errno;
                if (error == EAGAIN || error == EWOULDBLOCK || error == EINTR) {
                    // Normal timeout.  Only viable if SO_RCVTIMEO socket option
                    // is used
                    continue;
                }
                if (atomic_load_explicit(
                        &client->run_state, memory_order_acquire
                    ) != NNCLIENT_RUNNING) {
                    // Normal shutdown
                    break;
                }
                nnclient_log(
                    config,
                    NNCLIENT_LOG_ERROR,
                    error,
                    "Failed to receive command packet"
                );
                continue;
            }

            // Check packet source
            if (recv_addr.sin_addr.s_addr !=
                client->config.motive_addr.s_addr) {
                char addr_str[INET_ADDRSTRLEN] = {0};
                const char* str_ptr = inet_ntop(
                    AF_INET,
                    &recv_addr.sin_addr.s_addr,
                    addr_str,
                    INET_ADDRSTRLEN
                );
                if (str_ptr == NULL) {
                    int error = errno;
                    nnclient_log(
                        config,
                        NNCLIENT_LOG_WARNING,
                        error,
                        "Ignoring command packet from unexpected source"
                    );
                } else {
                    nnclient_log(
                        config,
                        NNCLIENT_LOG_WARNING,
                        0,
                        "Ignoring packet from unexpected source: %s",
                        addr_str
                    );
                }
                continue;
            }
        } else {
            continue;
        }

        // Ensure received bytes matches encoded size
        NnPacketView packet_view;
        int result =
            packet_decode_view(packet_in, (size_t)recv_bytes, &packet_view);
        if (result != 0) {
            nnclient_log(
                config, NNCLIENT_LOG_ERROR, 0, "Failed to decode packet"
            );
            continue;
        }

        nnclient_log(
            config,
            NNCLIENT_LOG_TRACE,
            0,
            "Received %s on command thread",
            packet_type_from_id(packet_view.message_id)
        );

        // Update recv success time
        if (clock_gettime(CLOCK_MONOTONIC, &last_recv_success) == -1) {
            int error = errno;
            nnclient_log(
                config,
                NNCLIENT_LOG_ERROR,
                error,
                "Failed to read monotonic clock"
            );
            nnclient_request_terminal_state(client, NNCLIENT_STOP_FAILURE);
            break;
        }
        if (packet_view.message_id == NNCLIENT_SERVERINFO) {
            atomic_store_explicit(
                &client->connected, true, memory_order_release
            );
        }
        // User-specified packet handling
        if (client->config.handle_command_packet != NULL) {
            client->config.handle_command_packet(
                &packet_view, client->config.handle_command_packet_data
            );
        }
    }

    if (packet_in != NULL) free(packet_in);
    return NULL;
}

/**
 * Creates the data communication socket
 *
 * Currently only handles unicast
 */
static int create_data_socket(
    const NnClientConfig* config, struct in_addr addr, uint16_t port
) {
    int data_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (data_sock == -1) {
        int error = errno;
        nnclient_log(
            config, NNCLIENT_LOG_ERROR, error, "Failed to create data socket"
        );
        errno = error;
        return -1;
    }

    struct sockaddr_in data_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
        .sin_addr = addr,
    };
    socklen_t data_addr_len = sizeof(data_addr);

    int reuse_addr = 1;
    int retval = setsockopt(
        data_sock, SOL_SOCKET, SO_REUSEADDR, &reuse_addr, sizeof(reuse_addr)
    );
    if (retval == -1) {
        int error = errno;
        nnclient_log(
            config,
            NNCLIENT_LOG_ERROR,
            error,
            "Failed set SO_REUSEADDR on data socket"
        );
        close(data_sock);
        errno = error;
        return -1;
    }

    retval = bind(data_sock, (struct sockaddr*)&data_addr, data_addr_len);
    if (retval < 0) {
        int error = errno;
        nnclient_log(
            config, NNCLIENT_LOG_ERROR, error, "Failed to bind data socket"
        );
        close(data_sock);
        errno = error;
        return -1;
    }

    int request_buf_size = NNCLIENT_MAX_PACKET_SIZE;
    socklen_t opt_size = sizeof(request_buf_size);
    retval = setsockopt(
        data_sock,
        SOL_SOCKET,
        SO_RCVBUF,
        &request_buf_size,
        sizeof(request_buf_size)
    );
    if (retval == -1) {
        int error = errno;
        nnclient_log(
            config,
            NNCLIENT_LOG_ERROR,
            error,
            "Failed set data socket receive buffer size"
        );
        close(data_sock);
        errno = error;
        return -1;
    }

    // Ensure buffer was actually changed
    int confirm_buf_size = 0;
    opt_size = sizeof(confirm_buf_size);
    retval = getsockopt(
        data_sock, SOL_SOCKET, SO_RCVBUF, &confirm_buf_size, &opt_size
    );
    if (retval == -1) {
        int error = errno;
        nnclient_log(
            config,
            NNCLIENT_LOG_ERROR,
            error,
            "Failed to read data socket receive buffer size"
        );
        close(data_sock);
        errno = error;
        return -1;
    }
    if (confirm_buf_size != 2 * request_buf_size) {
        nnclient_log(
            config,
            NNCLIENT_LOG_WARNING,
            0,
            "Data socket receive buffer size differs from expected: %d/%d",
            confirm_buf_size,
            2 * request_buf_size
        );
    }

    retval =
        getsockname(data_sock, (struct sockaddr*)&data_addr, &data_addr_len);
    if (retval == -1) {
        int error = errno;
        nnclient_log(
            config,
            NNCLIENT_LOG_WARNING,
            error,
            "Data socket created, but its local endpoint could not be read"
        );
        return data_sock;
    }

    char ip_addr_str[INET_ADDRSTRLEN];
    const char* str_ptr = inet_ntop(
        AF_INET, &data_addr.sin_addr.s_addr, ip_addr_str, INET_ADDRSTRLEN
    );
    if (str_ptr == NULL) {
        int error = errno;
        nnclient_log(
            config,
            NNCLIENT_LOG_WARNING,
            error,
            "Data socket created, but its local address could not be formatted"
        );
        return data_sock;
    }
    nnclient_log(
        config,
        NNCLIENT_LOG_INFO,
        0,
        "Data socket created at %s:%u",
        ip_addr_str,
        (unsigned)ntohs(data_addr.sin_port)
    );
    return data_sock;
}

static void* data_listen_thread(void* userdata) {
    // TODO (mmckenna): Why 48KB?
    NnClient* client = (NnClient*)userdata;
    const NnClientConfig* config = &client->config;
    uint8_t* packet_in = malloc(NNCLIENT_MAX_PACKET_SIZE);
    if (packet_in == NULL) {
        nnclient_log(
            config,
            NNCLIENT_LOG_ERROR,
            ENOMEM,
            "Failed to allocate data packet buffer"
        );
        nnclient_request_terminal_state(client, NNCLIENT_STOP_FAILURE);
        return NULL;
    }

    struct pollfd pfd = {
        .fd = client->data_socket,
        .events = POLLIN,
    };

    struct sockaddr_in recv_addr = {0};
    nnclient_log(config, NNCLIENT_LOG_INFO, 0, "Data listen thread started.");
    while (atomic_load_explicit(&client->run_state, memory_order_acquire) ==
           NNCLIENT_RUNNING) {
        int retval = poll(&pfd, 1, POLL_TIMEOUT_MS);

        if (retval < 0) {
            int error = errno;
            if (error == EINTR) continue;
            NnClientRunState state =
                atomic_load_explicit(&client->run_state, memory_order_acquire);
            if (state != NNCLIENT_RUNNING) break;

            nnclient_log(
                config, NNCLIENT_LOG_ERROR, error, "Failed to poll data socket"
            );
            nnclient_request_terminal_state(client, NNCLIENT_STOP_FAILURE);
            break;
        }

        // Timeout
        if (retval == 0) continue;

        if (pfd.revents & POLLNVAL) {
            nnclient_log(
                config, NNCLIENT_LOG_ERROR, EBADF, "Data socket fd is invalid"
            );
            nnclient_request_terminal_state(client, NNCLIENT_STOP_FAILURE);
            break;
        }

        if (pfd.revents & (POLLERR | POLLHUP)) {
            nnclient_log(
                config,
                NNCLIENT_LOG_ERROR,
                0,
                "Data socket entered an error state"
            );
            nnclient_request_terminal_state(client, NNCLIENT_STOP_FAILURE);
            break;
        }

        if (!(pfd.revents & POLLIN)) continue;

        socklen_t recv_addr_len = sizeof(recv_addr);
        ssize_t recv_bytes = recvfrom(
            client->data_socket,
            packet_in,
            NNCLIENT_MAX_PACKET_SIZE,
            0,
            (struct sockaddr*)&recv_addr,
            &recv_addr_len
        );

        if (recv_bytes < 0) {
            int error = errno;
            if (error == EAGAIN || error == EWOULDBLOCK || error == EINTR) {
                // Normal timeout
                continue;
            }
            if (atomic_load_explicit(
                    &client->run_state, memory_order_acquire
                ) != NNCLIENT_RUNNING) {
                // Normal shutdown
                break;
            }
            nnclient_log(
                config,
                NNCLIENT_LOG_WARNING,
                error,
                "Failed to receive data packet"
            );
            continue;
        }

        // Packet empty or truncated
        if (recv_bytes < NNCLIENT_PACKET_HEADER_SIZE) {
            if (atomic_load_explicit(
                    &client->run_state, memory_order_acquire
                ) != NNCLIENT_RUNNING) {
                continue;
            }
            nnclient_log(
                config,
                NNCLIENT_LOG_WARNING,
                0,
                "Received incomplete data packet"
            );
            continue;
        }

        NnPacketView packet_view;
        int error =
            packet_decode_view(packet_in, (size_t)recv_bytes, &packet_view);
        if (error != 0) {
            nnclient_log(
                config, NNCLIENT_LOG_WARNING, 0, "Failed to decode data packet"
            );
            continue;
        }

        if (client->config.handle_data_packet != NULL) {
            client->config.handle_data_packet(
                &packet_view, client->config.handle_data_packet_data
            );
        }
    }

    free(packet_in);
    return NULL;
}

/* PUBLIC DEFINITIONS */

int NnPacket_compose_empty_request(
    uint8_t* dst,
    size_t dst_capacity,
    NnMessageId message_id,
    size_t* packet_size
) {
    switch (message_id) {
        case NNCLIENT_REQUEST_MODELDEF:
        case NNCLIENT_REQUEST_FRAMEOFDATA:
        case NNCLIENT_KEEPALIVE:
        case NNCLIENT_DISCONNECT: {
            return packet_begin(
                dst, dst_capacity, message_id, 0, NULL, packet_size
            );
        } break;
        default:
            return -1;
    }
    return 0;
}

int NnPacket_compose_connect(
    uint8_t* dst, size_t dst_capacity, Version version, size_t* packet_size
) {
    uint8_t* payload = NULL;

    int result = packet_begin(
        dst,
        dst_capacity,
        NNCLIENT_CONNECT,
        CONNECT_PAYLOAD_SIZE,
        &payload,
        packet_size
    );
    if (result != 0) return result;

    memset(payload, 0, CONNECT_PAYLOAD_SIZE);
    // Skip writing sender info
    uint8_t* cursor = payload + SENDER_SIZE;
    // subscribedDataOnly
    *cursor++ = 0;
    // BitstreamVersion
    *cursor++ = version.major;
    *cursor++ = version.minor;
    *cursor++ = version.patch;
    *cursor++ = version.build;

    if (cursor != payload + CONNECT_PAYLOAD_SIZE) return -1;
    return 0;
}

int NnPacket_compose_modeldef_request(
    uint8_t* dst, size_t dst_capacity, size_t* packet_size
) {
    int result = packet_begin(
        dst, dst_capacity, NNCLIENT_REQUEST_MODELDEF, 0, NULL, packet_size
    );
    return result;
}

int NnClient_send_connect(const NnClient* client) {
    if (client == NULL) return EINVAL;

    // Generic buffer
    uint8_t packet_out[NNCLIENT_MAX_REQUEST_PACKET_SIZE] = {0};
    // Actual composed packet size;
    size_t packet_size;

    int result = NnPacket_compose_connect(
        packet_out, sizeof(packet_out), client->connect_version, &packet_size
    );
    if (result != 0) return EINVAL;

    return nnclient_send_command_packet(client, packet_out, packet_size);
}

int NnClient_send_modeldef_request(const NnClient* client) {
    if (client == NULL) return EINVAL;

    // Generic buffer
    uint8_t packet_out[NNCLIENT_MAX_REQUEST_PACKET_SIZE] = {0};
    // Actual composed packet size;
    size_t packet_size;

    int result = NnPacket_compose_modeldef_request(
        packet_out, sizeof(packet_out), &packet_size
    );
    if (result != 0) return EINVAL;

    return nnclient_send_command_packet(client, packet_out, packet_size);
}

int NnClient_create(const NnClientConfig* cfg, NnClient** out) {
    if (cfg == NULL || out == NULL) return EINVAL;

    NnClient* previous = *out;
    NnClient* client = calloc(1, sizeof(*client));
    if (client == NULL) return ENOMEM;

    client->config = *cfg;
    client->connect_version = cfg->version;
    client->motive_addr = (struct sockaddr_in){
        .sin_family = AF_INET,
        .sin_port = htons(NNCLIENT_PORT_COMMAND),
        .sin_addr = cfg->motive_addr,
    };
    client->cmd_socket = -1;
    client->data_socket = -1;

    atomic_init(&client->connected, false);
    atomic_init(&client->run_state, NNCLIENT_INITIALIZED);

    int error = NnClient_destroy(previous);
    if (error != 0) {
        free(client);
        return error;
    }
    *out = client;
    return 0;
}

int NnClient_start(NnClient* client) {
    if (client == NULL) return EINVAL;
    NnClientRunState state =
        atomic_load_explicit(&client->run_state, memory_order_acquire);
    if (state != NNCLIENT_INITIALIZED) return EALREADY;

    // Create sockets
    client->cmd_socket =
        create_cmd_socket(&client->config, client->config.host_addr, 0);
    if (client->cmd_socket == -1) {
        return errno;
    }

    client->data_socket =
        create_data_socket(&client->config, client->config.host_addr, 0);
    if (client->data_socket == -1) {
        int error = errno;
        close(client->cmd_socket);
        client->cmd_socket = -1;
        return error;
    }

    // Switch state
    NnClientRunState expected = NNCLIENT_INITIALIZED;
    bool transitioned = atomic_compare_exchange_strong_explicit(
        &client->run_state,
        &expected,
        NNCLIENT_RUNNING,
        memory_order_release,
        memory_order_relaxed
    );

    if (!transitioned) {
        close(client->cmd_socket);
        client->cmd_socket = -1;
        close(client->data_socket);
        client->data_socket = -1;
        return ECANCELED;
    }

    // Create listener threads
    int error = pthread_create(
        &client->cmd_thread, NULL, command_listen_thread, client
    );
    if (error != 0) {
        nnclient_log(
            &client->config,
            NNCLIENT_LOG_ERROR,
            error,
            "Failed to create command listener thread"
        );
        nnclient_request_terminal_state(client, NNCLIENT_STOP_FAILURE);
        close(client->cmd_socket);
        client->cmd_socket = -1;
        close(client->data_socket);
        client->data_socket = -1;
        return error;
    }
    client->cmd_thread_started = true;

    error =
        pthread_create(&client->data_thread, NULL, data_listen_thread, client);
    if (error != 0) {
        int create_error = error;
        nnclient_log(
            &client->config,
            NNCLIENT_LOG_ERROR,
            create_error,
            "Failed to create data listener thread"
        );
        nnclient_request_terminal_state(client, NNCLIENT_STOP_FAILURE);
        int join_error = pthread_join(client->cmd_thread, NULL);
        if (join_error != 0) {
            nnclient_log(
                &client->config,
                NNCLIENT_LOG_ERROR,
                join_error,
                "Failed to join command listener after data thread startup "
                "failure"
            );
            return join_error;
        }
        client->cmd_thread_started = false;

        close(client->cmd_socket);
        client->cmd_socket = -1;
        close(client->data_socket);
        client->data_socket = -1;

        return create_error;
    }
    client->data_thread_started = true;

    return 0;
}

static bool nnclient_request_terminal_state(
    NnClient* client, NnClientRunState requested_state
) {
    if (client == NULL) return false;
    NnClientRunState state =
        atomic_load_explicit(&client->run_state, memory_order_acquire);

    while (state == NNCLIENT_INITIALIZED || state == NNCLIENT_RUNNING) {
        bool transitioned = atomic_compare_exchange_weak_explicit(
            &client->run_state,
            &state,
            requested_state,
            memory_order_acq_rel,
            memory_order_acquire
        );
        if (transitioned) return true;
    }
    return false;
}

NnClientRunState NnClient_get_run_state(const NnClient* client) {
    if (client == NULL) return NNCLIENT_STOP_FAILURE;
    return atomic_load_explicit(&client->run_state, memory_order_acquire);
}

bool NnClient_is_connected(const NnClient* client) {
    if (client == NULL) return false;
    return atomic_load_explicit(&client->connected, memory_order_acquire);
}

bool NnClient_request_stop(NnClient* client) {
    return nnclient_request_terminal_state(client, NNCLIENT_STOP_REQUESTED);
}

int NnClient_destroy(NnClient* client) {
    if (client == NULL) return 0;

    NnClient_request_stop(client);

    // Join threads
    if (client->cmd_thread_started) {
        nnclient_log(
            &client->config,
            NNCLIENT_LOG_INFO,
            0,
            "Stopping command listener thread"
        );
        int error = pthread_join(client->cmd_thread, NULL);
        if (error != 0) {
            nnclient_log(
                &client->config,
                NNCLIENT_LOG_ERROR,
                error,
                "Failed to join command listener thread"
            );
            return error;
        }
        client->cmd_thread_started = false;
    }

    if (client->data_thread_started) {
        nnclient_log(
            &client->config,
            NNCLIENT_LOG_INFO,
            0,
            "Stopping data listener thread"
        );

        int error = pthread_join(client->data_thread, NULL);
        if (error != 0) {
            nnclient_log(
                &client->config,
                NNCLIENT_LOG_ERROR,
                error,
                "Failed to join data listener thread"
            );
            return error;
        }
        client->data_thread_started = false;
    }

    // Close sockets
    if (client->cmd_socket != -1) {
        close(client->cmd_socket);
        client->cmd_socket = -1;
    }
    if (client->data_socket != -1) {
        close(client->data_socket);
        client->data_socket = -1;
    }

    free(client);
    return 0;
}
