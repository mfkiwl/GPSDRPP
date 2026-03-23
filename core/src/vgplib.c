#include "vgplib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <gpiod.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <poll.h>
#include <time.h>
#include "vgpd_proto.h"

// ------------------------------
// Command processing
// ------------------------------
#define COMMAND_BUFFER_SIZE 512
#define OUTPUT_BUFFER_SIZE  1024

char command_buffer[COMMAND_BUFFER_SIZE];
char output_buffer[OUTPUT_BUFFER_SIZE];

// ------------------------------
// Fast MMIO register access
// ------------------------------
#define PMUGRF_MAP_SIZE 0x1000u
#define GRF_MAP_SIZE    0x10000u   // needs to cover offsets ~0x0e02c
#define GPIO_MAP_SIZE   0x1000u

static int mem_fd = -1;
static volatile unsigned char *pmugrf_map = NULL;
static volatile unsigned char *grf_map = NULL;
static volatile unsigned char *gpio_map[5] = { NULL, NULL, NULL, NULL, NULL };
static pthread_mutex_t regs_lock = PTHREAD_MUTEX_INITIALIZER;
static int regs_inited = 0;

// ------------------------------
// libgpiod cache (optional)
// ------------------------------
#define VGP_MAX_CHIPS 5
#define VGP_MAX_LINES 32

typedef struct {
    struct gpiod_line *line;
    int requested;      // 0=no, 1=yes
    int is_output;      // 0=input/as-is, 1=output
} VgpLineCache;

static struct gpiod_chip *g_chip_cache[VGP_MAX_CHIPS] = { NULL };
static VgpLineCache g_line_cache[VGP_MAX_CHIPS][VGP_MAX_LINES] = { 0 };
static pthread_mutex_t g_gpiod_cache_lock = PTHREAD_MUTEX_INITIALIZER;
static int g_gpiod_cache_enabled = 0;

// ------------------------------
// Misc.
// ------------------------------
char chip_name[] = "/dev/gpiochipn";

static int env_flag_enabled(const char *name) {
    const char *e = getenv(name);
    return (e && *e && strcmp(e, "0") != 0);
}

// Debug mask:
//  bit0: vgpd RPC debug
//  bit1: event dispatcher debug
static int vgp_debug_mask(void) {
    static int inited = 0;
    static int mask = 0;
    if (!inited) {
        if (env_flag_enabled("VGP_DEBUG")) mask |= 0x3;       // enable all
        if (env_flag_enabled("VGPD_DEBUG")) mask |= 0x1;      // rpc
        if (env_flag_enabled("VGP_EVT_DEBUG")) mask |= 0x2;   // events
        inited = 1;
    }
    return mask;
}

static int vgpd_debug_enabled(void) {
    return (vgp_debug_mask() & 0x1) != 0;
}

static int vgp_evt_debug_enabled(void) {
    return (vgp_debug_mask() & 0x2) != 0;
}

#define EVTDBG(...) do { \
    if (vgp_evt_debug_enabled()) fprintf(stderr, __VA_ARGS__); \
} while (0)

static uint64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

#ifndef VGPD_SOCK_PATH
#define VGPD_SOCK_PATH "/run/vgpd.sock"
#endif

// GET40 cache TTL: keep it tiny so each GUI refresh gets fresh data,
// but multiple get_* calls within the same refresh share one GET40.
#define VGP_GET40_CACHE_TTL_NS (1000ULL * 1000ULL) // 1ms

static int g_privd_fd = -1;
static uint32_t g_privd_seq = 1;
static uint64_t g_privd_last_try_ns = 0;
static pthread_mutex_t g_privd_lock = PTHREAD_MUTEX_INITIALIZER;

static int g_pinmap_inited = 0;
static int g_chln2pin[VGP_MAX_CHIPS][VGP_MAX_LINES] = { {0} };

static vgp_pin_state_t g_cache40[41];
static uint64_t g_cache40_ts_ns = 0;
static int g_cache40_valid = 0;

// -------------------------------------------
// Begin: Single-thread GPIO event dispatcher
// -------------------------------------------
MonitorThread * monitor_threads[MONITOR_THREADS];

static pthread_t g_evt_thread;
static int g_evt_thread_started = 0;
static int g_evt_stop = 0;
static int g_evt_wake_pipe[2] = { -1, -1 };
static pthread_mutex_t g_evt_lock = PTHREAD_MUTEX_INITIALIZER;

// vgpd-based events
static int g_evt_privd_fd = -1;
static uint32_t g_evt_privd_seq = 1;
static uint64_t g_evt_privd_last_try_ns = 0;
static int g_evt_use_privd = -1; // -1 unknown, 0 local, 1 vgpd

static void privd_evt_close_locked(void) {
    if (g_evt_privd_fd >= 0) {
        close(g_evt_privd_fd);
        g_evt_privd_fd = -1;
    }
}

static int privd_evt_connect_locked(void) {
    if (g_evt_privd_fd >= 0) return 0;
    uint64_t t = now_ns();
    if (g_evt_privd_last_try_ns && (t - g_evt_privd_last_try_ns) < 1000000000ULL) return -EAGAIN;
    g_evt_privd_last_try_ns = t;

    int fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
    if (fd < 0) return -errno;
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, VGPD_SOCK_PATH, sizeof(addr.sun_path) - 1);
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        int e = errno;
        close(fd);
        return -e;
    }
    g_evt_privd_fd = fd;
    return 0;
}

static int privd_evt_send_locked(uint16_t cmd, const void *req_body, size_t req_body_len) {
    uint8_t req[256];
    if (sizeof(vgp_req_hdr_t) + req_body_len > sizeof(req)) return -EINVAL;
    int cst = privd_evt_connect_locked();
    if (cst != 0) return cst;

    vgp_req_hdr_t hdr;
    hdr.magic = VGP_PRIVD_MAGIC;
    hdr.ver = VGP_PRIVD_VER;
    hdr.cmd = cmd;
    hdr.size = (uint32_t)(sizeof(vgp_req_hdr_t) + req_body_len);
    hdr.seq = g_evt_privd_seq++;
    memcpy(req, &hdr, sizeof(hdr));
    if (req_body_len) memcpy(req + sizeof(hdr), req_body, req_body_len);

    ssize_t n = send(g_evt_privd_fd, req, hdr.size, MSG_NOSIGNAL);
    if (n != (ssize_t)hdr.size) {
        int e = (n < 0) ? errno : EPIPE;
        privd_evt_close_locked();
        return -e;
    }
    return 0;
}

// If a line is already requested for edge events (by our dispatcher), regular
// gpiod_line_request() calls will fail with EBUSY. In that case, read the value
// via the already-requested monitoring handle.
static int vgp_read_value_via_monitor(int ch, int ln) {
    int val = -1;
    pthread_mutex_lock(&g_evt_lock);
    for (int pin = 1; pin < MONITOR_THREADS; pin++) {
        MonitorThread *mt = monitor_threads[pin];
        if (!mt || !mt->active || mt->closing || !mt->line) continue;
        char *pin_name = (char *)NAMES[pin];
        int mch = get_chip_number(pin_name);
        int mln = get_line_number(pin_name);
        if (mch == ch && mln == ln) {
            val = gpiod_line_get_value(mt->line);
            break;
        }
    }
    pthread_mutex_unlock(&g_evt_lock);
    return val;
}

static void vgp_evt_wake_locked(void) {
    if (g_evt_wake_pipe[1] >= 0) {
        unsigned char b = 1;
        (void)write(g_evt_wake_pipe[1], &b, 1);
        EVTDBG("[vgp-evt] wake drained\n");
    }
}

static void vgp_evt_drain_wake(void) {
    if (g_evt_wake_pipe[0] < 0) return;
    unsigned char buf[64];
    while (read(g_evt_wake_pipe[0], buf, sizeof(buf)) > 0) { /* drain */ }
}

static void vgp_evt_free_pin_locked(int pin) {
    if (pin <= 0 || pin >= MONITOR_THREADS) return;
    MonitorThread *mt = monitor_threads[pin];
    if (!mt) return;

    if (mt->active) {
        if (g_evt_use_privd == 1) {
            vgp_req_unsub_t u = { .pin = (uint8_t)mt->pin };
            (void)privd_evt_send_locked(VGP_CMD_UNSUBSCRIBE, &u, sizeof(u));
        }
        if (mt->line) gpiod_line_release(mt->line);
        if (mt->chip) gpiod_chip_close(mt->chip);
    }
    monitor_threads[pin] = NULL;
    free(mt);
}

static int vgp_evt_activate_pin_locked(MonitorThread *mt) {
    if (!mt) return -1;
    if (mt->active) return 0;
    if (mt->closing) return 0;

    if (g_evt_use_privd == 1) {
        vgp_req_sub_t s = {0};
        s.pin = (uint8_t)mt->pin;
        s.edges = (uint8_t)mt->wait_for;
        int rc = privd_evt_send_locked(VGP_CMD_SUBSCRIBE, &s, sizeof(s));
        if (rc == 0) {
            mt->chip = NULL;
            mt->line = NULL;
            mt->fd = -1;
            mt->active = 1;
            EVTDBG("[vgp-evt] subscribed via vgpd pin=%d wait_for=%d\n", mt->pin, mt->wait_for);
            return 0;
        }
        EVTDBG("[vgp-evt] vgpd SUBSCRIBE failed pin=%d: %s (%d)\n", mt->pin, strerror(-rc), rc);
        return -1;
    }

    char *pin_name = (char *)NAMES[mt->pin];
    int ch = get_chip_number(pin_name);
    int ln = get_line_number(pin_name);

    char chipname[32];
    snprintf(chipname, sizeof(chipname), "/dev/gpiochip%d", ch);
    EVTDBG("[vgp-evt] activating pin=%d (%s) chip=%d line=%d wait_for=%d\n", mt->pin, NAMES[mt->pin], ch, ln, mt->wait_for);
    mt->chip = gpiod_chip_open(chipname);
    if (!mt->chip) {
        EVTDBG("[vgp-evt] gpiod_chip_open(%s) failed: %s (errno=%d)\n", chipname, strerror(errno), errno);
        return -1;
    }

    mt->line = gpiod_chip_get_line(mt->chip, ln);
    if (!mt->line) {
        EVTDBG("[vgp-evt] gpiod_chip_get_line(%s,%d) failed: %s (errno=%d)\n", chipname, ln, strerror(errno), errno);
        gpiod_chip_close(mt->chip);
        mt->chip = NULL;
        return -1;
    }

    int rc = -1;
    if (mt->wait_for == GPIO_RISING_EDGE) {
        rc = gpiod_line_request_rising_edge_events(mt->line, "vgpw");
    } else if (mt->wait_for == GPIO_FALLING_EDGE) {
        rc = gpiod_line_request_falling_edge_events(mt->line, "vgpw");
    } else {
        rc = gpiod_line_request_both_edges_events(mt->line, "vgpw");
    }
    if (rc < 0) {
        EVTDBG("[vgp-evt] request edge events failed pin=%d (%s): %s (errno=%d)\n", mt->pin, NAMES[mt->pin], strerror(errno), errno);
        gpiod_chip_close(mt->chip);
        mt->chip = NULL;
        mt->line = NULL;
        return -1;
    }

    mt->fd = gpiod_line_event_get_fd(mt->line);
    if (mt->fd < 0) {
        EVTDBG("[vgp-evt] gpiod_line_event_get_fd failed pin=%d (%s): %s (errno=%d)\n", mt->pin, NAMES[mt->pin], strerror(errno), errno);
        gpiod_line_release(mt->line);
        gpiod_chip_close(mt->chip);
        mt->chip = NULL;
        mt->line = NULL;
        return -1;
    }
    mt->active = 1;
    EVTDBG("[vgp-evt] activated pin=%d fd=%d\n", mt->pin, mt->fd);
    return 0;
}

static void *vgp_evt_loop(void *unused) {
    (void)unused;

    while (1) {
        struct pollfd pfds[1 + MONITOR_THREADS];
        //int pins[1 + MONITOR_THREADS];
        MonitorThread *mts[1 + MONITOR_THREADS];
        int n = 0;

        pthread_mutex_lock(&g_evt_lock);

        if (g_evt_stop) {
            for (int pin = 1; pin < MONITOR_THREADS; pin++) {
                if (monitor_threads[pin]) vgp_evt_free_pin_locked(pin);
            }
            privd_evt_close_locked();
            pthread_mutex_unlock(&g_evt_lock);
            return NULL;
        }
        
        if (g_evt_use_privd < 0) {
            int c = privd_evt_connect_locked();
            g_evt_use_privd = (c == 0) ? 1 : 0;
            if (g_evt_use_privd == 0) privd_evt_close_locked();
            EVTDBG("[vgp-evt] event source: %s\n", g_evt_use_privd ? "vgpd" : "local gpiod");
        } else if (g_evt_use_privd == 1 && g_evt_privd_fd < 0) {
            (void)privd_evt_connect_locked();
        }

        time_t now = time(NULL);
        int timeout_ms = -1;

        // Apply pending stop/start and compute timeout for delayed starts
        for (int pin = 1; pin < MONITOR_THREADS; pin++) {
            MonitorThread *mt = monitor_threads[pin];
            if (!mt) continue;

            if (mt->closing) {
                vgp_evt_free_pin_locked(pin);
                continue;
            }

            if (!mt->active) {
                if (now >= mt->start_at) {
                    int arc = vgp_evt_activate_pin_locked(mt);
                    // If activation fails, periodically wake up and retry
                    if (arc < 0) {
                        if (timeout_ms < 0 || timeout_ms > 1000) timeout_ms = 1000;
                        EVTDBG("[vgp-evt] activation failed pin=%d, will retry in %dms\n", mt->pin, timeout_ms);
                    }
                } else {
                    int diff = (int)(mt->start_at - now);
                    int ms = diff * 1000;
                    if (timeout_ms < 0 || ms < timeout_ms) timeout_ms = ms;
                }
            }
        }

        pfds[n].fd = g_evt_wake_pipe[0];
        pfds[n].events = POLLIN;
        //pins[n] = 0;
        mts[n] = NULL;
        n++;

        int use_privd = (g_evt_use_privd == 1);
        int daemon_fd = g_evt_privd_fd;
        if (use_privd) {
            if (daemon_fd >= 0) {
                pfds[n].fd = daemon_fd;
                pfds[n].events = POLLIN;
                mts[n] = NULL;
                n++;
            } else {
                if (timeout_ms < 0 || timeout_ms > 1000) timeout_ms = 1000;
            }
        } else {
            for (int pin = 1; pin < MONITOR_THREADS; pin++) {
                MonitorThread *mt = monitor_threads[pin];
                if (!mt || !mt->active || mt->closing || mt->fd < 0) continue;
                pfds[n].fd = mt->fd;
                pfds[n].events = POLLIN;
                //pins[n] = pin;
                mts[n] = mt;
                n++;
            }
        }

        pthread_mutex_unlock(&g_evt_lock);

        int prc = poll(pfds, n, timeout_ms);
        if (prc <= 0) continue;

        if (use_privd) {
            if (pfds[0].revents & POLLIN) vgp_evt_drain_wake();
            if (daemon_fd < 0) continue;
            for (;;) {
                uint8_t buf[256];
                ssize_t rn = recv(daemon_fd, buf, sizeof(buf), MSG_DONTWAIT);
                if (rn < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                    pthread_mutex_lock(&g_evt_lock);
                    privd_evt_close_locked();
                    pthread_mutex_unlock(&g_evt_lock);
                    break;
                }
                if (rn == 0) {
                    pthread_mutex_lock(&g_evt_lock);
                    privd_evt_close_locked();
                    pthread_mutex_unlock(&g_evt_lock);
                    break;
                }
                if ((size_t)rn < sizeof(vgp_resp_hdr_t)) continue;
                vgp_resp_hdr_t hdr;
                memcpy(&hdr, buf, sizeof(hdr));
                if (hdr.magic != VGP_PRIVD_MAGIC || hdr.ver != VGP_PRIVD_VER || hdr.size != (uint32_t)rn) continue;
                if (hdr.cmd != VGP_CMD_EVENT) continue; // drop subscribe/unsub acks
                if ((size_t)rn != sizeof(vgp_resp_hdr_t) + sizeof(vgp_evt_t)) continue;
                vgp_evt_t evt;
                memcpy(&evt, buf + sizeof(vgp_resp_hdr_t), sizeof(evt));
                MonitorThread *mt = NULL;
                void (*cb)(void*) = NULL;
                pthread_mutex_lock(&g_evt_lock);
                if (evt.pin > 0 && evt.pin < MONITOR_THREADS) {
                    mt = monitor_threads[evt.pin];
                    if (mt && mt->active && !mt->closing) {
                        if (mt->wait_for == GPIO_BOTH_EDGES || mt->wait_for == evt.edge) {
                            mt->latest_event = evt.edge;
                            cb = mt->callback;
                        }
                    }
                }
                pthread_mutex_unlock(&g_evt_lock);
                if (cb && mt) cb((void*)mt);
            }
        } else {
            if (pfds[0].revents & POLLIN) vgp_evt_drain_wake();

            for (int i = 1; i < n; i++) {
                if (!(pfds[i].revents & POLLIN)) continue;

                pthread_mutex_lock(&g_evt_lock);
                MonitorThread *mt = mts[i];
                if (!mt || !mt->active || mt->closing || !mt->line) {
                    pthread_mutex_unlock(&g_evt_lock);
                    continue;
                }

                struct gpiod_line_event ev;
                if (gpiod_line_event_read(mt->line, &ev) < 0) {
                    pthread_mutex_unlock(&g_evt_lock);
                    continue;
                }
                mt->latest_event = (ev.event_type == GPIOD_LINE_EVENT_RISING_EDGE ? GPIO_RISING_EDGE : GPIO_FALLING_EDGE);
                void (*cb)(void*) = mt->callback;
                pthread_mutex_unlock(&g_evt_lock);

                if (cb) cb((void *)mt);
            }
        }
    }
}

static void vgp_evt_shutdown(void) {
    pthread_mutex_lock(&g_evt_lock);
    g_evt_stop = 1;
    vgp_evt_wake_locked();
    pthread_mutex_unlock(&g_evt_lock);

    if (g_evt_thread_started) {
        (void)pthread_join(g_evt_thread, NULL);
        g_evt_thread_started = 0;
    }
    if (g_evt_wake_pipe[0] >= 0) close(g_evt_wake_pipe[0]);
    if (g_evt_wake_pipe[1] >= 0) close(g_evt_wake_pipe[1]);
    g_evt_wake_pipe[0] = g_evt_wake_pipe[1] = -1;
    pthread_mutex_lock(&g_evt_lock);
    privd_evt_close_locked();
    g_evt_use_privd = -1;
    pthread_mutex_unlock(&g_evt_lock);
}

static int vgp_evt_ensure_started(void) {
    pthread_mutex_lock(&g_evt_lock);
    if (g_evt_thread_started) {
        pthread_mutex_unlock(&g_evt_lock);
        return 0;
    }

    // Re-decide next start
    g_evt_use_privd = -1;
    privd_evt_close_locked();

    if (pipe(g_evt_wake_pipe) < 0) {
        EVTDBG("[vgp-evt] pipe() failed: %s (errno=%d)\n", strerror(errno), errno);
        pthread_mutex_unlock(&g_evt_lock);
        return -1;
    }

    // Best-effort: CLOEXEC + NONBLOCK
    for (int i = 0; i < 2; i++) {
        int flags = fcntl(g_evt_wake_pipe[i], F_GETFD);
        if (flags >= 0) (void)fcntl(g_evt_wake_pipe[i], F_SETFD, flags | FD_CLOEXEC);
        flags = fcntl(g_evt_wake_pipe[i], F_GETFL);
        if (flags >= 0) (void)fcntl(g_evt_wake_pipe[i], F_SETFL, flags | O_NONBLOCK);
    }

    g_evt_stop = 0;
    int err = pthread_create(&g_evt_thread, NULL, vgp_evt_loop, NULL);
    if (err != 0) {
        EVTDBG("[vgp-evt] pthread_create failed: %s (errno=%d)\n", strerror(err), err);
        close(g_evt_wake_pipe[0]);
        close(g_evt_wake_pipe[1]);
        g_evt_wake_pipe[0] = g_evt_wake_pipe[1] = -1;
        pthread_mutex_unlock(&g_evt_lock);
        return -1;
    }

    g_evt_thread_started = 1;
    EVTDBG("[vgp-evt] event dispatcher thread started\n");
    atexit(vgp_evt_shutdown);

    pthread_mutex_unlock(&g_evt_lock);
    return 0;
}

void thread_cleanup_handler(void *p) {
    MonitorThread * params = (MonitorThread *)p;
    gpiod_line_release(params->line);
    gpiod_chip_close(params->chip);
}

void * monitor_pin(void *p) {
    pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);

    MonitorThread * params = (MonitorThread *)p;

    sleep(params->delay);

    char * pin_name = (char *)NAMES[params->pin];
    int ch = get_chip_number(pin_name);
    int ln = get_line_number(pin_name);

    char chipname[30];
    snprintf(chipname, sizeof(chipname), "/dev/gpiochip%d", ch);
    params->chip = gpiod_chip_open(chipname);
    if (!params->chip) {
        perror("Error opening GPIO chip");
        return NULL;
    }

    params->line = gpiod_chip_get_line(params->chip, ln);
    if (!params->line) {
        perror("Error getting GPIO line");
        gpiod_chip_close(params->chip);
        return NULL;
    }

    pthread_cleanup_push(thread_cleanup_handler, p);

    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

    // request event
    int ret;
    switch (params->wait_for) {
    case GPIO_RISING_EDGE:
        ret = gpiod_line_request_rising_edge_events(params->line, "vgplib");
        break;
    case GPIO_FALLING_EDGE:
        ret = gpiod_line_request_falling_edge_events(params->line, "vgplib");
        break;
    case GPIO_BOTH_EDGES:
        ret = gpiod_line_request_both_edges_events(params->line, "vgplib");
        break;
    default:
        ret = -1;
    }
    if (ret < 0) {
        perror("Error requesting GPIO line events");
        gpiod_line_release(params->line);
        gpiod_chip_close(params->chip);
        return NULL;
    }

    // wait for event
    while (1) {
        pthread_testcancel();

        struct gpiod_line_event event;
        ret = gpiod_line_event_wait(params->line, NULL);
        if (ret < 0) {
            perror("Error waiting for GPIO event");
            gpiod_line_release(params->line);
            gpiod_chip_close(params->chip);
            return NULL;
        }
        ret = gpiod_line_event_read(params->line, &event);
        if (ret < 0) {
            perror("Error reading GPIO event");
            gpiod_line_release(params->line);
            gpiod_chip_close(params->chip);
            return NULL;
        }
        params->latest_event = (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE ? GPIO_RISING_EDGE : GPIO_FALLING_EDGE);
        params->callback(p);
    }

    // release GPIO resources
    gpiod_line_release(params->line);
    gpiod_chip_close(params->chip);

    pthread_cleanup_pop(0);

    return NULL;
}

void create_monitor_thread(int pin, int delay, int wait_for, void (*callback)(void*)) {
    if (pin <= 0 || pin >= MONITOR_THREADS) return;
    EVTDBG("[vgp-evt] create pin=%d delay=%d wait_for=%d\n", pin, delay, wait_for);
    if (vgp_evt_ensure_started() < 0) return;

    // Release cached line to avoid EBUSY between cache and event request
    if (g_gpiod_cache_enabled) {
        char *pin_name = (char *)NAMES[pin];
        int ch = get_chip_number(pin_name);
        int ln = get_line_number(pin_name);
        pthread_mutex_lock(&g_gpiod_cache_lock);
        if (ch >= 0 && ch < VGP_MAX_CHIPS && ln >= 0 && ln < VGP_MAX_LINES) {
            VgpLineCache *e = &g_line_cache[ch][ln];
            if (e->line && e->requested) {
                gpiod_line_release(e->line);
                e->requested = 0;
                e->is_output = 0;
            }
        }
        pthread_mutex_unlock(&g_gpiod_cache_lock);
    }

    pthread_mutex_lock(&g_evt_lock);

    // If exists, mark closing (dispatcher will free it safely)
    if (monitor_threads[pin]) {
        monitor_threads[pin]->closing = 1;
    }

    MonitorThread *mt = (MonitorThread *)calloc(1, sizeof(MonitorThread));
    if (!mt) {
        pthread_mutex_unlock(&g_evt_lock);
        return;
    }

    mt->pin = pin;
    mt->delay = delay;
    mt->wait_for = wait_for;
    mt->callback = callback;
    mt->latest_event = 0;
    mt->fd = -1;
    mt->active = 0;
    mt->closing = 0;
    mt->start_at = time(NULL) + (delay > 0 ? delay : 0);

    monitor_threads[pin] = mt;

    vgp_evt_wake_locked();
    pthread_mutex_unlock(&g_evt_lock);
}

void destroy_monitor_thread(int pin) {
    if (pin <= 0 || pin >= MONITOR_THREADS) return;
    EVTDBG("[vgp-evt] destroy pin=%d\n", pin);
    pthread_mutex_lock(&g_evt_lock);
    if (monitor_threads[pin]) {
        monitor_threads[pin]->closing = 1;
        vgp_evt_wake_locked();
    }
    pthread_mutex_unlock(&g_evt_lock);
}
// -----------------------------------------
// End: Single-thread GPIO event dispatcher
// -----------------------------------------

// -----------------------------------------
// Begin: ADC (IIO sysfs)
// -----------------------------------------
enum { VGP_MAX_ADC_CHANNELS = 8 };
static pthread_once_t g_adc_once = PTHREAD_ONCE_INIT;
static pthread_mutex_t g_adc_lock = PTHREAD_MUTEX_INITIALIZER;
static int g_iio_dirfd = -1;
static int g_adc_fd[VGP_MAX_ADC_CHANNELS];

static int vgp_adc_channel_allowed(int ch) {
    return (ch == 0 || ch == 3 || ch == 4); // Vivid Unit exposes A0/A3/A4
}

static void vgp_adc_init_once(void) {
    for (int i = 0; i < VGP_MAX_ADC_CHANNELS; i++) g_adc_fd[i] = -1;
}

static int vgp_parse_int_fast(const char *s, ssize_t n, int *out) {
    if (!s || n <= 0 || !out) return -1;
    ssize_t i = 0;
    while (i < n && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) i++;
    int sign = 1;
    if (i < n && s[i] == '-') { sign = -1; i++; }
    int v = 0, any = 0;
    for (; i < n; i++) {
        char c = s[i];
        if (c < '0' || c > '9') break;
        any = 1;
        v = v * 10 + (c - '0');
    }
    if (!any) return -1;
    *out = v * sign;
    return 0;
}

static int vgp_adc_ensure_dir_locked(void) {
    if (g_iio_dirfd >= 0) return 0;
    g_iio_dirfd = open("/sys/bus/iio/devices/iio:device0", O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    return (g_iio_dirfd >= 0) ? 0 : -1;
}

static int vgp_adc_ensure_fd_locked(int ch) {
    if (ch < 0 || ch >= VGP_MAX_ADC_CHANNELS) return -1;
    if (g_adc_fd[ch] >= 0) return g_adc_fd[ch];
    if (vgp_adc_ensure_dir_locked() < 0) return -1;

    char name[64];
    snprintf(name, sizeof(name), "in_voltage%d_raw", ch);
    int fd = openat(g_iio_dirfd, name, O_RDONLY | O_CLOEXEC);
    if (fd < 0) return -1;
    g_adc_fd[ch] = fd;
    return fd;
}

int get_adc(int a_pin) {
    pthread_once(&g_adc_once, vgp_adc_init_once);
    if (a_pin < 0 || a_pin >= VGP_MAX_ADC_CHANNELS) return -1;
    if (!vgp_adc_channel_allowed(a_pin)) return -1;

    int fd;
    pthread_mutex_lock(&g_adc_lock);
    fd = vgp_adc_ensure_fd_locked(a_pin);
    pthread_mutex_unlock(&g_adc_lock);
    if (fd < 0) return -1;

    // pread() is ideal, fall back if unsupported
    char buf[32];
    ssize_t n = pread(fd, buf, (ssize_t)sizeof(buf) - 1, 0);
    if (n < 0 && (errno == ESPIPE || errno == EINVAL)) {
        pthread_mutex_lock(&g_adc_lock);
        if (lseek(fd, 0, SEEK_SET) >= 0) {
            n = read(fd, buf, (ssize_t)sizeof(buf) - 1);
        }
        pthread_mutex_unlock(&g_adc_lock);
    }
    if (n <= 0) return -1;
    buf[n] = '\0';

    int val;
    if (vgp_parse_int_fast(buf, n, &val) < 0) return -1;
    return val;
}

float get_voltage_by_adc(int adc) {
    return 5.0f * (float)adc / 1024;
}
// -----------------------------------------
// End: ADC (IIO sysfs)
// -----------------------------------------

// Allow libgpiod fallback only when vgpd service is unavailable
static int privd_err_is_unavailable(int st) {
    if (st == 0) return 0;
    if (st == -1) return 1;
    if (st > 0) return 0;
    switch (-st) {
        case ENOENT:
        case ECONNREFUSED:
        case ECONNRESET:
        case ECONNABORTED:
        case ENOTCONN:
        case EPIPE:
        case ETIMEDOUT:
        case EAGAIN:
            return 1;
        default:
            return 0;
    }
}

static void pinmap_init_once(void) {
    if (g_pinmap_inited) return;
    memset(g_chln2pin, 0, sizeof(g_chln2pin));
    for (int pin = 1; pin <= 40; pin++) {
        if (is_power_pin(pin)) continue;
        const char *name = NAMES[pin];
        if (!name) continue;
        int ch = get_chip_number((char *)name);
        int ln = get_line_number((char *)name);
        if (ch >= 0 && ch < VGP_MAX_CHIPS && ln >= 0 && ln < VGP_MAX_LINES) {
            g_chln2pin[ch][ln] = pin;
        }
    }
    g_pinmap_inited = 1;
}

static int chln_to_physical_pin(int ch, int ln) {
    if (!g_pinmap_inited) pinmap_init_once();
    if (ch < 0 || ch >= VGP_MAX_CHIPS) return 0;
    if (ln < 0 || ln >= VGP_MAX_LINES) return 0;
    return g_chln2pin[ch][ln];
}

static void privd_close_locked(void) {
    if (g_privd_fd >= 0) {
        close(g_privd_fd);
        g_privd_fd = -1;
    }
    g_cache40_valid = 0;
}

static int privd_connect_locked(void) {
    if (g_privd_fd >= 0) return 0;

    uint64_t t = now_ns();
    // 1s backoff on failures
    if (g_privd_last_try_ns && (t - g_privd_last_try_ns) < 1000000000ULL) return -EAGAIN;
    g_privd_last_try_ns = t;

    int fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
    if (fd < 0) return -errno;

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, VGPD_SOCK_PATH, sizeof(addr.sun_path) - 1);

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        int e = errno;
        if (vgpd_debug_enabled()) {
            fprintf(stderr, "vgplib: connect(%s) failed: %s\n", VGPD_SOCK_PATH, strerror(e));
            if (e == EACCES) {
                fprintf(stderr, "vgplib: hint: socket permissions deny access; check `ls -l %s` (need group access for your user)\n", VGPD_SOCK_PATH);
            }
        }
        close(fd);
        return -e;
    }

    g_privd_fd = fd;
    return 0;
}

static int privd_rpc(uint16_t cmd, const void *req_body, size_t req_body_len,
                     void *resp_buf, size_t resp_buf_max, size_t *out_resp_len) {
    uint8_t req[256];
    if (sizeof(vgp_req_hdr_t) + req_body_len > sizeof(req)) return -EINVAL;

    pthread_mutex_lock(&g_privd_lock);
    int cst = privd_connect_locked();
    if (cst != 0) {
        pthread_mutex_unlock(&g_privd_lock);
        return cst;
    }

    vgp_req_hdr_t hdr;
    hdr.magic = VGP_PRIVD_MAGIC;
    hdr.ver = VGP_PRIVD_VER;
    hdr.cmd = cmd;
    hdr.size = (uint32_t)(sizeof(vgp_req_hdr_t) + req_body_len);
    hdr.seq = g_privd_seq++;

    memcpy(req, &hdr, sizeof(hdr));
    if (req_body_len) memcpy(req + sizeof(hdr), req_body, req_body_len);

    ssize_t n = send(g_privd_fd, req, hdr.size, MSG_NOSIGNAL);
    if (n != (ssize_t)hdr.size) {
        privd_close_locked();
        pthread_mutex_unlock(&g_privd_lock);
        return -( (n < 0) ? errno : EPIPE );
    }

    ssize_t r = recv(g_privd_fd, resp_buf, resp_buf_max, 0);
    if (r < 0) {
        int e = errno;
        privd_close_locked();
        pthread_mutex_unlock(&g_privd_lock);
        return -e;
    }
    if (r < (ssize_t)sizeof(vgp_resp_hdr_t)) {
        privd_close_locked();
        pthread_mutex_unlock(&g_privd_lock);
        return -ENOTCONN;
    }

    vgp_resp_hdr_t resp_hdr;
    memcpy(&resp_hdr, resp_buf, sizeof(resp_hdr));
    if (resp_hdr.magic != VGP_PRIVD_MAGIC || resp_hdr.ver != VGP_PRIVD_VER || resp_hdr.cmd != cmd) {
        privd_close_locked();
        pthread_mutex_unlock(&g_privd_lock);
        return -EPROTO;
    }
    if (resp_hdr.size != (uint32_t)r) {
        privd_close_locked();
        pthread_mutex_unlock(&g_privd_lock);
        return -EPROTO;
    }

    if (out_resp_len) *out_resp_len = (size_t)r;
    int status = resp_hdr.status;
    pthread_mutex_unlock(&g_privd_lock);
    return status;
}

static int privd_get40_cached(void) {
    uint64_t t = now_ns();
    if (g_cache40_valid && (t - g_cache40_ts_ns) <= VGP_GET40_CACHE_TTL_NS) return 0;

    uint8_t resp[1024];
    size_t resp_len = 0;
    int st = privd_rpc(VGP_CMD_GET40, NULL, 0, resp, sizeof(resp), &resp_len);
    if (st != 0) return st;

    if (resp_len < sizeof(vgp_resp_hdr_t) + sizeof(vgp_resp_40_t)) return -1;
    vgp_resp_40_t body;
    memcpy(&body, resp + sizeof(vgp_resp_hdr_t), sizeof(body));
    memcpy(g_cache40, body.pins, sizeof(g_cache40));
    g_cache40_ts_ns = t;
    g_cache40_valid = 1;
    return 0;
}

static int privd_get_pin_state(int ch, int ln, vgp_pin_state_t *out) {
    vgp_req_pin_t req;
    req.ch = (uint8_t)ch;
    req.ln = (uint8_t)ln;

    uint8_t resp[256];
    size_t resp_len = 0;
    int st = privd_rpc(VGP_CMD_GET_PIN, &req, sizeof(req), resp, sizeof(resp), &resp_len);
    if (st != 0) return st;
    if (resp_len < sizeof(vgp_resp_hdr_t) + sizeof(vgp_resp_pin_t)) return -1;
    vgp_resp_pin_t body;
    memcpy(&body, resp + sizeof(vgp_resp_hdr_t), sizeof(body));
    *out = body.st;
    return 0;
}

static int privd_set_simple_u8i8(uint16_t cmd, int ch, int ln, int8_t val) {
    uint8_t resp[128];
    size_t resp_len = 0;
    int st;

    if (cmd == VGP_CMD_SET_ALT) {
        vgp_req_set_alt_t req = { .ch = (uint8_t)ch, .ln = (uint8_t)ln, .alt = val };
        st = privd_rpc(cmd, &req, sizeof(req), resp, sizeof(resp), &resp_len);
    } else if (cmd == VGP_CMD_SET_DIR) {
        vgp_req_set_dir_t req = { .ch = (uint8_t)ch, .ln = (uint8_t)ln, .dir = val };
        st = privd_rpc(cmd, &req, sizeof(req), resp, sizeof(resp), &resp_len);
    } else if (cmd == VGP_CMD_SET_VAL) {
        vgp_req_set_val_t req = { .ch = (uint8_t)ch, .ln = (uint8_t)ln, .val = val };
        st = privd_rpc(cmd, &req, sizeof(req), resp, sizeof(resp), &resp_len);
    } else {
        return -EINVAL;
    }

    if (st == 0) g_cache40_valid = 0;
    return st;
}

static struct gpiod_chip *vgp_cache_get_chip_locked(int ch) {
    if (ch < 0 || ch >= VGP_MAX_CHIPS) return NULL;
    if (g_chip_cache[ch]) return g_chip_cache[ch];

    char path[32];
    snprintf(path, sizeof(path), "/dev/gpiochip%d", ch);
    g_chip_cache[ch] = gpiod_chip_open(path);
    return g_chip_cache[ch];
}

static VgpLineCache *vgp_cache_get_line_locked(int ch, int ln) {
    if (ch < 0 || ch >= VGP_MAX_CHIPS) return NULL;
    if (ln < 0 || ln >= VGP_MAX_LINES) return NULL;

    VgpLineCache *e = &g_line_cache[ch][ln];
    if (e->line) return e;

    struct gpiod_chip *chip = vgp_cache_get_chip_locked(ch);
    if (!chip) return NULL;

    e->line = gpiod_chip_get_line(chip, ln);
    return e;
}

static void vgp_cache_release_line_locked(int ch, int ln) {
    if (ch < 0 || ch >= VGP_MAX_CHIPS) return;
    if (ln < 0 || ln >= VGP_MAX_LINES) return;

    VgpLineCache *e = &g_line_cache[ch][ln];
    if (e->requested && e->line) {
        gpiod_line_release(e->line);
    }
    e->requested = 0;
    e->is_output = 0;
}

void vgp_gpiod_cache_enable(int enable) {
    pthread_mutex_lock(&g_gpiod_cache_lock);
    g_gpiod_cache_enabled = enable ? 1 : 0;
    pthread_mutex_unlock(&g_gpiod_cache_lock);

    // Auto cleanup on exit when enabled (safe even if called multiple times)
    if (enable) atexit(vgp_gpiod_cache_cleanup);
}

void vgp_gpiod_cache_cleanup(void) {
    pthread_mutex_lock(&g_gpiod_cache_lock);

    for (int ch = 0; ch < VGP_MAX_CHIPS; ch++) {
        for (int ln = 0; ln < VGP_MAX_LINES; ln++) {
            vgp_cache_release_line_locked(ch, ln);
            g_line_cache[ch][ln].line = NULL; // invalid after chip close
        }
        if (g_chip_cache[ch]) {
            gpiod_chip_close(g_chip_cache[ch]);
            g_chip_cache[ch] = NULL;
        }
    }

    pthread_mutex_unlock(&g_gpiod_cache_lock);
}

int run_command(const char * cmd) {
    FILE *fp;
    if ((fp = popen(cmd, "r")) == NULL) {
        return -1;
    }
    int length = 0;
    if (fgets(output_buffer, OUTPUT_BUFFER_SIZE, fp) != NULL) {
        length = strlen(output_buffer);
    }
    if (pclose(fp)) {
        return -2;
    }
    return length;
}

static void regs_deinit_unlocked(void) {
    if (pmugrf_map) {
        munmap((void*)pmugrf_map, PMUGRF_MAP_SIZE);
        pmugrf_map = NULL;
    }
    if (grf_map)    {
        munmap((void*)grf_map,    GRF_MAP_SIZE);
        grf_map = NULL;
    }
    for (int i = 0; i < 5; i++) {
        if (gpio_map[i]) {
            munmap((void*)gpio_map[i], GPIO_MAP_SIZE);
            gpio_map[i] = NULL;
        }
    }
    if (mem_fd >= 0) {
        close(mem_fd);
        mem_fd = -1;
    }
    regs_inited = 0;
}

static int regs_init(void) {
    pthread_mutex_lock(&regs_lock);
    if (regs_inited) {
        pthread_mutex_unlock(&regs_lock);
        return 0;
    }

    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        fprintf(stderr, "open(/dev/mem) failed: %s\n", strerror(errno));
        pthread_mutex_unlock(&regs_lock);
        return -1;
    }

    pmugrf_map = (volatile unsigned char*)mmap(NULL, PMUGRF_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, (off_t)PMUGRF);
    if (pmugrf_map == (void*)MAP_FAILED) {
        pmugrf_map = NULL;
        fprintf(stderr, "mmap(PMUGRF) failed: %s\n", strerror(errno));
        regs_deinit_unlocked();
        pthread_mutex_unlock(&regs_lock);
        return -2;
    }

    grf_map = (volatile unsigned char*)mmap(NULL, GRF_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, (off_t)GRF);
    if (grf_map == (void*)MAP_FAILED) {
        grf_map = NULL;
        fprintf(stderr, "mmap(GRF) failed: %s\n", strerror(errno));
        regs_deinit_unlocked();
        pthread_mutex_unlock(&regs_lock);
        return -3;
    }

    for (int i = 0; i < 5; i++) {
        gpio_map[i] = (volatile unsigned char*)mmap(NULL, GPIO_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, (off_t)GPIO_BASE[i]);
        if (gpio_map[i] == (void*)MAP_FAILED) {
            gpio_map[i] = NULL;
            fprintf(stderr, "mmap(GPIO_BASE[%d]=0x%08x) failed: %s\n", i, GPIO_BASE[i], strerror(errno));
            regs_deinit_unlocked();
            pthread_mutex_unlock(&regs_lock);
            return -4;
        }
    }

    regs_inited = 1;
    pthread_mutex_unlock(&regs_lock);
    return 0;
}

static volatile uint32_t *addr_to_reg(uint32_t address) {
    // strict whitelist
    if (pmugrf_map && address >= PMUGRF && (address + 4u) <= (PMUGRF + PMUGRF_MAP_SIZE))
        return (volatile uint32_t*)(pmugrf_map + (address - PMUGRF));
    if (grf_map && address >= GRF && (address + 4u) <= (GRF + GRF_MAP_SIZE))
        return (volatile uint32_t*)(grf_map + (address - GRF));
    for (int i = 0; i < 5; i++) {
        uint32_t base = GPIO_BASE[i];
        if (gpio_map[i] && address >= base && (address + 4u) <= (base + GPIO_MAP_SIZE))
            return (volatile uint32_t*)(gpio_map[i] + (address - base));
    }
    return NULL;
}

int get_register(unsigned int address) {
    if (regs_init() != 0) return -1;
    volatile uint32_t *p = addr_to_reg((uint32_t)address);
    if (!p) return -2;
    return (int)(*p);
}

int set_register(unsigned int address, unsigned int value) {
    if (regs_init() != 0) return -1;
    volatile uint32_t *p = addr_to_reg((uint32_t)address);
    if (!p) return -2;
    *p = (uint32_t)value;
    __sync_synchronize();
    return 0;
}

int get_chip_number(char *pin_name) {
    return pin_name[0] - 0x30;
}

int get_line_number(char *pin_name) {
    return ((pin_name[1] - 0x41) << 3) + (pin_name[2] - 0x30);
}

int get_dir(int ch, int ln) {
    int pin = chln_to_physical_pin(ch, ln);
    if (pin > 0) {
        if (privd_get40_cached() == 0) {
            if (g_cache40[pin].flags & VGP_PINF_VALID) {
                return g_cache40[pin].dir;
            }
        }
    } else {
        vgp_pin_state_t st;
        if (privd_get_pin_state(ch, ln, &st) == 0) {
            return st.dir;
        }
    }

    if (geteuid() != 0) return -EPERM;
    int group = ln / 8;
    int index = ln % 8;
    int gpio_directions = get_register(GPIO_BASE[ch] + GPIO_SWPORTA_DDR);
    int directions = ((gpio_directions >> (group << 3)) & 0xff);
    return ((directions & (0x01 << index)) >> index);
}

int set_dir(int ch, int ln, int dir) {
    if (g_gpiod_cache_enabled) {
        pthread_mutex_lock(&g_gpiod_cache_lock);
        vgp_cache_release_line_locked(ch, ln);
        pthread_mutex_unlock(&g_gpiod_cache_lock);
    }
    int st = privd_set_simple_u8i8(VGP_CMD_SET_DIR, ch, ln, (int8_t)dir);
    if (st == 0) return 0;

    if (geteuid() != 0) return -EPERM;
    // Drop cached request for this line after changing direction
    if (g_gpiod_cache_enabled) {
        pthread_mutex_lock(&g_gpiod_cache_lock);
        vgp_cache_release_line_locked(ch, ln);
        pthread_mutex_unlock(&g_gpiod_cache_lock);
    }
    int group = ln / 8;
    int index = ln % 8;
    int gpio_directions = get_register(GPIO_BASE[ch] + GPIO_SWPORTA_DDR);
    int directions = ((gpio_directions >> (group << 3)) & 0xff);
    if (dir == GPIO_INPUT) {
        directions &= ~(0x01 << index);
    } else if (dir == GPIO_OUTPUT) {
        directions |= (0x01 << index);
    } else {
        fprintf(stderr, "Unknown direction %d\n", dir);
        return -3;
    }
    gpio_directions &= ~(0xFF << (group << 3));
    gpio_directions |= (directions << (group << 3));
    set_register(GPIO_BASE[ch] + GPIO_SWPORTA_DDR, gpio_directions);
    return 0;
}

int get_alt(int ch, int ln) {
    int pin = chln_to_physical_pin(ch, ln);
    if (pin > 0) {
        if (privd_get40_cached() == 0) {
            if (g_cache40[pin].flags & VGP_PINF_VALID) {
                return g_cache40[pin].alt;
            }
        }
    } else {
        vgp_pin_state_t st;
        if (privd_get_pin_state(ch, ln, &st) == 0) {
            return st.alt;
        }
    }

    if (geteuid() != 0) return -EPERM;
    int group = ln / 8;
    int index = ln % 8;
    if (GPIO_IOMUX[ch][group] == -1) return -1;
    int iomux = get_register((ch < 2 ? PMUGRF : GRF) + (uint32_t)GPIO_IOMUX[ch][group]);
    if (iomux < 0) return iomux;
    return ((iomux >> (index << 1)) & 0x03);
}

int set_alt(int ch, int ln, int alt) {
    if (g_gpiod_cache_enabled) {
        pthread_mutex_lock(&g_gpiod_cache_lock);
        vgp_cache_release_line_locked(ch, ln);
        pthread_mutex_unlock(&g_gpiod_cache_lock);
    }
    int st = privd_set_simple_u8i8(VGP_CMD_SET_ALT, ch, ln, (int8_t)alt);
    if (st == 0) return 0;

    if (geteuid() != 0) return -EPERM;
    // Drop cached request for this line after changing ALT
    if (g_gpiod_cache_enabled) {
        pthread_mutex_lock(&g_gpiod_cache_lock);
        vgp_cache_release_line_locked(ch, ln);
        pthread_mutex_unlock(&g_gpiod_cache_lock);
    }
    int group = ln / 8;
    int index = ln % 8;
    if (GPIO_IOMUX[ch][group] == -1) return -1;
    uint32_t addr = (ch < 2 ? PMUGRF : GRF) + (uint32_t)GPIO_IOMUX[ch][group];
    int cur = get_register(addr);
    if (cur < 0) return cur;
    uint32_t v = (uint32_t)cur & 0x0000FFFFu;
    v &= ~(0x03u << (index << 1));
    v |= ((uint32_t)alt << (index << 1));
    v |= (0x03u << ((index << 1) + 16));  // write mask
    return set_register(addr, v);
}

int get(int ch, int ln) {
    int pin = chln_to_physical_pin(ch, ln);
    if (pin > 0) {
        int st = privd_get40_cached();
        if (st == 0) {
            if (g_cache40[pin].flags & VGP_PINF_VALID && g_cache40[pin].val >= 0) {
                return g_cache40[pin].val;
            }
            return -EIO; // vgpd answered, but the payload is not usable.
        }
        if (!privd_err_is_unavailable(st)) return st;
    } else {
        vgp_pin_state_t st;
        int pst = privd_get_pin_state(ch, ln, &st);
        if (pst == 0) {
            if (st.val >= 0) return st.val;
            return -EIO;
        }
        if (!privd_err_is_unavailable(pst)) return pst;
    }

    // Fast path: cached gpiod line (optional)
    if (g_gpiod_cache_enabled) {
        pthread_mutex_lock(&g_gpiod_cache_lock);
        VgpLineCache *e = vgp_cache_get_line_locked(ch, ln);
        if (!e || !e->line) {
            pthread_mutex_unlock(&g_gpiod_cache_lock);
            return -1;
        }
        if (!e->requested) {
            struct gpiod_line_request_config cfg;
            memset(&cfg, 0, sizeof(cfg));
            cfg.consumer = "vgp";
            cfg.request_type = GPIOD_LINE_REQUEST_DIRECTION_AS_IS;
            int req = gpiod_line_request(e->line, &cfg, 0);
            if (req < 0) {
                int errsv = errno;
                pthread_mutex_unlock(&g_gpiod_cache_lock);
                if (errsv == EBUSY) return vgp_read_value_via_monitor(ch, ln);
                return -2;
            }
            e->requested = 1;
            e->is_output = 0;
        }
        int v = gpiod_line_get_value(e->line);
        pthread_mutex_unlock(&g_gpiod_cache_lock);
        return v;
    }

    // Compatibility path: open/request/release each call
    struct gpiod_chip *chip = NULL;
    struct gpiod_line *line = NULL;
    struct gpiod_line_request_config cfg;
    int req, value, ret = 0;

    chip_name[13] = ch + 0x30;
    chip = gpiod_chip_open(chip_name);
    if (chip) {
        line = gpiod_chip_get_line(chip, ln);
        if (line) {
            memset(&cfg, 0, sizeof(cfg));
            cfg.consumer = "vgp";
            cfg.request_type = GPIOD_LINE_REQUEST_DIRECTION_AS_IS;
            req = gpiod_line_request(line, &cfg, 0);
            if (req >= 0) {
                value = gpiod_line_get_value(line);
                ret = value;
            } else {
                if (errno == EBUSY) {
                    ret = vgp_read_value_via_monitor(ch, ln);
                } else {
                    ret = -3;
                }
            }
        } else {
            ret = -2;
        }
    } else {
        ret = -1;
    }
    if (line) gpiod_line_release(line);
    if (chip) gpiod_chip_close(chip);
    return ret;
}

int set(int ch, int ln, int val) {
    if (g_gpiod_cache_enabled) {
        pthread_mutex_lock(&g_gpiod_cache_lock);
        vgp_cache_release_line_locked(ch, ln);
        pthread_mutex_unlock(&g_gpiod_cache_lock);
    }
    int st = privd_set_simple_u8i8(VGP_CMD_SET_VAL, ch, ln, (int8_t)(val ? 1 : 0));
    if (st == 0) return 0;
    
    // Do not fall back to libgpiod if vgpd is reachable and explicitly rejects the operation
    if (st == -EPERM || st == -EINVAL) return st;
    if (!privd_err_is_unavailable(st)) return st;

    // Fast path: cached gpiod line (optional)
    if (g_gpiod_cache_enabled) {
        pthread_mutex_lock(&g_gpiod_cache_lock);
        VgpLineCache *e = vgp_cache_get_line_locked(ch, ln);
        if (!e || !e->line) {
            pthread_mutex_unlock(&g_gpiod_cache_lock);
            return -1;
        }
        if (!e->requested || !e->is_output) {
            // If previously requested as input/as-is, release and re-request as output.
            if (e->requested) {
                gpiod_line_release(e->line);
                e->requested = 0;
                e->is_output = 0;
            }
            int req = gpiod_line_request_output(e->line, "vgp", val ? 1 : 0);
            if (req < 0) {
                pthread_mutex_unlock(&g_gpiod_cache_lock);
                return -2;
            }
            e->requested = 1;
            e->is_output = 1;
        }
        int rc = gpiod_line_set_value(e->line, val ? 1 : 0);
        pthread_mutex_unlock(&g_gpiod_cache_lock);
        return (rc < 0) ? -3 : 0;
    }

    // Compatibility path: open/request/release each call
    struct gpiod_chip *chip = NULL;
    struct gpiod_line *line = NULL;
    int req, ret = 0;

    chip_name[13] = ch + 0x30;
    chip = gpiod_chip_open(chip_name);
    if (chip) {
        line = gpiod_chip_get_line(chip, ln);
        if (line) {
            req = gpiod_line_request_output(line, "vgp", 0);
            if (req >= 0) {
                gpiod_line_set_value(line, val);
            } else {
                ret = -3;
            }
        } else {
            ret = -2;
        }
    } else {
        ret = -1;
    }
    if (line) gpiod_line_release(line);
    if (chip) gpiod_chip_close(chip);
    return ret;
}

bool is_power_pin(int pin) {
    if (pin == 1 || pin == 2 || pin == 4 || pin == 6 || pin == 9 || pin == 14
            || pin == 17 || pin == 20 || pin == 25 || pin == 30 || pin == 34 || pin == 39) {
        return true;
    }
    return false;
}
