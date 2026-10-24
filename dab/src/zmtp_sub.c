// SPDX-License-Identifier: Apache-2.0
// Minimal ZeroMQ (ZMTP 3.0, NULL security) SUB endpoint that LISTENS for a publisher, as ODR-DabMux's "zmq+tcp://*:port"
// input does for ODR-AudioEnc. Own implementation of the wire protocol (RFC 23/ZMTP); no libzmq. One publisher at a time: a new
// connection replaces the old one, and a connection on which nothing arrives for IDLE_S seconds is closed (a sender that went
// away without closing, or a connection that a NAT router forgot), so that the publisher's automatic reconnection gets through.
#define _GNU_SOURCE
#include "zmtp_sub.h"
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define IDLE_S  10.0
#define MSG_MAX (1u << 24) // the largest frame and the largest message (all its frames) accepted

enum { ST_GREET, ST_HANDSHAKE, ST_RUN };

struct zmtp_sub {
    int lfd, cfd, state; // listening socket, connection (-1: none), ST_*
    uint8_t *buf;        // bytes received, not yet parsed
    size_t len, cap;
    uint8_t *msg; // the message being assembled (frames with the MORE flag)
    size_t mlen, mcap;
    char peer[48];  // the publisher's address
    double last_rx; // when the last bytes arrived (monotonic seconds)
};

static double mono_s(void){
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}

static int send_all(int fd, const uint8_t *p, size_t n){
    while (n){
        ssize_t w = send(fd, p, n, MSG_NOSIGNAL);
        if (w < 0){
            if (errno == EINTR) continue;
            if (errno == EAGAIN){
                usleep(1000);
                continue;
            }
            return -1;
        }
        p += w;
        n -= (size_t)w;
    }
    return 0;
}

zmtp_sub *zmtp_sub_listen(int port){
    zmtp_sub *z = calloc(1, sizeof *z);
    if (!z) return NULL;
    z->cfd = -1;
    z->lfd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    int one = 1;
    setsockopt(z->lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port = htons((uint16_t)port);
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (z->lfd < 0 || bind(z->lfd, (struct sockaddr *)&a, sizeof a) || listen(z->lfd, 2)){
        if (z->lfd >= 0) close(z->lfd);
        free(z);
        return NULL;
    }
    fcntl(z->lfd, F_SETFL, O_NONBLOCK);
    z->cap = 1 << 16;
    z->buf = malloc(z->cap);
    z->mcap = 1 << 14;
    z->msg = malloc(z->mcap);
    if (!z->buf || !z->msg){
        zmtp_sub_close(z);
        return NULL;
    }
    return z;
}

static void drop_peer(zmtp_sub *z){
    if (z->cfd >= 0) close(z->cfd);
    z->cfd = -1;
    z->len = 0;
    z->mlen = 0;
    z->peer[0] = 0;
}

void zmtp_sub_close(zmtp_sub *z){
    if (!z) return;
    drop_peer(z);
    if (z->lfd >= 0) close(z->lfd);
    free(z->buf);
    free(z->msg);
    free(z);
}

const char *zmtp_sub_peer(const zmtp_sub *z) { return z->cfd >= 0 ? z->peer : NULL; }

// our side of the greeting: signature, version 3.0, mechanism NULL, as-server 0, filler
static int send_greeting(zmtp_sub *z){
    uint8_t g[64];
    memset(g, 0, sizeof g);
    g[0] = 0xFF;
    g[8] = 0x01;
    g[9] = 0x7F;
    g[10] = 3;
    g[11] = 0;
    memcpy(g + 12, "NULL", 4);
    return send_all(z->cfd, g, sizeof g);
}
// READY with Socket-Type SUB, then the subscription to everything (ZMTP 3.0: a message whose body is 0x01 + topic)
static int send_ready_and_subscribe(zmtp_sub *z){
    static const uint8_t ready[] = { 0x04, 25,  5,   'R', 'E', 'A', 'D', 'Y', 11, 'S', 'o', 'c', 'k', 'e',
                                     't',  '-', 'T', 'y', 'p', 'e', 0,   0,   0,  3,   'S', 'U', 'B' };
    static const uint8_t sub[] = { 0x00, 1, 0x01 };
    return send_all(z->cfd, ready, sizeof ready) || send_all(z->cfd, sub, sizeof sub) ? -1 : 0;
}

int zmtp_sub_poll(zmtp_sub *z, uint8_t *out, size_t outcap){
    { // a waiting connection: the newest publisher wins (the old connection is often dead without having been closed)
        struct sockaddr_in p;
        socklen_t pl = sizeof p;
        int c = accept4(z->lfd, (struct sockaddr *)&p, &pl, SOCK_CLOEXEC);
        if (c >= 0){
            drop_peer(z);
            fcntl(c, F_SETFL, O_NONBLOCK);
            int one = 1;
            setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
            setsockopt(c, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof one);
            z->cfd = c;
            z->state = ST_GREET;
            z->len = 0;
            z->mlen = 0;
            z->last_rx = mono_s();
            inet_ntop(AF_INET, &p.sin_addr, z->peer, sizeof z->peer);
            if (send_greeting(z)){
                drop_peer(z);
                return 0;
            }
        }
    }
    if (z->cfd < 0) return 0;
    for (;;){ // read what is there
        if (z->len == z->cap){
            size_t nc = z->cap * 2;
            uint8_t *nb = realloc(z->buf, nc);
            if (!nb){
                drop_peer(z);
                return -1;
            }
            z->buf = nb;
            z->cap = nc;
        }
        ssize_t r = recv(z->cfd, z->buf + z->len, z->cap - z->len, 0);
        if (r > 0){
            z->len += (size_t)r;
            z->last_rx = mono_s();
            continue;
        }
        if (r == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)){
            drop_peer(z);
            return -1;
        }
        break;
    }
    if (mono_s() - z->last_rx > IDLE_S){ // silent too long: closed, the publisher reconnects
        drop_peer(z);
        return -1;
    }
    size_t pos = 0;
    int got = 0;
    if (z->state == ST_GREET){
        if (z->len < 64) return 0;
        // only ZMTP 3, NULL security
        if (z->buf[0] != 0xFF || z->buf[9] != 0x7F || z->buf[10] < 3 || memcmp(z->buf + 12, "NULL", 4)){
            drop_peer(z);
            return -1;
        }
        pos = 64;
        z->state = ST_HANDSHAKE;
        if (send_ready_and_subscribe(z)){
            drop_peer(z);
            return -1;
        }
    }
    while (!got){
        if (z->len - pos < 2) break;
        uint8_t fl = z->buf[pos];
        size_t hdr = (fl & 0x02) ? 9 : 2, sz = 0;
        if (z->len - pos < hdr) break;
        if (fl & 0x02){
            for (int i = 0; i < 8; i++) sz = (sz << 8) | z->buf[pos + 1 + i];
        } else sz = z->buf[pos + 1];
        if (sz > MSG_MAX || z->mlen + sz > MSG_MAX){ // also bounds a message of many MORE frames
            drop_peer(z);
            return -1;
        }
        if (z->len - pos < hdr + sz) break;
        const uint8_t *body = z->buf + pos + hdr;
        pos += hdr + sz;
        if (fl & 0x04){ // a command (READY from the peer, PING, ...): nothing to do
            if (z->state == ST_HANDSHAKE) z->state = ST_RUN;
            continue;
        }
        if (z->mlen + sz > z->mcap){
            size_t nc = (z->mlen + sz) * 2;
            uint8_t *nm = realloc(z->msg, nc);
            if (!nm){
                drop_peer(z);
                return -1;
            }
            z->msg = nm;
            z->mcap = nc;
        }
        memcpy(z->msg + z->mlen, body, sz);
        z->mlen += sz;
        if (!(fl & 0x01)){ // the last frame of the message
            size_t n = z->mlen < outcap ? z->mlen : outcap;
            memcpy(out, z->msg, n);
            z->mlen = 0;
            got = (int)n;
        }
    }
    memmove(z->buf, z->buf + pos, z->len - pos);
    z->len -= pos;
    return got;
}
