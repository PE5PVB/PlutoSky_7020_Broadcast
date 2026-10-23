// SPDX-License-Identifier: Apache-2.0
// Minimal ZeroMQ SUB endpoint (ZMTP 3.0, NULL security) that listens for one publisher, e.g. ODR-AudioEnc's
// "-o tcp://<host>:<port>" output. Non-blocking: call zmtp_sub_poll() regularly.
#ifndef ZMTP_SUB_H
#define ZMTP_SUB_H

#include <stddef.h>
#include <stdint.h>

typedef struct zmtp_sub zmtp_sub;

zmtp_sub   *zmtp_sub_listen(int port);
void        zmtp_sub_close(zmtp_sub *z);
// one complete message into out (at most outcap bytes): its length; 0 = none yet; -1 = the publisher went away
int         zmtp_sub_poll(zmtp_sub *z, uint8_t *out, size_t outcap);
const char *zmtp_sub_peer(const zmtp_sub *z);       // the publisher's address, or NULL

#endif
