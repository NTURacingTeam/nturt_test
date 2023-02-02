#ifndef IT_TRANSPORT_H
#define IT_TRANSPORT_H

// libc include
#include <stdbool.h>

// microros include
#include <uxr/client/transport.h>

bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport,
                              uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf,
                             size_t len, int timeout, uint8_t *err);

#endif // IT_TRANSPORT_H
