
#ifndef _MICROROS_TRANSPORTS_H_
#define _MICROROS_TRANSPORTS_H_

#ifdef __cplusplus
extern "C"
{
#endif

bool transport_serial_open(struct uxrCustomTransport * transport);
bool transport_serial_close(struct uxrCustomTransport * transport);
size_t transport_serial_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t transport_serial_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

#ifdef __cplusplus
}
#endif

#endif //_MICROROS_TRANSPORTS_H_
