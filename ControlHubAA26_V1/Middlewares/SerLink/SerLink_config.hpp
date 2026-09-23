
#ifndef SERLINK_CONFIG_HPP_
#define SERLINK_CONFIG_HPP_

//#define SERLINK_CONFIG__MAX_DATA_LEN 50

// Sockets each Transport can hold. transport0 currently uses seven:
// RAD00, LED01, MOTOR, DBG00, MQTT0, ADC00 and TACHO.
//
// Each slot costs sizeof(Socket) in every Transport object, and there are
// two of them. A Socket is dominated by its two static queues -
// SERLINK_CONFIG__SOCKET_*_MAX_MSGS messages of sizeof(SocketMsg), which
// is Frame::MAX_DATALEN plus a header - so a slot is roughly 600 bytes,
// i.e. about 1.2 kB of BSS per slot across both Transports. Headroom
// rather than a tight fit, but not free: keep an eye on the bss figure in
// the arm-none-eabi-size output if this is raised again.
#define SERLINK_CONFIG__MAX_SOCKETS 10

#define SERLINK_CONFIG__SOCKET_TX_MAX_MSGS 3
#define SERLINK_CONFIG__SOCKET_RX_MAX_MSGS 3


#endif /* SERLINK_CONFIG_HPP_ */