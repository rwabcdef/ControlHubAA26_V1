
#ifndef SERLINK_CONFIG_HPP_
#define SERLINK_CONFIG_HPP_

//#define SERLINK_CONFIG__MAX_DATA_LEN 50

// Sockets each Transport can hold. Currently in use (main_tasks.cpp):
//   transport0 (uart2)  5: MOTOR, CTRL0, ADC00, SDC00, DBG00
//   transport1 (radio)  5: LED01, BTN01, POT01, HBT01, DBG00
//   transport2 (MQTT)   3: DBG00, CTRL0, LIFT0
//
// Each slot costs sizeof(Socket) in every Transport object, and there are
// three of them. A Socket is dominated by its two static queues -
// SERLINK_CONFIG__SOCKET_*_MAX_MSGS messages of sizeof(SocketMsg), which
// is Frame::MAX_DATALEN plus a header - so a slot is roughly 600 bytes,
// i.e. about 1.8 kB of BSS per slot across the three Transports. Headroom
// rather than a tight fit, but not free: keep an eye on the bss figure in
// the arm-none-eabi-size output if this is raised again.
#define SERLINK_CONFIG__MAX_SOCKETS 10

#define SERLINK_CONFIG__SOCKET_TX_MAX_MSGS 3
#define SERLINK_CONFIG__SOCKET_RX_MAX_MSGS 3


#endif /* SERLINK_CONFIG_HPP_ */