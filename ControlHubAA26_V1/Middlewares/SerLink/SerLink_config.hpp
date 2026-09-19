
#ifndef SERLINK_CONFIG_HPP_
#define SERLINK_CONFIG_HPP_

//#define SERLINK_CONFIG__MAX_DATA_LEN 50

// Sockets each Transport can hold. transport0 currently uses all six:
// RAD00, LED01, MOTOR, DBG00, MQTT0 and ADC00. Raising this costs
// sizeof(Socket) per Transport object - two of them - so it is sized to
// what is actually acquired rather than left generous.
#define SERLINK_CONFIG__MAX_SOCKETS 6

#define SERLINK_CONFIG__SOCKET_TX_MAX_MSGS 3
#define SERLINK_CONFIG__SOCKET_RX_MAX_MSGS 3


#endif /* SERLINK_CONFIG_HPP_ */