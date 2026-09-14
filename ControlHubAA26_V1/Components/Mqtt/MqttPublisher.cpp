/*
 * MqttPublisher.cpp
 */

#include "MqttPublisher.hpp"
#include "lwip/ip_addr.h"
#include "lwip/tcpip.h"

// The broker drops the connection after 1.5x this with no traffic. lwIP's
// cyclic timer sends PINGREQ when needed, so it never gets there while the
// link is up.
#define MQTT_KEEP_ALIVE_S 60

MqttPublisher::MqttPublisher(const char* brokerIp, uint16_t brokerPort, const char* clientId)
  : brokerIp(brokerIp),
    brokerPort(brokerPort),
    client(nullptr),
    clientInfo(),
    status(MQTT_CONNECT_DISCONNECTED)
{
  clientInfo.client_id  = clientId;
  clientInfo.keep_alive = MQTT_KEEP_ALIVE_S;
  // user, pass and will stay NULL: none used
}

bool MqttPublisher::connect()
{
  ip_addr_t brokerAddr;
  if(!ipaddr_aton(brokerIp, &brokerAddr))
  {
    return false;
  }

  err_t err = ERR_MEM;

  LOCK_TCPIP_CORE();
  /* Allocated here rather than in the constructor: global objects are
     constructed before main(), long before lwIP's heap exists. Kept for the
     life of the program and reused on every reconnect - lwIP resets it once
     the previous connection has closed. */
  if(client == nullptr)
  {
    client = mqtt_client_new();
  }
  if(client != nullptr)
  {
    // ERR_ISCONN while an earlier attempt is still in progress
    err = mqtt_client_connect(client, &brokerAddr, brokerPort,
                              connectionCallback, this, &clientInfo);
  }
  UNLOCK_TCPIP_CORE();

  return err == ERR_OK;
}

bool MqttPublisher::isConnected()
{
  bool connected;

  LOCK_TCPIP_CORE();
  connected = (client != nullptr) && mqtt_client_is_connected(client);
  UNLOCK_TCPIP_CORE();

  return connected;
}

bool MqttPublisher::publish(const char* topic, const char* payload, uint16_t payloadLen)
{
  err_t err = ERR_CONN;

  LOCK_TCPIP_CORE();
  if(client != nullptr)
  {
    // qos 0, retain 0: no request is tracked, so no completion callback
    err = mqtt_publish(client, topic, payload, payloadLen, 0, 0, nullptr, nullptr);
  }
  UNLOCK_TCPIP_CORE();

  return err == ERR_OK;
}

// tcpip thread. Called once with the CONNACK result, and again whenever the
// connection later drops (MQTT_CONNECT_DISCONNECTED / MQTT_CONNECT_TIMEOUT).
void MqttPublisher::connectionCallback(mqtt_client_t* client, void* arg,
                                       mqtt_connection_status_t status)
{
  (void)client;
  static_cast<MqttPublisher*>(arg)->status = status;
}
