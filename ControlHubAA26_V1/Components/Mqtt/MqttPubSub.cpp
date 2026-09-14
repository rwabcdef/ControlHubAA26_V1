/*
 * MqttPubSub.cpp
 */

#include <string.h>
#include "MqttPubSub.hpp"
#include "lwip/ip_addr.h"
#include "lwip/tcpip.h"

// The broker drops the connection after 1.5x this with no traffic. lwIP's
// cyclic timer sends PINGREQ when needed, so it never gets there while the
// link is up.
#define MQTT_KEEP_ALIVE_S 60

MqttPubSub::MqttPubSub(const char* brokerIp, uint16_t brokerPort, const char* clientId)
  : rxQueue(nullptr),
    brokerIp(brokerIp),
    brokerPort(brokerPort),
    client(nullptr),
    clientInfo(),
    subscriptions(),
    numSubscriptions(0),
    rxMsg(),
    status(MQTT_CONNECT_DISCONNECTED),
    droppedCount(0),
    subscribeFailCount(0)
{
  clientInfo.client_id  = clientId;
  clientInfo.keep_alive = MQTT_KEEP_ALIVE_S;
  // user, pass and will stay NULL: none used
}

void MqttPubSub::init()
{
  rxQueue = xQueueCreateStatic(MQTT_PUBSUB__RX_QUEUE_LENGTH, sizeof(Message),
                               rxQueueStorageArea, &rxStaticQueue);
}

bool MqttPubSub::connect()
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
     life of the program and reused on every reconnect. */
  if(client == nullptr)
  {
    client = mqtt_client_new();
  }
  if(client != nullptr)
  {
    // ERR_ISCONN while an earlier attempt is still in progress
    err = mqtt_client_connect(client, &brokerAddr, brokerPort,
                              connectionCallback, this, &clientInfo);

    /* After the connect, not before: mqtt_client_connect() zeroes the whole
       client, callbacks included. Nothing can arrive in between, since the
       tcpip thread is locked out until UNLOCK below. And never skip it -
       lwIP calls the data callback without checking it for NULL. */
    if(err == ERR_OK)
    {
      mqtt_set_inpub_callback(client, incomingPublishCallback, incomingDataCallback, this);
    }
  }
  UNLOCK_TCPIP_CORE();

  return err == ERR_OK;
}

bool MqttPubSub::isConnected()
{
  bool connected;

  LOCK_TCPIP_CORE();
  connected = (client != nullptr) && mqtt_client_is_connected(client);
  UNLOCK_TCPIP_CORE();

  return connected;
}

bool MqttPubSub::publish(const char* topic, const char* payload, uint16_t payloadLen,
                         uint8_t qos, bool retain)
{
  err_t err = ERR_CONN;

  LOCK_TCPIP_CORE();
  if(client != nullptr)
  {
    err = mqtt_publish(client, topic, payload, payloadLen,
                       (qos > 2) ? 2 : qos, retain ? 1 : 0, nullptr, nullptr);
  }
  UNLOCK_TCPIP_CORE();

  return err == ERR_OK;
}

bool MqttPubSub::subscribe(const char* topic, uint8_t qos)
{
  bool ok = false;

  if(qos > 2)
  {
    qos = 2;   // lwIP asserts on anything higher
  }

  LOCK_TCPIP_CORE();
  if(numSubscriptions < MAX_SUBSCRIPTIONS)
  {
    subscriptions[numSubscriptions].topic = topic;
    subscriptions[numSubscriptions].qos   = qos;
    numSubscriptions++;
    ok = true;

    /* Only if the broker has already accepted the connection. Before that -
       including while connecting - connectionCallback() sends it along with
       the rest of the list, so it never goes out twice. */
    if((client != nullptr) && mqtt_client_is_connected(client))
    {
      ok = (mqtt_subscribe(client, topic, qos, subscribeCallback, this) == ERR_OK);
    }
  }
  UNLOCK_TCPIP_CORE();

  return ok;
}

bool MqttPubSub::receive(Message& msg, uint32_t timeoutMs)
{
  TickType_t ticks = (timeoutMs == WAIT_FOREVER) ? portMAX_DELAY : pdMS_TO_TICKS(timeoutMs);

  return xQueueReceive(rxQueue, &msg, ticks) == pdTRUE;
}

//--------------------------------------------------------------
// lwIP callbacks. All run in the tcpip thread, core lock held.

// Called with the CONNACK result, and again whenever the connection later
// drops (MQTT_CONNECT_DISCONNECTED / MQTT_CONNECT_TIMEOUT).
void MqttPubSub::connectionCallback(mqtt_client_t* client, void* arg,
                                    mqtt_connection_status_t status)
{
  MqttPubSub* self = static_cast<MqttPubSub*>(arg);

  self->status = status;

  if(status == MQTT_CONNECT_ACCEPTED)
  {
    // Clean session: the broker holds no subscriptions for us, so send them all
    for(uint8_t i = 0; i < self->numSubscriptions; i++)
    {
      if(mqtt_subscribe(client, self->subscriptions[i].topic, self->subscriptions[i].qos,
                        subscribeCallback, self) != ERR_OK)
      {
        self->subscribeFailCount++;
      }
    }
  }
}

// Start of a message: the topic, and the total payload length to follow.
// The topic string is only valid for the duration of the call.
void MqttPubSub::incomingPublishCallback(void* arg, const char* topic, u32_t totLen)
{
  Message& msg = static_cast<MqttPubSub*>(arg)->rxMsg;
  size_t topicLen = strlen(topic);

  msg.truncated = (topicLen >= MAX_TOPIC_LEN) || (totLen > MAX_PAYLOAD_LEN);

  if(topicLen >= MAX_TOPIC_LEN)
  {
    topicLen = MAX_TOPIC_LEN - 1;
  }
  memcpy(msg.topic, topic, topicLen);
  msg.topic[topicLen] = '\0';

  msg.payloadLen = 0;
}

// One payload fragment. Always called at least once per message, with
// MQTT_DATA_FLAG_LAST on the final fragment - even for an empty payload.
void MqttPubSub::incomingDataCallback(void* arg, const u8_t* data, u16_t len, u8_t flags)
{
  MqttPubSub* self = static_cast<MqttPubSub*>(arg);
  Message&    msg  = self->rxMsg;

  uint16_t space   = MAX_PAYLOAD_LEN - msg.payloadLen;
  uint16_t copyLen = (len < space) ? len : space;   // the rest was flagged truncated

  if(copyLen > 0)
  {
    memcpy(msg.payload + msg.payloadLen, data, copyLen);
    msg.payloadLen += copyLen;
  }

  if(flags & MQTT_DATA_FLAG_LAST)
  {
    msg.payload[msg.payloadLen] = '\0';

    // Never block the tcpip thread: drop instead
    if(xQueueSendToBack(self->rxQueue, &msg, 0) != pdTRUE)
    {
      self->droppedCount++;
    }
  }
}

// SUBACK received (ERR_OK, or ERR_ABRT if the broker refused the topic), or
// no SUBACK within MQTT_REQ_TIMEOUT (ERR_TIMEOUT).
void MqttPubSub::subscribeCallback(void* arg, err_t err)
{
  if(err != ERR_OK)
  {
    static_cast<MqttPubSub*>(arg)->subscribeFailCount++;
  }
}
