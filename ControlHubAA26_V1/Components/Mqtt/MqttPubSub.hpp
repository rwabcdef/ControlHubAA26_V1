/*
 * MqttPubSub.hpp
 *
 * MQTT client on top of lwIP's MQTT client (lwip/apps/mqtt.h): publish, and
 * subscribe with received messages delivered through a queue.
 *
 *   task --publish()--> lwIP ---------------------------------> broker
 *   broker --> lwIP (tcpip thread) --> callbacks --> rxQueue --> receive()
 *
 * Threading
 * ---------
 * lwIP's MQTT client is built on the raw API, which is not thread-safe.
 * Every call into it from here is made holding the tcpip core lock
 * (LOCK_TCPIP_CORE), so connect(), isConnected(), publish() and subscribe()
 * can be called from any task - but only once MX_LWIP_Init() has run, since
 * that is what creates the lock. lwIP's callbacks run in the tcpip thread
 * with the lock already held; they only copy into rxQueue and never block,
 * so a slow consumer cannot stall the stack.
 *
 * Subscriptions
 * -------------
 * lwIP always connects with a clean session, so the broker forgets every
 * subscription whenever the connection drops. subscribe() therefore records
 * the topic, and the whole list is resubscribed each time a connection is
 * accepted: call it once per topic, before or after connecting.
 *
 * Received messages
 * -----------------
 * lwIP hands a message over in fragments; they are reassembled and posted to
 * rxQueue as one Message. A topic or payload too long for Message is cut
 * short and flagged truncated. If rxQueue is full the message is dropped and
 * counted. Separately, the whole incoming topic must fit lwIP's own receive
 * buffer (MQTT_VAR_HEADER_BUFFER_LEN, 128 bytes less a few of header) - a
 * longer one makes lwIP close the connection.
 */

#ifndef MQTT_PUBSUB_HPP_
#define MQTT_PUBSUB_HPP_

#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "lwip/apps/mqtt.h"

#define MQTT_PUBSUB__RX_QUEUE_LENGTH 4

class MqttPubSub
{
  public:
    // All subscriptions are sent together on connect, and each holds one of
    // lwIP's MQTT_REQ_MAX_IN_FLIGHT (4) request slots until its SUBACK - as
    // does any QoS 1/2 publish. Keep this at or below that.
    static const uint8_t  MAX_SUBSCRIPTIONS = 4;
    static const uint16_t MAX_TOPIC_LEN     = 64;    // including the NUL
    static const uint16_t MAX_PAYLOAD_LEN   = 128;   // excluding the NUL
    static const uint32_t WAIT_FOREVER      = 0xFFFFFFFF;

    struct Message
    {
      char     topic[MAX_TOPIC_LEN];           // NUL terminated
      char     payload[MAX_PAYLOAD_LEN + 1];   // NUL added for convenience; use
                                               // payloadLen if it may be binary
      uint16_t payloadLen;
      bool     truncated;                      // topic or payload was cut short
    };

    // Received messages, one per item, as Message. Valid after init().
    QueueHandle_t rxQueue;

    // brokerIp is dotted decimal, e.g. "192.168.0.196". clientId must be
    // unique on the broker: a second connection with the same id kicks the
    // first off. Both strings must outlive the object. Touches neither lwIP
    // nor the RTOS, so fine for a static instance.
    MqttPubSub(const char* brokerIp, uint16_t brokerPort, const char* clientId);

    // Creates rxQueue. Safe before the scheduler starts; must come before
    // connect().
    void init();

    // Starts a connection attempt and returns straight away; the outcome
    // arrives later, so poll isConnected(). Returns false if no attempt was
    // started: bad broker address, out of memory, or an attempt already in
    // progress.
    bool connect();

    bool isConnected();

    // Queues payload on topic. Returns false if not connected, if lwIP's
    // output buffer (MQTT_OUTPUT_RINGBUF_SIZE, 256 bytes, which must hold
    // topic + payload) is full, or for QoS 1/2 if no request slot is free.
    bool publish(const char* topic, const char* payload, uint16_t payloadLen,
                 uint8_t qos = 0, bool retain = false);

    // Adds topic - a filter, so + and # wildcards are allowed - to the
    // subscription list, and subscribes straight away if connected. topic
    // must outlive the object. Returns false if the list is full, or if
    // connected and lwIP could not queue the SUBSCRIBE; in that case it is
    // still retried on the next connect.
    bool subscribe(const char* topic, uint8_t qos = 0);

    // Waits up to timeoutMs, or WAIT_FOREVER, for a received message. Task
    // context only.
    bool receive(Message& msg, uint32_t timeoutMs = WAIT_FOREVER);

    // For the debugger when things go quiet.
    mqtt_connection_status_t getStatus() const { return status; }
    uint32_t getDroppedCount() const { return droppedCount; }             // rxQueue full
    uint32_t getSubscribeFailCount() const { return subscribeFailCount; } // not sent, refused or timed out

  private:
    struct Subscription
    {
      const char* topic;
      uint8_t     qos;
    };

    static void connectionCallback(mqtt_client_t* client, void* arg,
                                   mqtt_connection_status_t status);
    static void incomingPublishCallback(void* arg, const char* topic, u32_t totLen);
    static void incomingDataCallback(void* arg, const u8_t* data, u16_t len, u8_t flags);
    static void subscribeCallback(void* arg, err_t err);

    const char*    brokerIp;
    uint16_t       brokerPort;
    mqtt_client_t* client;
    struct mqtt_connect_client_info_t clientInfo;

    // Guarded by the tcpip core lock: written by subscribe(), read by
    // connectionCallback() in the tcpip thread.
    Subscription subscriptions[MAX_SUBSCRIPTIONS];
    uint8_t      numSubscriptions;

    Message rxMsg;   // being reassembled - tcpip thread only

    // Written by the tcpip thread
    volatile mqtt_connection_status_t status;
    volatile uint32_t droppedCount;
    volatile uint32_t subscribeFailCount;

    StaticQueue_t rxStaticQueue;
    uint8_t       rxQueueStorageArea[MQTT_PUBSUB__RX_QUEUE_LENGTH * sizeof(Message)];
};

#endif /* MQTT_PUBSUB_HPP_ */
