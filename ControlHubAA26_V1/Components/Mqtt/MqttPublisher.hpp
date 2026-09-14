/*
 * MqttPublisher.hpp
 *
 * Minimal MQTT publisher on top of lwIP's MQTT client (lwip/apps/mqtt.h).
 * Publish only: QoS 0, no retain, no subscriptions.
 *
 * lwIP's MQTT client is built on the raw API, which is not thread-safe.
 * Every call into it here is made holding the tcpip core lock
 * (LOCK_TCPIP_CORE), so the methods can be called from any task - but only
 * once MX_LWIP_Init() has run, since that is what creates the lock. The
 * connection callback runs in the tcpip thread.
 */

#ifndef MQTT_PUBLISHER_HPP_
#define MQTT_PUBLISHER_HPP_

#include <stdint.h>
#include "lwip/apps/mqtt.h"

class MqttPublisher
{
  public:
    // brokerIp is dotted decimal, e.g. "192.168.0.196". clientId must be
    // unique on the broker: a second connection with the same id kicks the
    // first off. Both strings must outlive the object.
    MqttPublisher(const char* brokerIp, uint16_t brokerPort, const char* clientId);

    // Starts a connection attempt and returns straight away; the outcome
    // arrives later, so poll isConnected(). Returns false if no attempt was
    // started: bad broker address, out of memory, or an attempt already in
    // progress.
    bool connect();

    bool isConnected();

    // Queues payload on topic at QoS 0. Returns false if not connected, or
    // if lwIP's output buffer (MQTT_OUTPUT_RINGBUF_SIZE, 256 bytes, which
    // must hold topic + payload) is full.
    bool publish(const char* topic, const char* payload, uint16_t payloadLen);

    // Last status reported by lwIP's connection callback - useful in the
    // debugger when isConnected() stays false.
    mqtt_connection_status_t getStatus() const { return status; }

  private:
    static void connectionCallback(mqtt_client_t* client, void* arg,
                                   mqtt_connection_status_t status);

    const char* brokerIp;
    uint16_t brokerPort;
    mqtt_client_t* client;
    struct mqtt_connect_client_info_t clientInfo;
    volatile mqtt_connection_status_t status;   // written by the tcpip thread
};

#endif /* MQTT_PUBLISHER_HPP_ */
