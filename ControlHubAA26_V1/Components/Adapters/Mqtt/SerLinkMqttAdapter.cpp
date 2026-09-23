/*
 * SerLinkMqttAdapter.cpp
 *
 * See SerLinkMqttAdapter.hpp for the link model, the ownership rule on the
 * MqttPubSub instance, and the bring-up order.
 */

#include "SerLinkMqttAdapter.hpp"
#include "uart2.h"      // UART_MSG_TYPE__FRAME_RX
#include <string.h>

SerLinkMqttAdapter::SerLinkMqttAdapter(MqttPubSub* mqtt, const char* topicUp,
                                       const char* topicDown)
  : rxDataQueue(nullptr),
    mqtt(mqtt),
    topicUp(topicUp),
    topicDown(topicDown),
    started(false),
    rxMessage(),
    rxMsg(),
    rxFrames(0U),
    txFrames(0U),
    rxDropped(0U),
    txDropped(0U),
    truncated(0U),
    otherTopic(0U),
    rxDataStaticQueue(),
    rxDataQueueStorageArea()
{
  /* Stores its arguments and nothing else - no HAL, no RTOS, no lwIP - so
     a file scope instance is safe. This runs from __libc_init_array,
     before HAL_Init() and long before MX_LWIP_Init(). */
}

bool SerLinkMqttAdapter::init()
{
  if((mqtt == nullptr) || (topicUp == nullptr) || (topicDown == nullptr))
  {
    return false;
  }

  /* Publishing on the topic we subscribe to would feed every frame - and
     every ack - straight back into our own Reader. MQTT brokers deliver
     to every subscriber, the publisher included. */
  if(0 == strcmp(topicUp, topicDown))
  {
    return false;
  }

  rxDataQueue = xQueueCreateStatic(SERLINK_MQTT__RXDATA_QUEUE_LENGTH, sizeof(UartMessage_t),
    rxDataQueueStorageArea, &rxDataStaticQueue);
  if(rxDataQueue == nullptr)
  {
    return false;
  }

  /* This adapter owns the client outright, so it is what brings it up.
     init() only creates the client's rxQueue - no lwIP is touched, which
     is what makes this callable from initTasks(). */
  mqtt->init();

  return true;
}

bool SerLinkMqttAdapter::start()
{
  if(rxDataQueue == nullptr)
  {
    return false;   // init() was not called, or it failed
  }

  /* Recorded now and re-sent on every accepted connection: lwIP always
     connects with a clean session, so the broker forgets its
     subscriptions whenever the link drops.

     We are not connected yet, so a false return here can only mean the
     subscription list is full - a real error, not a transient one. */
  if(!mqtt->subscribe(topicDown))
  {
    return false;
  }

  started = true;

  /* First attempt. It returns false if none could be started; run()
     retries either way, so the return is not worth propagating. */
  mqtt->connect();

  return true;
}

void SerLinkMqttAdapter::run()
{
  /* The timeout is what lets one task do both jobs. Blocking forever
     would be the obvious thing, but then nothing would ever poll the
     connection - and a link that has dropped delivers no messages to wake
     us, which is precisely when the reconnect below is needed. On a busy
     link this branch is taken every time and the housekeeping never
     runs, which is correct: traffic is proof the connection is alive. */
  if(mqtt->receive(rxMessage, SERLINK_MQTT__POLL_MS))
  {
    deliver();
    return;
  }

  if(started && !mqtt->isConnected())
  {
    /* Returns false, harmlessly, while an earlier attempt is still
       pending - the same pattern startMqttTask uses. */
    mqtt->connect();
  }
}

bool SerLinkMqttAdapter::deliver()
{
  /* MqttPubSub has one rxQueue for all of its subscriptions and no topic
     filter on receive(), so anything else subscribed on this client would
     arrive here too. Nothing should be - see the ownership note in the
     header - and a non-zero otherTopic says something is. */
  if(0 != strncmp(rxMessage.topic, topicDown, MqttPubSub::MAX_TOPIC_LEN))
  {
    otherTopic++;
    return false;
  }

  uint16_t len = rxMessage.payloadLen;

  /* Message::payload is MAX_PAYLOAD_LEN + 1 bytes; UartMessage_t::data is
     UART2__BUFFER_LEN, one byte shorter. A well formed frame is at most
     Frame::MAX_FRAME_LEN, so this cannot bite on real traffic - which is
     exactly why it would go unnoticed the first time it did. */
  if(len >= sizeof(this->rxMsg.data))
  {
    len = (uint16_t)(sizeof(this->rxMsg.data) - 1U);
    truncated++;
  }

  memcpy(this->rxMsg.data, rxMessage.payload, len);
  this->rxMsg.data[len] = '\0';
  this->rxMsg.len = len;

  /* Reader::checkUartFrameRx() tests this against UART_MSG_TYPE__FRAME_RX
     and discards anything else without a word. Leave it zeroed and you
     get a link that looks perfectly healthy and never delivers a frame. */
  this->rxMsg.type = UART_MSG_TYPE__FRAME_RX;

  if(xQueueSend(this->rxDataQueue, &this->rxMsg, 0) != pdTRUE)
  {
    /* The Reader is not keeping up. Dropping the newest frame loses a
       command; SerLink's ack timeout is what reports it to the sender. */
    rxDropped++;
    return false;
  }

  rxFrames++;
  return true;
}

uint8_t SerLinkMqttAdapter::write(char* buffer)
{
  if((buffer == nullptr) || !mqtt->isConnected())
  {
    txDropped++;
    return 1U;
  }

  /* Frame::toString() clears eofIndex + 2 bytes before writing and puts
     the '\n' at eofIndex, so the frame is NUL terminated just past the
     newline: strlen() gives its full length, terminator included. */
  size_t len = strlen(buffer);

  if((len == 0U) || (len > MqttPubSub::MAX_PAYLOAD_LEN))
  {
    txDropped++;
    return 1U;
  }

  /* QoS 0 and retain false - see the wire format note in the header.
     publish() refuses if lwIP's 256 byte output ring buffer is full,
     which a Writer and a Reader publishing at once can manage. */
  if(!mqtt->publish(topicUp, buffer, (uint16_t)len, 0U, false))
  {
    txDropped++;
    return 1U;
  }

  txFrames++;
  return 0U;
}

bool SerLinkMqttAdapter::isConnected()
{
  return mqtt->isConnected();
}

uint32_t SerLinkMqttAdapter::getRxFrames() const   { return rxFrames; }
uint32_t SerLinkMqttAdapter::getTxFrames() const   { return txFrames; }
uint32_t SerLinkMqttAdapter::getRxDropped() const  { return rxDropped; }
uint32_t SerLinkMqttAdapter::getTxDropped() const  { return txDropped; }
uint32_t SerLinkMqttAdapter::getTruncated() const  { return truncated; }
uint32_t SerLinkMqttAdapter::getOtherTopic() const { return otherTopic; }
