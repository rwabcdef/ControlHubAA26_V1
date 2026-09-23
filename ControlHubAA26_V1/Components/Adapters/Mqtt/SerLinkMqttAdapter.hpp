/*
 * SerLinkMqttAdapter.hpp
 *
 * SerLink over MQTT. Carries serialised SerLink frames in both directions
 * on a topic pair, so a Writer/Reader pair can use a broker in place of a
 * uart or the radio:
 *
 *   lwIP tcpip thread --> MqttPubSub::rxQueue --> run() --> rxDataQueue --> Reader
 *   Writer ------------------------------------- write() ----------------> broker
 *   Reader (acks) ------------------------------ write() ----------------> broker
 *
 * Received frames are published as the same UartMessage_t that uart2Queue
 * and Radio::rxDataQueue carry, so a Reader consumes them unchanged:
 *
 *   reader2.init(mqtt2.rxDataQueue, &writer2, transport2.queue);
 *   writer2.init([](char* b) -> uint8_t { return mqtt2.write(b); });
 *
 * Why the transmit side does not go through run()
 * -----------------------------------------------
 * Radio funnels both directions through its own task because the nRF24L01
 * is half-duplex and not thread-safe - one owner, everything else posts to
 * a queue. MQTT has no such constraint: MqttPubSub::publish() takes the
 * tcpip core lock itself and is callable from any task. So write()
 * publishes directly, from whichever task called it, and this class needs
 * no event queue and no TX message type. run() owns only the receive path
 * and the connection.
 *
 * Note that both a Writer (frames) and a Reader (acks) will call write()
 * from their own tasks, concurrently. That is safe, but they share lwIP's
 * output ring buffer (MQTT_OUTPUT_RINGBUF_SIZE, 256 bytes, which must hold
 * topic + payload), so a burst can make publish() refuse. write() reports
 * that - see the warning on the return value below.
 *
 * Why this class owns its MqttPubSub outright
 * -------------------------------------------
 * MqttPubSub has ONE rxQueue for all of its subscriptions, and receive()
 * is a plain queue read with no topic filter. Two tasks calling receive()
 * on the same client would steal each other's messages, whatever topic
 * those messages arrived on. So the client handed to this adapter must be
 * a dedicated instance that nothing else ever receives from - init() calls
 * its init(), run() is its only reader, and its client id must differ from
 * every other client on the broker (a second connection with the same id
 * kicks the first off).
 *
 * Publishing to it from elsewhere would in fact be harmless, since that
 * never touches rxQueue - but don't. Keeping one owner is what makes the
 * ownership rule easy to check.
 *
 * Wire format
 * -----------
 * The payload is exactly what Frame::toString() produced, trailing '\n'
 * included. Frame::fromString() parses by fixed offsets and never looks
 * for the terminator, so the newline costs nothing and buys a frame on
 * MQTT that is byte-identical to one on the uart - the strings documented
 * at the top of main_tasks.cpp can be pasted straight into mosquitto_pub,
 * and mosquitto_sub reads the link as if it were a terminal.
 *
 * QoS 0, retain false, deliberately. SerLink's Writer matches an incoming
 * ack to the frame in flight by PROTOCOL only, not roll code, so the
 * duplicate delivery QoS 1 explicitly permits could let a redelivered ack
 * satisfy the wrong frame. QoS 0 plus SerLink's own 'T'/ack retry is one
 * reliability mechanism rather than two fighting. A retained frame would
 * be redelivered on every reconnect carrying a stale roll code.
 *
 * Bring-up order
 * --------------
 * initTasks() runs before MX_LWIP_Init(), and every MqttPubSub method bar
 * receive() takes the tcpip core lock that MX_LWIP_Init() creates. So the
 * split is the same one Adc uses:
 *
 *   init()   queues only. Safe before the scheduler, touches no lwIP.
 *   start()  subscribe + first connect. After the netif is up.
 *   run()    the pump. Loop on it from the owning task.
 *
 * The netif wait itself is left to the owning task, as startMqttTask does
 * it, so this class needs no knowledge of which netif the application uses.
 *
 * Threading
 * ---------
 * run() belongs to one task and is the only caller of receive(). write()
 * is callable from any task. The counters are diagnostics only: write()
 * can be re-entered from two tasks at once, so txDropped and txFrames are
 * plain increments that may under-count under contention. They are there
 * to tell you a link is unhealthy, not to be audited.
 */

#ifndef SERLINK_MQTT_ADAPTER_HPP_
#define SERLINK_MQTT_ADAPTER_HPP_

#include <stdint.h>
#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "MqttPubSub.hpp"

// Frames buffered between the broker and the Reader. The Reader blocks on
// this queue and does nothing else, so it drains fast; this only has to
// absorb a burst arriving while the Reader is in its ACKDELAY wait.
#define SERLINK_MQTT__RXDATA_QUEUE_LENGTH 5

// How long run() blocks in receive() before falling through to the
// connection housekeeping. It is a timeout, not a poll interval: a busy
// link never reaches it. It has to be finite, though - see run().
#define SERLINK_MQTT__POLL_MS 500

class SerLinkMqttAdapter
{
  public:
    // Frames received from the broker, one per item, as the same
    // UartMessage_t that uart2Queue carries. Valid after init().
    QueueHandle_t rxDataQueue;

    // mqtt must be an instance used by nothing else - see the ownership
    // note above. topicUp is what this board publishes on, topicDown what
    // it subscribes to; they must not be the same string, or the board
    // receives its own frames back. Both must outlive this object.
    //
    // Stores its arguments and nothing more, so a file scope instance
    // constructed before HAL_Init() is fine.
    SerLinkMqttAdapter(MqttPubSub* mqtt, const char* topicUp, const char* topicDown);

    // Creates rxDataQueue and the client's rxQueue. Safe before the
    // scheduler starts, and touches no lwIP. Must precede start().
    bool init();

    // Records the subscription and starts the first connection attempt.
    // Call once, from the owning task, after the netif is up - before then
    // the tcpip core lock this reaches through does not exist yet.
    bool start();

    // Services the link: one received frame, or the connection. Call
    // repeatedly from the one task that owns this adapter. Blocks
    // internally, so the loop needs nothing else.
    void run();

    // Publishes buffer, a NUL-terminated serialised frame, on topicUp.
    // Callable from any task. Returns 0 if the broker accepted it, 1 if
    // not - which is SerLink's WriteDataFunc convention.
    //
    // A non-zero return means the frame did NOT go out. Writer::idle()
    // currently discards this, so a refused write costs a full
    // WRITER_ACK_TIMEOUT_MS with no other symptom; getTxDropped() is what
    // tells you that is what happened.
    uint8_t write(char* buffer);

    // True once the broker has accepted the connection. Not const, because
    // MqttPubSub::isConnected() takes the tcpip core lock to ask.
    bool isConnected();

    // Diagnostics. In a healthy link every one of these but the frame
    // counts stays at zero.
    uint32_t getRxFrames() const;        // posted to rxDataQueue
    uint32_t getTxFrames() const;        // accepted by publish()
    uint32_t getRxDropped() const;       // rxDataQueue full - Reader too slow
    uint32_t getTxDropped() const;       // not connected, or lwIP refused
    uint32_t getTruncated() const;       // payload longer than UartMessage_t
    uint32_t getOtherTopic() const;      // arrived on a topic we did not expect

  private:
    MqttPubSub* mqtt;
    const char* topicUp;
    const char* topicDown;

    bool started;

    // Members rather than locals in run(): a MqttPubSub::Message is around
    // 200 bytes and a UartMessage_t 132, which is most of a small task's
    // stack if both sit on it. Radio keeps eventMsg the same way.
    MqttPubSub::Message rxMessage;
    UartMessage_t       rxMsg;

    volatile uint32_t rxFrames;
    volatile uint32_t txFrames;
    volatile uint32_t rxDropped;
    volatile uint32_t txDropped;
    volatile uint32_t truncated;
    volatile uint32_t otherTopic;

    StaticQueue_t rxDataStaticQueue;
    uint8_t rxDataQueueStorageArea[SERLINK_MQTT__RXDATA_QUEUE_LENGTH * sizeof(UartMessage_t)];

    // Converts a received MQTT message into rxMsg and posts it. Returns
    // false if it was not for us, or could not be queued.
    bool deliver();
};

#endif /* SERLINK_MQTT_ADAPTER_HPP_ */
