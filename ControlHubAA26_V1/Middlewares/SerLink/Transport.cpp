#include "Transport.hpp"
#include "Writer.hpp"
#include "Reader.hpp"

using namespace SerLink;

Transport::Transport(Writer* writer, Reader *reader)
: writer(writer), reader(reader)
{
}

void Transport::init(QueueHandle_t queue, onReceiveCallback receiveCallback,
  onReceiveCallback ackCallback)
{
  this->queue = queue;
  this->receiveCallback = receiveCallback;
  this->ackCallback = ackCallback;
}

void Transport::run()
{
  FrameMsg frameMsg;

  if(xQueueReceive(this->queue, &frameMsg, portMAX_DELAY) == pdTRUE)
  {
    if(frameMsg.type == FrameMsg::TYPE_RX)
    {
      // find the acquired socket this frame's protocol belongs to, if any
      for(uint8_t i = 0; i < SERLINK_CONFIG__MAX_SOCKETS; i++)
      {
        if(this->sockets[i].isAcquired() && this->sockets[i].matchesProtocol(frameMsg.frame.protocol))
        {
          // The corresp[onding socket has been found - deliver the received data to it.
          this->sockets[i].deliverReceivedData(frameMsg.frame.data, frameMsg.frame.dataLen);
          break;
        }
      }

      // call the receiveCallback if it exists (for app-level logging, etc.)
      if(this->receiveCallback != nullptr)
      {
        this->receiveCallback(frameMsg.frame.data, frameMsg.frame.dataLen);   
      }
    }
    else if(frameMsg.type == FrameMsg::TYPE_TX)
    {
      // send the frame via the writer
      this->writer->sendFrame(&frameMsg.frame);
    }
    // TYPE_ACK
    else if(frameMsg.type == FrameMsg::TYPE_ACK)
    {
      // Ack packets can (typically don't) contain data - via the piggyback mechanism.
      // If this is the case then the data is delivered to the socket that matches the protocol of the ack frame.
      if(frameMsg.frame.dataLen <= Frame::MAX_DATALEN)
      {
        for(uint8_t i = 0; i < SERLINK_CONFIG__MAX_SOCKETS; i++)
        {
          if(this->sockets[i].isAcquired() && this->sockets[i].matchesProtocol(frameMsg.frame.protocol))
          {
            // The corresp[onding socket has been found - deliver the received data to it.
            this->sockets[i].deliverReceivedData(frameMsg.frame.data, frameMsg.frame.dataLen);
            break;
          }
        }
      }

      // call the ackCallback if it exists (for app-level logging, etc.)
      if(this->ackCallback != nullptr)
      {
        this->ackCallback(frameMsg.frame.data, frameMsg.frame.dataLen);   
      }
    }
  }
}

bool Transport::sendFrame(Frame* frame)
{
  FrameMsg frameMsg;
  frameMsg.type = FrameMsg::TYPE_TX;
  frameMsg.frame = *frame;

  return (xQueueSend(this->queue, &frameMsg, 0) == pdTRUE);
}

Socket* Transport::acquireSocket(char* protocol, onReceiveCallback callback, readHandler instantHandler)
{
  if(instantHandler != nullptr)
  {
    this->reader->registerInstantCallback(protocol, instantHandler);
  }

  for(uint8_t i = 0; i < SERLINK_CONFIG__MAX_SOCKETS; i++)
  {
    if(!this->sockets[i].isAcquired())
    {
      this->sockets[i].init(protocol, this, callback);
      return &this->sockets[i];
    }
  }

  return nullptr; // no free socket slots
}
