/*
 * HubApp.cpp
 *
 * See HubApp.hpp for the modes, the rules between them and the threading
 * contract.
 */

#include "HubApp.hpp"
#include <string.h>
#include "task.h"

// The low `width` digits of value, zero padded, no NUL - the same as
// main_tasks.cpp's helper, which is file static there.
static void writeDigits(uint32_t value, uint8_t width, char* dst)
{
  for(uint8_t i = width; i > 0U; i--)
  {
    dst[i - 1U] = (char)('0' + (value % 10U));
    value /= 10U;
  }
}

HubApp::HubApp(const HubAppConfig& config)
: config(config), valid(false), statusSocket(nullptr), ledSocket(nullptr),
  staticQueue(), queueStorage(), queue(nullptr),
  currentMode(mode::idle), runSource(source::none),
  selectedDir(ControllerDirection::forward), targetRpm(config.bootRpm),
  potRpm(0U), potSeen(false), remoteLastTick(0U), remoteSeen(false),
  statusPasses(0U), ledPasses(0U), ledRunState(), ledDirState('\0')
{
  /* Only stores the config: a file scope HubApp is constructed before
     main(), when neither the HAL nor the RTOS exists. */
}

bool HubApp::init(SerLink::Socket* statusSocket, SerLink::Socket* ledSocket)
{
  this->statusSocket = statusSocket;
  this->ledSocket = ledSocket;

  valid = (config.controller != nullptr) && (config.lift != nullptr) &&
          (config.getCurrentMa != nullptr) && (config.onLiftDone != nullptr) &&
          (config.periodMs > 0U) &&
          (config.statusPeriodMs >= config.periodMs) &&
          (config.ledRefreshMs >= config.periodMs);

  queue = xQueueCreateStatic(QUEUE_LENGTH, sizeof(AppCmd), queueStorage, &staticQueue);

  /* A target without enable(): nothing moves at boot, but a start - or a
     lift move, which reads the controller's demand - has a speed. */
  config.controller->setRequiredRpm(targetRpm);

  return valid && (queue != nullptr);
}

bool HubApp::post(const AppCmd& cmd)
{
  return (queue != nullptr) && (xQueueSend(queue, &cmd, 0U) == pdTRUE);
}

bool HubApp::setTargetRpm(uint16_t rpm)
{
  /* A lift move keeps the speed it started with. The mode is read from
     another task, so a set racing a lift start can still land - run()
     only applies the target in Control, so the move is unaffected and the
     new target waits for the next start. */
  if(currentMode == mode::lift)
  {
    return false;
  }
  targetRpm = rpm;
  return true;
}

uint16_t HubApp::getTargetRpm() const                { return targetRpm; }
HubApp::mode HubApp::getMode() const                 { return currentMode; }
HubApp::source HubApp::getSource() const             { return runSource; }
ControllerDirection HubApp::getSelectedDirection() const { return selectedDir; }

void HubApp::remoteAlive()
{
  remoteLastTick = xTaskGetTickCount();
  remoteSeen = true;
}

void HubApp::setPotPercent(uint8_t percent)
{
  if(percent > 100U) { percent = 100U; }
  potRpm = (uint16_t)(((uint32_t)percent * config.potRpmMax) / 100U);
  potSeen = true;
}

char HubApp::modeToChar(mode m)
{
  switch(m)
  {
    case mode::control: return 'C';
    case mode::lift:    return 'L';
    default:            return 'I';
  }
}

char HubApp::sourceToChar(source s)
{
  switch(s)
  {
    case source::pc:     return 'P';
    case source::remote: return 'R';
    default:             return '-';
  }
}

//--------------------------------------------------------------
void HubApp::run()
{
  if(!valid)
  {
    return;
  }

  /* Commands first, drained completely, so a start and a stop sent
     together are both seen, in order. */
  AppCmd cmd;
  while(xQueueReceive(queue, &cmd, 0U) == pdTRUE)
  {
    apply(cmd);
  }

  /* The remote has gone quiet. Only a run it started is stopped - see
     The remote in HubApp.hpp. The tick difference is unsigned, so a
     wrapped count is still right. */
  if((currentMode == mode::control) && (runSource == source::remote))
  {
    const TickType_t silence = xTaskGetTickCount() - remoteLastTick;
    if(!remoteSeen || (silence > pdMS_TO_TICKS(config.heartbeatTimeoutMs)))
    {
      stopAll();
    }
  }

  /* The speed, live in Control: a change of target, or a turn of the pot,
     reaches the run on the next controller pass. A lift move is left at
     the speed apply() started it with. */
  if(currentMode == mode::control)
  {
    config.controller->setRequiredRpm(runRpm());
  }

  /* Ends a move whose distance is up, then runs the controller - or holds
     it while the direction is idle. See Lift::run(). */
  config.lift->run();

  if((currentMode == mode::lift) &&
     (config.lift->getStatus() == Lift::status::idle))
  {
    /* Arrived, ground, or the controller's tacho timeout ended it short. */
    config.onLiftDone();
    enterIdle();
  }
  else if((currentMode == mode::control) && config.controller->isTachoFault())
  {
    /* The controller has already stopped the motor (0%, direction idle)
       and latched the fault, which CTRL0 BGF reads. */
    enterIdle();
  }

  if(currentMode != mode::idle)
  {
    if(++statusPasses >= (uint32_t)(config.statusPeriodMs / config.periodMs))
    {
      statusPasses = 0U;
      sendStatus();
    }
  }

  updateLeds(false);
}

void HubApp::apply(const AppCmd& cmd)
{
  switch(cmd.op)
  {
    case AppCmd::start:
      if(currentMode == mode::idle) { startControl(source::pc); }
      break;

    case AppCmd::stop:
      stopAll();
      break;

    case AppCmd::direction:
      if((currentMode == mode::idle) &&
         ((cmd.dir == ControllerDirection::forward) ||
          (cmd.dir == ControllerDirection::reverse)))
      {
        selectedDir = cmd.dir;
      }
      break;

    case AppCmd::liftStart:
    case AppCmd::liftGround:
    {
      if(currentMode != mode::idle)
      {
        break;
      }

      /* Lift::start() reads the controller's demand and refuses zero. */
      config.controller->setRequiredRpm(targetRpm);

      Lift* lift = config.lift;
      bool accepted = (cmd.op == AppCmd::liftStart)
                        ? lift->start(cmd.liftDir, cmd.distance)
                        : lift->toGroundLevel(cmd.distance);

      if(accepted && (lift->getStatus() == Lift::status::moving))
      {
        currentMode = mode::lift;
        runSource = source::pc;
        statusPasses = 0U;
        sendStatus();
      }
      else if(accepted)
      {
        /* A ground move with the lift already on the ground: it never
           moved, but the PC still waits for a done frame. */
        config.onLiftDone();
      }
      break;
    }

    case AppCmd::remoteStartStop:
      if(currentMode == mode::idle) { startControl(source::remote); }
      else                          { stopAll(); }
      break;

    case AppCmd::remoteDirection:
      if(currentMode == mode::idle)
      {
        selectedDir = (selectedDir == ControllerDirection::reverse)
                        ? ControllerDirection::forward
                        : ControllerDirection::reverse;
      }
      break;

    default:
      break;
  }
}

void HubApp::startControl(source from)
{
  runSource = from;   // before runRpm(), which depends on it
  config.controller->setRequiredRpm(runRpm());

  /* Direction, then enable - the order Lift::beginMove() uses, so the
     first controller pass has everything it needs. */
  config.controller->setDirection(selectedDir);
  config.controller->enable();

  currentMode = mode::control;
  statusPasses = 0U;
  sendStatus();
}

void HubApp::stopAll()
{
  if(currentMode == mode::lift)
  {
    /* Disables the controller and idles the direction - and the PC
       learns the move ended from the done frame, as for any other end. */
    config.lift->stop();
    config.onLiftDone();
  }
  else if(currentMode == mode::control)
  {
    /* What Lift::end() does: disable first, so no later controller pass
       can write the output, then idle, which stops the bridge. */
    config.controller->disable();
    config.controller->setDirection(ControllerDirection::idle);
  }

  if(currentMode != mode::idle)
  {
    enterIdle();
  }
}

void HubApp::enterIdle()
{
  currentMode = mode::idle;
  runSource = source::none;

  /* One last status frame, so the PC sees the run end rather than the
     frames just stopping. */
  sendStatus();
}

uint16_t HubApp::runRpm() const
{
  /* The pot only steers a run the remote started, and only once it has
     been heard from - until then the target stands. */
  return ((runSource == source::remote) && potSeen) ? potRpm : targetRpm;
}

void HubApp::writeStatus(char* dst) const
{
  /* The run's direction while running - which is the selected one, since
     direction changes are refused - and the selection while Idle. */
  ControllerDirection dir = config.controller->getDirection();
  if(dir == ControllerDirection::idle) { dir = selectedDir; }

  /* writeDigits() keeps only the low digits, so clamp to the field rather
     than report an unrelated number. The duty is at most 100. */
  uint32_t rpm = config.controller->getRpm();
  if(rpm > 9999U) { rpm = 9999U; }
  uint32_t currentMa = config.getCurrentMa();
  if(currentMa > 9999U) { currentMa = 9999U; }

  dst[0] = modeToChar(currentMode);
  dst[1] = (dir == ControllerDirection::reverse) ? 'R' : 'F';
  writeDigits(config.controller->getPwmPercent(), 3U, &dst[2]);
  dst[5] = '.';
  writeDigits(rpm, 4U, &dst[6]);
  dst[10] = '.';
  writeDigits(currentMa, 4U, &dst[11]);
}

void HubApp::sendStatus()
{
  if(statusSocket == nullptr)
  {
    return;
  }

  char data[STATUS_LEN];
  writeStatus(data);

  /* Non-blocking: queued for the writer, and dropped while MQTT is down. */
  statusSocket->sendData(data, STATUS_LEN, false);
}

void HubApp::updateLeds(bool force)
{
  if(ledSocket == nullptr)
  {
    return;
  }

  /* LED01 data is the remote's LedEvent format (Led.hpp in the Arduino
     repo): <id>1 on, <id>0 off, <id>F<nn><on><off><final> flash - nn 00
     is for ever, the periods are 250 ms ticks. 9 chars at most, the
     remote's frame data limit. */
  char run[LED_DATA_MAX + 1U];
  run[0] = config.ledRunId;
  switch(currentMode)
  {
    case mode::control:
      run[1] = '1'; run[2] = '\0';
      break;
    case mode::lift:
      memcpy(&run[1], "F0002020", 9U);   // slow flash, 500 ms on / 500 ms off
      break;
    default:
      run[1] = '0'; run[2] = '\0';
      break;
  }

  const char dirState = (selectedDir == ControllerDirection::reverse) ? '1' : '0';

  const bool refresh = (++ledPasses >= (uint32_t)(config.ledRefreshMs / config.periodMs));
  if(refresh)
  {
    ledPasses = 0U;
  }

  if(force || refresh || (strcmp(&run[1], ledRunState) != 0))
  {
    strcpy(ledRunState, &run[1]);
    ledSocket->sendData(run, (uint16_t)strlen(run), false);
  }

  if(force || refresh || (dirState != ledDirState))
  {
    ledDirState = dirState;
    char dir[2] = { config.ledDirectionId, dirState };
    ledSocket->sendData(dir, 2U, false);
  }
}
