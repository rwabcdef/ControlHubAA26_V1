/*
 * HubApp.hpp
 *
 * The hub's top level mode - Idle, Control or Lift - and the one place
 * that decides what a command may do in it.
 *
 * Commands reach the motor from three places: the PC over MQTT (CTRL0,
 * LIFT0), the serial console (CTRL0, MOTOR) and the remote hub over the
 * radio (BTN01, POT01, HBT01). Before HubApp, each socket handler drove
 * controllerB or liftB directly, from whichever SerLink task it ran in,
 * and the last command won. Now the handlers only parse: anything that
 * starts, stops or steers the motor is posted to HubApp's queue and
 * applied by run(), in the one task that owns the controller and the lift.
 *
 * Modes
 * -----
 *   Idle     the controller is disabled and the direction idle.
 *   Control  the controller holds the target speed until stopped.
 *            Started by the PC (CTRL0 BS) or the remote (button 1).
 *   Lift     a liftB move is in progress. Started by the PC only (LIFT0).
 *
 *   Idle    + start (PC or remote)     -> Control, in the selected direction
 *   Idle    + lift start / ground      -> Lift
 *   Control / Lift + stop (from anywhere, remote button 1 included) -> Idle
 *   Lift move ends (arrived, ground, tacho fault)                  -> Idle
 *   Control + tacho fault                                          -> Idle
 *   Control (remote started) + remote heartbeat lost               -> Idle
 *
 * A start of either kind is ignored unless Idle, and so is a direction
 * change - the motor is never reversed at speed. The target speed may be
 * changed at any time: it applies to a run in progress (Lift, or a PC
 * started Control) and to the next start.
 *
 * The remote
 * ----------
 * Button 1 is start/stop, button 2 toggles the selected direction (Idle
 * only). The pot is a speed trim for runs the REMOTE started: while one is
 * going, the pot sets the speed (0..potRpmMax), and a run the PC started
 * ignores it. Every frame from the remote (heartbeat, pot, button) counts
 * as a sign of life. A remote started run is stopped once none has
 * arrived for heartbeatTimeoutMs - with the remote gone, nothing else
 * would stop it. A PC started run carries on: the PC is watching it.
 *
 * Outbound
 * --------
 * run() sends, all as 'U' (fire and forget):
 *   - the CTRL0 status frame on statusSocket, every statusPeriodMs while
 *     running and once on reaching Idle - see writeStatus();
 *   - LED01 frames to the remote on ledSocket, on a change and every
 *     ledRefreshMs, which is also the remote's sign that the hub is there;
 *   - the LIFT0 done frame, through onLiftDone, when a move ends.
 *
 * Threading
 * ---------
 * run() belongs to one task (controlBTask), which must also be the only
 * task running the controller and the lift - run() calls lift->run().
 * Everything else here is safe from any task: post() is a non-blocking
 * queue send, and the setters and getters each read or write one volatile
 * byte, half word or word.
 *
 * Like the drivers, a HubApp is a file scope object: the constructor only
 * stores its config, and init() (before the scheduler starts) creates the
 * queue.
 */

#ifndef HUBAPP_HPP_
#define HUBAPP_HPP_

#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "Controller.hpp"
#include "Lift.hpp"
#include "Socket.hpp"

struct HubAppConfig
{
  Controller*       controller;
  Lift*             lift;

  // Motor current, mA, for the status frame.
  uint16_t (*getCurrentMa)();

  // Sends the LIFT0 done frame. The LIFT0 format lives with the rest of
  // the LIFT0 protocol, in main_tasks.cpp.
  void (*onLiftDone)();

  uint16_t bootRpm;             // the target speed until one is set
  uint16_t potRpmMax;           // the speed at pot 100%
  uint16_t periodMs;            // how often run() is called
  uint16_t statusPeriodMs;      // multiple of periodMs
  uint16_t ledRefreshMs;        // multiple of periodMs
  uint16_t heartbeatTimeoutMs;
  char     ledRunId;            // LED01 ids on the remote
  char     ledDirectionId;
};

// A command for run(). One queue item per command, so its fields always
// arrive together.
struct AppCmd
{
  enum op_t : uint8_t
  {
    start,            // Control, from the PC
    stop,             // whatever is running, from the PC
    direction,        // select a direction (dir) - Idle only
    liftStart,        // liftDir, distance
    liftGround,       // distance is the maximum
    remoteStartStop,  // remote button 1: start if Idle, else stop
    remoteDirection   // remote button 2: toggle the selected direction
  };

  op_t                op;
  ControllerDirection dir;
  Lift::direction     liftDir;
  uint32_t            distance;
};

class HubApp
{
  public:
    enum class mode : uint8_t { idle, control, lift };
    enum class source : uint8_t { none, pc, remote };

    explicit HubApp(const HubAppConfig& config);

    // Before the scheduler starts, once the sockets are acquired: creates
    // the queue and sets the boot target speed. statusSocket carries the
    // CTRL0 status frame (MQTT), ledSocket LED01 to the remote (radio).
    // Either may be nullptr - an acquire past SERLINK_CONFIG__MAX_SOCKETS
    // returns one - and that output is then skipped. False if the config
    // is unusable, and run() then does nothing.
    bool init(SerLink::Socket* statusSocket, SerLink::Socket* ledSocket);

    // Owning task only, every config.periodMs.
    void run();

    // Any task. False if the queue is full and the command was dropped -
    // over a 'T' frame the ack has already gone, so read the mode back.
    bool post(const AppCmd& cmd);

    // Any task. The target speed: a PC run's, a lift move's, and a remote
    // run's until the pot is moved. Applied by the next run().
    void     setTargetRpm(uint16_t rpm);
    uint16_t getTargetRpm() const;

    // Any task, from the remote's socket handlers. Every remote frame
    // calls remoteAlive(); setPotPercent() takes 0..100.
    void remoteAlive();
    void setPotPercent(uint8_t percent);

    // Any task.
    mode                getMode() const;
    source              getSource() const;   // none while Idle
    ControllerDirection getSelectedDirection() const;   // forward or reverse

    // The CTRL0 status frame's data, STATUS_LEN chars, no NUL:
    //   <mode I|C|L><dir F|R><duty ddd>.<measured rpm dddd>.<current mA dddd>
    // e.g. CF030.0350.1234. Getters only, so any task.
    static const uint8_t STATUS_LEN = 15U;
    void writeStatus(char* dst) const;

    static char modeToChar(mode m);
    static char sourceToChar(source s);

  private:
    static const uint8_t QUEUE_LENGTH = 8U;

    HubAppConfig config;
    bool valid;
    SerLink::Socket* statusSocket;
    SerLink::Socket* ledSocket;

    StaticQueue_t staticQueue;
    uint8_t       queueStorage[QUEUE_LENGTH * sizeof(AppCmd)];
    QueueHandle_t queue;

    volatile mode                currentMode;
    volatile source              runSource;
    volatile ControllerDirection selectedDir;
    volatile uint16_t            targetRpm;
    volatile uint16_t            potRpm;
    volatile bool                potSeen;
    volatile TickType_t          remoteLastTick;
    volatile bool                remoteSeen;

    // LED01 data: the remote takes 9 chars at most (its UART_BUFF_LEN).
    static const uint8_t LED_DATA_MAX = 9U;

    // Owning task only.
    uint32_t statusPasses;
    uint32_t ledPasses;
    char     ledRunState[LED_DATA_MAX];   // the run LED's last data, id stripped, NUL ended
    char     ledDirState;

    void apply(const AppCmd& cmd);
    void startControl(source from);
    void stopAll();
    void enterIdle();
    uint16_t runRpm() const;
    void sendStatus();
    void updateLeds(bool force);
};

#endif /* HUBAPP_HPP_ */
