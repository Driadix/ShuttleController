# ShuttleController

STM32 firmware for the shuttle racking system controller. This module runs the real machine: motion control, lifter control, safety checks, sensors, BMS integration, persistent configuration, direct display UART, and E22 radio communication with the handheld remote.

## Build
- Production build/upload is done only with Arduino IDE and the team-approved STM32 Arduino board/core settings.
- `platformio.ini` exists only for simple compile verification. PlatformIO is not the production build path and must not define project behavior.

Open `Cntrl_V2/Cntrl_V2.ino` in Arduino IDE to build or upload.

## Main Files
- `Cntrl_V2/Cntrl_V2.ino` - main controller sketch and state machine.
- `Cntrl_V2/ShuttleProtocol.h` - shared packed wire protocol used by controller, remote, and display.
- `Cntrl_V2/E22Radio.hpp` - header-only E22 radio helper for controller/remote links.
- `Cntrl_V2/TOF_Sense.*` - TOF sensor support.
- `Cntrl_V2/BmsDdA5*.hpp` - BMS integration.
- `Cntrl_V2/AlertManager.h` - fault/warning handling.

## Communication
- `SerialLora` connects to the E22 radio module for the remote.
- `SerialDisplay` connects directly to the display/Wi-Fi bridge.
- `SerialRS485`, CAN, GPIO sensors, and TOF are local controller interfaces.

Radio should stay mostly request-response: remote polling for heartbeat/status, sensors, stats, and config. Display UART can receive pushed telemetry/sensors/stats because it is wired.

## Reliability Notes
Target radio operation is up to 100 m in warehouse shuttle-racking conditions. Manual movement over radio must keep working with delayed packets: about 80 ms at 4k-class air data rate and up to about 150 ms in bad 2k-class scenarios.

Preserve ACK handling, command acceptance gates, error/channel checks, and the manual radio hold watchdog when changing motion or communication code.

## Validation
Documentation-only changes do not require a firmware compile. Firmware changes should be verified in Arduino IDE first, with PlatformIO only as an extra compile check. Hardware validation is required for motion, safety, radio, protocol, BMS, CAN, TOF, and display UART changes.
