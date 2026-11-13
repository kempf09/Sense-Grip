# Continuous approach development branch

Branch: `feature/continuous-approach-touch-window`

Base: Release 3.0.3 on `main`.

## Behaviour

- FSM START sends one CF35-12 position command to the saved three-finger TOUCH encoder position.
- The controller polls motor feedback every 10 ms.
- The approach stops when the measured encoder position enters the final 5% of the calibrated OPEN-to-TOUCH travel. The configured travel safety margin can widen this stop window.
- The controller then commands a position hold at the measured encoder position and enters HOLDING.
- Existing FSR maximum-force fault protection remains active during approach and hold.
- The SEN0628 acquisition, calibration data, 3D page, FSR pin map, and saved OPEN/TOUCH references are unchanged.
- Position speed is capped at 61 command units (about 44.7 RPM using the CF35-12 speed-unit conversion). Existing saved speed values are clamped on load.
- The continuous approach timeout defaults to 8 seconds.

## Timing limit

Saved references OPEN=1968 and TOUCH=19202 span 17,234 counts, or about 4.21 revolutions at 4,096 counts/revolution. At the motor's published approximately 45 RPM no-load speed, the full travel takes about 5.6 seconds; reaching the 95% stop window takes about 5.3 seconds before acceleration and load. This branch removes the repeated-step command overhead, but it cannot achieve a sub-3-second full-travel approach with the existing CF35-12 and current geometry.

## Validation status

The branch update was checked for the intended source changes and consistent configuration ranges. It has not been compiled with the Arduino ESP32 toolchain or tested on physical hardware. Verify motor direction, current limits, FSR stop behaviour, and the 5% encoder window with the gripper unloaded and an accessible emergency stop before testing with an object.
