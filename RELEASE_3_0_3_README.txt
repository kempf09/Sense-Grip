SENSE-GRIP RELEASE 3.0.3
Calibrated travel references and simplified, documented FSM settings

CALIBRATED REFERENCES
- OPEN zero fallback: encoder 1968
- Three-finger TOUCH fallback: encoder 19202
- Closing direction: increasing encoder counts
- Available calibrated travel: 17,234 counts
- Travel safety margin default: 100 counts
Valid references already stored in ESP32 NVS take precedence over these fallback values.

FSM DEFAULTS
- Per-finger contact threshold: 0.25 N
- Contact confirmation: 3 consecutive samples (about 60 ms at 20 ms FSR sampling)
- Minimum active FSR channels for grasp-force confirmation: 3 of 6
- Average grasp-force target: 1.00 N
- Grasp-force confirmation: 3 consecutive samples (about 60 ms)
- Maximum force per individual FSR: 3.00 N
- Contact-search step: 50 encoder counts
- Grasp step: 10 encoder counts
- Contact-search speed: 15 CF35-12 command units
- Grasp speed: 10 CF35-12 command units
- Release speed: 20 CF35-12 command units
- Contact-search timeout: 30 seconds
- Grasp timeout: 15 seconds
- Release timeout: 20 seconds
- Force-loss warning: 25% drop for 3 consecutive samples; warning only, no automatic tightening
- No automatic grasp retries

The force limits are conservative commissioning starting values, not validated safe limits for every object or fixture. Confirm FSR calibration and use a low-risk test object. Set lower force values if the mechanism/object requires them. Maximum force is a per-channel limit.

SETTINGS PAGE
The FSM settings are displayed as one row per parameter with its purpose, allowed range and editable value. On narrow screens, each row adapts to a stacked layout. Settings can only be saved when the FSM is IDLE and the servo is stopped.

PERSISTENCE / MIGRATION
- The FsmConfig structure remains the same size and keeps the existing FSM_MAGIC, so existing travel references and user-modified settings remain readable.
- A one-time config version key is added. During first boot, values that exactly match Release 3.0.2 factory defaults are updated to Release 3.0.3 defaults. User-customized values are preserved.
- OPEN=1968 and TOUCH=19202 are only inserted if no references were saved. Existing valid NVS references take precedence.
- FSR calibration namespace `fsr402`, SEN0628 calibration namespace `senCal2`, servo namespace `cf35`, and the SEN0628 embedded page are preserved.
- No flash erase is required. Do not select Erase All Flash Before Sketch Upload.

INSTALLATION
1. Extract the ZIP file.
2. Keep the .ino and .h files together in the same sketch folder.
3. Open SENSE_GRIP_Release_3_0_3_Simple_FSM.ino.
4. Keep ESP32-S3 board, memory and pin settings unchanged.
5. Do not erase flash/NVS.
6. Compile and upload.
7. Verify `/api/config` reports OPEN=1968 and TOUCH=19202 (unless you had different valid references already stored).
8. Test with the mechanism clear and a low-risk object. Watch all six FSR readings, motor current and temperature.

VALIDATION
- Embedded main-page JavaScript syntax checked with Node.js.
- SEN0628 page compared byte-for-byte with Release 3.0.2.
- Config API key/range list checked against the UI field list.
- ZIP integrity checked.
- No full Arduino ESP32 toolchain compile or physical gripper test was performed in this session.
