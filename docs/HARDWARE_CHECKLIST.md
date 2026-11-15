# Hardware checklist

Run once on real hardware. Gripper unloaded, hand on the E-stop.

1. **Position mode, turns counted.** Servo tab: if `Mode` is not 0, press Set position mode (0).
   Jog at speed 20 for several turns in one direction: `Position` must pass 4096 and keep counting
   (no wrap back to 0..4095), and `Mode` stays 0 the whole time. If it wraps, stop and report: the
   firmware would have to count turns itself. Then, with the fingers half closed, power-cycle the
   servo: `Mode` must still read 0 and `Position` must be the same as before. If the position
   changed, the count is lost at power-up: clear and re-set the limits after every servo power-up
   (and don't rely on "press Open" alone) until this is solved.
2. **Limits.** Set OPEN zero and close limit; press Open. IDLE shows only "not armed".
3. **Trigger.** Arm with nothing in front. Wave a hand at under 80 mm for over 200 ms: it closes.
   Touch an FSR: it slows. Press with 1 N or more on 2 channels: MONITOR.
4. **No object.** Arm and trigger with nothing to grasp: FAULT at the close limit, with a reason.
5. **E-stop.** Press it in PRE_TOUCH, GRASP and OPEN: each gives FAULT with torque off.
6. **Real grasp.** Grasp a soft object. Press Open: IDLE, disarmed.
7. **MONITOR hold.** When GRASP ends, torque goes off (the jaw cannot be back-driven). Check the
   object does not slip and the position stays put.
8. **Scan.** Scan blocks the controller for up to about 0.6 s (8 baud rates) and E-stop is not serviced during it.
   Only scan with the gripper idle and clear, and keep a hand on the power switch while scanning: the software E-stop does not work during a scan.
9. **Open from mid-travel.** Jog about halfway closed; press Open. The fingers return to OPEN zero,
   `Mode` stays 0, `Position from OPEN zero` ends near 0 and the state returns to IDLE.
10. **Multi-turn retained.** Jog the fingers about half closed. Power-cycle the servo and the ESP32
    together. After boot, Last action reads "servo stopped at boot" and the fingers have not moved.
    Before pressing Open, check that the position in the Servo tab is the same as before the power
    cycle. If it changed, the servo does not keep its multi-turn count: from now on, after every
    power-up, jog to fully open and Set OPEN zero, jog to fully closed and Set close limit, then press Open.
11. **Boot stop.** Arm and trigger a close; during PRE_TOUCH press the ESP32 reset button. The servo
    must stop within about a second of the reset (torque off) and the dashboard comes back in IDLE.
12. **Disarm while closing.** Arm and trigger a close; press Disarm during PRE_TOUCH: state goes to
    OPEN with "Disarmed during PRE_TOUCH — opening", then IDLE.
13. **Open moves.** From IDLE away from OPEN, press Open: the fingers move to OPEN zero and the state
    returns to IDLE (no timeout). Full travel at speed 61 takes about 6 s.
14. **FAULT stop.** Cause a FAULT during PRE_TOUCH (E-STOP). The fingers must stop and stay put;
    pressing Open while in FAULT must not move them (it says "Reset first").
15. **SEN0628 dropouts.** Leave the gripper idle for 2 minutes, then jog for 2 minutes. Note on the
    SEN0628 tab for each sensor: dropouts (moving), max read ms, init fails (stage), plus control
    loop max and servo bus errors. Report these numbers.
