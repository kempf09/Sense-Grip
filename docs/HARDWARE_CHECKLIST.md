# Hardware checklist

Run once on real hardware. Gripper unloaded, hand on the E-stop.

1. **Servo gate.** Servo tab: Jog at speed 400 (steps/s, about 6 RPM). Fingers move and `mode` reads 1. Release: it stops.
   Power-cycle: `mode` reads the value from before the jog (the mode write is not persisted).
   After a jog, press Open: no jump on the 1 to 0 switch. If it jumps on the 1 to 0 switch, stop and use step 9 (`runViaPosition`). Any other failure: tick `runViaPosition`.
2. **Limits.** Set OPEN zero and close limit; press Open. IDLE shows only "not armed".
3. **Trigger.** Arm with nothing in front. Wave a hand at under 80 mm for over 200 ms: it closes.
   Touch an FSR: it slows. Press with 1 N or more on 2 channels: MONITOR.
4. **No object.** Arm and trigger with nothing to grasp: FAULT at the close limit, with a reason.
5. **E-stop.** Press it in PRE_TOUCH, GRASP and OPEN: each gives FAULT with torque off.
6. **Real grasp.** Grasp a soft object. Press Open: IDLE, disarmed.
7. **MONITOR hold.** When GRASP ends, the servo switches from speed mode to position hold.
   Torque is briefly off during the mode change. Check the object does not slip.
8. **Scan.** Scan blocks the controller for up to about 0.6 s (8 baud rates) and E-stop is not serviced during it.
   Only scan with the gripper idle and clear, and keep a hand on the power switch while scanning: the software E-stop does not work during a scan.
9. **Fallback.** If step 1 fails, tick `runViaPosition` (Servo tab) and repeat steps 3 to 6.
10. **Multi-turn retained.** Jog the fingers about half closed. Power-cycle the servo and the ESP32
    together. After boot, Last action reads "servo stopped at boot" and the fingers have not moved.
    Before pressing Open, check that the position in the Servo tab is the same as before the power
    cycle. If it changed, the servo does not keep its multi-turn count: from now on, after every
    power-up, jog to fully open and Set OPEN zero, jog to fully closed and Set close limit, then press Open.
11. **Boot stop.** Arm and trigger a close; during PRE_TOUCH press the ESP32 reset button. The servo
    must stop within about a second of the reset (torque off) and the dashboard comes back in IDLE.
12. **Disarm while closing.** Arm and trigger a close; press Disarm during PRE_TOUCH: state goes to
    OPEN with "Disarmed during PRE_TOUCH — opening", then IDLE.
13. **Speeds.** With an object in front, Arm: PRE_TOUCH must reach the object well inside the 8 s
    pre-touch timeout (about 5 s for full travel at 2000 steps/s). If the gripper crawls, check the
    Servo tab speed reading while it moves; raise or lower Approach/Open speed (max 3400).
14. **FAULT stop.** Cause a FAULT during PRE_TOUCH (E-STOP). The fingers must stop and stay put;
    pressing Open while in FAULT must not move them (it says "Reset first").
