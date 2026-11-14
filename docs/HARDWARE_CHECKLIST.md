# Hardware checklist

Run once on real hardware. Gripper unloaded, hand on the E-stop.

1. **Servo gate.** Servo tab: Jog at speed 10. Fingers move and `mode` reads 1. Release: it stops.
   Power-cycle: `mode` reads the value from before the jog (the mode write is not persisted).
   After a jog, press Open: no jump on the 1 to 0 switch. Any failure: tick `runViaPosition`.
2. **Limits.** Set OPEN zero and close limit; press Open. IDLE shows only "not armed".
3. **Trigger.** Arm with nothing in front. Wave a hand at under 80 mm for over 200 ms: it closes.
   Touch an FSR: it slows. Press with 1 N or more on 2 channels: MONITOR.
4. **No object.** Arm and trigger with nothing to grasp: FAULT at the close limit, with a reason.
5. **E-stop.** Press it in PRE_TOUCH, GRASP and OPEN: each gives FAULT with torque off.
6. **Real grasp.** Grasp a soft object. Press Open: IDLE, disarmed.
7. **MONITOR hold.** When GRASP ends, the servo switches from speed mode to position hold.
   Torque is briefly off during the mode change. Check the object does not slip.
8. **Scan.** Scan blocks the controller for about 0.2 s and E-stop is not serviced during it.
   Only scan with the gripper idle and clear.
9. **Fallback.** If step 1 fails, tick `runViaPosition` (Servo tab) and repeat steps 3 to 6.
