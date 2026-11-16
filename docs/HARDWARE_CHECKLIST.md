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
16. **FSR filters.** FSR tab, pick a pad in the chart. Press it by hand and tap it sharply: compare
    raw (grey) and filtered (blue) for each filter, and note the delay readout. With the median
    filter on, a sharp press above the force limit must still trip the FAULT.
17. **FSR sample time at 100 Hz.** FSR tab (Filter panel) or SEN0628 tab: "FSR sample pass max"
    must stay at or below 4 ms, and the control loop max below 10 ms. If the pass is above 4 ms,
    report it: the FSR oversampling is then reduced from 16 to 8.
18. **First calibration sweep.** Gripper empty, nothing in view. Jog to fully open, Set OPEN zero,
    Run calibration sweep. Note the touch position it reports. The opening plot is dashed from
    170 mm at OPEN (estimated: the SEN0628 cannot see the fingers there) to the first point where a
    fingertip is seen, then solid down to the touch point; finger zones are outlined on the heatmaps.
    Re-run the sweep after changing FSR resistors or contact force (the touch point moves).
19. **Home after power cycles.** Three times, from different finger positions: power-cycle the
    servo (and ESP32), press Home. Note the residual each time; the fingers must end at OPEN.
20. **Home with an object** between the fingers must fail ("residual ... object in the gripper?")
    without opening past OPEN.
21. **ESP32 restart only** (servo stays powered): the dashboard shows NOT HOMED; Home works.
22. **Jump guard:** with the ESP32 on, power-cycle only the servo → NOT HOMED appears.
23. **Running check:** homed, idle, hold an object in the SEN0628 view → no "position check failed".

### Phase 3 — object perception and grasp

24. **Max force.** FSR tab: Max force N above the large class force (default 8 N; an older config
    keeps 3 N and large grasps are refused until it is raised).
25. **Opening table first.** Open `/api/sweep` (or the Servo tab plot): solid curve falling
    smoothly from ~100 mm to ~11 mm, no spikes. Then tick **Approach only**, arm, and present a
    rigid object (tube or mug): the fingers must stop short of it. With a ruler, the fingertip
    gap to the surface should be about x mm + 10 % of the range shown. Only then untick it.
26. **Capture background** with the gripper empty at the filming location (SEN0628 tab).
27. **Width of each demo object** (egg, tube, mug, apple, drill handle, asymmetric object), held
    centred: note width / range / shape / class on the SEN0628 tab against a ruler.
28. **Nothing measurable:** arm with an object outside the 100 mm zone (or hold a hand far away):
    START gives up after 3 s with "object not measured ...", disarmed.
29. **One grasp per class:** Combined tab shows PRE_TOUCH step "approach" then "slow", the grasp
    stops at the class force, the contact pattern is shown (wrap for large, pinch for small).
30. **Delicate egg:** tick Delicate (badge shown), grasp the egg: 0.4 N, grasp speed 3.
31. **Drill by its handle:** override LARGE (badge "CLASS: LARGE (forced)"), grasp holds ~500 g.
