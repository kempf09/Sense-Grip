# Recording protocol — slip-detection data (phase 4)

Goal: real recordings of quiet holds, harmless disturbances and deliberate slips on every test
object, with the moment of each slip marked. The slip detector's thresholds are tuned from these
files, so what is not recorded cannot be tuned.

## 0. Before you start (once)

1. Flash the latest `feature/continuous-approach-touch-window` build. The dashboard header must
   start with **`phase4-slip`** (or a later build name). "last reset" should not say CRASH or
   BROWN-OUT; if it does, note it.
2. Use a **laptop** for the dashboard, not a phone (a phone that sleeps stops the collection).
   Keep the Combined tab open and visible the whole session (a hidden tab is slowed down by the
   browser). Reloading is fine (recordings continue), but closing the page for more than ~3
   minutes during a grasp leaves a gap (marked "gap" in the list).
3. Press **Home**. A calibration sweep must be stored (Servo tab plot shows a curve).
4. FSR tab: **Max force N ≥ 8** (otherwise large grasps are refused).
5. SEN0628 tab: with the gripper empty and nothing in view, press **Capture background**.
6. Check the Recordings panel shows **≈100 rows/s**.
7. **Test grasp first:** grasp any object, press **P** then **E** while it is held, Open.
   Download that recording: the `.csv` must contain `mark:pull` and `mark:end` in the `event`
   column, and the `_tof.csv` must have frames. The recording list shows **frames/s**: expect
   roughly 20–45 (three sensors); fewer when nothing moves in front of the sensors (unchanged
frames are not stored). Note the number. If the Recordings panel says "FRAME LOG
   UNAVAILABLE", stop and report it (the PSRAM settings of the build are wrong).
8. SEN0628 tab → Slip detection: leave **Tighten on slip unticked**. The detector still runs
   with placeholder thresholds and may show SLIP badges; ignore them (they are logged and help
   the tuning). Do not change the slip settings during the session.

**If you Arm, present an object and nothing happens (stays IDLE, armed):** the object is not
"present" yet: SEN0628 tab → Config → **Detect distance mm** → 150, Save, try again. Small
centred objects sit ~85–100 mm from the sensors; the default 80 mm can miss them.

## 1. Mark keys (dashboard, Combined tab — work anywhere on the page)

| Key | Mark | Press it… |
|---|---|---|
| **P** | pull | the moment you start pulling the object out of the hand |
| **T** | twist | the moment you start twisting / rotating it |
| **S** | slide | the moment you start pushing it along its long axis through the fingers |
| **O** | other | at a harmless disturbance that is *not* a slip (bump, tap, hand in view) |
| **E** | end | when the slip / disturbance is over (object stopped, came free, or you let go) |

Press the key **as the motion starts**, not after. A late mark is worse than none.
If someone helps, one person handles the object and the other presses keys.

## 2. Per object

Objects: egg, tube, mug, apple, drill (by its handle), the asymmetric object, and any other
object you want the detector to work on. Present each one as in the demo (palm up, centred).
**Place the object (or hang it on its string) and take your hand out of the sensor view before
pressing Arm**: a hand near the object is measured as part of it and can pick the wrong class.
If START keeps giving up ("object size not stable"), the object is near a class boundary:
use the class override.

| # | Trial | What to do | Repeats |
|---|---|---|---|
| A | **Quiet hold** | Arm, let it grasp, **do not touch anything for 30 s**, then Open. | 2 |
| B | **Disturbances** | Grasp. Every ~5 s, one of these, each with **O** then **E**: tap the table, tap the gripper base, touch the object lightly, wave a hand through the sensor view. Then Open. | 1 |
| C | **Pull** | Grasp, wait 3 s. **P**, pull the object slowly upward (out of the palm) until it moves or comes free, **E**. Re-grasp and do it again faster. A third time, pull only until it starts to move, then stop. | 3 |
| D | **Twist** | Grasp, wait 3 s. **T**, rotate the object (about the gripper axis, then tilt it sideways), **E**. | 3 |
| C2 | **Pull from outside the view** | Same as C, but pull by the string (hanging objects) or with a tool / a cord tied to the object, so **no hand enters the sensor view**. Keeps the detector from learning "a hand appeared" instead of "the object moved". | 2 |
| B2 | **Hand grips, object stays** | Grasp. **O**, take hold of the object with your hand as if to pull, but do **not** move it for 2 s, let go, **E**. | 2 |
| E | **Slide** | Only for elongated objects (tube, drill handle, anything long). Grasp, wait 3 s. **S**, push/pull it along its length through the fingers, **E**. | 3 |

Egg: only A, B and a very gentle C (with **Delicate** ticked on the SEN0628 tab).
Drill: set the class override to **large** (SEN0628 tab) before grasping.

**After each object:** Recordings panel → **Download all (.zip)**, check the zip opened, then
**Clear all**. Clear all only deletes recordings that are in a downloaded zip (the list marks the
others "not downloaded"), so a grasp that was still recording is never lost. This keeps the browser's storage small (a session can reach 100+ MB).

Each grasp is its own recording automatically (from 1 s before START to 1 s after it is back in
IDLE). One grasp per trial is easiest to analyse; several slips inside one grasp are fine as
long as each has its own mark.

## 3. If the automatic grasp does not work

Phase 3 grasping (measured approach) has not run on the gripper before this session.
1. If START gives up ("object not measured", "size not stable"): set the **class override**
   (SEN0628 tab) and try again.
2. If it still fails or FAULTs: **Start manual recording** (Recordings panel), Disarm, jog the
   fingers closed onto the object on the Servo tab until the pads show force, then do the trial
   with the marks, then **Stop manual recording**. Note "manual" in the notes file.
3. Write down any FAULT text from the dashboard (or take a screenshot).

## 4. Notes file

Keep a small table while you record (any text/Excel file is fine):

```
file,object,trial,notes
grasp_3_medium_20261118-101502.csv,mug,C pull fast,came free
grasp_4_small_20261118-101640.csv,tube,E slide,
```

The file names appear in the Recordings panel as soon as a grasp finishes.

## 5. Hand-over

Recordings panel → **Download all (.zip)**: one zip with every finished recording
(`name.csv` + `name_tof.csv`). Send that zip plus the notes file. If the panel ever shows
"BROWSER STORAGE ERROR", download the zip right away and press Clear all before continuing. Recordings stay in the
browser until you press Clear all, so nothing is lost if a download fails.
