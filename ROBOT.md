# FTC Starter Bot: goBILDA StarterBot (mecanum) on Pedro Pathing 3 + Ivy + Turtle Tracer

The goBILDA StarterBot with mecanum wheels, a goBILDA Pinpoint, an intake roller with two corner servos,
and a flywheel launcher fed by a windmill servo. The robot logic comes from the
`BioBuzzStarterbotTeleopMecanum` TeleOp; the structure (Ivy commands, subsystems, Pedro Pathing,
Turtle Tracer autos, test OpModes, Panels) is the same as the Ivy sample robot project.

Built on the [Pedro Pathing Quickstart](https://github.com/Pedro-Pathing/Quickstart) (FTC SDK 12.0, 2026-27 season).

| Library | Version | Why |
|---|---|---|
| Pedro Pathing (`revhub`, `tuning`) | 3.0.2 / 1.0.1 | Path following, Pinpoint localizer, tuning web page |
| Ivy (`com.pedropathing.ivy:core` + `:pedro`) | 1.1.1 | Command scheduler. **1.1.x is the Pedro 3 build.** The old `com.pedropathing:ivy:1.0.0` only works with Pedro 2 |
| TurtleTracerLib | v3.1.1 (JitPack) | Loads `.turt` files and runs event markers in Turtle Tracer exports |
| Sloth (`dev.frozenmilk.sinister:Sloth`) + Load plugin | 0.3.2 | Fast deploys. Pedro's tuning library already depends on it |
| Panels, Sloth build (`com.bylazar.sloth:fullpanels`) | 0.3.2+1.0.13 | Dashboard: telemetry, graphs, live-editable values |

No LED subsystem and no Limelight on this robot (yet).

## Code layout (`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`)

```
Robot.java                    builds the subsystems, bulk caching, Auto → TeleOp handoff
pedro/Constants.java          drivetrain, Pinpoint, and Foresight config: paste tuner output here
pedro/Tuning.java             registers the tuners on the Pedro tuning page
subsystems/
  DriveSubsystem.java         mecanum + Pinpoint through the Pedro Follower
  LauncherSubsystem.java      flywheel (closed-loop velocity) + windmill feeder servo
  IntakeSubsystem.java        intake motor + 2 corner servos, with the launcher's feed assist
util/GamepadControls.java     every driver control in one list (continuous + on-press)
opmodes/MainTeleOp.java       the TeleOp
opmodes/test/                 bring-up test OpModes (see "Bring-up order")
autos/TurtleTracerAuto.java   base class for every auto
autos/paths/                  put Turtle Tracer exports here
```

## What came from the BioBuzz StarterBot TeleOp

| BioBuzz TeleOp | Here |
|---|---|
| `mecanumDrive(-left_stick_y, left_stick_x, right_stick_x)`, powers scaled so none passes 1 | `DriveSubsystem.teleopDrive` through Pedro's `follower.manual()`, which does the same scaling. Robot-centric, like BioBuzz |
| Drive motors BRAKE, left side REVERSE | Directions in `pedro/Constants.drivetrainConfig`. Pedro brakes while driving by hand (`manualBrakeMode`, on by default) |
| `intakePower = right_trigger - left_trigger` | `IntakeSubsystem.driverControl` |
| Intake motor + both corner servos get the same power; right servo reversed | `IntakeSubsystem.periodic` writes all three once per loop |
| Launcher `RUN_USING_ENCODER`, PIDF (40, 0, 0, 12.5) | `LauncherSubsystem`: `LAUNCHER_P/I/D/F`, re-sent when changed in Panels |
| Right bumper held: `setVelocity(1250)`, else `setVelocity(0)` (brakes the flywheel hard) | `LauncherSubsystem.driverControl`. **One change:** released, the flywheel coasts down instead of braking (`COAST_WHEN_OFF`, below) |
| Windmill runs only while bumper held AND velocity > 1200 | `LauncherSubsystem.periodic`: same check in TeleOp and Auto |
| `intakePower += 0.5` while the windmill feeds | `FEED_ASSIST_POWER`, added in `IntakeSubsystem.periodic` |
| Windmill reversed; CR servos set to 0 at init | Same |

The order matters, just as in the BioBuzz loop: commands set what's wanted, then `robot.periodic()` runs the
launcher (decides if it's feeding) before the intake (adds the assist). That order is in `Robot`.

## Robot configuration names

These are the names the BioBuzz TeleOp used, so an existing StarterBot configuration works unchanged
(plus the Pinpoint).

| Type | Name | Notes |
|---|---|---|
| Motor | `left_front_drive`, `right_front_drive`, `left_back_drive`, `right_back_drive` | Mecanum drive. Directions are set by the Mecanum Tuner |
| Motor | `intake` | Positive power pulls game pieces in |
| Motor | `launcher` | Flywheel. Encoder must be plugged in (28 ticks/rev): velocity control needs it |
| CR Servo | `left_intake_servo`, `right_intake_servo` | Corner servos. Right one is reversed in code |
| CR Servo | `windmillServo` | Feeds the launcher. Reversed in code |
| I²C | `pinpoint` | goBILDA Pinpoint Odometry Computer |

## Bring-up order

Each test OpMode is in the **Test** group on the Driver Station, numbered in order, with its controls and
what to check written at the top of its file and shown on screen.

| OpMode | What it checks |
|---|---|
| **1. Hardware Check** | Every name above against the Robot Configuration, all at once: OK / MISSING / WRONG TYPE. Then live encoder counts for every motor and the Pinpoint pods. Nothing moves |
| **2a. Drive Test** | One wheel at a time (X / Y / A / B): right wheel, spins forward, encoder counts up. Then stick driving and the odometry pose, before any Pedro tuning |
| **2b. Intake Test** | Triggers run the roller and both corner servos. All three should pull pieces in on the right trigger |
| **2c. Launcher Test** | Windmill direction (X), flywheel holds speed (right bumper), dpad changes the test speed, A feeds only above `LAUNCHER_MIN_VELOCITY`. Shows how far each shot drags the flywheel down |
| **3. Pedro tuning** | The Pedro tuning web page (below) |
| **4. Autos** | "Hand-Written Ivy Auto" first, then Turtle Tracer autos |

Numbers found in a test only last for that run. Copy them into the code when they're right.

## Panels and Sloth

**Panels** is a web dashboard: with the laptop on the robot's Wi-Fi, open <http://192.168.43.1:8001>.
- Everything `Robot` puts in telemetry (TeleOp and Auto) shows there as well as on the Driver Station.
  The test OpModes only use the Driver Station.
- **Configurables:** every tuning value in `LauncherSubsystem`, `IntakeSubsystem`, `DriveSubsystem`,
  `MainTeleOp`, the Drive Test, and the Launcher Test can be changed live from Panels while an OpMode runs,
  for example `LAUNCHER_TARGET_VELOCITY`, `LAUNCHER_F`, `FEED_ASSIST_POWER`, `FIELD_CENTRIC`.
  **Changes made in Panels are lost when the app restarts.** Copy the good values into the code.
  (These are the classes marked `@Configurable`. `pedro/Constants` isn't: paste tuner output there instead.)

**Sloth** makes deploys take seconds instead of reinstalling the whole app. It also has to be set up for Panels
to work in this project, because Pedro's tuning library already uses Sloth. That's why this uses the Sloth build of
Panels: don't swap it for the regular `com.bylazar:fullpanels`. One-time setup in Android Studio:
1. Deploy once the normal way (the green Run button), after syncing Gradle.
2. *Run → Edit Configurations → + → Gradle*, task `deploySloth`. Save.
3. Edit the normal **TeamCode** run configuration: under *Before launch*, add a Gradle task `removeSlothRemote`
   (Gradle project `:TeamCode`) and move it to the top. Save.
4. From now on, use the `deploySloth` configuration for code changes, and the normal TeamCode one when you
   change Gradle files, add libraries, or something seems stuck.

Sloth only hot-reloads code in `TeamCode`. If uploads ever stop taking effect, deploy with the normal TeamCode
configuration (which runs `removeSlothRemote` first). Sloth's README has the details and screenshots.

## Driver controls (gamepad 1)

The same as the BioBuzz StarterBot TeleOp, plus Options for field-centric.

| Input | Action |
|---|---|
| Left stick | Drive / strafe |
| Right stick X | Turn |
| Right trigger | Intake in. Proportional: pull further for more speed |
| Left trigger | Intake out (reverse). Proportional. Both triggers fully pulled = off |
| Right bumper | Hold: spin up the launcher. Once it's above `LAUNCHER_MIN_VELOCITY`, the windmill feeds and the intake gets `FEED_ASSIST_POWER` extra. Release: the windmill stops and the flywheel coasts down |
| Options | Field-centric only: make "forward" the way the robot faces now |

A button already held when START is pressed is ignored until it is released and pressed again (Options only;
the sticks, triggers and bumper are read live, as in BioBuzz). Nothing moves until START.

Printable control sheets: `docs/Starter-Bot-Driver-Controls.pdf` (Xbox-style), `docs/Starter-Bot-Driver-Controls-PS4.pdf`
and `docs/Starter-Bot-Driver-Controls-PS5.pdf` (DualSense). They're built from `docs/source/driver-controls.html`:
after changing the controls, edit that file and run `docs/source/make-pdfs.sh` (needs Google Chrome).

### Robot-centric or field-centric

Driving is **robot-centric** by default, like the BioBuzz StarterBot: stick forward drives the way the robot faces.
Set `DriveSubsystem.FIELD_CENTRIC = true` (in code or Panels) for field-centric. Then:

- **Heading from an Auto:** every Auto records its final pose and the side its paths are drawn for when it ends,
  including when it's stopped early. "Forward" is `BLUE_DRIVER_FORWARD_DEG` or `RED_DRIVER_FORWARD_DEG` in
  `MainTeleOp`, picked by that side: the Pedro field heading pointing away from that side's driver. Red drivers
  stand at the x = 0 edge of the Turtle Tracer field, so red is 0°. Blue drivers stand at x = 144, so blue is 180°.
- **No heading (no Auto, or ignored with A before START):** "forward" is the way the robot faces at START, so point
  it away from the driver.

Options re-sets "forward" during the match. TeleOp clears the recording at START, so a second TeleOp run starts
with no heading. Restarting the Robot Controller app also clears it.

### Launcher

The flywheel runs under the hub's velocity control (`RUN_USING_ENCODER`), so it holds `LAUNCHER_TARGET_VELOCITY`
(1250 ticks/s, about 2,678 RPM) as the battery drops and as shots slow it down. The windmill only turns while the
flywheel is above `LAUNCHER_MIN_VELOCITY` (1200 ticks/s), so a game piece is never fed into a slow wheel: hold the
bumper and the windmill pauses by itself while the flywheel recovers between shots.

**Coasting when off:** with `COAST_WHEN_OFF = true` (the default), turning the launcher off switches the motor to
`RUN_WITHOUT_ENCODER` at 0 power with FLOAT, so the flywheel coasts to a stop. (Velocity control with a target of 0,
which BioBuzz uses, makes the hub brake the wheel hard.) Spinning up switches back to `RUN_USING_ENCODER`. Pressing the
bumper again while it's still coasting picks up from the current speed, so the next shot is ready sooner. Set
`COAST_WHEN_OFF = false` for the BioBuzz behavior.

If the flywheel speed jumps far past the target and never comes back, the encoder is unplugged, on a different
port than the motor, or the motor polarity is reversed somewhere in the wiring.

Tuning, all in `LauncherSubsystem` (Panels), with the Launcher Test:
1. `LAUNCHER_F` first, with `LAUNCHER_P` at 0: raise F until the speed settles close to the target.
2. `LAUNCHER_P`: raise it until the speed recovers quickly after a shot without swinging around the target.
3. `LAUNCHER_TARGET_VELOCITY` for where the shots land; keep `LAUNCHER_MIN_VELOCITY` a little under it.
4. `FIRE_FEED_MS` (Auto only): long enough to feed everything in the hopper.

## Tuning the drive for autonomous

1. Connect to the robot's Wi-Fi and open the Pedro tuning page at <http://192.168.43.1:10158> (Control Hub address).
2. Run the tuners in order. Each one prints code to paste into `pedro/Constants.java`:
   1. **Mecanum Tuner**: motor names and directions (`drivetrainConfig`)
   2. **Pinpoint Tuner**: pod directions and offsets (`localizerConfig`)
   3. **Foresight Tuner**: velocities, braking, and translational and heading gains (`foresightConfig`)
   4. **Tests**: Localization and Odometry first, then Line, Curve, and Hold
3. `foresightConfig` ships with **placeholder** values so the project builds. Don't run an auto until step 2.3 is done.

Keep the robot still while it initializes: the Pinpoint recalibrates its IMU at init.

## Adding an autonomous from Turtle Tracer

1. In Turtle Tracer: **Export Code → Sequential → Ivy, Java**, package `org.firstinspires.ftc.teamcode.autos.paths`.
   Constants class: `org.firstinspires.ftc.teamcode.pedro.Constants` (the default).
2. Save the exported class in `autos/paths/`.
   If you export **without** "hardcode values", also copy the `.turt` file to `TeamCode/src/main/assets/AutoPaths/`.
3. Copy `autos/BlueSampleAuto.java`, rename it, and return your export:

   ```java
   @Autonomous(name = "Blue Close", group = "Competition", preselectTeleOp = "TeleOp")
   public class BlueClose extends TurtleTracerAuto {
       protected Alliance alliance() { return Alliance.BLUE; }

       protected Command routine(Robot robot) throws Exception {
           return new BlueClosePaths(robot.drive.follower(), hardwareMap).command();
       }
   }
   ```

   To add actions before or after the paths, wrap the export:
   `sequential(new BlueClosePaths(...).command(), robot.launcher.fire())`.
   `TurtleTracerAuto` runs the routine without reserving any subsystem, so this is safe to mix with `fire` markers.
   Each `fire` marker is one volley: a marker that comes while another marker's volley is running waits for it to finish.

   The auto's init screen lists every registered marker name. A marker whose name isn't on that list does nothing.

### Event marker names

Type these exactly as marker names in Turtle Tracer. They are registered before your export is built.

| Marker | Does |
|---|---|
| `intakeOn` / `intakeOff` / `intakeReverse` | Intake roller and corner servos |
| `launcherOn` / `launcherOff` | Spin the flywheel up / stop it. Put `launcherOn` early in the path before a shot so it's at speed on arrival |
| `fire` | One volley: spin up if needed, wait for speed (up to `SPIN_UP_TIMEOUT_MS`), feed for `FIRE_FEED_MS`. Leaves the flywheel spinning |

Leave the robot at the shooting spot long enough for a volley to finish (spin-up + `FIRE_FEED_MS`, about 2 s)
before the next `launcherOff`. A volley still running sets the flywheel back on as it finishes.

To add one, call `NamedCommands.registerCommand("name", () -> ...)` in that subsystem's `registerNamedCommands()`.

`autos/paths/SampleAutoPaths.java` is an example of what the export looks like, with markers. Its poses are placeholders.
`autos/HandWrittenIvyAuto.java` shows the same kind of auto written by hand with subsystem commands.

## Adding a Limelight or LEDs later

The Ivy sample robot project has both (`VisionSubsystem` + `vision/LimelightCamera`, `LedSubsystem` + `prism/`).
Copy those files across, add them to the `subsystems` list in `Robot`, and add their names to `HardwareCheck`.
