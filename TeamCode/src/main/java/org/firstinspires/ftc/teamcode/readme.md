## TeamCode Module

This module is the home of our competition robot controller. The standout features are custom PedroPathing paths, Limelight-powered AprilTag localization, and live telemetry dashboards—everything you need to drive autos, keep localization honest, and see robot state at a glance.

---

### Custom PedroPathing paths

- **Where to look:** Auto OpModes such as `opmode/auto/Auto_9_0_Blue.java` declare their poses/control points up top and then construct `Path` objects in `buildPaths()`. All of those helpers live in `opmode/auto/AutonomousMethods.java`, while `commands/drivecommand/PathCommand.java` or `PathChainCommand.java` actually hand each path to the follower.
- **How to add/edit a route:** Update the pose constants in your auto file, then extend `buildPaths()` with additional `buildPath()`/`buildCurve()` calls and schedule them inside the command group. For TeleOp-only maneuvers, follow the lazy `follower.pathBuilder()` pattern demonstrated in `opmode/TeleOp_Solo.java` to generate a curve on demand.
- **Tuning knobs:** Drivetrain and follower constants (PIDF, constraints, Pinpoint layout) are centralized in `pedroPathing/PedroPathingConstants.java`. Edit those before swapping gearboxes/odo pods or when you need tighter tracking.

---

### AprilTag localization (Limelight 3A)

- **Sensor pipeline:** `hardware/subsystems/CameraSubsystem.java` wraps the Limelight 3A, polls `LLResult`, and filters fiducials so we only home in on our alliance basket (IDs 20 or 24). It also tracks the “obelisk” tag IDs (21–23) so autonomous routines know which pattern is up.
- **Pose + range cues:** The class converts tag pitch into a distance estimate using the `CAMERA_HEIGHT_M`, `TAG_HEIGHT_M`, and `CAMERA_PITCH_RAD` fields at the top of the file. It then classifies the shot as `INRANGE` or `OUTOFRANGE`, exposing the results through `Robot.data`.
- **Adjustments:** Tweak the geometry constants for new mounts, use `setCameraYawOffsetDeg()` for alliance-dependent trims, or change `minRange`/`maxRange` to widen the “green light” window for shooters.

---

### Live telemetry dashboards

- **Shared data model:** `hardware/RobotData.java` stores loop timing, follower pose/busy state, subsystem enumerations, shooter velocity, Limelight information, and renders it to both the Driver Station and Panels dashboards when `Robot.write()` is called.
- **Panels integration:** `opmode/TeleOp_Solo.java`, `TeleOp_SoloSlow.java`, and `pedroPathing/Tuning.java` create a `TelemetryManager` via `PanelsTelemetry.INSTANCE`. Any `telemetryM.debug()/info()` call instantly shows up on the dashboard alongside the FTC telemetry stream.
- **Extend it:** Set new fields on `Robot.data` inside your subsystems’ `updateData()` hooks, then print them in `RobotData.write()`. For quick experiments drop `telemetryM` logging anywhere you already have an OpMode reference.

---

### Quick start

1. `Robot robot = Robot.getInstance(); robot.initialize(hardwareMap, telemetry);`
2. Build or fetch a PedroPathing route (`buildPaths()` in your auto or `follower.pathBuilder()` in TeleOp).
3. Schedule drive/subsystem commands with FTCLib’s `CommandScheduler`.
4. Inside your loop call `robot.periodic()`, `robot.updateData()`, and `robot.write()` so localization, Limelight, and telemetry stay in sync.

Between the PedroPathing builders, Limelight AprilTag localization, and the live dashboards, this module covers every entry point you need to iterate on Decode League strategies without touching the stock FTC samples.
