# Auto-align and shoot-on-the-move

## Controls (driver Xbox)

| Hold | What happens |
|---|---|
| **X** | Auto-align: drives to `SWEET_SPOT_DISTANCE` in front of the HUB and points the shooter at it. Only starts from inside our ALLIANCE ZONE. |
| **B** | Shoot on the move: you keep translation (capped at 2 m/s); the robot turns to aim, sets flywheel speed, and runs the indexer only when the shot is good. |

PathPlanner named commands: `AlignToHub`, `AimAndShoot` (3 s each). Run them **after** a path, not in parallel with one.

## How it works

1. **Pose**: wheel odometry + Pigeon run every 20 ms in a `SwerveDrivePoseEstimator`. The Limelight 4 (MegaTag2) only corrects drift, so 15 fps is plenty. Nothing aims straight off the camera.
2. **Virtual target**: FUEL keeps the robot's velocity after it leaves, so we aim at `HUB - velocity * timeOfFlight` instead of the HUB.
3. **Flywheel**: closed-loop velocity (`VelocityVoltage`) from a distance table, indexed by distance to the virtual target. Driving toward the HUB lowers the speed and driving away raises it.
4. **Heading**: PID plus a feedforward for how fast the aim point sweeps. In simulation the feedforward keeps aim error under 0.6° while strafing at 2 m/s; without it the robot lags about 9° and never fires.
5. **Feed gate**: aimed, flywheel within 2 rps, distance inside the table, and bumpers in our ALLIANCE ZONE (G407, MAJOR FOUL otherwise).

## Test in simulation first

Run `./gradlew simulateJava` (IntelliJ: Gradle → `simulateJava`, or VS Code: "WPILib: Simulate Robot Code" with the GUI).

1. **Joysticks**: plug the Xbox controller into the laptop. In the Sim GUI, drag it from *System Joysticks* into *Joysticks* slot 0 (the driver controller port). Untick "Map gamepad" if the axes look wrong.
2. **Alliance**: pick Blue 1 or Red 1 in the *FMS* / *DS* panel. The sim robot starts 2.5 m from your alliance wall, in line with your HUB and facing it.
3. **Field view**: NetworkTables → SmartDashboard → `Field`. You'll see the robot, `ShotVirtualTarget` (where it aims while moving), and `AlignTarget` (where X drives to). AdvantageScope can also connect to the sim at `localhost`.
4. **Enable Teleop**, then check:
   - Hold **X** from inside the zone. The robot should drive onto the `AlignTarget` and `Align/Aligned` turns true. From the neutral zone, `Align/Status` should say "not in alliance zone".
   - Hold **B** standing still. The flywheel spins up (`Shot/FlywheelRps` climbs to `Shot/TableRps`), then `Shot/Aimed`, `Shot/AtSpeed` and `Shot/Firing` turn true.
   - Hold **B** while strafing. `ShotVirtualTarget` should slide away from the HUB opposite your motion, the robot should lead the shot, and `Shot/Firing` should stay true. Drive past the zone line and `Shot/Firing` should drop out (G407).
   - Run an auto from the Auto Chooser in *Autonomous* mode.

What the sim models: the drivetrain as an ideal swerve (no wheel slip, acceleration capped by `SimConstants`) and the flywheel as a physics model with both Krakens. It does **not** model the Limelight (the sim pose is exact), the FUEL flight, or your real motor gains. Use it to catch logic, direction, sign and binding mistakes; the RPS/TOF tables still have to be tuned on the real robot.

## Limelight setup (web UI)

- Hostname `limelight` (or change `VisionConstants.LIMELIGHT_NAME`).
- AprilTag pipeline, 2026 field map, **MegaTag2 on**.
- **Robot-space camera pose** measured and entered. MegaTag2 is only as good as this.
- To run cooler: use detector downscale and crop. The code also sets `throttle_set` = 100 while disabled.
- Dashboard: `Vision/LimelightTempC`, `Vision/LimelightFps`, `Vision/SecondsSinceUpdate`, `Vision/FieldHeadingTrusted`.

## Debugging switches and readouts

- `Shot/BlockedBy`: the one thing stopping the shot right now ("aiming: 4.0 deg off, need under 2.0", "flywheel at 50 of 65 rps", "outside alliance zone (G407)", ...). It says `FIRING` while firing.
- `Vision/Status`: what happened to the last camera frame ("accepted: 2 tag(s) at 2.3 m", "rejected: spinning too fast", ...).
- `Align/Status`: "driving: 0.85 m and 12.0 deg to go", "aligned", or why it didn't start.
- `Shot/DryRun`: everything runs except the indexer. Use it until aiming and the flywheel are proven.
- `Shot/IgnoreAllianceZone`: for practice fields only. It is ignored automatically when connected to FMS.
- `Vision/Enabled`: turn it off to drive on odometry alone if vision makes the pose jump.
- `Tune/AimP`, `Tune/AimD`, `Tune/MaxAimOmega`, `Tune/AlignP`, `Tune/AlignMaxSpeed`, `Tune/MaxDriveSpeedWhileShooting`, `Tune/FlywheelToleranceRps`, `Tune/LookaheadSec`, `Tune/TofScale`: live gains. They reset to `Constants.kt` on reboot, so copy good values into the code.
- The robot logs a line for every burst of shots ("Shot: firing at 2.41 m, 66.0 rps ...") and every vision seed. Find them in AdvantageScope's console view.

## Tuning order

1. **Shooter geometry** (`ShotConstants`): `SHOOTER_OFFSET_X/Y`, and `SHOOTER_FACING_RAD` (0 = shooter faces the robot front).
2. **Flywheel gains** (`LaunchConstants`): spin to a fixed rps and check `shooter/flywheel rps` tracks it. Tune kV first, then raise kP until it recovers quickly after each FUEL.
3. **RPS table**: turn on `Shot/TuningMode`. Park at about 1, 2, 3, 4 and 5 m (`Shot/DistanceM`), hold B, and adjust `Shot/TuningRps` until shots drop in. Put the pairs in `FLYWHEEL_RPS_BY_DISTANCE`.
4. **Time-of-flight table**: film a few shots in slow motion and count frames from exit to entry. This only affects shots while moving.
5. **Moving shots**: strafe past the HUB with B held. If shots land ahead of your direction of travel, the robot isn't leading enough: raise `Tune/TofScale` (or the TOF values). If they land behind, lower it. If only the first ball of each burst misses, raise `Tune/LookaheadSec`.
6. **Auto-align**: set `SWEET_SPOT_DISTANCE` to your most reliable distance.
