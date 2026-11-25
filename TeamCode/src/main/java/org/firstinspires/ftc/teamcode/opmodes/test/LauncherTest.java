package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

/**
 * Step 2c: the launcher flywheel and the windmill feeder. Keep hands clear of the flywheel.
 *
 *   Right bumper (hold)    spin the flywheel at the test speed
 *   Dpad up / down         test speed +/- 50 ticks/s
 *   A (hold)               windmill feeds, but only while the flywheel is above LAUNCHER_MIN_VELOCITY
 *   X (hold)               windmill on by itself, flywheel ignored (direction check only, no game pieces)
 *   Y                      copy the test speed into LAUNCHER_TARGET_VELOCITY for the rest of this run
 *
 * What to check and tune, in order:
 *   1. X: the windmill turns toward the flywheel. If not, flip its direction in LauncherSubsystem.
 *   2. Right bumper: the speed climbs to the target and holds within about 20 t/s. If it shoots far
 *      past the target and doesn't come back, the encoder is unplugged or on the wrong port.
 *   3. Tune the PIDF in Panels while holding the bumper: F first (speed close to the target with
 *      P = 0), then P until it recovers quickly after a shot without oscillating.
 *   4. Shoot real pieces (bumper + A) and change the test speed until shots land. Then set
 *      LAUNCHER_TARGET_VELOCITY, and LAUNCHER_MIN_VELOCITY a little under it, in LauncherSubsystem.
 */
@Configurable
@TeleOp(name = "2c. Launcher Test", group = "Test")
public class LauncherTest extends LinearOpMode {
    public static double SPEED_STEP = 50;

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        LauncherSubsystem launcher = new LauncherSubsystem(hardwareMap);
        launcher.init();

        double testSpeed = LauncherSubsystem.LAUNCHER_TARGET_VELOCITY;
        double peakDrop = 0;
        boolean lastUp = false, lastDown = false, lastY = false;

        telemetry.addLine("Hands clear of the flywheel. Hold the right bumper to spin it.");
        telemetry.update();
        waitForStart();

        try {
            while (opModeIsActive()) {
                if (gamepad1.dpad_up && !lastUp) testSpeed += SPEED_STEP;
                if (gamepad1.dpad_down && !lastDown) testSpeed = Math.max(0, testSpeed - SPEED_STEP);
                if (gamepad1.y && !lastY) LauncherSubsystem.LAUNCHER_TARGET_VELOCITY = testSpeed;
                lastUp = gamepad1.dpad_up;
                lastDown = gamepad1.dpad_down;
                lastY = gamepad1.y;

                boolean spinning = gamepad1.right_bumper;
                launcher.testVelocity(spinning ? testSpeed : 0);

                double velocity = launcher.velocity();
                boolean atSpeed = spinning && launcher.isAtSpeed();
                double windmill = gamepad1.x || (gamepad1.a && atSpeed) ? LauncherSubsystem.WINDMILL_POWER : 0;
                launcher.testWindmill(windmill);

                // Biggest drop below the test speed while feeding: how hard each shot slows the flywheel.
                if (spinning && windmill > 0) peakDrop = Math.max(peakDrop, testSpeed - velocity);
                if (!spinning) peakDrop = 0;

                telemetry.addData("Flywheel", "%.0f / %.0f t/s  (%.0f RPM)  %s", velocity,
                        spinning ? testSpeed : 0, launcher.rpm(),
                        !spinning ? "off" : atSpeed ? "above MIN" : "below MIN");
                telemetry.addData("Windmill", windmill > 0 ? "ON" : "off");
                telemetry.addData("Test speed", "%.0f t/s (dpad up/down)", testSpeed);
                telemetry.addData("Biggest drop while feeding", "%.0f t/s", peakDrop);
                telemetry.addLine();
                telemetry.addData("TARGET / MIN", "%.0f / %.0f t/s",
                        LauncherSubsystem.LAUNCHER_TARGET_VELOCITY, LauncherSubsystem.LAUNCHER_MIN_VELOCITY);
                telemetry.addData("PIDF", "%.1f  %.1f  %.1f  %.1f", LauncherSubsystem.LAUNCHER_P,
                        LauncherSubsystem.LAUNCHER_I, LauncherSubsystem.LAUNCHER_D, LauncherSubsystem.LAUNCHER_F);
                telemetry.update();
            }
        } finally {
            launcher.stop();
        }
    }
}
