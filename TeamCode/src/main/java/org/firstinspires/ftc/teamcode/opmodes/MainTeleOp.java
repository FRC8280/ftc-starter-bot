package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.GamepadControls;

/* FTC Starter Bot: the goBILDA StarterBot (mecanum) on Ivy + Pedro Pathing */
/**
 * Driver controls (gamepad 1). The same as the BioBuzz StarterBot TeleOp, plus Options.
 *
 * After START:
 *   Left stick          drive / strafe
 *   Right stick X       turn
 *   Right trigger       intake in (proportional: pull further = faster)
 *   Left trigger        intake out (proportional). Both triggers = off
 *   Right bumper        hold: spin up the launcher. Once it's at speed, the windmill feeds and the
 *                       intake gets a little extra power. Release: launcher and windmill stop
 *   Options             set field-centric "forward" to where the robot faces (field-centric only)
 *
 * Before START (field-centric only):
 *   A                   use / ignore the heading the last Auto recorded
 *
 * Driving is robot-centric by default, like the BioBuzz StarterBot (DriveSubsystem.FIELD_CENTRIC).
 * With field-centric on and a recorded heading, "forward" is the driver direction below for the side
 * the Auto's paths are drawn for. With none, the robot must face away from the driver at START.
 */
@Configurable
@TeleOp(name = "TeleOp", group = "Competition")
public class MainTeleOp extends LinearOpMode {
    /**
     * The Pedro field heading that points straight away from each alliance's driver, in the coordinates
     * of paths drawn for that side. Red drivers stand at the x = 0 edge and look toward +x; blue drivers
     * stand at x = 144 and look toward -x.
     */
    public static double BLUE_DRIVER_FORWARD_DEG = 180;
    public static double RED_DRIVER_FORWARD_DEG = 0;

    @Override
    public void runOpMode() {
        Scheduler.reset();

        Alliance pathSide = Robot.autoEndPathSide();
        boolean hasRecording = pathSide != null;
        boolean useRecordedHeading = hasRecording;

        Robot robot = new Robot(hardwareMap, telemetry, Robot.autoEndPose());
        robot.init();

        // Every driver control after START, in one place. The button before START is in the init loop below.
        GamepadControls controls = new GamepadControls()
                // Left stick: drive / strafe. Right stick X: turn.
                .whileActive(robot.drive.teleopDrive(
                        () -> -gamepad1.left_stick_y,
                        () -> -gamepad1.left_stick_x,
                        () -> -gamepad1.right_stick_x))
                // Right trigger in, left trigger out. Proportional.
                .whileActive(robot.intake.driverControl(() -> gamepad1.right_trigger, () -> gamepad1.left_trigger))
                // Right bumper: hold to spin up the launcher and feed once it's at speed.
                .whileActive(robot.launcher.driverControl(() -> gamepad1.right_bumper))
                // Options: set "driver forward" to where the robot faces now (field-centric only).
                .onPress(() -> DriveSubsystem.FIELD_CENTRIC && gamepad1.options, robot.drive::setDriverForward);

        boolean lastA = false;
        while (opModeInInit()) {
            robot.clearBulkCache();
            robot.drive.periodic(); // keeps the heading right if the robot gets bumped before START

            // A: use / ignore the recorded heading. A toggle, so a wrong press can be undone.
            if (gamepad1.a && !lastA && hasRecording) {
                useRecordedHeading = !useRecordedHeading;
            }
            lastA = gamepad1.a;

            if (!DriveSubsystem.FIELD_CENTRIC) {
                telemetry.addData("Drive", "robot-centric: stick forward drives the way the robot faces");
            } else if (useRecordedHeading) {
                telemetry.addData("Heading", "%.1f° (from Auto, paths drawn for %s)",
                        AngleUnit.normalizeDegrees(Math.toDegrees(robot.drive.pose().heading())), pathSide);
                telemetry.addLine("A = ignore the Auto heading");
            } else {
                telemetry.addData("Heading", "not set: face the robot away from the driver before START");
                if (hasRecording) telemetry.addLine("A = use the Auto heading");
            }
            telemetry.addLine();
            telemetry.addLine("Ready. Nothing moves until START.");
            telemetry.update();
        }

        if (isStopRequested()) return;

        Robot.clearAutoEnd();
        robot.start(); // driver forward starts as the way the robot faces now
        if (DriveSubsystem.FIELD_CENTRIC && useRecordedHeading) {
            // The pose is in the coordinates of the side the Auto's paths are drawn for.
            robot.drive.setDriverForwardHeading(Math.toRadians(
                    pathSide == Alliance.RED ? RED_DRIVER_FORWARD_DEG : BLUE_DRIVER_FORWARD_DEG));
        }
        controls.schedule();

        try {
            while (opModeIsActive()) {
                robot.clearBulkCache();
                Scheduler.execute();
                robot.periodic();
                robot.telemetry();
            }
        } finally {
            Scheduler.reset();
            robot.stop();
        }
    }
}
