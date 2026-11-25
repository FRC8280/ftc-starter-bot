package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Step 2b: intake roller and the two corner servos. The launcher is not used.
 *
 *   Right trigger      intake in  (proportional)
 *   Left trigger       intake out (proportional)
 *
 * Check, pulling the right trigger:
 *   - the roller pulls game pieces IN (else flip the motor direction in IntakeSubsystem)
 *   - BOTH corner servos pull pieces in toward the roller. If one pushes out, flip that servo's
 *     direction in IntakeSubsystem (the right one is reversed in code).
 */
@TeleOp(name = "2b. Intake Test", group = "Test")
public class IntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, () -> false);
        intake.init();

        telemetry.addLine("Right trigger = in, left trigger = out.");
        telemetry.update();
        waitForStart();

        try {
            while (opModeIsActive()) {
                intake.testPower(gamepad1.right_trigger - gamepad1.left_trigger);
                intake.telemetry(telemetry);
                telemetry.addLine("Check: roller and BOTH corner servos pull pieces in on the right trigger.");
                telemetry.update();
            }
        } finally {
            intake.stop();
        }
    }
}
