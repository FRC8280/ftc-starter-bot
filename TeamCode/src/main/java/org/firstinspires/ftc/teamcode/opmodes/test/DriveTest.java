package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Step 2a: drivetrain + Pinpoint, before any Pedro tuning. Put the robot on blocks first.
 *
 *   X / Y / A / B (hold)   spin ONE wheel forward: front-left / front-right / back-left / back-right
 *   Sticks                 drive at DRIVE_SCALE (only when no wheel button is held)
 *   Dpad down              reset the odometry pose to (0, 0, 0)
 *
 * Check, for each wheel button:
 *   - the right wheel spins (else the names in Constants.drivetrainConfig are on the wrong ports)
 *   - it spins forward (else flip its direction in Constants.drivetrainConfig)
 *   - its encoder counts UP (else the encoder cable is on a different port than the motor)
 *
 * Then, robot on the floor, push it by hand: forward = x goes up, left = y goes up,
 * counter-clockwise = heading goes up. If not, the Pinpoint Tuner will sort out the pod directions.
 */
@Configurable
@TeleOp(name = "2a. Drive Test", group = "Test")
public class DriveTest extends LinearOpMode {
    public static double WHEEL_TEST_POWER = 0.3;
    public static double DRIVE_SCALE = 0.5;

    @Override
    public void runOpMode() {
        DcMotorEx leftFront = motor(Constants.drivetrainConfig.frontLeftName.get(),
                Constants.drivetrainConfig.frontLeftDirection.get());
        DcMotorEx rightFront = motor(Constants.drivetrainConfig.frontRightName.get(),
                Constants.drivetrainConfig.frontRightDirection.get());
        DcMotorEx leftBack = motor(Constants.drivetrainConfig.backLeftName.get(),
                Constants.drivetrainConfig.backLeftDirection.get());
        DcMotorEx rightBack = motor(Constants.drivetrainConfig.backRightName.get(),
                Constants.drivetrainConfig.backRightDirection.get());

        // Pedro's Pinpoint localizer, with the pod settings from Constants.localizerConfig.
        Localizer odometry = Constants.localizer(hardwareMap);

        telemetry.addLine("Robot on blocks. Hold X / Y / A / B to spin one wheel.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double lf, rf, lb, rb;
            if (gamepad1.x || gamepad1.y || gamepad1.a || gamepad1.b) {
                lf = gamepad1.x ? WHEEL_TEST_POWER : 0;
                rf = gamepad1.y ? WHEEL_TEST_POWER : 0;
                lb = gamepad1.a ? WHEEL_TEST_POWER : 0;
                rb = gamepad1.b ? WHEEL_TEST_POWER : 0;
            } else {
                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;
                lf = forward + strafe + turn;
                rf = forward - strafe - turn;
                lb = forward - strafe + turn;
                rb = forward + strafe - turn;
                double max = Math.max(1, Math.max(Math.max(Math.abs(lf), Math.abs(rf)), Math.max(Math.abs(lb), Math.abs(rb))));
                lf = lf / max * DRIVE_SCALE;
                rf = rf / max * DRIVE_SCALE;
                lb = lb / max * DRIVE_SCALE;
                rb = rb / max * DRIVE_SCALE;
            }
            leftFront.setPower(lf);
            rightFront.setPower(rf);
            leftBack.setPower(lb);
            rightBack.setPower(rb);

            if (gamepad1.dpad_down) {
                odometry.setPose(Pose.zero());
            }
            odometry.update();
            Pose pose = odometry.pose();

            telemetry.addData("Front left  (X)", "power %.2f  ticks %d", lf, leftFront.getCurrentPosition());
            telemetry.addData("Front right (Y)", "power %.2f  ticks %d", rf, rightFront.getCurrentPosition());
            telemetry.addData("Back left   (A)", "power %.2f  ticks %d", lb, leftBack.getCurrentPosition());
            telemetry.addData("Back right  (B)", "power %.2f  ticks %d", rb, rightBack.getCurrentPosition());
            telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            telemetry.update();
        }
    }

    private DcMotorEx motor(String name, DcMotorSimple.Direction direction) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }
}
