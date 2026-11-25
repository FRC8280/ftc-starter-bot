package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.function.DoubleSupplier;

/**
 * Mecanum drivetrain + goBILDA Pinpoint odometry, both owned by the Pedro {@link Follower}.
 *
 * Pedro drives the wheels with open-loop power and gets position from the Pinpoint, so the
 * drive motor encoders are not used for localization. They are still read here for telemetry,
 * which is handy for spotting a dead wheel or a loose encoder cable.
 *
 * TeleOp driving is the BioBuzz StarterBot's: left stick drives and strafes, right stick X turns,
 * and the powers are scaled down together if any wheel would go past full power (Pedro's
 * follower.manual() does that scaling).
 */
@Configurable
public class DriveSubsystem implements Subsystem {
    /**
     * false (the BioBuzz StarterBot way) = robot-centric: stick forward drives the way the robot faces.
     * true = field-centric: stick forward drives in the "driver forward" direction whichever way
     * the robot faces. Driver forward is the way the robot faced at START, unless TeleOp sets it
     * from an Auto's recorded heading; the driver can re-set it (see {@link #setDriverForward()}).
     */
    public static boolean FIELD_CENTRIC = false;

    private final Follower follower;
    private final DcMotorEx leftFront, rightFront, leftBack, rightBack;

    // Kept separate from the odometry heading so re-setting it never changes the robot's pose.
    private double driverForwardHeading = 0;

    public DriveSubsystem(HardwareMap hardwareMap, Pose startPose) {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        leftFront = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.frontLeftName.get());
        rightFront = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.frontRightName.get());
        leftBack = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.backLeftName.get());
        rightBack = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.backRightName.get());
    }

    /** Turtle Tracer exports and PedroCommands need the follower itself. */
    public Follower follower() {
        return follower;
    }

    public Pose pose() {
        return follower.pose();
    }

    /** The follower must update every loop, in TeleOp and in Auto. */
    @Override
    public void periodic() {
        follower.update();
    }

    @Override
    public void start() {
        driverForwardHeading = follower.pose().heading();
    }

    @Override
    public void stop() {
        follower.stop();
    }

    /** Set driver forward to a known field heading. Call after {@link #start()}. */
    public void setDriverForwardHeading(double radians) {
        driverForwardHeading = radians;
    }

    // ---------------------------------------------------------------- commands

    /**
     * Default TeleOp command: runs until something else requires the drive.
     * Pedro's convention: forward +, strafe left +, turn counter-clockwise +.
     */
    public Command teleopDrive(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
        return infinite(() -> {
            DrivePowers powers = new DrivePowers(forward.getAsDouble(), strafe.getAsDouble(), turn.getAsDouble());
            if (FIELD_CENTRIC) {
                powers = ManualDrive.fieldCentric(powers, follower.pose().heading() - driverForwardHeading);
            }
            follower.manual(powers);
        })
                .requiring(this)
                .setEnd(endCondition -> follower.stop());
    }

    /** Follow a Pedro path. Finishes when the robot reaches the end of the path. */
    public Command follow(Path path) {
        return PedroCommands.follow(follower, path).requiring(this);
    }

    /**
     * Field-centric: press while the robot faces straight away from the driver.
     * Only the driving direction changes; the odometry pose is left alone.
     */
    public Command setDriverForward() {
        return instant(() -> driverForwardHeading = follower.pose().heading());
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        Pose pose = follower.pose();
        telemetry.addData("Drive", FIELD_CENTRIC ? "field-centric" : "robot-centric");
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Wheel ticks", "LF %d  RF %d  LB %d  RB %d",
                leftFront.getCurrentPosition(), rightFront.getCurrentPosition(),
                leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
    }
}
