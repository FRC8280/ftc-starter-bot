package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing setup for the drivetrain, Pinpoint odometry, and the Foresight path follower.
 *
 * Every value here comes from a tuner. Run them from the Pedro tuning web page
 * (connect to the robot's Wi-Fi, then open http://192.168.43.1:10158) in this order:
 *
 * 1. Mecanum Tuner: paste its output over drivetrainConfig
 * 2. Pinpoint Tuner: paste its output over localizerConfig
 * 3. Foresight Tuner: paste its output over foresightConfig
 * 4. Tests: check Localization, then Line, Curve, and Hold
 *
 * Turtle Tracer exports import this class by name (org.firstinspires.ftc.teamcode.pedro.Constants),
 * so keep the package, class name, create(), and foresightConfig as they are.
 */
public class Constants {

    // Motor names must match the Robot Configuration on the Driver Station.
    // Names and directions are the ones the BioBuzz StarterBot TeleOp used, so the same
    // Robot Configuration works. The Mecanum Tuner checks the directions.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("left_front_drive");
        c.frontRightName.set("right_front_drive");
        c.backLeftName.set("left_back_drive");
        c.backRightName.set("right_back_drive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // goBILDA Pinpoint computer with two goBILDA odometry pods.
    // Offsets are placeholders: the Pinpoint Tuner measures them.
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.0);
        c.yPodOffset.set(0.0);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // PLACEHOLDER VALUES. They let the code build and run the tuners.
    // Do not run an autonomous until you have replaced this with the Foresight Tuner output.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.1);
                Controller secondaryTranslationalForward = Controller.proportional(0.05);
                Controller primaryTranslationalLateral = Controller.proportional(0.1);
                Controller secondaryTranslationalLateral = Controller.proportional(0.05);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01));
                c.brake.set(Controller.proportionalFeedforward(0.01));

                c.headingFeedback.set(Controller.proportional(1.0));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05, 0.002));

                c.linearBrakeCoefficients.set(Matrix.diag(0.05, 0.05));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002, 0.002));

                c.maxAchievableForwardVelocity.set(60.0);
                c.maxAchievableStrafeVelocity.set(45.0);
                c.naturalForwardDeceleration.set(30.0);
                c.naturalStrafeDeceleration.set(60.0);
            }
    );

    public static Drivetrain drivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    public static Localizer localizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, localizerConfig);
    }

    public static Algorithm algorithm() {
        return new Foresight(foresightConfig);
    }

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(localizer(hardwareMap), drivetrain(hardwareMap), algorithm());
    }
}
