package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

import java.util.ArrayList;
import java.util.List;

/**
 * Step 1 of bring-up: checks every device name the code uses against the Robot Configuration,
 * all at once, instead of crashing on the first missing one. Nothing moves.
 *
 * After INIT it keeps showing every motor's encoder, so you can turn each wheel / roller /
 * flywheel by hand and confirm its encoder counts. Push the robot to check the Pinpoint pods.
 *
 * The names come from the same constants the robot code uses, so this can't get out of date.
 */
@TeleOp(name = "1. Hardware Check", group = "Test")
public class HardwareCheck extends LinearOpMode {

    private static class Device {
        final String label;
        final String name;
        final Class<? extends HardwareDevice> type;
        HardwareDevice found;
        String status;

        Device(String label, String name, Class<? extends HardwareDevice> type) {
            this.label = label;
            this.name = name;
            this.type = type;
        }
    }

    private final List<Device> devices = new ArrayList<>();

    @Override
    public void runOpMode() {
        motor("Drive front left", Constants.drivetrainConfig.frontLeftName.get());
        motor("Drive front right", Constants.drivetrainConfig.frontRightName.get());
        motor("Drive back left", Constants.drivetrainConfig.backLeftName.get());
        motor("Drive back right", Constants.drivetrainConfig.backRightName.get());
        motor("Intake roller", IntakeSubsystem.INTAKE_MOTOR);
        motor("Launcher", LauncherSubsystem.LAUNCHER_MOTOR);
        crServo("Intake servo L", IntakeSubsystem.LEFT_INTAKE_SERVO);
        crServo("Intake servo R", IntakeSubsystem.RIGHT_INTAKE_SERVO);
        crServo("Windmill servo", LauncherSubsystem.WINDMILL_SERVO);
        devices.add(new Device("Pinpoint", Constants.localizerConfig.name.get(), GoBildaPinpointDriver.class));

        int problems = 0;
        for (Device device : devices) {
            device.found = hardwareMap.tryGet(device.type, device.name);
            if (device.found != null) {
                device.status = "OK";
                continue;
            }
            // Is something configured under that name, just not the right kind of device?
            HardwareDevice other = hardwareMap.tryGet(HardwareDevice.class, device.name);
            device.status = other != null
                    ? "WRONG TYPE: configured as " + other.getDeviceName() + ", expected " + device.type.getSimpleName()
                    : "MISSING";
            problems++;
        }

        GoBildaPinpointDriver pinpoint = (GoBildaPinpointDriver) find(GoBildaPinpointDriver.class);

        while (!isStopRequested()) {
            telemetry.addLine(problems == 0
                    ? "All devices found."
                    : problems + " device(s) need fixing in the Robot Configuration:");
            for (Device device : devices) {
                telemetry.addData(device.label, "\"%s\"  %s", device.name, device.status);
            }

            telemetry.addLine();
            // Raw counts: the code hasn't set motor directions here, so only check that each one changes.
            // Direction (forward = count up) is checked in the subsystem tests.
            telemetry.addLine("Encoders: turn each one by hand. Every count should change.");
            for (Device device : devices) {
                if (device.found instanceof DcMotorEx) {
                    DcMotorEx motor = (DcMotorEx) device.found;
                    telemetry.addData(device.label, "%d ticks  %.0f t/s",
                            motor.getCurrentPosition(), motor.getVelocity());
                }
            }

            if (pinpoint != null) {
                pinpoint.update();
                telemetry.addLine();
                telemetry.addData("Pinpoint", "%s", pinpoint.getDeviceStatus());
                telemetry.addData("Pinpoint pods", "x %d  y %d", pinpoint.getEncoderX(), pinpoint.getEncoderY());
                telemetry.addData("Pinpoint pose", "x %.1f in  y %.1f in  h %.1f°",
                        pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH),
                        pinpoint.getHeading(AngleUnit.DEGREES));
            }
            telemetry.update();
        }
    }

    private void motor(String label, String name) {
        devices.add(new Device(label, name, DcMotorEx.class));
    }

    private void crServo(String label, String name) {
        devices.add(new Device(label, name, CRServo.class));
    }

    private HardwareDevice find(Class<? extends HardwareDevice> type) {
        for (Device device : devices) {
            if (device.type == type) return device.found;
        }
        return null;
    }
}
