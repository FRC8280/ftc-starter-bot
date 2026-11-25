package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.turtletracerlib.pathing.NamedCommands;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * The goBILDA StarterBot intake: one roller motor, plus two continuous-rotation servos that pull
 * game pieces out of corners. All three always run at the same power.
 *
 * From the BioBuzz StarterBot TeleOp:
 *   - Power = right trigger minus left trigger, so the intake speed is proportional and the left
 *     trigger runs it in reverse. Both triggers fully pulled = off.
 *   - While the launcher's windmill is feeding, FEED_ASSIST_POWER (0.5) is added to help push
 *     pieces up out of the hopper.
 *
 * Commands set the requested power; {@link #periodic()} adds the feed assist and writes the motor
 * and both servos once per loop, so they're never set twice in one loop.
 */
@Configurable
public class IntakeSubsystem implements Subsystem {
    // Robot Configuration names
    public static final String INTAKE_MOTOR = "intake";
    public static final String LEFT_INTAKE_SERVO = "left_intake_servo";
    public static final String RIGHT_INTAKE_SERVO = "right_intake_servo";

    /** Below this the intake counts as off (for telemetry and isRunning()). */
    public static double DEADBAND = 0.05;
    /** The speed the intake runs at in Auto ("intakeOn"). */
    public static double INTAKE_POWER = 1.0;
    public static double REVERSE_POWER = -1.0;

    private final DcMotorEx intakeMotor;
    private final CRServo leftServo, rightServo;
    private final BooleanSupplier launcherFeeding;

    private double requestedPower = 0;
    private double appliedPower = 0;

    /**
     * @param launcherFeeding true while the launcher's windmill is feeding, which adds FEED_ASSIST_POWER
     */
    public IntakeSubsystem(HardwareMap hardwareMap, BooleanSupplier launcherFeeding) {
        this.launcherFeeding = launcherFeeding;

        intakeMotor = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD); // positive power should pull game pieces in
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftServo = hardwareMap.get(CRServo.class, LEFT_INTAKE_SERVO);
        rightServo = hardwareMap.get(CRServo.class, RIGHT_INTAKE_SERVO);
        // The servos face each other, so one is reversed and both pull pieces in on positive power.
        rightServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /** The power commands ask for (before the feed assist). Written in {@link #periodic()}. */
    public void setPower(double power) {
        requestedPower = power;
    }

    /** Write a power to the motor and both servos right now. */
    private void apply(double power) {
        appliedPower = Range.clip(power, -1, 1);
        intakeMotor.setPower(appliedPower);
        leftServo.setPower(appliedPower);
        rightServo.setPower(appliedPower);
    }

    public boolean isRunning() {
        return Math.abs(appliedPower) > DEADBAND;
    }

    /** For the Intake Test only: run everything at a raw power right now. */
    public void testPower(double power) {
        apply(power);
    }

    @Override
    public void init() {
        // Sends the CR servos a value so the servo controller is enabled, as the BioBuzz TeleOp did.
        apply(0);
    }

    @Override
    public void periodic() {
        double assist = launcherFeeding.getAsBoolean() ? LauncherSubsystem.FEED_ASSIST_POWER : 0;
        apply(requestedPower + assist);
    }

    @Override
    public void stop() {
        requestedPower = 0;
        apply(0);
    }

    // ---------------------------------------------------------------- commands

    /** TeleOp, from the BioBuzz StarterBot: power = in - out (right trigger - left trigger). */
    public Command driverControl(DoubleSupplier in, DoubleSupplier out) {
        return infinite(() -> setPower(in.getAsDouble() - out.getAsDouble()))
                .requiring(this)
                .setEnd(endCondition -> setPower(0));
    }

    public Command run(double power) {
        return instant(() -> setPower(power)).requiring(this);
    }

    @Override
    public void registerNamedCommands() {
        NamedCommands.registerCommand("intakeOn", () -> setPower(INTAKE_POWER));
        NamedCommands.registerCommand("intakeOff", () -> setPower(0));
        NamedCommands.registerCommand("intakeReverse", () -> setPower(REVERSE_POWER));
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Intake", "power %.2f%s  ticks %d", appliedPower,
                launcherFeeding.getAsBoolean() ? " (feed assist)" : "", intakeMotor.getCurrentPosition());
    }
}
