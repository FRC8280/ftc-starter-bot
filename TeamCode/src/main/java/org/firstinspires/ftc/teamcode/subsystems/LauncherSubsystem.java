package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.turtletracerlib.pathing.NamedCommands;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

/**
 * The goBILDA StarterBot launcher: one high-speed flywheel motor under closed-loop velocity
 * control, and a continuous-rotation "windmill" servo that feeds game pieces into it.
 *
 * From the BioBuzz StarterBot TeleOp:
 *   - The flywheel runs RUN_USING_ENCODER, so setVelocity() holds a speed in encoder ticks per
 *     second. The hub's velocity PIDF is replaced with gains that suit a flywheel (40, 0, 0, 12.5).
 *   - The windmill only feeds while the flywheel is above LAUNCHER_MIN_VELOCITY, so a piece is
 *     never fed into a wheel that is too slow to throw it.
 *   - While the windmill feeds, the intake gets FEED_ASSIST_POWER extra to push pieces up
 *     (IntakeSubsystem reads {@link #isFeeding()}).
 *
 * Commands only set what is wanted (a target speed, feed or not). {@link #periodic()} writes the
 * motor and servo once per loop, and decides if the windmill may feed, the same in TeleOp and Auto.
 *
 * Encoder: 28 ticks per motor revolution, so RPM = ticks/second / 28 * 60.
 */
@Configurable
public class LauncherSubsystem implements Subsystem {
    // Robot Configuration names
    public static final String LAUNCHER_MOTOR = "launcher";
    public static final String WINDMILL_SERVO = "windmillServo";

    public static final double TICKS_PER_REV = 28;

    /** Flywheel speed to hold, in encoder ticks per second (1250 = about 2,678 RPM). */
    public static double LAUNCHER_TARGET_VELOCITY = 1250;
    /** The windmill only feeds above this speed (1200 = about 2,571 RPM). */
    public static double LAUNCHER_MIN_VELOCITY = 1200;

    // Velocity PIDF for the flywheel. Changes in Panels are sent to the hub on the next loop.
    public static double LAUNCHER_P = 40;
    public static double LAUNCHER_I = 0;
    public static double LAUNCHER_D = 0;
    public static double LAUNCHER_F = 12.5;

    public static double WINDMILL_POWER = 1.0;
    /** Added to the intake power while the windmill feeds, to help push pieces up out of the hopper. */
    public static double FEED_ASSIST_POWER = 0.5;

    // Auto shots (the fire() command and the "fire" marker)
    /**
     * Longest an Auto shot waits for the flywheel to reach speed. After that the feed time starts
     * anyway, but the windmill still only turns while the flywheel is fast enough.
     */
    public static double SPIN_UP_TIMEOUT_MS = 2000;
    /** How long an Auto shot runs the windmill. Long enough to feed every piece in the hopper. */
    public static double FIRE_FEED_MS = 1500;

    private final DcMotorEx launcher;
    private final CRServo windmill;

    private double targetVelocity = 0;
    private boolean feedRequested = false;
    private boolean feeding = false;
    private final double[] appliedPidf = new double[4];

    public LauncherSubsystem(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, LAUNCHER_MOTOR);
        windmill = hardwareMap.get(CRServo.class, WINDMILL_SERVO);

        // If the speed jumps far past the target with no control, check that the encoder cable is
        // in the port next to the motor's power port, and that the motor polarity is consistent.
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        applyPidf();

        // Reversed, as in the BioBuzz TeleOp, so positive power feeds toward the flywheel.
        windmill.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void applyPidf() {
        appliedPidf[0] = LAUNCHER_P;
        appliedPidf[1] = LAUNCHER_I;
        appliedPidf[2] = LAUNCHER_D;
        appliedPidf[3] = LAUNCHER_F;
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(LAUNCHER_P, LAUNCHER_I, LAUNCHER_D, LAUNCHER_F));
    }

    private boolean pidfChanged() {
        return appliedPidf[0] != LAUNCHER_P || appliedPidf[1] != LAUNCHER_I
                || appliedPidf[2] != LAUNCHER_D || appliedPidf[3] != LAUNCHER_F;
    }

    /** Flywheel speed in ticks per second. */
    public double velocity() {
        return launcher.getVelocity();
    }

    public double rpm() {
        return velocity() / TICKS_PER_REV * 60;
    }

    public boolean isAtSpeed() {
        return velocity() > LAUNCHER_MIN_VELOCITY;
    }

    /** True while the windmill is actually feeding (asked to feed AND the flywheel is fast enough). */
    public boolean isFeeding() {
        return feeding;
    }

    public boolean isSpinning() {
        return targetVelocity > 0;
    }

    /** Set the flywheel target (ticks/second, 0 = off) and whether to feed. Written in {@link #periodic()}. */
    public void set(double velocity, boolean feed) {
        targetVelocity = velocity;
        feedRequested = feed;
    }

    /**
     * For the Launcher Test only: run the windmill at a raw power right now, ignoring the speed
     * check. {@link #periodic()} isn't called in that test.
     */
    public void testWindmill(double power) {
        windmill.setPower(power);
    }

    /** For the Launcher Test only: set the flywheel speed right now. */
    public void testVelocity(double velocity) {
        if (pidfChanged()) applyPidf();
        launcher.setVelocity(velocity);
    }

    @Override
    public void init() {
        // Sends the CR servo a value so the servo controller is enabled, as the BioBuzz TeleOp did.
        windmill.setPower(0);
    }

    @Override
    public void periodic() {
        if (pidfChanged()) applyPidf();
        launcher.setVelocity(targetVelocity);
        // Feed only when asked AND the flywheel is fast enough to make the throw.
        feeding = feedRequested && targetVelocity > 0 && isAtSpeed();
        windmill.setPower(feeding ? WINDMILL_POWER : 0);
    }

    @Override
    public void stop() {
        set(0, false);
        feeding = false;
        launcher.setVelocity(0);
        launcher.setPower(0);
        windmill.setPower(0);
    }

    // ---------------------------------------------------------------- commands

    /**
     * TeleOp, from the BioBuzz StarterBot: hold the button to spin the flywheel up, and the
     * windmill feeds once it reaches LAUNCHER_MIN_VELOCITY. Release to stop both.
     */
    public Command driverControl(BooleanSupplier launch) {
        return infinite(() -> {
            boolean held = launch.getAsBoolean();
            set(held ? LAUNCHER_TARGET_VELOCITY : 0, held);
        })
                .requiring(this)
                .setEnd(endCondition -> set(0, false));
    }

    /** Spin the flywheel up and leave it spinning (to be ready for a shot later in an Auto). */
    public Command spinUp() {
        return instant(() -> set(LAUNCHER_TARGET_VELOCITY, false)).requiring(this);
    }

    /** Flywheel and windmill off. */
    public Command spinDown() {
        return instant(() -> set(0, false)).requiring(this);
    }

    /**
     * Auto: one volley. Spin up (if it isn't already), wait until it's at speed (up to
     * SPIN_UP_TIMEOUT_MS), then feed for FIRE_FEED_MS. The windmill still only turns while the
     * flywheel is fast enough. Leaves the flywheel spinning; use {@link #spinDown()} or the
     * "launcherOff" marker when done shooting.
     *
     * Scheduled on its own, it is ignored if another shot is already running (CANCEL).
     */
    public Command fire() {
        return shot().setConflictBehavior(ConflictBehavior.CANCEL);
    }

    private CommandBuilder shot() {
        return sequential(
                instant(() -> set(LAUNCHER_TARGET_VELOCITY, false)),
                race(waitUntil(this::isAtSpeed), waitMs(SPIN_UP_TIMEOUT_MS)),
                instant(() -> set(LAUNCHER_TARGET_VELOCITY, true)),
                waitMs(FIRE_FEED_MS),
                instant(() -> set(LAUNCHER_TARGET_VELOCITY, false))
        ).requiring(this);
    }

    @Override
    public void registerNamedCommands() {
        NamedCommands.registerCommand("launcherOn", () -> set(LAUNCHER_TARGET_VELOCITY, false));
        NamedCommands.registerCommand("launcherOff", () -> set(0, false));
        // Named commands are plain Runnables, so this one hands the shot to the Ivy scheduler.
        // QUEUE: a marker that comes while another shot is running waits its turn.
        NamedCommands.registerCommand("fire",
                () -> shot().setConflictBehavior(ConflictBehavior.QUEUE).schedule());
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Launcher", "%s  %.0f / %.0f t/s  (%.0f RPM)",
                feeding ? "FEEDING" : isSpinning() ? (isAtSpeed() ? "at speed" : "spinning up") : "off",
                velocity(), targetVelocity, rpm());
    }
}
