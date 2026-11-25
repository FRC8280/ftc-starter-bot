package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.turtletracerlib.pathing.NamedCommands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.Arrays;
import java.util.List;

/**
 * Builds every subsystem and runs the parts of the loop that are the same in TeleOp and Auto.
 *
 * Every OpMode loop should do, in this order:
 * <pre>
 * robot.clearBulkCache();
 * Scheduler.execute();   // commands say what each mechanism should do
 * robot.periodic();      // follower.update(), then the launcher and intake write their hardware
 * robot.telemetry();
 * </pre>
 */
public class Robot {
    /**
     * What the last Auto left for the next TeleOp. Static fields live until the Robot Controller
     * app restarts, so TeleOp clears them once it has started: a second TeleOp run must not start
     * from where the first one ended.
     *
     * pathSide is the side the Auto's paths are drawn for. The pose is in that side's coordinates,
     * so the field-centric heading handoff uses it.
     */
    private static Pose autoEndPose = null;
    private static Alliance autoEndPathSide = null;

    public static void saveAutoEnd(Pose pose, Alliance pathSide) {
        autoEndPose = pose;
        autoEndPathSide = pathSide;
    }

    /** The pose the last Auto ended at, or (0, 0, 0) if there is none. Doesn't clear it. */
    public static Pose autoEndPose() {
        return autoEndPose != null ? autoEndPose : Pose.zero();
    }

    /** The side the last Auto's paths are drawn for, or null if no Auto has been recorded. */
    public static Alliance autoEndPathSide() {
        return autoEndPathSide;
    }

    /**
     * Call after START, not in init, so re-initializing TeleOp (INIT, stop, INIT again)
     * still gets what the Auto recorded.
     */
    public static void clearAutoEnd() {
        autoEndPose = null;
        autoEndPathSide = null;
    }

    public final DriveSubsystem drive;
    public final LauncherSubsystem launcher;
    public final IntakeSubsystem intake;

    private final List<Subsystem> subsystems;
    private final List<LynxModule> hubs;
    private final Telemetry telemetry;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Pose startPose) {
        // Everything Robot reports goes to both the Driver Station and the Panels dashboard.
        this.telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        // Read every motor encoder on a hub in one bulk read per loop instead of one read per call.
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        drive = new DriveSubsystem(hardwareMap, startPose);
        launcher = new LauncherSubsystem(hardwareMap);
        // The intake adds a little power while the launcher feeds (BioBuzz StarterBot).
        intake = new IntakeSubsystem(hardwareMap, launcher::isFeeding);
        // Order matters: the launcher decides if it is feeding before the intake reads it.
        subsystems = Arrays.asList(drive, launcher, intake);
    }

    /**
     * Register every subsystem's Turtle Tracer named commands. Call this BEFORE building an
     * exported auto: the export looks the names up in its constructor.
     */
    public void registerNamedCommands() {
        // NamedCommands is static and outlives the OpMode, so drop the last OpMode's commands
        // (they point at old hardware objects).
        NamedCommands.clearAllCommands();
        for (Subsystem subsystem : subsystems) {
            subsystem.registerNamedCommands();
        }
    }

    /** Call once right after building the Robot, during INIT. Nothing moves. */
    public void init() {
        for (Subsystem subsystem : subsystems) {
            subsystem.init();
        }
    }

    public void clearBulkCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    public void start() {
        for (Subsystem subsystem : subsystems) {
            subsystem.start();
        }
    }

    public void periodic() {
        for (Subsystem subsystem : subsystems) {
            subsystem.periodic();
        }
    }

    public void stop() {
        for (Subsystem subsystem : subsystems) {
            subsystem.stop();
        }
    }

    public void telemetry() {
        for (Subsystem subsystem : subsystems) {
            subsystem.telemetry(telemetry);
        }
        telemetry.update();
    }
}
