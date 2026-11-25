package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.turtletracerlib.pathing.NamedCommands;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;

/**
 * Base class for every autonomous. A new auto is one small subclass:
 *
 * <pre>
 * &#64;Autonomous(name = "Blue Close", group = "Competition", preselectTeleOp = "TeleOp")
 * public class BlueClose extends TurtleTracerAuto {
 *     protected Alliance alliance() { return Alliance.BLUE; }
 *
 *     protected Command routine(Robot robot) throws Exception {
 *         return new BlueClosePaths(robot.drive.follower(), hardwareMap).command();
 *     }
 * }
 * </pre>
 *
 * where BlueClosePaths is the file Turtle Tracer exported (Export Code, then Sequential, then Ivy).
 * You can also wrap the export with robot actions:
 * {@code sequential(paths.command(), robot.launcher.fire())}.
 *
 * This class handles everything around the routine: Scheduler reset, named commands for event
 * markers, the loop, saving the final pose and path side for TeleOp, and shutting everything off.
 */
public abstract class TurtleTracerAuto extends LinearOpMode {
    protected Robot robot;

    /** The side this Auto's paths are drawn for. TeleOp uses it for the field-centric heading. */
    protected abstract Alliance alliance();

    /**
     * Build the autonomous command. Named commands are already registered when this runs.
     * The Turtle Tracer export sets the starting pose itself.
     */
    protected abstract Command routine(Robot robot) throws Exception;

    @Override
    public void runOpMode() {
        Scheduler.reset();
        // The routine sets the real start pose (Turtle Tracer exports do it in their constructor).
        robot = new Robot(hardwareMap, telemetry, Pose.zero());
        robot.init();
        robot.registerNamedCommands();

        Command routine;
        try {
            routine = withoutRequirements(routine(robot));
        } catch (Exception e) {
            // Usually a missing .turt file in TeamCode/src/main/assets/AutoPaths/
            telemetry.addLine("Could not build the auto:");
            telemetry.addLine(String.valueOf(e));
            telemetry.update();
            waitForStart();
            return;
        }

        // A marker name that doesn't match one of these silently does nothing, so list them.
        String markerNames = String.join(", ", NamedCommands.getAllCommandNames());

        // Keep odometry running during init so you can check the start pose before the match.
        while (opModeInInit()) {
            robot.clearBulkCache();
            robot.drive.periodic();
            telemetry.addData("Paths drawn for", alliance());
            telemetry.addLine("Ready. Nothing moves until START.");
            telemetry.addData("Marker names", markerNames);
            robot.drive.telemetry(telemetry);
            telemetry.update();
        }
        if (isStopRequested()) return;

        robot.start();
        Scheduler.schedule(routine);

        try {
            while (opModeIsActive()) {
                robot.clearBulkCache();
                Scheduler.execute();
                robot.periodic();
                robot.telemetry();
            }
        } finally {
            Robot.saveAutoEnd(robot.drive.pose(), alliance());
            Scheduler.reset();
            robot.stop();
        }
    }

    /**
     * Ivy gives a group every requirement of every command inside it, for the group's whole run.
     * Scheduled directly, an auto that fires the launcher at the end would hold the launcher from
     * the first second, and every "fire" marker before then would be rejected.
     *
     * This runs the same routine but reserves nothing at the top level. The commands
     * inside still work the same; fire() itself ignores a second shot while one is in progress.
     */
    private static Command withoutRequirements(Command routine) {
        return Command.build()
                .setStart(routine::start)
                .setExecute(routine::execute)
                .setDone(routine::done)
                .setEnd(routine::end);
    }
}
