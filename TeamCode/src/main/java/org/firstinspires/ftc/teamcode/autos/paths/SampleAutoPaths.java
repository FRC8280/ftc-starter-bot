// EXAMPLE of a Turtle Tracer "Sequential / Ivy" export with hardcoded values and event markers.
// Replace this file with your own export. Poses are placeholders: lay out real paths in Turtle Tracer.

package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.turtletracerlib.pathing.ProgressTracker;
import com.turtletracerlib.pathing.NamedCommands;

import java.io.IOException;

/**
 * Call {@code Scheduler.reset();} in init(), start it with
 * {@code new SampleAutoPaths(follower, hardwareMap).command().schedule();}, and call
 * {@code follower.update();} and {@code Scheduler.execute();} every loop.
 */
public class SampleAutoPaths {

    private final Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private ProgressTracker tracker;

    // Poses
    private Pose startPoint;
    private Pose scorePose;
    private Pose pickupPose;
    private Pose parkPose;

    // Path chains
    private Path scorePreload;
    private Path pickup;
    private Path returnToScore;
    private Path park;

    public SampleAutoPaths(final Follower follower, HardwareMap hw) throws IOException {
        this.follower = follower;

        tracker = new ProgressTracker(follower, null);

        // Load poses
        startPoint = p.of(56.000, 8.000, 90.000);
        scorePose = p.of(60.000, 84.000, 135.000);
        pickupPose = p.of(24.000, 84.000, 180.000);
        parkPose = p.of(38.000, 33.000, 90.000);

        follower.setPose(startPoint);
    }

    public void buildPaths() {
        scorePreload = line(startPoint, scorePose)
            .linear(Math.toRadians(90.0), Math.toRadians(135.0));

        pickup = curve(scorePose, p.of(48.000, 90.000, 0.0), pickupPose)
            .linear(Math.toRadians(135.0), Math.toRadians(180.0));

        returnToScore = line(pickupPose, scorePose)
            .linear(Math.toRadians(180.0), Math.toRadians(135.0));

        park = line(scorePose, parkPose)
            .tangent();
    }

    public Command command() {
        buildPaths();
        return Groups.sequential(
                Commands.instant(() -> {
                        tracker.clearPathEvents();
                        tracker.onParametric(0, 0.050, NamedCommands.getCommand("launcherOn"));
                        tracker.onParametric(0, 0.950, NamedCommands.getCommand("fire"));
                        tracker.setCurrentPath(scorePreload);
                    }),
                Groups.race(
                    PedroCommands.follow(follower, scorePreload),
                    Commands.infinite(() -> tracker.update())
                ),
                Commands.waitMs(2000),
                Commands.instant(() -> {
                        tracker.clearPathEvents();
                        tracker.onParametric(0, 0.100, NamedCommands.getCommand("launcherOff"));
                        tracker.onParametric(0, 0.100, NamedCommands.getCommand("intakeOn"));
                        tracker.setCurrentPath(pickup);
                    }),
                Groups.race(
                    PedroCommands.follow(follower, pickup),
                    Commands.infinite(() -> tracker.update())
                ),
                Commands.instant(() -> {
                        tracker.clearPathEvents();
                        tracker.onParametric(0, 0.500, NamedCommands.getCommand("intakeOff"));
                        tracker.onParametric(0, 0.500, NamedCommands.getCommand("launcherOn"));
                        tracker.onParametric(0, 0.950, NamedCommands.getCommand("fire"));
                        tracker.setCurrentPath(returnToScore);
                    }),
                Groups.race(
                    PedroCommands.follow(follower, returnToScore),
                    Commands.infinite(() -> tracker.update())
                ),
                Commands.waitMs(2000),
                Commands.instant(() -> {
                        tracker.clearPathEvents();
                        tracker.onParametric(0, 0.050, NamedCommands.getCommand("launcherOff"));
                        tracker.setCurrentPath(park);
                    }),
                Groups.race(
                    PedroCommands.follow(follower, park),
                    Commands.infinite(() -> tracker.update())
                )
        );
    }
}
