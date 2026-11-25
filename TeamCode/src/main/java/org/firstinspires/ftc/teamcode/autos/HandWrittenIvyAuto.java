package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;

/**
 * The same kind of auto written by hand with the subsystem commands, no Turtle Tracer.
 * Good for a first test after tuning: drive 24" forward while the launcher spins up, shoot,
 * then intake on the way back.
 */
@Autonomous(name = "Hand-Written Ivy Auto", group = "Samples", preselectTeleOp = "TeleOp")
public class HandWrittenIvyAuto extends TurtleTracerAuto {
    private final PoseFactory p = PoseFactory.degrees();

    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }

    @Override
    protected Command routine(Robot robot) {
        Pose start = p.of(72, 72, 0);
        Pose forward = p.of(96, 72, 0);
        robot.drive.follower().setPose(start);

        Path driveOut = line(start, forward).constant(Math.toRadians(0));
        Path driveBack = line(forward, start).constant(Math.toRadians(0));

        return sequential(
                robot.launcher.spinUp(),      // spin up while driving, so the shot starts sooner
                robot.drive.follow(driveOut),
                robot.launcher.fire(),        // waits for speed, then feeds for FIRE_FEED_MS
                robot.launcher.spinDown(),
                parallel(
                        robot.drive.follow(driveBack),
                        sequential(
                                robot.intake.run(1.0),
                                waitMs(1500),
                                robot.intake.run(0)
                        )
                )
        );
    }
}
