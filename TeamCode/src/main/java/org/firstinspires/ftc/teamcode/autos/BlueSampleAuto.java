package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.autos.paths.SampleAutoPaths;
import org.firstinspires.ftc.teamcode.util.Alliance;

/**
 * Runs the example Turtle Tracer export. Copy this file for each new auto and change
 * the name, alliance, and paths class.
 */
@Autonomous(name = "Sample Auto (Blue)", group = "Samples", preselectTeleOp = "TeleOp")
public class BlueSampleAuto extends TurtleTracerAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }

    @Override
    protected Command routine(Robot robot) throws Exception {
        return new SampleAutoPaths(robot.drive.follower(), hardwareMap).command();
    }
}
