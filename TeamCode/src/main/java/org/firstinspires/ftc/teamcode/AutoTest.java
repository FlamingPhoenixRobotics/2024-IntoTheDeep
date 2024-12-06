package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.ParallelAction;
@Config
@Autonomous
public class AutoTest extends AutoBase{
    @Override
    public void runOpMode() {
        initialize();
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0)); //update with real starting position
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        //score preload and intake sample from sub  (numbers aren't exact)
        TrajectoryActionBuilder preload = drive.actionBuilder(initialPose)
                //.setTangent(90)
                .splineTo(new Vector2d(10,0), Math.toRadians(0))
                // llTune();
                ;
        //score sample (numbers aren't exact)
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                //this needs to be replaced with the output of previous traj
                //.splineToSplineHeading(x,y);
                //llTune();
                ;

        //intake from basket pos
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)//this needs to be replaced with the output of previous traj
                //.splineTo(x,y)
                //llTune();
                //intakeSub();
                ;

        // actions that need to happen on init; for instance, a claw tightening.
        //intake();
        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(new SequentialAction(preload.build())//, lift.raiseSpecimen()
        );
        //intakeSub();
        /*Actions.runBlocking(new ParallelAction(tab2.build())//, lift.raiseLiftSample()
        );
        //outtake();
        Actions.runBlocking(new ParallelAction(tab3.build())//, lift.setIntake()
        );
        //intakeSub();
        Actions.runBlocking(new ParallelAction(tab2.build())//, lift.raiseLiftSample()
        );*/

    }
}
