package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.IntakeActions;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ServoDegreeController;

import java.util.Arrays;

@Autonomous
public class SpecimenAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(12, -65, Math.toRadians(90)); //update with real starting position
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        IntakeActions.Lift lift = new IntakeActions.Lift(hardwareMap);
        IntakeActions.Intake intake = new IntakeActions.Intake(hardwareMap);

        ServoDegreeController sdc=  new ServoDegreeController(hardwareMap.get(Servo.class,"linkage"),255);
        IntakeActions.Linkage linkage = new IntakeActions.Linkage(sdc);
        //ALL TRAJECTORIES
        int[] samplesPoss = new int[] {48,58,68};
        Vector2d stationPos = new Vector2d(30,-36);
        Vector2d stationPos2 = new Vector2d(48,-42);
        double obsZoneHeading = Math.atan2(-60 - -48, 60 - 36);
        VelConstraint normal = new TranslationalVelConstraint(50);
        //basically once entering the observation slow set max speed to 10in/s
        VelConstraint obszoneslow = (robotPose,_path,_disp) -> {if(robotPose.position.y.value()<-50){return 10;} else{return 60;}};
//        Pose2d initialPose = new Pose2d(12, -60, Math.toRadians(90)); //update with real starting position
        VelConstraint abitslow = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(30),
                new AngularVelConstraint(Math.toRadians(90))
        ));
        VelConstraint slow10 = new TranslationalVelConstraint(10);
        //NOTE TO HUMAN PLAYER: QUICKLY SNATCH FIRST SAMPLE AFTER ROBOT FULLY LEAVES ZONE
        //AND CREATE A SPECIMEN TO HANG ON WALL ABOVE TILE SEAM
        TrajectoryActionBuilder toFirst = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(5,-36),Math.toRadians(90),normal);
        TrajectoryActionBuilder push3 = drive.actionBuilder(new Pose2d(5,-36,Math.toRadians(90)))
                .setTangent(Math.toRadians(-60))
                .splineToConstantHeading(new Vector2d(38,-30),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(48,-10),Math.toRadians(0))
                .strafeTo(new Vector2d(48,-50))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(60,-8),Math.toRadians(45))
                .strafeTo(new Vector2d(60,-50))
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(110))
//                .splineToSplineHeading(new Pose2d(42,-48,Math.atan2(-62 - -48, 60 - 42)),Math.toRadians(-90))
//                        .afterTime(0.2,()->{
//                            System.out.println("Linkage to 6-8 inch");
//                        })
                .splineToLinearHeading(new Pose2d(56,-32

                        ,Math.atan2(-24- -36,samplesPoss[2]-50)),Math.toRadians(90))
                .stopAndAdd(()->sdc.setPosition(68f/255))
                .waitSeconds(0.9)
                .setTangent(Math.toRadians(-75))
                .splineToLinearHeading(new Pose2d(56,-44,Math.atan2(-62 - -48, 60 - 48)),Math.toRadians(-90))
                .stopAndAdd(()->sdc.setPosition(0.0))
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(-90))
                .stopAndAdd(lift.liftUp(470))
                .splineToSplineHeading(new Pose2d(56,-50,Math.toRadians(-90)),Math.toRadians(-90))
                .strafeTo(new Vector2d(56,-55),slow10)
                ;
        TrajectoryActionBuilder cycle = push3.endTrajectory().fresh()
                .setTangent(Math.toRadians(110))
                .splineToSplineHeading(new Pose2d(8,-33,Math.toRadians(90.000001)),Math.toRadians(90),normal)
                .setTangent(Math.toRadians(-70))
                .splineToLinearHeading(new Pose2d(40,-54,Math.toRadians(-89.999999)),Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(40,-65,Math.toRadians(-90)),Math.toRadians(-90),obszoneslow)
                .setTangent(Math.toRadians(110))
                .splineToSplineHeading(new Pose2d(4,-33,Math.toRadians(90.000001)),Math.toRadians(90),normal);
//                .setTangent(Math.toRadians(-70))
//                .splineToLinearHeading(new Pose2d(48,-54,Math.toRadians(-89.999999)),Math.toRadians(-90))
//                .splineToSplineHeading(new Pose2d(48,-65,Math.toRadians(-90)),Math.toRadians(-90),obszoneslow)
//                .setTangent(Math.toRadians(110))
//                .splineToSplineHeading(new Pose2d(2,-33,Math.toRadians(90.000001)),Math.toRadians(90),normal)
//                .setTangent(Math.toRadians(-70))
//                .splineToLinearHeading(new Pose2d(48,-54,Math.toRadians(-89.999999)),Math.toRadians(-90))
//                .splineToSplineHeading(new Pose2d(48,-65,Math.toRadians(-90)),Math.toRadians(-90),obszoneslow)
//                .setTangent(Math.toRadians(110))
//                .splineToSplineHeading(new Pose2d(0,-33,Math.toRadians(90.000001)),Math.toRadians(90),normal);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.update();
        Actions.runBlocking(new SequentialAction(
                new InstantAction(()->sdc.setPosition(0.0)),
                toFirst.build(),
                push3.build(),
                cycle.build()
                //                moveForward.build(),
//                samplesToZone.build()
//                cycles.build()
        ));
    }
}
