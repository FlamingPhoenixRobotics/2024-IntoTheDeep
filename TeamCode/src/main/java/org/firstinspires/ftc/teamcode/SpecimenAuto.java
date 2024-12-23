package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.IntakeActions;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ServoDegreeController;

@Autonomous
public class SpecimenAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(12, -60, Math.toRadians(90)); //update with real starting position
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        IntakeActions.Lift lift = new IntakeActions.Lift(hardwareMap);
        IntakeActions.Intake intake = new IntakeActions.Intake(hardwareMap);

        ServoDegreeController sdc=  new ServoDegreeController(hardwareMap.get(Servo.class,"linkage"),255);
        IntakeActions.Linkage linkage = new IntakeActions.Linkage(sdc);

        //ALL TRAJECTORIES
        int[] samplesPoss = new int[] {48,58,68};
        Vector2d stationPos = new Vector2d(42,-42);
        Vector2d stationPos2 = new Vector2d(48,-42);
        double obsZoneHeading = Math.atan2(-60 - stationPos.y, 60 - stationPos.x);
        Action pickUp1 = new SequentialAction(
                new ParallelAction(
                        linkage.extendLinkage(10.6),
                        intake.intakeIn()
                ),
                new SleepAction(1000),
                lift.liftUp(0),
                new SleepAction(500),
                intake.intakeStop(),
                lift.liftUp(100),
                linkage.extendLinkage(0)
        );
        Action dropOff = new SequentialAction(
                linkage.extendLinkage(17),
                new SleepAction(1000),
                intake.intakeOut(),
                new SleepAction(500),
                intake.intakeStop(),
                linkage.extendLinkage(0)
        );

        TrajectoryActionBuilder moveForward = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(7,-36),Math.toRadians(90));
        TrajectoryActionBuilder samplesToZone = moveForward.endTrajectory().fresh()
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(stationPos.x,stationPos.y,Math.atan2(-24-stationPos.y,samplesPoss[0]-stationPos.x)),Math.toRadians(0))

                .stopAndAdd(new SequentialAction(
                        new ParallelAction(
                                linkage.extendLinkage(10.6),
                                intake.intakeIn()
                        ),
                        new SleepAction(1000),
                        lift.liftUp(0),
                        new SleepAction(500),
                        intake.intakeStop(),
                        lift.liftUp(100),
                        linkage.extendLinkage(0)
                ))
                .afterTime(1,
                        linkage.extendLinkage(17))

                .turnTo(obsZoneHeading)

                .stopAndAdd(dropOff)

                .turnTo(Math.atan2(-24-stationPos.y,samplesPoss[1]-stationPos.x))

                .stopAndAdd(new SequentialAction(
                        new ParallelAction(
                                linkage.extendLinkage(15.7),
                                intake.intakeIn()
                        ),
                        new SleepAction(1000),
                        lift.liftUp(0),
                        new SleepAction(500),
                        intake.intakeStop(),
                        lift.liftUp(100),
                        linkage.extendLinkage(0)
                ))
                .afterTime(1,
                        linkage.extendLinkage(17))

                .turnTo(obsZoneHeading)
                .stopAndAdd(dropOff)

                .splineToLinearHeading(new Pose2d(stationPos2,Math.atan2(-24-stationPos2.y,samplesPoss[2]-stationPos2.x)),Math.toRadians(0))
                .stopAndAdd(new SequentialAction(
                        new ParallelAction(
                                linkage.extendLinkage(18.5),
                                intake.intakeIn()
                        ),
                        new SleepAction(1000),
                        lift.liftUp(0),
                        new SleepAction(500),
                        intake.intakeStop(),
                        lift.liftUp(100),
                        linkage.extendLinkage(0)
                ))
                .turnTo(Math.atan2(-60-stationPos2.y,60-stationPos2.x))
                .stopAndAdd(dropOff)
                .splineToSplineHeading(new Pose2d(40,-40,Math.toRadians(0)),Math.toRadians(200))
                .splineToLinearHeading(new Pose2d(38,-58,0),Math.toRadians(0));
        TrajectoryActionBuilder cycleTo = samplesToZone.endTrajectory().fresh()
//                .setTangent(Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(48,-60),Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(48,-42,Math.toRadians(-89.99999)),Math.toRadians(0))
//                .strafeTo(new Vector2d(48,-60))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(7,-33,Math.toRadians(90.000001)),Math.toRadians(90));
        TrajectoryActionBuilder cycleBack = cycleTo.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(38,-58,Math.toRadians(0)),Math.toRadians(0));
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.update();
        Actions.runBlocking(new SequentialAction(
                moveForward.build(),
                samplesToZone.build()
        ));
    }
}
