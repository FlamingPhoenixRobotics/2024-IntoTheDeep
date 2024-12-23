package org.firstinspires.ftc.teamcode.old;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
public class SpecimenAuto2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(12, -60, Math.toRadians(90)); //update with real starting position
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        IntakeActions.Lift lift = new IntakeActions.Lift(hardwareMap);
        IntakeActions.Intake intake = new IntakeActions.Intake(hardwareMap);


        TrajectoryActionBuilder moveForward = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(7,-36),Math.toRadians(90));
        TrajectoryActionBuilder samplesToZone = moveForward.endTrajectory().fresh()
                .setTangent(Math.toRadians(-35))
//                        .splineToLinearHeading(new Pose2d(36,-24,Math.toRadians(-90.001)),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(38,-24),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(46,-8),Math.toRadians(0))
                .strafeTo(new Vector2d(46,-56))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(54,-8),Math.toRadians(45))
                .strafeTo(new Vector2d(54,-54))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(61,-8),Math.toRadians(45))
                .strafeTo(new Vector2d(61,-54))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(40,-40,Math.toRadians(45)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(38,-58,Math.toRadians(0)),Math.toRadians(0));
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
