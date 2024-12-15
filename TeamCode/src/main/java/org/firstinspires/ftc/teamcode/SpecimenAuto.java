package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class SpecimenAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(12, -60, Math.toRadians(90)); //update with real starting position
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action trajectory1 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(7,-36),Math.toRadians(90))
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
//                        .splineToLinearHeading(new Pose2d(36,-24,Math.toRadians(-90.001)),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(38,-24),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(46,-8),Math.toRadians(0))
                .strafeTo(new Vector2d(46,-56))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(57,-8),Math.toRadians(0))
                .strafeTo(new Vector2d(57,-54))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(61,-8),Math.toRadians(45))
                .strafeTo(new Vector2d(61,-54))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(40,-40,Math.toRadians(0)),Math.toRadians(200))
                .splineToConstantHeading(new Vector2d(38,-58),Math.toRadians(0))
//                .setTangent(Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(48,-60),Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(48,-42,Math.toRadians(-89.99999)),Math.toRadians(0))
//                .strafeTo(new Vector2d(48,-60))
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(7,-33,Math.toRadians(90.000001)),Math.toRadians(90))
                .waitSeconds(1)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(38,-58,Math.toRadians(0)),Math.toRadians(0))
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(7,-33,Math.toRadians(90.000001)),Math.toRadians(90))
                .waitSeconds(1)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(38,-58,Math.toRadians(0)),Math.toRadians(0))
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(7,-33,Math.toRadians(90.000001)),Math.toRadians(90))
                .waitSeconds(1)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(38,-58,Math.toRadians(0)),Math.toRadians(0))
                .build();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.update();
        Actions.runBlocking(trajectory1);
    }
}
