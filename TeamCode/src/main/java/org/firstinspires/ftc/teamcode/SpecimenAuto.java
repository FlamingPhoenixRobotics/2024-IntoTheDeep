package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
        Vector2d stationPos = new Vector2d(30,-36);
        Vector2d stationPos2 = new Vector2d(48,-42);
        double obsZoneHeading = Math.atan2(-60 - -48, 60 - 36);
        VelConstraint normal = new TranslationalVelConstraint(60);
        //basically once entering the observation slow set max speed to 10in/s
        VelConstraint obszoneslow = (robotPose,_path,_disp) -> {if(robotPose.position.y.value()<-60){return 10;} else{return 60;}};
//        Pose2d initialPose = new Pose2d(12, -60, Math.toRadians(90)); //update with real starting position

        //NOTE TO HUMAN PLAYER: QUICKLY SNATCH FIRST SAMPLE AFTER ROBOT FULLY LEAVES ZONE
        //AND CREATE A SPECIMEN TO HANG ON WALL ABOVE TILE SEAM
        TrajectoryActionBuilder allActions = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(5,-33),Math.toRadians(90),normal)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(18,-42,Math.atan2(-24-stationPos.y,samplesPoss[0]-stationPos.x)+0.5), 0)
                .afterTime(0.9,
                        linkage.extendLinkage(14)
                )
                .splineToSplineHeading(new Pose2d(stationPos.x,stationPos.y,Math.atan2(-24-stationPos.y,samplesPoss[0]-stationPos.x)+0.12),Math.toRadians(45))//linakge to 14 inch
                .waitSeconds(0.1)
                .setTangent(-45)
                .splineToSplineHeading(new Pose2d(36,-48,Math.atan2(-62 - -48, 60 - 36)),Math.toRadians(-45))//linakge to 6-8 inch
                .afterTime(0.1,
                        linkage.extendLinkage(8)
                )
                .splineToLinearHeading(new Pose2d(40,-36,Math.atan2(-24- -36,samplesPoss[1]-40)+0.12),Math.toRadians(75))
                //linkage to 14 inch
                .stopAndAdd(
                        linkage.extendLinkage(14)
                )
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(-75))
                .splineToSplineHeading(new Pose2d(42,-48,Math.atan2(-62 - -48, 60 - 42)),Math.toRadians(-75))
                .afterTime(0.2,
                        linkage.extendLinkage(8)
                )
                .splineToLinearHeading(new Pose2d(46,-36,Math.atan2(-24- -36,samplesPoss[2]-50)),Math.toRadians(45))
                .stopAndAdd(linkage.extendLinkage(14))
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(-75))
                .splineToLinearHeading(new Pose2d(48,-48,Math.atan2(-62 - -48, 60 - 48)),Math.toRadians(-90))
                .waitSeconds(0.1)
                .stopAndAdd(linkage.extendLinkage(0))
                .setTangent(90)
                .splineToSplineHeading(new Pose2d(48,-42,Math.toRadians(-90)),Math.toRadians(90))
//                .waitSeconds(0.4)
                .strafeTo(new Vector2d(48,-65),obszoneslow)


                .setTangent(Math.toRadians(110))
                .splineToSplineHeading(new Pose2d(8,-33,Math.toRadians(90.000001)),Math.toRadians(90),normal)
                .setTangent(Math.toRadians(-70))
                .splineToLinearHeading(new Pose2d(48,-54,Math.toRadians(-90)),Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(48,-65,Math.toRadians(-90)),Math.toRadians(-90),obszoneslow)
                .setTangent(Math.toRadians(110))
                .splineToSplineHeading(new Pose2d(4,-33,Math.toRadians(90.000001)),Math.toRadians(90),normal)
                .setTangent(Math.toRadians(-70))
                .splineToLinearHeading(new Pose2d(48,-54,Math.toRadians(-90)),Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(48,-65,Math.toRadians(-90)),Math.toRadians(-90),obszoneslow)
                .setTangent(Math.toRadians(110))
                .splineToSplineHeading(new Pose2d(2,-33,Math.toRadians(90.000001)),Math.toRadians(90),normal)
                .setTangent(Math.toRadians(-70))
                .splineToLinearHeading(new Pose2d(48,-54,Math.toRadians(-90)),Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(48,-65,Math.toRadians(-90)),Math.toRadians(-90),obszoneslow)
                .setTangent(Math.toRadians(110))
                .splineToSplineHeading(new Pose2d(0,-33,Math.toRadians(90.000001)),Math.toRadians(90),normal);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.update();
        Actions.runBlocking(new SequentialAction(
                linkage.extendLinkage(10),
                lift.liftUp(-150)
//                moveForward.build(),
//                samplesToZone.build()
//                cycles.build()
        ));
    }
}
