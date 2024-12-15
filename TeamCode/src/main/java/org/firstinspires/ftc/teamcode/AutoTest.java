package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.ParallelAction;
@Config
@Autonomous
public class AutoTest extends AutoBase{
    //private Limelight3A limelight;

    IMU imu;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        /*limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();*/
        Pose2d initialPose = new Pose2d(-12, -62, Math.toRadians(90)); //update with real starting position
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        //score preload and intake sample from sub  (numbers aren't exact)
        TrajectoryActionBuilder preload = drive.actionBuilder(initialPose)
                .splineToSplineHeading(new Pose2d(-6,-33, Math.toRadians(90)), Math.toRadians(75))
                .setReversed(true)
                .splineTo(new Vector2d(-52,-52),Math.toRadians(180))

                ;

        /*limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botPoseMT1 = result.getBotpose();
                Pose3D botPoseMT2 = result.getBotpose_MT2();

                double x_MT1 = botPoseMT1.getPosition().x;
                double y_MT1 = botPoseMT1.getPosition().y;

                double x_MT2 = botPoseMT2.getPosition().x;
                double y_MT2 = botPoseMT2.getPosition().y;

                telemetry.addData("MT1 Location:", "(" + x_MT1 + ", " + y_MT1 + ")");
                telemetry.addData("MT2 Location:", "(" + x_MT2 + ", " + y_MT2 + ")");
            }
        }*/
        telemetry.update();
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
