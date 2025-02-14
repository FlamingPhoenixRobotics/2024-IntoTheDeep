package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SleepAction;
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
import org.firstinspires.ftc.teamcode.IntakeActions;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ServoDegreeController;

import com.acmerobotics.roadrunner.ParallelAction;

import java.util.Vector;

@Config
@Autonomous
public class AutoPreserve extends AutoBase{
    private Limelight3A limelight;
    //first -2770
    //second -2441
    IMU imu;
    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        IntakeActions.Lift lift = new IntakeActions.Lift(hardwareMap);
        IntakeActions.Intake intake = new IntakeActions.Intake(hardwareMap);
        ServoDegreeController sdc=  new ServoDegreeController(hardwareMap.get(Servo.class,"linkage"),255);
        IntakeActions.Linkage linkage = new IntakeActions.Linkage(sdc);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();

        Pose2d initialPose = new Pose2d(-12, -62, Math.toRadians(90)); //update with real starting position
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        //score preload and intake sample from sub  (numbers aren't exact)
        TrajectoryActionBuilder preload = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-6, -30.5))
                ;
        TrajectoryActionBuilder firstIntake = drive.actionBuilder(new Pose2d(-6,-30.5, Math.toRadians(90)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-49,-46, Math.toRadians(90)),Math.toRadians(180))
                .afterTime(0,intake.intakeOut())
                ;
        TrajectoryActionBuilder firstScore = drive.actionBuilder(new Pose2d(-49,-46, Math.toRadians(90)))
                .stopAndAdd(new SequentialAction(linkage.extendLinkagePos(0.33), new SleepAction(1), lift.liftDown(-45), new SleepAction(1), intake.intakeStop()))
                .stopAndAdd(linkage.extendLinkagePos(0.1))
                .stopAndAdd(new SleepAction(0.5))
                .afterTime(0, new SequentialAction(lift.liftUp(4500)))
                .strafeToLinearHeading(new Vector2d(-56,-46),Math.toRadians(215))
                .stopAndAdd(new SequentialAction(new SleepAction(2), linkage.extendLinkagePos(0.22),new SleepAction(1),intake.intakeIn(), new SleepAction(0.5f), intake.intakeStop(), linkage.extendLinkagePos(0), new SleepAction(0.35), lift.liftDown(100)))
                ;
        TrajectoryActionBuilder secondIntake = drive.actionBuilder(new Pose2d(-56,-46, Math.toRadians(215)))
                .strafeToLinearHeading(new Vector2d(-55,-47),Math.toRadians(90))
                .afterTime(0, new ParallelAction(intake.intakeOut()))
                .stopAndAdd(new SequentialAction(linkage.extendLinkagePos(0.31), new SleepAction(1), lift.liftDown(-45), new SleepAction(1), intake.intakeStop()))
                .stopAndAdd(linkage.extendLinkagePos(0.1))
                .stopAndAdd(new SleepAction(0.5))
                .afterTime(0, new SequentialAction(lift.liftUp(4500)))
                .strafeToLinearHeading(new Vector2d(-56,-46),Math.toRadians(215))
                .stopAndAdd(new SequentialAction(new SleepAction(2), linkage.extendLinkagePos(0.22),new SleepAction(1),intake.intakeIn(), new SleepAction(0.5f), intake.intakeStop(), linkage.extendLinkagePos(0), new SleepAction(0.5)))
                ;


        /*limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();*/

        waitForStart();


        if (isStopRequested()) return;
        Actions.runBlocking(new ParallelAction(new SequentialAction(new SleepAction(0.5f), preload.build()), lift.liftUp(2100)));
        Actions.runBlocking(new SequentialAction(lift.liftDown(400)));
        Actions.runBlocking(new SequentialAction(firstIntake.build()));
        limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();
        //drive.pose = new Pose2d(llCheckX(result, -49), llCheckY(result, -47), Math.toRadians(90));
        Actions.runBlocking(new SequentialAction(firstScore.build()));
        Actions.runBlocking(new SequentialAction(secondIntake.build()));

    }
}
