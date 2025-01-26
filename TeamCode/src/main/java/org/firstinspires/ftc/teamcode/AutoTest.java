package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
public class AutoTest extends AutoBase{
    //private Limelight3A limelight;
    //first -2770
    //second -2441
    IMU imu;
    @Override
    public void runOpMode() {
        IntakeActions.Lift lift = new IntakeActions.Lift(hardwareMap);
        IntakeActions.Intake intake = new IntakeActions.Intake(hardwareMap);
        ServoDegreeController sdc=  new ServoDegreeController(hardwareMap.get(Servo.class,"linkage"),255);
        IntakeActions.Linkage linkage = new IntakeActions.Linkage(sdc);
        /*limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();*/

        Pose2d initialPose = new Pose2d(-12, -62, Math.toRadians(90)); //update with real starting position
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        //score preload and intake sample from sub  (numbers aren't exact)
        TrajectoryActionBuilder preload = drive.actionBuilder(initialPose)
                .afterTime(0, lift.liftUp(2100))
                .strafeTo(new Vector2d(-6, -31.5))
                .stopAndAdd(lift.liftDown(400))
                .setReversed(true)
                .afterTime(0,intake.intakeOut())
                .splineToConstantHeading(new Vector2d(-57,-46),Math.toRadians(180))
                .stopAndAdd(new SequentialAction(linkage.extendLinkagePos(0.23), new SleepAction(1), lift.liftDown(-35), new SleepAction(1), intake.intakeStop()))
                .setReversed(false)
                .stopAndAdd(linkage.extendLinkagePos(0.1))
                .stopAndAdd(new SleepAction(0.5))
                .afterTime(0, new SequentialAction(lift.liftUp(4500)))
                .strafeToLinearHeading(new Vector2d(-56,-46),Math.toRadians(190))
                .stopAndAdd(new SequentialAction(linkage.extendLinkagePos(0.31),new SleepAction(1),intake.intakeIn(), new SleepAction(0.5f), intake.intakeStop()))
                ;


        /*limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();*/

        waitForStart();


        if (isStopRequested()) return;
        Actions.runBlocking(new ParallelAction(preload.build()));
    }
}
