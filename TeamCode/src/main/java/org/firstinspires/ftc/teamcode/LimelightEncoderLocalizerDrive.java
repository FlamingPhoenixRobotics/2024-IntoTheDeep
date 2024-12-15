package org.firstinspires.ftc.teamcode;



import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

/**
 * Experimental extension of MecanumDrive that uses the Limelight sensor for localization.
 */
public class LimelightEncoderLocalizerDrive extends MecanumDrive {
    public static class Params {

    }

    public Limelight3A limelight;
    public static LimelightEncoderLocalizerDrive.Params PARAMS = new LimelightEncoderLocalizerDrive.Params();
    public IMU imu;
    double headingOffset;

    ElapsedTime timer = new ElapsedTime();
    double lastTime = timer.milliseconds()/1000; // divide by 1000 to convert to seconds
    DriveLocalizer localizer;
    Pose2d lastLLPose = pose;
    Pose2d lastLLVelocity = new Pose2d(0, 0, 0);
    //Pose2d lastLLAcceleration = new Pose2d(0, 0, 0);
    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);

    public LimelightEncoderLocalizerDrive(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();


        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        headingOffset = pose.heading.toDouble();

        // Initialize drive encoder localizer (just in case Limelight cannot find target)
        localizer = new DriveLocalizer();
    }
    @Override
    public PoseVelocity2d updatePoseEstimate() {
        if (lastLLPose != pose) {
            //something else modify pose??
        }
        // RR localizer note:
        // The values are passed by reference, so we create variables first,
        // then pass them into the function, then read from them.
        // Reading acceleration worsens loop times by 1ms,
        // but not reading it would need a custom driver and would break compatibility.
        // The same is true for speed: we could calculate speed ourselves from pose and time,
        // but it would be hard, less accurate, and would only save 1ms of loop time.

        // Update Limelight orientation - Required for MegaTag2
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        // Initialize pose, velocity, and acceleration for reading
        Pose2d LLpose;
        Pose2d LLvelocity; // = new Pose2d(0, 0, 0);
        //Pose2d LLacceleration = new Pose2d(0, 0, 0);

        // Read Limelight data
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            // Get Limelight MegaTag2 position data
            Pose3D botPoseMT2 = result.getBotpose_MT2();

            double x_MT2 = botPoseMT2.getPosition().x;
            double y_MT2 = botPoseMT2.getPosition().y;

            // Delta time for velocity calculation
            double deltatime = timer.milliseconds() - lastTime;
            lastTime = timer.milliseconds()/1000; // divide by 1000 to convert to seconds

            // Calculate pose, velocity, and acceleration
            LLpose = new Pose2d(x_MT2, y_MT2, orientation.getYaw(AngleUnit.RADIANS) - headingOffset);
            LLvelocity = new Pose2d((LLpose.position.x - lastLLPose.position.x) / deltatime,
                    (LLpose.position.y - lastLLPose.position.y) / deltatime,
                    (LLpose.heading.toDouble() - lastLLPose.heading.toDouble()) / deltatime);
            // LLacceleration is not necessary for this implementation!

            /*
            LLacceleration = new Pose2d((LLvelocity.position.x - lastLLVelocity.position.x) / deltatime,
            (LLvelocity.position.y - lastLLVelocity.position.y) / deltatime,
            (LLvelocity.heading.toDouble() - lastLLVelocity.heading.toDouble()) / deltatime);
            lastLLAcceleration = LLacceleration;
            */

            lastLLPose = LLpose;
//            lastLLVelocity = LLvelocity;


            //get pose
            pose = LLpose;
            // RR standard
            poseHistory.add(pose);
            while (poseHistory.size() > 100) {
                poseHistory.removeFirst();
            }

            estimatedPoseWriter.write(new PoseMessage(pose));

            //return new PoseVelocity2d(new Vector2d(Vel.x, Vel.y),Vel.h);
            return new PoseVelocity2d(new Vector2d(LLvelocity.position.x, LLvelocity.position.y), LLvelocity.heading.toDouble());
        } else {
            // If Limelight cannot find target, use drive encoder localizer
            Twist2dDual<Time> twist = localizer.update();
            pose = pose.plus(twist.value());
            lastLLPose = pose;

            poseHistory.add(pose);
            while (poseHistory.size() > 100) {
                poseHistory.removeFirst();
            }

            estimatedPoseWriter.write(new PoseMessage(pose));

            return twist.velocity().value();
        }
    }
}
