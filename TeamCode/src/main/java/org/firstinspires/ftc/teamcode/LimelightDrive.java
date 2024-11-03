package org.firstinspires.ftc.teamcode;



import static com.acmerobotics.roadrunner.ftc.OTOSKt.OTOSPoseToRRPose;
import static com.acmerobotics.roadrunner.ftc.OTOSKt.RRPoseToOTOSPose;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

/**
 * Experimental extension of MecanumDrive that uses the Limelight sensor for localization.
 */
public class LimelightDrive extends MecanumDrive {
    public static class Params {

    }

    public static LimelightDrive.Params PARAMS = new LimelightDrive.Params();
    private Pose2d lastOtosPose = pose;

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);

    public LimelightDrive(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
        FlightRecorder.write("OTOS_PARAMS",PARAMS);
        //initialize
    }
    @Override
    public PoseVelocity2d updatePoseEstimate() {
        if (lastOtosPose != pose) {
            //something else modify pose??
        }
        // RR localizer note:
        // The values are passed by reference, so we create variables first,
        // then pass them into the function, then read from them.

        // Reading acceleration worsens loop times by 1ms,
        // but not reading it would need a custom driver and would break compatibility.
        // The same is true for speed: we could calculate speed ourselves from pose and time,
        // but it would be hard, less accurate, and would only save 1ms of loop time.

        //get pose
        lastOtosPose = pose;

        // RR standard
        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));

        // RR localizer note:
        // OTOS velocity units happen to be identical to Roadrunners, so we don't need any conversion!
//        return new PoseVelocity2d(new Vector2d(Vel.x, Vel.y),Vel.h);
        return null;
    }


}
