package org.firstinspires.ftc.teamcode;

import android.provider.CalendarContract;
import android.util.Log;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import java.util.List;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvWebcam;

public abstract class AutoBase extends LinearOpMode {
    DcMotor fr;
    DcMotor fl;
    DcMotor br;
    DcMotor bl;
    private Limelight3A limelight;
    IMU imu;






    //BNO055IMU imu;

    public void initialize() {

        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        limelight.start();

        imu.resetYaw();

    }
    public void driveToPoint(double x_start, double y_start, double x_target, double y_target, double speed, double desiredHeading) {
        double deltaX = x_target - x_start;
        double deltaY = y_target - y_start;

        double distance = Math.hypot(deltaX, deltaY);
        double directionX = deltaX / distance;
        double directionY = deltaY / distance;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentHeading = orientation.getYaw();

        double headingError = desiredHeading - currentHeading;

        headingError = (headingError + 360) % 360;
        if (headingError > 180) {
            headingError -= 360;
        }

        double kP = 0.01;
        double headingCorrection = kP * headingError;

        double flPower = directionY + directionX - headingCorrection;
        double blPower = directionY - directionX - headingCorrection;
        double frPower = directionY - directionX + headingCorrection;
        double brPower = directionY + directionX + headingCorrection;

        double maxPower = Math.max(Math.abs(flPower), Math.abs(blPower));
        maxPower = Math.max(maxPower, Math.abs(frPower));
        maxPower = Math.max(maxPower, Math.abs(brPower));
        if (maxPower > 1.0) {
            flPower /= maxPower;
            blPower /= maxPower;
            frPower /= maxPower;
            brPower /= maxPower;
        }

        fl.setPower(flPower * speed);
        bl.setPower(blPower * speed);
        fr.setPower(frPower * speed);
        br.setPower(brPower * speed);

        while (!hasReachedTarget(x_target, y_target)) {
            orientation = imu.getRobotYawPitchRollAngles();
            currentHeading = orientation.getYaw();

            headingError = desiredHeading - currentHeading;

            headingError = (headingError + 360) % 360;
            if (headingError > 180) {
                headingError -= 360;
            }

            headingCorrection = kP * headingError;

            flPower = directionY + directionX - headingCorrection;
            blPower = directionY - directionX - headingCorrection;
            frPower = directionY - directionX + headingCorrection;
            brPower = directionY + directionX + headingCorrection;

            maxPower = Math.max(Math.abs(flPower), Math.abs(blPower));
            maxPower = Math.max(maxPower, Math.abs(frPower));
            maxPower = Math.max(maxPower, Math.abs(brPower));
            if (maxPower > 1.0) {
                flPower /= maxPower;
                blPower /= maxPower;
                frPower /= maxPower;
                brPower /= maxPower;
            }

            fl.setPower(flPower * speed);
            bl.setPower(blPower * speed);
            fr.setPower(frPower * speed);
            br.setPower(brPower * speed);
        }

        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }

    private boolean hasReachedTarget(double x_target, double y_target) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));

        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();
        double x = 0;
        double y = 0;

        if (result != null) {
            if (result.isValid()) {
                Pose3D botPoseMT1 = result.getBotpose();

                x = 39.3701*botPoseMT1.getPosition().x;
                y = 39.3701*botPoseMT1.getPosition().y;

            }
        }
        else{
            return false;
        }

        double tolerance = 1.0;

        return (Math.abs(x - x_target) < tolerance && Math.abs(y - y_target) < tolerance);
    }



}