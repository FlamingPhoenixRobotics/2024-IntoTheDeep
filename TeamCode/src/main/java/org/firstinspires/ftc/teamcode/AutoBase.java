package org.firstinspires.ftc.teamcode;

import android.provider.CalendarContract;
import android.util.Log;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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
import org.firstinspires.ftc.teamcode.utility.LinkageArm;
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
    public float PPR = 537.7f;

    DcMotor liftL,liftR;
    LinkageArm arm;
    CRServo intakeL, intakeR;

    Servo linkage;
    double linkPos = 0;
    double liftPos = 0;



    //BNO055IMU imu;

    public void initialize() {
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
    public void llTune(double endX, double endY){
        
    }

    public void highRaise(){
        while (liftPos>-2770){
            liftR.setPower(1);
            liftL.setPower(1);
            liftPos = liftR.getCurrentPosition();
        }
    }
    public void lowRaise(){
        while (liftPos<-2700){
            liftR.setPower(-1);
            liftL.setPower(-1);
            liftPos = liftR.getCurrentPosition();
        }
    }
    public void outtake(){

    }
    public void Drive(double distance){
        double x = (PPR * distance)/(2 * (float)Math.PI);

        long targetEncoderValue = Math.round(x);

        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = 0;

        while (currentPosition < targetEncoderValue && opModeIsActive()) {
            currentPosition = Math.abs(fr.getCurrentPosition());

            fl.setPower(0.2f);
            fr.setPower(0.2f);
            bl.setPower(0.2f);
            br.setPower(0.2f);
            telemetry.addData("Current Pos %d", currentPosition);
            telemetry.addData("target", targetEncoderValue);
            telemetry.update();
        }
        StopAllWheels();

    }
    public void StopAllWheels(){
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }
    public double llCheckX(LLResult result, double def){
        if (result != null) {
            if (result.isValid()) {
                Pose3D botPoseMT = result.getBotpose_MT2();
                return 39.3701*botPoseMT.getPosition().x;

            }
        }
        return def;
    }
    public double llCheckY(LLResult result, double def){
        if (result != null) {
            if (result.isValid()) {
                Pose3D botPoseMT = result.getBotpose_MT2();
                return 39.3701*botPoseMT.getPosition().y;

            }
        }
        return def;
    }



}