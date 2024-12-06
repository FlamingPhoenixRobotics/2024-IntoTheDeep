package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.utility.LinkageArm;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    FieldCentricDrive drive;
    DcMotor liftL,liftR;
    LinkageArm arm;
    CRServo intakeL, intakeR;

    Servo linkage;
    double linkPos = 0;
    double liftPos = 0;
    boolean highReached = true;
    boolean reset = false;
    int exp = 1;
    @Override
    public void init() {
        drive = new FieldCentricDrive(hardwareMap, true);
        liftL = hardwareMap.dcMotor.get("liftl");
        liftR = hardwareMap.dcMotor.get("liftr");
        intakeL = hardwareMap.crservo.get("intakeL");
        intakeR = hardwareMap.crservo.get("intakeR");
        linkage = hardwareMap.servo.get("linkage");
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linkage.setPosition(0);

    }

    @Override
    public void loop() {
        //drive.drive(gamepad1,exp);
        liftL.setPower(-gamepad2.right_stick_y);
        liftR.setPower(-gamepad2.right_stick_y);
        if(gamepad1.right_stick_button){
            drive.resetIMU();
        }
        if(gamepad1.left_bumper){
            intakeL.setPower(1);
            intakeR.setPower(-1);
        }
        if(gamepad1.right_bumper){
            intakeL.setPower(-1);
            intakeR.setPower(1);
        }
        if (!gamepad1.left_bumper && !gamepad1.right_bumper){
            intakeL.setPower(0);
            intakeR.setPower(0);
        }
        if(gamepad1.x){
            linkage.setPosition(0.45);
            linkPos = 0.45;
        }
        if(gamepad1.y){
            linkage.setPosition(0);
            linkPos = 0;
        }
        /*if(gamepad2.dpad_up){
            if(linkPos<0.5){
                linkPos+=0.025;
                linkage.setPosition(linkPos);
            }
        }
        if(gamepad2.dpad_down){
            if(linkPos>0.025){
                linkPos-=0.025;
                linkage.setPosition(linkPos);
            }
        }*/
        if(gamepad1.dpad_left){
        }
        if(gamepad1.dpad_right){
        }
        if(gamepad1.left_bumper){
        }
        if(gamepad1.dpad_right){
            highReached = false;
        }
        if(Math.abs(gamepad2.right_stick_y)<0.1){
            liftL.setPower(0.15);
            liftR.setPower(0.15);
        }//-2850
        //-2672
        if(!highReached){
            if (liftPos>-4700){
                liftR.setPower(1);
                liftL.setPower(-1);
            }
            else{
                highReached = true;
            }
        }
        if(linkPos>0.3){
            exp = 3;
        }
        else{
            exp = 1;
        }
        liftPos = liftR.getCurrentPosition();
        telemetry.addData("things: ", liftPos);
        telemetry.update();
    }
}