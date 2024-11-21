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
    @Override
    public void init() {
        drive = new FieldCentricDrive(hardwareMap, true);
        liftL = hardwareMap.dcMotor.get("liftl");
        liftR = hardwareMap.dcMotor.get("liftr");
        intakeL = hardwareMap.crservo.get("intakeL");
        intakeR = hardwareMap.crservo.get("intakeR");
        linkage = hardwareMap.servo.get("linkage");
    }

    @Override
    public void loop() {
        drive.drive(gamepad1);
        liftL.setPower(gamepad2.right_stick_y);
        liftR.setPower(-gamepad2.right_stick_y);



        if(gamepad1.a){
            intakeL.setPower(- 1);
            intakeR.setPower(-1);
        }
        if(gamepad1.b){
            intakeL.setPower(-1);
            intakeR.setPower(1);
        }
        if(gamepad1.x){
            linkage.setPosition(0.5);
        }
        if(gamepad1.y){

        }
        if(gamepad1.dpad_up){
        }
        if(gamepad1.dpad_down){
        }
        if(gamepad1.dpad_left){
        }
        if(gamepad1.dpad_right){
        }
        if(gamepad1.left_bumper){
        }
        if(gamepad1.right_bumper){
        }

    }
}
