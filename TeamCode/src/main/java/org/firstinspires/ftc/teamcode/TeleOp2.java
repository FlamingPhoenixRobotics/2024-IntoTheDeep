package org.firstinspires.ftc.teamcode;


import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.FieldCentricDrive;

@TeleOp
public class TeleOp2 extends OpMode{
    DcMotor liftl, liftr,hangl, hangr;
    CRServo intakeL, intakeR;
    FieldCentricDrive drive;
    Servo linkage;
    int targetLiftPos;
    boolean runLift = false;
    boolean liftGoUp;
    double linkagePos = 0;
    @Override
    public void init() {
        drive = new FieldCentricDrive(hardwareMap);
        liftl = hardwareMap.dcMotor.get("liftl");
        liftr = hardwareMap.dcMotor.get("liftr");
        intakeL = hardwareMap.crservo.get("intakeL");
        intakeR = hardwareMap.crservo.get("intakeR");
        linkage = hardwareMap.servo.get("linkage");
        hangl = hardwareMap.dcMotor.get("hang1");
        hangr = hardwareMap.dcMotor.get("hang2");
        liftr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    @Override
    public void loop() {
        //gamepad2 LIFT AND INTAKE CONTROLS
        if(gamepad2.y){
            targetLiftPos = 4500; //high basket
            runLift = true;
        }
        if(gamepad2.x){
            targetLiftPos = 2100;//specimen/low basket
            runLift = true;
        }
        if(gamepad2.a){
            targetLiftPos = 0;
            runLift = true;
        }
        if(gamepad2.b){
            targetLiftPos = 520;
            runLift = true;
        }
        if(gamepad2.dpad_up){
            linkagePos+=0.01;
            linkagePos = Math.min(Math.max(0, linkagePos), 0.6);
        }
        if(gamepad2.dpad_down){
            linkagePos-=0.01;
            linkagePos = Math.min(Math.max(0, linkagePos), 0.6);
        }
        linkage.setPosition(linkagePos);
        if(abs(gamepad2.right_stick_y) > 0.1){
//            liftl.setPower(-Math.pow(gamepad2.right_stick_y, 3));
//            liftr.setPower(-Math.pow(gamepad2.right_stick_y, 3));
            liftl.setPower(-gamepad2.right_stick_y);
            liftr.setPower(-gamepad2.right_stick_y);
            runLift = false;
        } else{
            if(!runLift){
                liftl.setPower(0.115);
                liftr.setPower(0.115);
            }
        }
        if(gamepad2.left_bumper){
            intakeL.setPower(1);
            intakeR.setPower(-1);
        }else if(gamepad2.right_bumper){
            intakeL.setPower(-1);
            intakeR.setPower(1);
        }
        else{
            intakeL.setPower(0);
            intakeR.setPower(0);
        }



        if(runLift){
            // compare positions and determine direction, then run in that direction until reached target
            if(-liftr.getCurrentPosition() < targetLiftPos) {
                if(Math.abs(liftr.getCurrentPosition() - targetLiftPos) > 100) {
                    liftl.setPower(0.8);
                    liftr.setPower(0.8);}
                    else{
                     liftr.setPower(0.4);
                        liftl.setPower(0.4);
                    }
                liftGoUp = true;
            }else if(-liftr.getCurrentPosition() > targetLiftPos) {
                if(Math.abs(liftr.getCurrentPosition() - targetLiftPos) > 100) {
                    liftl.setPower(-0.5);
                    liftr.setPower(-0.5);
                }else{
                    liftl.setPower(-0.1);
                    liftr.setPower(-0.1);
                }
                liftGoUp = false;
            }
            if(liftGoUp){
                if(-liftr.getCurrentPosition() >= targetLiftPos){
                    liftl.setPower(0.115);
                    liftr.setPower(0.115);
                    runLift = false;
                }
            }else{
                if(-liftr.getCurrentPosition() <= targetLiftPos){
                    liftl.setPower(0.115);
                    liftr.setPower(0.115);
                    runLift = false;
                }
            }
        }

        if(gamepad2.left_trigger > 0.2){
            linkage.setPosition(0.45);
            linkagePos = 0.45;
        }
        if(gamepad2.right_trigger > 0.2){
            linkage.setPosition(0);
            linkagePos = 0;
        }
        //gamepad1 drive contrls
        if(gamepad1.left_stick_button){
            drive.resetIMU();
        }
        if(gamepad1.left_trigger>0.3){
            drive.setMultiplier(0.25);
        } else if (gamepad1.right_trigger > 0.3){
            drive.setMultiplier(1);
        }
        else{
            drive.setMultiplier(0.8);
        }
        if(linkagePos > 0.35){
            drive.setMultiplier(0.5);
        }
        if(gamepad1.dpad_up){
            hangl.setPower(-1);
            hangr.setPower(-1);
        }
        else if(gamepad1.dpad_down){
            hangl.setPower(1);
            hangr.setPower(1);
        }else{
            hangl.setPower(0);
            hangr.setPower(0);
        }
        telemetry.addData("liftPos", liftr.getCurrentPosition());
        drive.drive(gamepad1, 1);
    }
}
