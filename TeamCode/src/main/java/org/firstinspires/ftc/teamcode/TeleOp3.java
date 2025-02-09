package org.firstinspires.ftc.teamcode;


import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.FieldCentricDrive;

@TeleOp
public class TeleOp3 extends OpMode{
    DcMotor liftl, liftr,hangl, hangr;
    CRServo intakeL, intakeR;
    FieldCentricDrive drive;
    Servo linkage;
    int targetLiftPos;
    int exp = 1;
    boolean runLift = false;
    boolean liftGoUp;
    boolean linkStat = false;
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
        liftl.setDirection(DcMotorSimple.Direction.REVERSE);
        liftr.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void loop() {
        //gamepad2 LIFT AND INTAKE CONTROLS
        if(gamepad1.y){
            targetLiftPos = 4500; //high basket
            runLift = true;
        }
        if(gamepad1.x){
            targetLiftPos = 2100;//specimen/low basket
            runLift = true;
        }

        if(abs(gamepad2.right_stick_y) > 0.1){
            liftl.setPower(-gamepad2.right_stick_y);
            liftr.setPower(-gamepad2.right_stick_y);
            runLift = false;
        } else{
            if(!runLift){
                liftl.setPower(0.115);
                liftr.setPower(0.115);
            }
        }
        if(gamepad1.right_bumper){
            intakeL.setPower(1);
            intakeR.setPower(-1);
        }else if(gamepad1.left_bumper){
            intakeL.setPower(-1);
            intakeR.setPower(1);
        }
        else{
            intakeL.setPower(0);
            intakeR.setPower(0);
        }
        if(gamepad1.a){
            linkagePos = 0.6*gamepad1.left_trigger;
            linkStat = true;
        }
        if(gamepad1.b){
            linkStat = false;
        }
        if(linkStat){
            double range = 0.6-linkagePos;
            linkage.setPosition(gamepad1.left_trigger*range+linkagePos);
        }
        else{
            linkage.setPosition(0.6*gamepad1.left_trigger);
        }
        if(runLift){
            // compare positions and determine direction, then run in that direction until reached target
            if(-liftr.getCurrentPosition() < targetLiftPos) {
                if(Math.abs(liftr.getCurrentPosition() - targetLiftPos) > 100) {
                    liftl.setPower(1);
                    liftr.setPower(1);}
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

        if(gamepad2.x){
            drive.resetIMU();
        }
        /**HANG**/
        if(gamepad2.dpad_up){
            hangl.setPower(-1);
            hangr.setPower(-1);
        }
        else if(gamepad2.dpad_down){
            hangl.setPower(1);
            hangr.setPower(1);
        }else{
            hangl.setPower(0);
            hangr.setPower(0);
        }
        if(linkagePos>0.4){
            exp = 3;
        }
        else{
            exp = 1;
        }
        telemetry.addData("liftPos", liftr.getCurrentPosition());
        drive.drive(gamepad1, exp);
    }
}
