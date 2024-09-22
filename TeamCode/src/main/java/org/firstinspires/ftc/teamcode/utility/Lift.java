package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.jetbrains.annotations.NotNull;


public class Lift {
    public int liftposl,liftposr;
    double encoderDifference;
    DcMotor liftl,liftr;
    DcMotor[] liftMotors;
    Gamepad gamepad2;
    private final double holdValue = 0.15;

    private double min = 0;
    private double max = 537;

    public Lift(@NotNull DcMotor l, @NotNull DcMotor r, @NotNull Gamepad gp2){
        liftposl = l.getCurrentPosition();
        liftposr = r.getCurrentPosition();
        liftl = l;
        liftr = r;
        liftl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gamepad2 = gp2;
    }
    public Lift(@NotNull DcMotor l, @NotNull DcMotor r){
        liftposl = l.getCurrentPosition();
        liftposr = r.getCurrentPosition();
        liftl = l;
        liftr = r;
        liftl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public Lift(@NotNull DcMotor l, @NotNull DcMotor r, @NotNull Gamepad gp2, double min, double max){
        liftposl = l.getCurrentPosition();
        liftposr = r.getCurrentPosition();
        liftl = l;
        liftr = r;
        liftl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gamepad2 = gp2;

        this.min = min;
        this.max = max;
        setLiftDualMotorPos(min);
    }
    public Lift(@NotNull DcMotor l, @NotNull DcMotor r, double min, double max){
        liftposl = l.getCurrentPosition();
        liftposr = r.getCurrentPosition();
        liftl = l;
        liftr = r;
        liftl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        this.min = min;
        this.max = max;
        setLiftDualMotorPos(min);
    }
    public Lift(DcMotor[] motors, Gamepad gp2){//WIP
        liftMotors = motors;
    }
    public void setLiftDualMotor(){
        liftposl = liftl.getCurrentPosition();
        liftposr = liftr.getCurrentPosition();
        encoderDifference = liftposl - liftposr;
        double liftlPower,liftrPower = 0;
        if(gamepad2.right_stick_y>0.1){//up
            liftlPower = gamepad2.right_stick_y;
        }else if(gamepad2.right_stick_y<-0.1) {//down
            liftlPower = gamepad2.right_stick_y/3;
        }else{
            liftlPower = holdValue;
        }
        liftrPower = liftlPower - encoderDifference/1000;
        if(liftposl >= max){
            liftlPower = Math.min(liftlPower, holdValue);
            liftrPower = Math.min(liftrPower, holdValue);
        }
        if(liftposl <= min){
            liftlPower = Math.max(liftlPower, holdValue);
            liftrPower = Math.max(liftrPower, holdValue);
        }

        liftl.setPower(liftlPower);
        liftr.setPower(liftrPower);
    }
    public void setLiftDualMotor(double rightStickY){
        liftposl = liftl.getCurrentPosition();
        liftposr = liftr.getCurrentPosition();
        encoderDifference = liftposl - liftposr;
        double liftlPower,liftrPower = 0;
        if(rightStickY>0.1){//up
            liftlPower = rightStickY;
        }else if(rightStickY<-0.1) {//down
            liftlPower = rightStickY/2.5;
        }else{
            liftlPower = holdValue;
        }
        liftrPower = liftlPower - encoderDifference/1000;
        if(liftposl >= max){
            liftlPower = Math.min(liftlPower, holdValue);
            liftrPower = Math.min(liftrPower, holdValue);
        }
        if(liftposl <= min){
            liftlPower = Math.max(liftlPower, holdValue);
            liftrPower = Math.max(liftrPower, holdValue);
        }

        liftl.setPower(liftlPower);
        liftr.setPower(liftrPower);
    }
    public void setLiftDualMotorPos(double pos){
        /**
         * set the lift to a certain encoder position
        @param pos encoder value
        */
        liftposl = liftl.getCurrentPosition();
        liftposr = liftr.getCurrentPosition();
        encoderDifference = liftposl - liftposr;
        double liftlPower,liftrPower = 0;
        if(pos > liftposl) {//up
            liftlPower = 0.28;
        }else if(pos < liftposl) {//down
            liftlPower = -0.1;
        }else{
            liftlPower = holdValue;
        }
        liftl.setPower(liftlPower);
        liftr.setPower(liftrPower + encoderDifference/1000);

    }
    public int getLiftPos(){
        return liftl.getCurrentPosition();
    }
    public void retractDualMotor(){
        while(liftl.getCurrentPosition() < 10){
            liftl.setPower(-0.5);
            liftr.setPower(-0.5);
        }
        liftl.setPower(-0.06);
        liftr.setPower(-0.06);
    }
}
