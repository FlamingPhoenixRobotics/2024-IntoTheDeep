package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.HashMap;

public class AsyncMotorRunner {
    DcMotorEx motor, motor2;
    int targetPosition;
    int power;
    boolean positive = true;
    int lastPosition = 0;
//    HashMap<String,Integer> presets = new HashMap<>();

    /**
     * Constructor for AsyncMotorRunner
     * @param motor
     * @param power
     */
    public AsyncMotorRunner(DcMotorEx motor, int power){
        this.motor = motor;
        this.power = power;
    }
    public AsyncMotorRunner(DcMotorEx motor, DcMotorEx motor2, int power){
        this.motor = motor;
        this.motor2 = motor2;
        this.power = power;
    }
    public void setTargetPosition(int target){
        targetPosition = target;
    }
    public void setPower(int power){
        this.power = power;
    }

    public int runNormal(){
        if(motor.getCurrentPosition() < targetPosition){
            motor.setPower(power);
            return power;
        }
        else if (motor.getCurrentPosition() > targetPosition){
            motor.setPower(-power);
            return -power;
        }
        else{
            motor.setPower(0);
            return 0;
        }
    }
    public int runDual(){
        if(motor.getCurrentPosition() < targetPosition){
            motor.setPower(power);
            motor2.setPower(power);
            return power;
        }
        else if (motor.getCurrentPosition() > targetPosition){
            motor.setPower(-power);
            motor2.setPower(-power);
            return -power;
        }
        else{
            motor.setPower(0);
            motor2.setPower(0);
            return 0;
        }
    }
//    public void runAutoDirection(){
//        if(motor.getCurrentPosition() < targetPosition){
//            motor.setPower(power);
//        }
//        else if (motor.getCurrentPosition() > targetPosition){
//            motor.setPower(-power);
//        }
//        else{
//            motor.setPower(0);
//        }
//        lastPosition = motor.getCurrentPosition();
//    }
}
