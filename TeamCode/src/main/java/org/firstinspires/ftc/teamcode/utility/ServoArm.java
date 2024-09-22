package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.Servo;

import org.jetbrains.annotations.NotNull;


public class ServoArm { //12-3: still in progress
    Servo arml, armr;
    double maxPos = 1; //extended
    double minPos = 0; //retracted
    /**
     * @param arml left arm servo
     * @param armr right arm servo
     * @param maxPos extended servo position
     * @param minPos retracted servo position
     */
    public ServoArm(@NotNull Servo arml, @NotNull Servo armr, double minPos, double maxPos){
        this.arml = arml;
        this.armr = armr;
        arml.setPosition(minPos);
        armr.setPosition(1-minPos);
        this.maxPos = maxPos;
        this.minPos = minPos;
    }

    public void setPosition(double pos){
     arml.setPosition(pos);
     armr.setPosition(1-pos);
    }
    public void extend(){
        arml.setPosition(maxPos);
        armr.setPosition(1-maxPos);
    }
    public void retract(){
        arml.setPosition(minPos);
        armr.setPosition(1-minPos);
    }
}
