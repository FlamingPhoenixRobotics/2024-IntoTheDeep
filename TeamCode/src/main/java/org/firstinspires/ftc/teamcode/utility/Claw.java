package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Servo claw;
    double[] clawposs;
    double openPos,halfOpenPos, closePos;
    boolean throwError = true;
    /**
     * Initialize Claw
     * @param claw Servo for controlling the claw
     * @param openPos position of the claw when it is open - drop all pixels
     * @param halfOpenPos position of the claw when it is half open - drop one pixel
     * @param closePos position of the claw when it is closed - pick up pixels
     */
    public Claw(Servo claw, double closePos, double halfOpenPos, double openPos){
        this.claw = claw;
        this.openPos = openPos;
        this.halfOpenPos = halfOpenPos;
        this.closePos = closePos;
        clawposs = new double[]{closePos, halfOpenPos, openPos};
        claw.setPosition(openPos);
    }
    /**
     * Initialize Claw
     * @param claw Servo for controlling the claw
     * @param openPos position of the claw when it is open - drop all pixels
     * @param halfOpenPos position of the claw when it is half open - drop one pixel
     * @param closePos position of the claw when it is closed - pick up pixels
     * @param throwError whether or not to throw an error when an invalid position is given in ezSetPos
     */
    public Claw(Servo claw, double closePos, double halfOpenPos, double openPos, boolean throwError){
        this.claw = claw;
        this.openPos = openPos;
        this.halfOpenPos = halfOpenPos;
        this.closePos = closePos;
        this.throwError = throwError;
        clawposs = new double[]{closePos, halfOpenPos, openPos};
        claw.setPosition(openPos);
    }
    public void open(){
        claw.setPosition(openPos);
    }
    public void halfOpen(){
        claw.setPosition(halfOpenPos);
    }
    public void close(){
        claw.setPosition(closePos);
    }
    public void setPos(double pos){
        claw.setPosition(pos);
    }
    public double getPos(){
        return claw.getPosition();
    }
    public void ezSetPos(int posID){
        if(posID > clawposs.length){
            if(throwError) throw new IllegalArgumentException("posID is out of bounds");
        } else {
            claw.setPosition(clawposs[posID]);
        }
    }
    public void ezSetPos(String posName){
        switch(posName){
            case "open":
                open();
                break;
            case "halfOpen":
                halfOpen();
                break;
            case "close":
                close();
                break;
            default:
                if(throwError) throw new IllegalArgumentException("posName is not a valid position name. Valid names are \"open\", \"halfOpen\", and \"close\"");
                else break;
        }
    }

}
