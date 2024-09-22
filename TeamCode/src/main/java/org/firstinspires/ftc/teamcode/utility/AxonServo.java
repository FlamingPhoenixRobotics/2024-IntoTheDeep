package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;


public class AxonServo  {
    ServoImplEx motor;
    AnalogInput feedback;
    double restPos = 0;
    double zeroPosOffset = 0;
    private double range = 255;

    /**
     * Initialize AxonServo
     * @param motor servo for controlling the servo
     * @param potentiometer analog feedback input
     */
    public AxonServo(ServoImplEx motor, AnalogInput potentiometer){
        this.motor = motor;
        this.feedback = potentiometer;
    }
    public AxonServo(ServoImplEx motor){
        this.motor = motor;
    }
    /**
     * Initialize AxonServo
     * @param motor servo for controlling the servo
     * @param potentiometer analog feedback input
     */
    public AxonServo(Servo motor, AnalogInput potentiometer) {
        this.motor = (ServoImplEx) motor;
        this.feedback = potentiometer;
    }
    public void setRange(double range){
        this.range = range;
    }
    public double getRange(){
        return range;
    }
    /**
     * Reset the servo to the rest position
     */
    public void reset(){
        motor.setPosition(0);
    }
    /**
     * set where the zero position is
     * @return position in radians
     */
    public void setZeroPosOffset(double offset){
        zeroPosOffset = offset;
    }
    /**
     * Get the position of the servo in radians
     * @return position in radians
     */
    public double getReading(){
        return feedback.getVoltage()/3.3*Math.toRadians(360);
    }
    /**
     * Get the position of the servo in degrees
     * @return position in radians
     */
    public double getReadingDegrees(){
        return feedback.getVoltage()/3.3*360;
    }
    /**
     * Get the position of the servo between 0 and 1
     * @return position between 0 and 1
     */
    public double getPos() {
        return motor.getPosition() - restPos;
    }
    /**
     * Get the position of the servo between 0 and 1
     * @return position in radians
     */
    public void setPos(double pos){
        motor.setPosition(pos+restPos);
    }
    /**
     * Set the position of the servo in degrees
     * @param pos position in degrees
     */
    public void setPosDegrees(double pos){
        motor.setPosition(pos/range + restPos);
    }
    /**
     * Set the position of the servo in radians
     * @param pos position in radians
     */
    public void setPosRadians(double pos){
        motor.setPosition(Math.toDegrees(pos)/range + restPos);
    }

    /**
     * Calibrate the servo
     * sets the current position to be "0"
     */
    public void calibrate(){
        //PUT THE SERVO IN THE REST POSITION
        restPos = getReading();
    }
    /**
     * Calibrate the servo
     * sets the current position to be "restPos"
     * @param restPos position of the servo when it is at rest
     */
    public void calibrate(double restPos){
        this.restPos = restPos;
    }


}
