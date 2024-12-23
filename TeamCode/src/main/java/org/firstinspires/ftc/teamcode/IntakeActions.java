package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.ServoDegreeController;
import org.firstinspires.ftc.teamcode.utility.UnitConversions;

import kotlin.Unit;

public class IntakeActions {
    public static class Lift {
        private final DcMotorEx liftL, liftR;

        public Lift(HardwareMap hardwareMap) {
            liftL = hardwareMap.get(DcMotorEx.class, "liftL");
            liftR = hardwareMap.get(DcMotorEx.class, "liftR");
            liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftL.setDirection(DcMotorSimple.Direction.FORWARD);
            liftR.setDirection(DcMotorSimple.Direction.FORWARD); //lol they are reversed electrically. if the wire motoring is correct then change this.
        }

        public class LiftUp implements Action {
            private boolean initialized = false;
            private final int targetPos;
            private boolean up = true;
            public LiftUp(int targetPos){
                this.targetPos = targetPos;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    if(targetPos> liftL.getCurrentPosition()){
                        up = true;
                        liftL.setPower(0.8);
                    } else {
                        up = false;
                        liftL.setPower(-0.8);
                    }
                    initialized = true;
                }

                double pos = liftL.getCurrentPosition();
                if(up && pos > targetPos - 10){
                    //if the lift is going up, stop the lift once it is within 10 ticks of the target position
                    liftL.setPower(0);
                    return false;
                } else if(!up && pos < targetPos + 10){
                    //if the lift is going down, stop the lift once it is within 10 ticks of the target position
                    liftL.setPower(0);
                    return false;
                } else if(Math.abs(pos - targetPos) < 50){
                    //if the lift is is within 50 ticks of the target position, slow down the lift
                    if(up){
                        liftL.setPower(0.2);
                    } else {
                        liftL.setPower(-0.2);
                    }
                    return true;
                }
                if(up && pos > targetPos+50){
                    //if the lift is going up and has passed the target position, stop the lift
                    liftL.setPower(-0.2);
                    return true;
                } else if(!up && pos < targetPos-50){
                    //if the lift is going down and has passed the target position, stop the lift
                    liftL.setPower(0.2);
                    return true;
                }


                packet.put("liftPos", pos);
                if (pos < targetPos) {
                    return true;
                } else {
                    liftL.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp(int targetPos) {
            return new LiftUp(targetPos);
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftL.setPower(-0.8);
                    initialized = true;
                }
                double pos = liftL.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 10.0) {
                    return true;
                } else {
                    liftL.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown() {
            return new LiftDown();
        }
    }
    public static class Intake{
        private final CRServoImplEx intakeL, intakeR;
        public Intake(HardwareMap hardwareMap){
            intakeL = hardwareMap.get(CRServoImplEx.class, "intakeL");
            intakeR = hardwareMap.get(CRServoImplEx.class, "intakeR");
        }
        public class intakeIn implements Action{
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeL.setPower(1);
                    intakeR.setPower(-1);
                    initialized = true;
                }
                return false;
            }
        }
        public Action intakeIn(){
            return new intakeIn();
        }
        public class intakeOut implements Action{
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeL.setPower(-1);
                    intakeR.setPower(1);
                    initialized = true;
                }
                return false;
            }
        }
        public Action intakeOut(){
            return new intakeOut();
        }
        public class intakeStop implements Action{
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeL.setPower(0);
                    intakeR.setPower(0);
                    initialized = true;
                }
                return false;
            }
        }
        public Action intakeStop(){
            return new intakeStop();
        }
        public class intakeInUntil implements Action{
            private boolean initialized = false;
            private final double time;
            private final ElapsedTime timer = new ElapsedTime();
            public intakeInUntil(double time){
                this.time = time;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeL.setPower(1);
                    intakeR.setPower(-1);
                    initialized = true;
                    timer.reset();
                }
                if(timer.milliseconds()>=time){
                    intakeL.setPower(0);
                    intakeR.setPower(0);
                    return true;
                }
                else{
                    return false;
                }
            }
        }
        public Action intakeInUntil(double time){
            return new intakeInUntil(time);
        }
        public class intakeOutUntil implements Action{
            private boolean initialized = false;
            private final double time;
            private final ElapsedTime timer = new ElapsedTime();
            public intakeOutUntil(double time){
                this.time = time;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeL.setPower(-1);
                    intakeR.setPower(1);
                    initialized = true;
                }
                if(timer.milliseconds()>=time){
                    intakeL.setPower(0);
                    intakeR.setPower(0);
                    return true;
                }
                else{
                    return false;
                }
            }
        }
        public Action intakeOutUntil(double time){
            return new intakeOutUntil(time);
        }
    }
    public static class Linkage{
        private ServoImplEx linakgeServo;
        private ServoDegreeController sdc;

        /**
         * Initialize Linkage
         * @param sdc calibrated servo for controlling the linkage.
         */
        public Linkage(ServoDegreeController sdc){
            this.sdc = sdc;
        }
        private static class LinkageMath{
            private final double pinonPitchRadius, linkLen, linkWidth;
            int n;
            public LinkageMath(double pinonPitchRadius, double linkLen, double linkWidth, int stages){
                this.pinonPitchRadius = pinonPitchRadius;
                this.linkLen = linkLen;
                this.n = stages;
                this.linkWidth = linkWidth;
            }
            public double gearRotationToHorizLen(double rotationRadians){
                return pinonPitchRadius*rotationRadians * 2; //times 2 because of symmetry
            }
            public double lenToExtension(double horizLen){
                return n * Math.sqrt(Math.pow(linkLen,2) - Math.pow(horizLen,2));
            }
            public double extensionToLen(double extension){
                return Math.sqrt(Math.pow(linkLen,2) - Math.pow((extension+n*linkWidth)/n,2)); // critical error point - check here first fpr linkage errors.
            }
            public double horizLenToGearRotation(double horizLen){
                return horizLen/pinonPitchRadius;
            }

        }
        public class ExtendLinkage implements Action {
            double length;
            private final double GearRotation;
            LinkageMath lm = new LinkageMath(12.7 * UnitConversions.MM_TO_INCH, 122 * UnitConversions.MM_TO_INCH,12* UnitConversions.MM_TO_INCH, 6);
            public ExtendLinkage(double length){
                this.length = length;
                double horizLen = lm.extensionToLen(length);
                this.GearRotation = lm.horizLenToGearRotation(horizLen);
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                sdc.setPositionDegrees(GearRotation);
                return false;
            }
        }
        public Action extendLinkage(double length){
            return new ExtendLinkage(length);
        }
    }
}
