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

import org.firstinspires.ftc.teamcode.utility.LinkageArm;
import org.firstinspires.ftc.teamcode.utility.ServoDegreeController;
import org.firstinspires.ftc.teamcode.utility.UnitConversions;

import kotlin.Unit;

public class IntakeActions {
    public static class Lift {
        private final DcMotorEx liftL, liftR;

        public Lift(HardwareMap hardwareMap) {
            liftL = hardwareMap.get(DcMotorEx.class, "liftl");
            liftR = hardwareMap.get(DcMotorEx.class, "liftr");
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
                    if(targetPos> -liftL.getCurrentPosition()){
                        up = true;
                        liftL.setPower(0.8);
                        liftR.setPower(0.8);
                    } else {
                        up = false;
                        liftL.setPower(-0.8);
                        liftR.setPower(-0.8);
                    }
                    initialized = true;
                }

                double pos = -liftL.getCurrentPosition();
                if(Math.abs(targetPos - pos) < 50){
                    if(up){
                        liftL.setPower(0.2);
                        liftR.setPower(0.2);
                    } else {
                        liftL.setPower(-0.2);
                        liftR.setPower(-0.2);
                    }
                    return true;
                }

                packet.put("liftPos", pos);
                if(Math.abs(targetPos - pos) < 10){
                    liftL.setPower(0);
                    liftR.setPower(0);
                    return true;
                }
                else{
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
        private LinkageArm linkageArm;

        /**
         * Initialize Linkage
         * @param sdc calibrated servo for controlling the linkage.
         */
        public Linkage(ServoDegreeController sdc){
            this.sdc = sdc;
            linkageArm = new LinkageArm(sdc,UnitConversions.mmToIn(257),UnitConversions.mmToIn(302));
        }
        public class ExtendLinkage implements Action {
            double length;
            public ExtendLinkage(double length){
                this.length = length;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                linkageArm.setLen(UnitConversions.mmToIn(length));
                return false;
            }
        }
        public Action extendLinkage(double length){
            return new ExtendLinkage(length);
        }
    }
}
