package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeActions {
    public static class Lift {
        private DcMotorEx lift;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;
            private int targetPos;
            public LiftUp(int targetPos){
                this.targetPos = targetPos;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < targetPos) {
                    return true;
                } else {
                    lift.setPower(0);
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
                    lift.setPower(-0.8);
                    initialized = true;
                }
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 0.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown() {
            return new LiftDown();
        }
    }
    public static class Intake{
        private CRServoImplEx intakeL;
        private CRServoImplEx intakeR;
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
            private double time;
            public intakeInUntil(double time){
                this.time = time;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeL.setPower(1);
                    intakeR.setPower(-1);
                    initialized = true;
                }
                if (time > 0){
                    time -= 0.02;
                    return false;
                }
                else{
                    intakeL.setPower(0);
                    intakeR.setPower(0);
                    return true;
                }
            }
        }
        public Action intakeInUntil(double time){
            return new intakeInUntil(time);
        }
        public class intakeOutUntil implements Action{
            private boolean initialized = false;
            private double time;
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
                if (time > 0){
                    time -= 0.02;
                    return false;
                }
                else{
                    intakeL.setPower(0);
                    intakeR.setPower(0);
                    return true;
                }
            }
        }
        public Action intakeOutUntil(double time){
            return new intakeOutUntil(time);
        }
    }
}
