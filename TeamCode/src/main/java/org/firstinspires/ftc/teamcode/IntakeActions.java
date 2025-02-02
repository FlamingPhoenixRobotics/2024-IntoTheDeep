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
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
            liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftL.setDirection(DcMotorSimple.Direction.FORWARD);
            liftR.setDirection(DcMotorSimple.Direction.FORWARD); //lol they are reversed electrically. if the wire motoring is correct then change this.
            liftL.setPower(0.115);
            liftR.setPower(0.115);//keep it with a bit of power
        }
        public class SetPower implements Action{
            double power;
            public SetPower(double power){
                this.power = power;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                liftL.setPower(power);
                liftR.setPower(power);
                return false;
            }
        }
        public Action setPower(double power){
            return new SetPower(power);
        }
        public class LiftUp implements Action {
            private boolean initialized = false;
            private final int targetPos;
            private boolean up = true;
            public LiftUp(int targetPos){
                this.targetPos = -targetPos;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(targetPos < liftR.getCurrentPosition()){
                    liftL.setPower(1);
                    liftR.setPower(1);
                    return true;
                }
                else{
                    liftL.setPower(0.1);
                    liftR.setPower(0.1);
                    return false;
                }
            }
        }
        public Action liftUp(int targetPos) {
            return new LiftUp(targetPos);
        }

        public class LiftDown implements Action {
            private boolean initialized = false;
            private int targetpos;
            public LiftDown(int targetpos){
                this.targetpos = -targetpos;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(targetpos > liftR.getCurrentPosition()){
                    //150
                    //0
                    liftL.setPower(-0.7);
                    liftR.setPower(-0.7);
                    return true;
                }
                else{
                    liftL.setPower(0.1);
                    liftR.setPower(0.1);
                    return false;
                }
            }
        }
        public Action liftDown(int targetpos) {
            return new LiftDown(targetpos);
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
                linkageArm.setLen(length);
                return false;
            }
        }
        public Action extendLinkage(double length){
            return new ExtendLinkage(length);
        }
        public class ExtendLinkagePos implements Action {
            double pos;
            public ExtendLinkagePos(double pos){
                this.pos = pos;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                linkageArm.setPos(pos);
                return false;
            }
        }
        public Action extendLinkagePos(double pos){
            return new ExtendLinkagePos(pos);
        }
    }
}
