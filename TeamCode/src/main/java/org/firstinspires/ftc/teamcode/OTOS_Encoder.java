package org.firstinspires.ftc.teamcode;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

enum OTOS_DIRECTION{
    X,
    Y
}
public class OTOS_Encoder implements Encoder {
    private final SparkFunOTOS otos;
    public OTOS_DIRECTION xOrY;
    double offset;
    public OTOS_Encoder(SparkFunOTOS otos,OTOS_DIRECTION direction,double offset){
        this.otos = otos;
        xOrY = direction;
        this.offset = offset;
    }
    public void initOTOS(){
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);
        otos.resetTracking();
        otos.setOffset(new SparkFunOTOS.Pose2D(0,0,0));
        //calibrate OTOS
        otos.setAngularScalar(1.0);
        otos.setLinearScalar(1.0);
        otos.calibrateImu();
    }
    private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
    @NonNull
    @Override
    public DcMotorController getController() {
        return null;
    }

    @NonNull
    @Override
    public DcMotorSimple.Direction getDirection() {
        return direction;
    }

    @Override
    public void setDirection(@NonNull DcMotorSimple.Direction direction) {
        this.direction = direction;
    }
    private int applyDirection(int x) {
        if (direction == DcMotorSimple.Direction.REVERSE) {
            x = -x;
        }

        return x;
    }

    @NonNull
    @Override
    public PositionVelocityPair getPositionAndVelocity() {
        SparkFunOTOS.Pose2D pose = otos.getPosition();
        SparkFunOTOS.Pose2D velocity = otos.getVelocity();
        int px = (int) (pose.x - offset * sin(pose.h)) * 10000;
        int py = (int) (pose.y - offset * cos(pose.h)) * 10000;
        int vx = (int) (velocity.x - offset * sin(velocity.h)) * 10000;
        int vy = (int) (velocity.y - offset * cos(velocity.h)) * 10000;
        if(xOrY == OTOS_DIRECTION.X)
            return new PositionVelocityPair(px, vx, px, vx);
        else
            return new PositionVelocityPair(py, vy, py, vy);
    }
}
