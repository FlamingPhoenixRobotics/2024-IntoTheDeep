package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utility.FieldCentricDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    FieldCentricDrive drive;
    @Override
    public void init() {
        drive = new FieldCentricDrive(hardwareMap, true);
    }

    @Override
    public void loop() {
        drive.drive(gamepad1);



        if(gamepad1.a){
        }
        if(gamepad1.b){
        }
        if(gamepad1.x){
        }
        if(gamepad1.y){
        }
        if(gamepad1.dpad_up){
        }
        if(gamepad1.dpad_down){
        }
        if(gamepad1.dpad_left){
        }
        if(gamepad1.dpad_right){
        }
        if(gamepad1.left_bumper){
        }
        if(gamepad1.right_bumper){
        }

    }
}
