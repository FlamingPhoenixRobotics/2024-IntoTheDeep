package org.firstinspires.ftc.teamcode.old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.old.OTOS_DIRECTION;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
@TeleOp
public class LocalizationTestOTOS extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDriveOTOS drive = new MecanumDriveOTOS(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));


                drive.updatePoseEstimate();
                if(gamepad1.a){
                    Action goback = drive.actionBuilder(drive.pose)
                            .lineToX(12)
                            .build();
                    com.acmerobotics.roadrunner.ftc.Actions.runBlocking(goback);
                }


                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                SparkFunOTOS.Pose2D spose = drive.otos.getPosition();
                telemetry.addData("oX", spose.x);
                telemetry.addData("oY", spose.y);
                OTOS_Encoder XENC = new OTOS_Encoder(drive.otos, OTOS_DIRECTION.X,6.5,-9.5);
                telemetry.addData("xENC",XENC.getPositionAndVelocity().position);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        }

    }

}
