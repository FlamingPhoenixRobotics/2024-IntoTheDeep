package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.List;

public class TeleOpRoadrunner extends OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    MecanumDrive drive;
    IntakeActions.Intake intake = new IntakeActions.Intake(hardwareMap);
    IntakeActions.Lift lift = new IntakeActions.Lift(hardwareMap);
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }
    @Override
    public void loop() {
        telemetry.addData("Status", "Running");
        TelemetryPacket packet = new TelemetryPacket();

        //place your code here
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                gamepad1.right_stick_x
        ));
        if(gamepad1.right_bumper){
            runningActions.add(intake.intakeIn());
        }
        else if (gamepad1.left_bumper){
            runningActions.add(intake.intakeOut());
        }
        else{
            runningActions.add(intake.intakeStop());
        }



        //end of your code
        //run actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
    }
}
