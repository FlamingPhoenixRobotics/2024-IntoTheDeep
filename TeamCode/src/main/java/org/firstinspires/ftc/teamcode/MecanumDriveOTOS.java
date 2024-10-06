package org.firstinspires.ftc.teamcode;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public final class MecanumDriveOTOS {
    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // drive model parameters
        public double inPerTick = 1/10000;
        public double lateralInPerTick = inPerTick;
        public double trackWidthTicks = 0;

        // feedforward parameters (in tick units)
        public double kS = 0;
        public double kV = 0;
        public double kA = 0;

        // path profile parameters (in inches)
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double axialGain = 0.0;
        public double lateralGain = 0.0;
        public double headingGain = 0.0; // shared with turn

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // shared with turn
    }

    public static Params PARAMS = new Params();

    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public final VoltageSensor voltageSensor;

    public final LazyImu lazyImu;

    public final Localizer localizer;
    public Pose2d pose;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);


    SparkFunOTOS otos;
    public class OTOSLocalizer implements Localizer {
        SparkFunOTOS otos = MecanumDriveOTOS.this.otos;
        private boolean initialized;
        private Rotation2d lastHeading;
        private int lastX, lastY;
        public Encoder par,perp;
        public final IMU imu;
        private final double sensorDistance = 10.6213228931;

        public OTOSLocalizer(){

            imu = lazyImu.get();
            //reset OTOS
            otos.setLinearUnit(DistanceUnit.INCH);
            otos.setAngularUnit(AngleUnit.RADIANS);
            otos.resetTracking();
            otos.setOffset(new SparkFunOTOS.Pose2D(0,0,0));
            //calibrate OTOS
            otos.setAngularScalar(1.0);
            otos.setLinearScalar(1.0);
            otos.calibrateImu();
            //OTOS virtual encoders
            par = new OTOS_Encoder(otos,OTOS_DIRECTION.Y,10.6213228931);
            perp = new OTOS_Encoder(otos,OTOS_DIRECTION.X,10.6213228931);


        }
        @Override
        public Twist2dDual<Time> update(){
            SparkFunOTOS.Pose2D pose = otos.getPosition();
            SparkFunOTOS.Pose2D vel = otos.getVelocity();
            PositionVelocityPair XPosVel = new PositionVelocityPair((int) pose.x * 10000, (int) vel.x * 10000,(int) pose.x * 10000,(int) vel.x * 10000);
            PositionVelocityPair YPosVel = new PositionVelocityPair((int) pose.y * 10000, (int) vel.y * 10000,(int) pose.y * 10000,(int) vel.y * 10000);
            Rotation2d heading = Rotation2d.exp(pose.h);
            double headingVel = vel.h;
            if(!initialized){
                initialized = true;
                lastHeading = heading;
                lastX = (int) pose.x * 10000;
                lastY = (int) pose.y * 10000;
                return new Twist2dDual<>(
                        Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                        DualNum.constant(0.0, 2)
                );

            }
            int xDelta = (int) pose.x * 10000 - lastX;
            int yDelta = (int) pose.y *10000 - lastY;
            double headingDelta = heading.minus(lastHeading);
            // subtract headingDelta/vel because when the sensor is offset from the center, the rotation affects the x and y
            // we need to find the absolute x and y movement without the rotation
            // With dead wheels, it simply uses the heading to find the effect of the rotation
            // With OTOS, we need to find the effect of the rotation on the x and y with trigonometry
            // The OTOS gets the absolute movement regardless of heading while the dead wheels turn with the robot.
            Twist2dDual<Time> twist = new Twist2dDual<>(
                    new Vector2dDual<>(
                            new DualNum<Time>(new double[] {
                                    xDelta - (sensorDistance * 10000) * sin(headingDelta),
                                    XPosVel.velocity - (sensorDistance * 10000) * sin(headingVel),
                            }).times((double) 1 / 10000),
                            new DualNum<Time>(new double[] {
                                    yDelta - (sensorDistance*10000) * cos(headingDelta),
                                    YPosVel.velocity - (sensorDistance * 10000) * cos(headingVel),
                            }).times((double) 1 / 10000)

                    ),
                    new DualNum<Time>(new double[]{
                            headingDelta,
                            headingVel,
                    })
            );
            lastX = (int)pose.x*10000;
            lastY = (int)pose.y*10000;
            lastHeading = heading;

            return twist;
//9.5x6.5
//forward 9.5 left 6.5
//           Twist2dDual<Time> twist = new Twist2dDual<>(
//                new Vector2dDual<>(
//                        new DualNum<Time>(new double[] {
//                                parPosDelta - PARAMS.parYTicks * headingDelta,
//                                parPosVel.velocity - PARAMS.parYTicks * headingVel,
//                        }).times(inPerTick),
//                        new DualNum<Time>(new double[] {
//                                perpPosDelta - PARAMS.perpXTicks * headingDelta,
//                                perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
//                        }).times(inPerTick)
//                ),
//                new DualNum<>(new double[] {
//                        headingDelta,
//                        headingVel,
//                })
//        );
        }
    }
//    public class DriveLocalizer implements Localizer {
//        public final Encoder leftFront, leftBack, rightBack, rightFront;
//        public final IMU imu;
//
//        private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
//        private Rotation2d lastHeading;
//        private boolean initialized;
//
//        public DriveLocalizer() {
//            leftFront = new OverflowEncoder(new RawEncoder(MecanumDriveOTOS.this.leftFront));
//            leftBack = new OverflowEncoder(new RawEncoder(MecanumDriveOTOS.this.leftBack));
//            rightBack = new OverflowEncoder(new RawEncoder(MecanumDriveOTOS.this.rightBack));
//            rightFront = new OverflowEncoder(new RawEncoder(MecanumDriveOTOS.this.rightFront));
//
//            imu = lazyImu.get();
//
//            // TODO: reverse encoders if needed
//            //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        }
//
//        @Override
//        public Twist2dDual<Time> update() {
//            PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
//            PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
//            PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
//            PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();
//
//            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
//
//            FlightRecorder.write("MECANUM_LOCALIZER_INPUTS", new MecanumLocalizerInputsMessage(
//                    leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles));
//
//            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));
//
//            if (!initialized) {
//                initialized = true;
//
//                lastLeftFrontPos = leftFrontPosVel.position;
//                lastLeftBackPos = leftBackPosVel.position;
//                lastRightBackPos = rightBackPosVel.position;
//                lastRightFrontPos = rightFrontPosVel.position;
//
//                lastHeading = heading;
//
//                return new Twist2dDual<>(
//                        Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
//                        DualNum.constant(0.0, 2)
//                );
//            }
//
//            double headingDelta = heading.minus(lastHeading);
//            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
//                    new DualNum<Time>(new double[]{
//                            (leftFrontPosVel.position - lastLeftFrontPos),
//                            leftFrontPosVel.velocity,
//                    }).times(PARAMS.inPerTick),
//                    new DualNum<Time>(new double[]{
//                            (leftBackPosVel.position - lastLeftBackPos),
//                            leftBackPosVel.velocity,
//                    }).times(PARAMS.inPerTick),
//                    new DualNum<Time>(new double[]{
//                            (rightBackPosVel.position - lastRightBackPos),
//                            rightBackPosVel.velocity,
//                    }).times(PARAMS.inPerTick),
//                    new DualNum<Time>(new double[]{
//                            (rightFrontPosVel.position - lastRightFrontPos),
//                            rightFrontPosVel.velocity,
//                    }).times(PARAMS.inPerTick)
//            ));
//
//            lastLeftFrontPos = leftFrontPosVel.position;
//            lastLeftBackPos = leftBackPosVel.position;
//            lastRightBackPos = rightBackPosVel.position;
//            lastRightFrontPos = rightFrontPosVel.position;
//
//            lastHeading = heading;
//
//            return new Twist2dDual<>(
//                    twist.line,
//                    DualNum.cons(headingDelta, twist.angle.drop(1))
//            );
//        }
//    }

    public MecanumDriveOTOS(HardwareMap hardwareMap, Pose2d pose) {

        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        this.pose = pose;

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: reverse motor directions if needed
        //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
//        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = new OTOSLocalizer();

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            rightFront.setPower(rightFrontPower);

            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));

        return twist.velocity().value();
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }


}
