package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Mechanism.Shooter;
import org.firstinspires.ftc.teamcode.Mechanism.System_Init;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous 3 Segment", group = "Autonomous")
@Configurable // Panels
public class Full_Auto extends OpMode {

    System_Init System_Init = new System_Init();
    Shooter shooter = new Shooter();
    DcMotorEx backLeftDrive;
    DcMotorEx backRightDrive;
    DcMotorEx intake;
    DcMotorEx shoot;
    Servo Hoodleft;
    Servo Hoodright;
    Limelight3A limelight3A;
    IMU imu;

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)

    private double start_X;
    private double start_Y;
    private Pose startPose = new Pose(start_X, start_Y, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose SmallscorePose = new Pose(56, 8, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose BigscorePose = new Pose(45, 98, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose pickup1Pose = new Pose(13, 36, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1PoseReverse = new Pose(34, 36, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup2Pose = new Pose(15, 64, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup2PoseReverse = new Pose(34, 64, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup3Pose = new Pose(13, 86, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup3PoseReverse = new Pose(34, 86, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.

    double currentX, currentY;
    private final Pose Goal = new Pose(5,144);
    private  Pose Current = new Pose(5,144);
    double distance;
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        System_Init.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        YawPitchRollAngles orientatiuon = imu.getRobotYawPitchRollAngles();
        limelight3A.updateRobotOrientation(orientatiuon.getYaw(AngleUnit.DEGREES));
        LLResult result = limelight3A.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                start_X = result.getBotpose().getPosition().x;
                start_Y = result.getBotpose().getPosition().y;
            }
        }
        follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight3A.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult result = limelight3A.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                currentX = result.getBotpose().getPosition().x;
                currentY = result.getBotpose().getPosition().y;
            }
        }
        follower.setStartingPose(startPose);

        distance = Current.distanceFrom(Goal);
        double servoPos = distance/1000;

        Hoodleft.setPosition(servoPos);
        Hoodright.setPosition(1-servoPos);
        double shooterSpeed = shooter.SpeedCalc(distance);
        shoot.setVelocity(shooterSpeed);


        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    PathChain preload,Path1, Path2, Path3, Path4;
    public void Paths(Follower follower) {

        preload = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                startPose,
                                SmallscorePose
                        )
                )
                .setLinearHeadingInterpolation(imu.getRobotYawPitchRollAngles().getYaw(), 110)
                .build();

        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                startPose,
                                new Pose(36.000, 41.000),
                                pickup1Pose
                        )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .addPath(
                            new BezierLine (
                                    pickup1Pose,
                                    SmallscorePose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .build();

            Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                SmallscorePose,
                                new Pose(63.000, 65.000),
                                pickup2Pose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))

                .addPath(
                        new BezierCurve(
                                pickup2Pose,
                                new Pose(61.000, 74.000),
                                BigscorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    BigscorePose,
                                    new Pose(17.000, 103.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-90))

                    .addPath(
                            new BezierCurve(
                                    new Pose(17.000, 103.000),
                                    new Pose(16.000, 79.000),
                                    pickup3PoseReverse
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(45))

                    .addPath(
                            new BezierLine(
                                    pickup3PoseReverse,
                                    BigscorePose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(135))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    BigscorePose,
                                    new Pose(106.000, 33.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                    .build();
        }


    public int autonomousPathUpdate() {
        switch (pathState){
            case 0:
                follower.followPath(preload);
                setPathState(1);
                break;
            case 1:


        }
        return pathState;
    }

    public void setPathState(int pState){
        pathState = pState;
    }
}