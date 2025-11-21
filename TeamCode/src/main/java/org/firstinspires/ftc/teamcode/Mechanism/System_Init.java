package org.firstinspires.ftc.teamcode.Mechanism;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class System_Init {
    DcMotorEx backLeftDrive;
    DcMotorEx backRightDrive;
    DcMotorEx intake;
    DcMotorEx shooter;
    Servo Hoodleft;
    Servo Hoodright;
    Limelight3A limelight3A;
    IMU imu;

    public void init (HardwareMap Hwmap){
        DcMotorEx frontLeftDrive = Hwmap.get(DcMotorEx.class, "front_left_drive");
        DcMotorEx frontRightDrive = Hwmap.get(DcMotorEx.class, "front_right_drive");
        backLeftDrive = Hwmap.get(DcMotorEx.class, "back_left_drive");
        backRightDrive = Hwmap.get(DcMotorEx.class, "back_right_drive");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = Hwmap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        intake = Hwmap.get(DcMotorEx.class, "intake");
        shooter = Hwmap.get(DcMotorEx.class, "shooter");
        Hoodleft = Hwmap.get(Servo.class, "Hoodleft");
        Hoodright = Hwmap.get(Servo.class, "Hoodright");
        limelight3A = Hwmap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(6);
    }
}
