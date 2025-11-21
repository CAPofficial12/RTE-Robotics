package org.firstinspires.ftc.teamcode.Mechanism;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {
    DcMotorEx Shooter;
    Servo Servoleft;
    Servo Servoright;
    double ShootErrorSum;
    ElapsedTime timerS = new ElapsedTime();
    double speed;
    double hood;
    double lastShooterError;
    double lastShooterPos;

    double Kp_Shoot = Constants_TeleOp.Kp_Shoot, Kd_shoot = Constants_TeleOp.Kd_Shoot, Ki_shoot = Constants_TeleOp.Ki_Shoot, Kf_Shoot = Constants_TeleOp.Kf_Shoot;


    public double SpeedCalc(double distance){
        speed = 5*distance;
        hood = distance/10;
        double dtS = Math.max(timerS.seconds(), 1e-6);
        double velocity = (Shooter.getCurrentPosition() - lastShooterPos)/ timerS.seconds();
        double errorS = speed - velocity;
        ShootErrorSum += errorS * dtS;
        ShootErrorSum = Math.max(-5000, Math.min(5000, ShootErrorSum)); // keep clamped always
        double derivativeS = (errorS - lastShooterError) / dtS;
        lastShooterError = errorS;
        double shoot = (Kp_Shoot * errorS) + (Kd_shoot * derivativeS) + (Ki_shoot * ShootErrorSum) + (Kf_Shoot * speed);
        Shooter.setVelocity(shoot);
        lastShooterPos = Shooter.getCurrentPosition();
        timerS.reset();
        return shoot;
    }
}
