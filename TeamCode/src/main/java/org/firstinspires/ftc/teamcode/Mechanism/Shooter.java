package org.firstinspires.ftc.teamcode.Mechanism;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Shooter Tuning")
public class Shooter extends OpMode {
    System_Init system_init = new System_Init();

    double[] step = {100,10,1,0.1,0.01,0.001};
    int a = 0;
    int mode = 1;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0;
    double[] target_speeds = {1000,2000};
    double target_speed = target_speeds[1];

    @Override
    public void init() {
        system_init.init(hardwareMap);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(Kp, Ki, Kd, Kf);
        system_init.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper){
            target_speed = target_speeds[1];
        } else if (gamepad1.right_bumper) {
            target_speed = target_speeds[0];
        }
        
        if (gamepad1.dpadUpWasPressed()){
            a += 1;
        } else if (gamepad1.dpadDownWasPressed()) {
            a -= 1;
        }
        
        if (gamepad1.dpad_right){
            mode = 1;
        } else if (gamepad1.dpad_left){
            mode = -1;
        }
        
        if (gamepad1.y) {
            Kp += mode * step[a];
        } else if (gamepad1.b) {
            Ki += mode * step[a];
        } else if (gamepad1.a) {
            Kd += mode * step[a];
        } else if (gamepad1.x) {
            Kf += mode * step[a];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(Kp, Ki, Kd, Kf);
        system_init.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        system_init.shooter.setVelocity(target_speed);

        double currentVelocity = system_init.shooter.getVelocity();
        double error =  target_speed - currentVelocity;

        telemetry.addData("Target Velocity", target_speed);
        telemetry.addData("Current velocity", currentVelocity);
        telemetry.addData("Error", error);
        telemetry.addData("Mode", mode);
        telemetry.addData("Step", step[a]);
        telemetry.addLine("------------------------------------------");
        telemetry.addData("Kp", Kp);
        telemetry.addData("Ki", Ki);
        telemetry.addData("Kd", Kd);
        telemetry.addData("Kf", Kf);
        telemetry.update();
    }
}