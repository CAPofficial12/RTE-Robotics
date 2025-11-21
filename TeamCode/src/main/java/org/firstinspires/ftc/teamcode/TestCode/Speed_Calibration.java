package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Mechanism.System_Init;

@TeleOp(name = "Speed_Calibration")
public class Speed_Calibration extends OpMode {
    System_Init system_init = new System_Init();
    DcMotorEx Shooter;
    Servo Servoleft;
    Servo Servoright;
    double shooter_speed;
    double servoPos;

    @Override
    public void init(){
        system_init.init(hardwareMap);
    }


    public void loop(){
        Shooter.setVelocity(shooter_speed);
        Servoleft.setPosition(servoPos);
        Servoright.setPosition(1-servoPos);

        if (gamepad1.dpad_up){
            shooter_speed += 50;
        } else if (gamepad1.dpad_down) {
            shooter_speed -= 50;
        }

        if (gamepad1.left_bumper){
            servoPos += 0.05;
        } else if (gamepad1.right_bumper) {
            servoPos -= 0.05;
        }

        telemetry.addData("Shooter Speed", Shooter.getVelocity());
        telemetry.addData("Target Speed", shooter_speed);
        telemetry.addData("Servo Position", Servoleft.getPosition());
        telemetry.update();
    }
}
