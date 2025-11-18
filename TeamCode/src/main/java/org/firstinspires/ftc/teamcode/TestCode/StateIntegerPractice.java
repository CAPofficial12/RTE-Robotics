package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class StateIntegerPractice extends OpMode {
    System_Init system_init = new System_Init();
    enum State {
        WaitForA,
        WaitForB,
        WaitForX,
        Finished
    }
    State state = State.WaitForA;

    @Override
    public void init(){
        system_init.init(hardwareMap);
        state = State.WaitForA;
    }

    @Override
    public void loop(){
        telemetry.addData("Cur State", state);
        switch (state) {
            case WaitForA:
                telemetry.addLine("A: Exit State");
                if (gamepad1.a) {
                    state = State.WaitForB;
                }
                break;
            case WaitForB:
                telemetry.addLine("B: Exit State");
                if (gamepad1.b) {
                    state = State.WaitForX;
                }
                break;
            case WaitForX:
                telemetry.addLine("X: Exit State");
                if (gamepad1.x) {
                    state = State.Finished;
                }
                break;
            default:
                telemetry.addLine("State machine finished");
        }
    }
}
