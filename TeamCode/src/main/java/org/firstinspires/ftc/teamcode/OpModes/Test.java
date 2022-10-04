package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.TestRobot;

@TeleOp(name = "Test version")
public class Test extends OpMode {
    private TestRobot robot;
    @Override
    public void init() {
       robot = new TestRobot(this);
       robot.init();
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        robot.loop();
    }
}
