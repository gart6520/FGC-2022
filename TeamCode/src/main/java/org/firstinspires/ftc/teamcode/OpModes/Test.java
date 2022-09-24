package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@TeleOp(name = "Test")
public class Test extends OpMode {

    private CRServo intakeLeft;
    private CRServo intakeRight;
    private Drivebase drivebase;
    private TouchSensor upLimit;
    private TouchSensor downLimit;
    private Intake intake;
    private boolean atAngle = false;
    @Override
    public void init() {
        intake = new Intake(this);
        intake.init();

        upLimit = hardwareMap.get(TouchSensor.class, "upLimit");
        downLimit = hardwareMap.get(TouchSensor.class, "downLimit");

        drivebase = new Drivebase(this);
        drivebase.init();
    }

    @Override
    public void loop() {
        double angle = 0;
        double MAX_SPEED = 0.8;
        if(gamepad1.left_bumper) {
            MAX_SPEED = 1;
        }
        double left = gamepad1.left_stick_y*MAX_SPEED;
        double right = gamepad1.right_stick_y*MAX_SPEED;
        if(upLimit.isPressed()) {
            intake.reverseIntake(DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD);
        }

        else if(downLimit.isPressed()) {
            intake.reverseIntake(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);
        }

        if(gamepad1.triangle) {
            angle = 0.8;
        }
        else if (gamepad1.cross) {
            angle = -0.8;
        }

        intake.movingIntake(angle);
        drivebase.tankController(right, left);
        telemetry.addData("Speed", angle);
        telemetry.addData("left", left);
        telemetry.addData("right", right);
        telemetry.update();
    }
}