package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Intake {
    private CRServo intakeLeft;
    private CRServo intakeRight;
    private TouchSensor upLimit;
    private TouchSensor downLimit;
    private HardwareMap hardwareMap;

    public Intake(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }

    public void init() {
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");

        upLimit = hardwareMap.get(TouchSensor.class, "upLimit");
        downLimit = hardwareMap.get(TouchSensor.class, "downLimit");

    }

    public void movingIntake(double position) {
        intakeRight.setPower(position);
        intakeLeft.setPower(position);
    }

    public void reverseIntake(DcMotorSimple.Direction left, DcMotorSimple.Direction right) {
        intakeLeft.setDirection(left);
        intakeRight.setDirection(right);
    }

    public boolean getUpLimit() {
        return upLimit.isPressed();
    }

    public boolean getDownLimit() {
        return downLimit.isPressed();
    }

}
