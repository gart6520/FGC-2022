package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake {
    private CRServo intakeLeft;
    private CRServo intakeRight;
    private HardwareMap hardwareMap;

    public Intake(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }

    public void init() {
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");

        intakeRight.setDirection(CRServo.Direction.FORWARD);
        intakeLeft.setDirection(CRServo.Direction.REVERSE);
    }

    public void movingIntake(double position) {
        intakeRight.setPower(position);
        intakeLeft.setPower(position);
    }

    public void reverseIntake(DcMotorSimple.Direction left, DcMotorSimple.Direction right) {
        intakeLeft.setDirection(left);
        intakeRight.setDirection(right);
    }
}
