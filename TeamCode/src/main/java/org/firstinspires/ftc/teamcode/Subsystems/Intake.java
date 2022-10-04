package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Class for controlling the intake
public class Intake {
    private DcMotor intake;
    private HardwareMap hardwareMap;

    public Intake(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }

    // Initialize the intake
    public void init() {
       intake = hardwareMap.get(DcMotor.class, "intake");
       intake.setDirection(DcMotorSimple.Direction.FORWARD);
       intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Set the speed for the intake
    public void suck(double speed) {
        intake.setPower(speed);
    }
}
