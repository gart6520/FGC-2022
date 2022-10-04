package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Class for controlling the climber, with 2 motors
public class Climber {
    private DcMotor clb1;
    private DcMotor clb2;
    private HardwareMap hardwareMap;
    public Climber(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }

    public void init() {
        clb1 = hardwareMap.get(DcMotor.class, "clb1");
        clb2 = hardwareMap.get(DcMotor.class, "clb2");

        // Each motor will operate in the opposite direction eith the other one,
        // so that the mechanism will move in a single direction
        clb1.setDirection(DcMotorSimple.Direction.FORWARD);
        clb2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set the Brake mode for the motor
        clb1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clb2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Function for climbing
    public void climb(double speed) {
        clb1.setPower(speed);
        clb2.setPower(speed);
    }
}
