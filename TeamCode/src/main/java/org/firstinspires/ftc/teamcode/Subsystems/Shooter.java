package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.SHOOT.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Our class to control the shooter. Our shooter using 2 dc motor and a pid controller to reach a certain velocity.
public class Shooter {
    private DcMotorEx sr1;
    private PIDController controller;
    private HardwareMap hardwareMap;

    public Shooter(OpMode opMode) {
        controller = new PIDController(KP, KI, KD);
        hardwareMap = opMode.hardwareMap;
    }

    public void init() {
        sr1 = hardwareMap.get(DcMotorEx.class, "sr1");

        sr1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sr1.setDirection(DcMotorSimple.Direction.REVERSE);

        // PID controller for shooting
        controller.setTolerance(POSITION_TOLERANCE, VELOCITY_TOLERANCE);
        controller.setIntegrationBounds(MIN_INTERGRAL, MAX_INTERGRAL);
    }

    // Both motors move with the same speed and direction.
    public void shoot(double velocity) {
        sr1.setPower(velocity);
    }

    // Return the angle rates of the motor using encoder.
    public double getVelocity() {
        return sr1.getVelocity(AngleUnit.RADIANS);
    }

    // Use the pid controller to allow the shooter to reach the needed velocity with high accuracy
    public double calculate(double position, double measurement) {
        return controller.calculate(measurement, position);
    }
}
