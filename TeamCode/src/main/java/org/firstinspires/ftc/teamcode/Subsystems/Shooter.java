package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.SHOOT.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter {
    private DcMotorEx sr1;
    private DcMotorEx sr2;
    private PIDController controller;
    private HardwareMap hardwareMap;

    public Shooter(OpMode opMode) {
        controller = new PIDController(KP, KI, KD);
        hardwareMap = opMode.hardwareMap;
    }

    public void init() {
        sr1 = hardwareMap.get(DcMotorEx.class, "sr1");
        sr2 = hardwareMap.get(DcMotorEx.class, "sr2");

        sr1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sr2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sr1.setDirection(DcMotorSimple.Direction.FORWARD);
        sr2.setDirection(DcMotorSimple.Direction.FORWARD);

        controller.setTolerance(POSITION_TOLERANCE, VELOCITY_TOLERANCE);
        controller.setIntegrationBounds(MIN_INTERGRAL, MAX_INTERGRAL);
    }

    public void shoot(double velocity) {
        sr1.setVelocity(velocity, AngleUnit.RADIANS);
        sr2.setVelocity(velocity, AngleUnit.RADIANS);
    }

    public double getVelocity() {
        return sr1.getVelocity(AngleUnit.RADIANS);
    }

    public double calculate(double position, double measurement) {
        return controller.calculate(measurement, position);
    }
}
