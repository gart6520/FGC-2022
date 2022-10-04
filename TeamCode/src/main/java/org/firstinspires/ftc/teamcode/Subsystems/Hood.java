package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.HOOD_CONST.*;

public class Hood {
    private CRServo hood;
    private HardwareMap hardwareMap;
    private PIDController controller;
    private AnalogInput potentiometer;
    public Hood(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
        controller = new PIDController(HD_KP, HD_KI, HD_KD);
    }

    // Initialize the hood and the potentiometer
    public void init() {
        potentiometer = hardwareMap.get(AnalogInput.class, "potent");
        hood = hardwareMap.get(CRServo.class, "hood");
        hood.setDirection(DcMotorSimple.Direction.FORWARD);
        hood.resetDeviceConfigurationForOpMode();

        // PID controller to calculate the angle of the hood
        controller.setIntegrationBounds(0, 1);
        controller.setTolerance(P_TOLERANCE, V_TOLERANCE);
    }

    // Set the hood to run in a specific speed
    public void calibrate(double velocity) {
        hood.setPower(velocity);
    }

    // Calculate the needed speed to reach an angle
    public double speedCalculator(double position, double measurement) {
        return controller.calculate(measurement, position);
    }

    // Return the value of the potentiometer
    public double getPotenVoltage() {
        return potentiometer.getVoltage();
    }

    // Check whether the hood has reached the setpoint
    public boolean atSetpoint() {
        return controller.atSetPoint();
    }
}
