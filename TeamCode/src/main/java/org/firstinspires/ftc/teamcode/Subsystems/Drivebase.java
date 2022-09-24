package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.ROTATION.INTERGRAL_MAX;
import static org.firstinspires.ftc.teamcode.Constants.ROTATION.INTERGRAL_MIN;
import static org.firstinspires.ftc.teamcode.Constants.ROTATION.ROTATE_KD;
import static org.firstinspires.ftc.teamcode.Constants.ROTATION.ROTATE_KI;
import static org.firstinspires.ftc.teamcode.Constants.ROTATION.ROTATE_KP;
import static org.firstinspires.ftc.teamcode.Constants.ROTATION.TOLERANCE;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Drivebase {
    private DcMotorEx mr1;
    private DcMotorEx mr2;
    private DcMotorEx mr3;
    private DcMotorEx mr4;
    private final HardwareMap hardwareMap;
    private final PIDController controller;

    public Drivebase(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.controller = new PIDController(ROTATE_KP, ROTATE_KI, ROTATE_KD);
    }

    public void init() {

        mr1 = hardwareMap.get(DcMotorEx.class, "mr1");
        mr2 = hardwareMap.get(DcMotorEx.class, "mr2");
        mr3 = hardwareMap.get(DcMotorEx.class, "mr3");
        mr4 = hardwareMap.get(DcMotorEx.class, "mr4");

        mr1.setDirection(DcMotorSimple.Direction.FORWARD);
        mr2.setDirection(DcMotorSimple.Direction.FORWARD);
        mr3.setDirection(DcMotorSimple.Direction.REVERSE);
        mr4.setDirection(DcMotorSimple.Direction.FORWARD);

        mr1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mr2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mr3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        mr4.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        controller.setTolerance(TOLERANCE);
        controller.setIntegrationBounds(INTERGRAL_MIN, INTERGRAL_MAX);
    }

    public void tankController(double right, double left) {
        mr1.setPower(left);
        mr2.setPower(left);

        mr3.setPower(right);
        mr4.setPower(right);
    }

    public void arcadeController(double forward, double rotation) {
        double left = forward + rotation;
        double right = forward - rotation;
        double max = Math.max(left, right);
        if(max > 1) {
            left /= max;
            right /= max;
        }
        mr1.setPower(left);
        mr2.setPower(left);
        mr3.setPower(right);
        mr4.setPower(right);
    }

    public int getLeftPosition() {
        return mr1.getCurrentPosition();
    }

    public int getRightPosition() {
        return mr3.getCurrentPosition();
    }

    public double rotateAngle(double angle, double setPoint) {
        controller.setSetPoint(setPoint);
        return controller.calculate(angle);
    }

    public boolean atSetpoint() {
        return controller.atSetPoint();
    }
}
