package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name = "Test")
public class Test extends OpMode {
    //    private DcMotor testMotor;
//    private IMU imu;
    private CRServo intakeLeft;
    private CRServo intakeRight;
    //private Drivebase drivebase;
    private DcMotor mr1;
    private DcMotor mr2;
    private DcMotor mr3;
    private DcMotor mr4;
    private TouchSensor upLimit;
    private TouchSensor downLimit;
    private boolean atAngle = false;
    @Override
    public void init() {
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");

        upLimit = hardwareMap.get(TouchSensor.class, "upLimit");
        downLimit = hardwareMap.get(TouchSensor.class, "downLimit");

        mr1 = hardwareMap.get(DcMotor.class, "mr1");
        mr2 = hardwareMap.get(DcMotor.class, "mr2");
        mr3 = hardwareMap.get(DcMotor.class, "mr3");
        mr4 = hardwareMap.get(DcMotor.class, "mr4");

        mr1.setDirection(DcMotorSimple.Direction.FORWARD);
        mr2.setDirection(DcMotorSimple.Direction.FORWARD);
        mr3.setDirection(DcMotorSimple.Direction.REVERSE);
        mr4.setDirection(DcMotorSimple.Direction.FORWARD);

        mr1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mr2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mr3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mr4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeRight.setDirection(CRServo.Direction.FORWARD);
        intakeLeft.setDirection(CRServo.Direction.REVERSE);

    }

    @Override
    public void loop() {
        double velocity = 0;
        double speed = 0;
        double angle = 0;
        double left = gamepad1.left_stick_y;
        double right = gamepad1.right_stick_y;
        if(upLimit.isPressed()) {
            intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        else if(downLimit.isPressed()) {
            intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if(gamepad1.triangle) {
            angle = 1;
        }
        else if(gamepad1.cross) {
            angle = -1;
        }
        intakeRight.setPower(angle);
        intakeLeft.setPower(angle);
//        climber.climb(angle);

//        if(atAngle) {
//            angle = 0.8;
//            servo.setPosition(angle);
//        }
//        telemetry.addData("Position y", imu.getPosition());
//        telemetry.update();
//        double speed = gamepad1.left_stick_y;
//        if (gamepad1.circle) {
//            testMotor.setPower(speed);
//        }
        mr1.setPower(left);
        mr2.setPower(left);
        mr3.setPower(right);
        mr4.setPower(right);
        telemetry.addData("Speed", angle);
        telemetry.addData("left", left);
        telemetry.addData("right", right);
        telemetry.update();
    }
}
