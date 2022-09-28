package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.Hood;

@TeleOp(name = "Test version")
public class Test extends OpMode {
    private DcMotor sr1;
//    private DcMotor sr2;
    private Hood hood;
    @Override
    public void init() {
        sr1 = hardwareMap.get(DcMotor.class, "sr1");
//        sr2 = hardwareMap.get(DcMotor.class, "sr2");
//
        sr1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        sr2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
        sr1.setDirection(DcMotorSimple.Direction.FORWARD);
//        sr2.setDirection(DcMotorSimple.Direction.FORWARD);

        hood = new Hood(this);
        hood.init();
    }

    @Override
    public void loop() {
        double speed = gamepad1.right_stick_y;
        double left = gamepad1.left_stick_y;
//        double right = gamepad1.right_stick_y;

//        sr1.setPower(left);
//        sr2.setPower(right);

//        telemetry.addData("right", right);
//        telemetry.addData("left", left);
        sr1.setPower(left);
        hood.calibrate(speed);
        telemetry.addData("Potentiometer", hood.getPotenVoltage());
        telemetry.addData("Intake", speed);
        telemetry.update();
    }
}
