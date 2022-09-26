package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.IMU;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Loader;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.Odometry;
import org.firstinspires.ftc.teamcode.utils.ProjectileCalculator;

import static org.firstinspires.ftc.teamcode.Constants.ODOMETRY.*;
import static org.firstinspires.ftc.teamcode.Constants.PROJECTILE_MOTION.*;

public class Robot {
    private Loader loader;
    private Drivebase drivebase;
    private Intake intake;
    private IMU imu;
    private Shooter shooter;
    private Hood hood;
    private Odometry odometry;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private Telemetry telemetry;
    private boolean hdAutoMode = false;
    private boolean rotateMode = false;

    public Robot(OpMode opMode) {
        intake = new Intake(opMode);
        drivebase = new Drivebase(opMode);
        loader = new Loader(opMode);
        imu = new IMU(opMode);
        shooter = new Shooter(opMode);
        hood = new Hood(opMode);

        gamepad1 = opMode.gamepad1;
        gamepad2 = opMode.gamepad2;
        telemetry = opMode.telemetry;
    }

    public void init() {
        intake.init();
        drivebase.init();
        loader.init();
        imu.init();
        shooter.init();
        hood.init();

        ProjectileCalculator.init(SINK_HEIGHT);
    }

    public void start() {
        odometry = new Odometry(
                new Pose2d(INIT_X, INIT_Y, new Rotation2d(INIT_THETA)),
                new Rotation2d(imu.getYaw())
        );
    }

    public void loop() {
        double load = 0;
        double ikV = 0;
        double hdV = 0;
        double shoot = 0;
        double MAX_SPEED = 0.8;
        double VoltSetpoint = 0;
        boolean cross = gamepad1.cross;
        DcMotorSimple.Direction ikRight = DcMotorSimple.Direction.FORWARD;
        DcMotorSimple.Direction ikLeft = DcMotorSimple.Direction.REVERSE;
        odometry.update(
                drivebase.getLeftPosition(),
                drivebase.getRightPosition(),
                0,
                new Rotation2d(imu.getYaw())
        );

        ProjectileCalculator.update(odometry.getDistance());

        if(gamepad1.right_trigger > 0.8) {
            MAX_SPEED = 1;
        }

        double left = gamepad1.left_stick_y*MAX_SPEED;
        double right = gamepad1.right_stick_y*MAX_SPEED;

        if(gamepad1.left_stick_button) {
            rotateMode =! rotateMode;
        }

        if(gamepad1.right_stick_button) {
            odometry.resetPosition(0, 0, 0);
        }

        if(cross) {
            ikV = 1;
            if(gamepad1.left_trigger > 0.8) {
                ikV = -ikV;
            }
            else if(gamepad1.right_bumper) {
                if(intake.getUpLimit()) {
                    ikLeft = DcMotorSimple.Direction.REVERSE;
                    ikRight = DcMotorSimple.Direction.FORWARD;
                }

                else if(intake.getDownLimit()) {
                    ikLeft = DcMotorSimple.Direction.FORWARD;
                    ikRight = DcMotorSimple.Direction.REVERSE;
                }
            }
        }

        if(gamepad2.square) {
            load = 1;
        }

        if(gamepad2.cross) {
            shoot = shooter.calculate(ProjectileCalculator.getAngleRate(), shooter.getVelocity());
        }

        if(gamepad2.triangle) {
            hdV = 0.8;
            if(gamepad2.left_trigger > 0.8) {
                hdV = -hdV;
            }
            else if(gamepad2.right_trigger > 0.8) {
                hdAutoMode = true;
            }
        }

        if(hdAutoMode) {
            VoltSetpoint = ProjectileCalculator.getAngle()*Math.PI/180;
            VoltSetpoint = 445.5*(VoltSetpoint - 270)/(Math.pow(VoltSetpoint, 2)-2700*VoltSetpoint-36450);

            hdV = hood.speedCalculator(VoltSetpoint, hood.getPotenVoltage());
            if(hood.atSetpoint()) {
                hdAutoMode = false;
            }
        }

        if(rotateMode) {
            double speed = drivebase.rotateAngle(odometry.getRelativeAngle(), 0);
            left = speed;
            right = -speed;
            if(drivebase.atSetpoint()) {
                rotateMode = false;
            }
        }

        if(gamepad2.left_trigger > 0.8) {
            load = -load;
        }

        intake.reverseIntake(ikLeft, ikRight);
        intake.movingIntake(ikV);
        drivebase.tankController(right, left);
        hood.calibrate(hdV);
        loader.load(load);
        shooter.shoot(shoot);
        telemetry.addData("intake speed", ikV);
        telemetry.addData("Shooter angle rate", shooter.getVelocity());
        telemetry.addData("Hood speed", hdV);
        telemetry.addData("left speed", left);
        telemetry.addData("right speed", right);
        telemetry.addData("Pose x", odometry.getRobotPose().getX());
        telemetry.addData("Pose y", odometry.getRobotPose().getY());
        telemetry.addData("Heading", odometry.getRobotPose().getHeading());
        telemetry.update();
    }
}
