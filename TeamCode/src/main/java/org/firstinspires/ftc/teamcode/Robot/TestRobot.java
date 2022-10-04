package org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Climber;
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
import static org.firstinspires.ftc.teamcode.Constants.SPEED.*;

public class TestRobot {
    // Generate the subsystems
    private Loader loader;
    private Drivebase drivebase;
    private Intake intake;
    private IMU imu;
    private Shooter shooter;
    private Hood hood;
    private Odometry odometry;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private Climber climber;
    private Telemetry telemetry;
    private boolean testMode = false;
    //    private boolean hdAutoMode = false;
    private boolean autoRotateMode = false;

    public TestRobot(OpMode opMode) {
        intake = new Intake(opMode);
        drivebase = new Drivebase(opMode);
        loader = new Loader(opMode);
        imu = new IMU(opMode);
        shooter = new Shooter(opMode);
        hood = new Hood(opMode);
        climber = new Climber(opMode);

        gamepad1 = opMode.gamepad1;
        gamepad2 = opMode.gamepad2;
        telemetry = opMode.telemetry;
    }

    // Initialize the robot
    public void init() {
        drivebase.init();
        loader.init();
        imu.init();
        shooter.init();
        hood.init();
        intake.init();
        climber.init();

        ProjectileCalculator.init(SINK_HEIGHT);
    }

    public void start() {
        odometry = new Odometry(
                new Pose2d(INIT_X, INIT_Y, new Rotation2d(INIT_THETA)),
                new Rotation2d(imu.getYaw())
        );
    }

    // Run the code continuously
    public void loop() {
        double lrV = 0;
        double ikV = 0;
        double hdV = 0;
        double stV = 0;
        double clbV = 0;
        double MAX_SPEED = NORMAL_DrB;
        double leftDrB = 0;
        double rightDrB = 0;

        odometry.update(
                drivebase.getLeftPosition(),
                drivebase.getRightPosition(),
                0,
                new Rotation2d(imu.getYaw())
        );

        ProjectileCalculator.update(odometry.getDistance());
        ProjectileCalculator.calculate();

        /*
        There are two modes in our program:
        ** The main mode which contains the program using in competitions.
        ** The test mode for testing the mechanisms.

    */
        // Trigger by left bumper of gamepad 1, switching between test and main automatically after each time pressing LB
        if(gamepad2.left_bumper) {
            testMode =! testMode;
        }

        // The test mode for the robot
        if(testMode) {
            // Only one mechanism can be tested at a time
            if(gamepad1.cross) {
                stV = gamepad1.left_stick_y;
            }

            else if(gamepad1.triangle) {
                lrV = gamepad1.left_stick_y;
            }

            else if(gamepad1.circle) {
                ikV = gamepad1.left_stick_y;
            }

            else if(gamepad1.square) {
                hdV = gamepad1.left_stick_y;
            }
            else {
                MAX_SPEED = gamepad1.left_trigger;
                leftDrB = gamepad1.left_stick_y * MAX_SPEED;
                rightDrB = gamepad1.right_stick_y * MAX_SPEED;
            }
        }

        // The main mode to run the robot
        else {
            // Pressing RB will boost the speed of the chassis
            if(gamepad1.right_bumper) {
                MAX_SPEED = BOOST_DrB;
            }

            // Control the speed of the chassis by 2 joysticks on the gamepad
            leftDrB = gamepad1.left_stick_y*MAX_SPEED;
            rightDrB = gamepad1.right_stick_y*MAX_SPEED;

            // Pressing the left stick button will switch between auto rotate mode and manual mode of the robot
            if(gamepad1.left_stick_button) {
                autoRotateMode =!autoRotateMode;
            }

            // Pressing the right stick button of gamepad 1 will reset the position of the odometry,
            // and start a new series of calculation with new position
            if(gamepad1.right_stick_button) {
                odometry.resetPosition(0, 0, 0);
            }

            // Pressing the right trigger of the gamepad 1 until it reaches its end will activate the intake
            // While then, pressing the left bumper will boost the speed of intake
            if(gamepad1.right_trigger > 0.8) {
                ikV = INTAKE;
                if(gamepad1.left_bumper) {
                    ikV = BOOST_Intake;
                }
            }

            // Run the loader at specific speed
            if(gamepad2.circle) {
                lrV = LOADER;
            }

            // Run the shooter at specific speed
            if(gamepad2.triangle) {
                stV = SHOOTER;
            }

            // Run the hood at spedcific speed
            if(gamepad2.cross) {
                hdV = HOOD;
            }

            // Run the climber at specific speed
            if(gamepad2.right_trigger > 0.8) {
                clbV = CLIMBER;
            }

            // During the auto rotate mode, the pid controller will calculate the velocity of the chassis to face the sink
            if(autoRotateMode) {
                double speed = drivebase.rotateAngle(
                        odometry.getRelativeAngle(),
                        0
                );
                leftDrB = speed;
                rightDrB = -speed;
                if(drivebase.atSetpoint()) {
                    autoRotateMode = false;
                }
            }

            // Triggering the L2 will reverse the intake
            if(gamepad1.left_trigger > 0.8) {
                ikV = -ikV;
            }

            // While the devices are working, Triggering the L2 of gamepad 2 will reverse the direction of all of them. (in their manual mode)
            if(gamepad2.left_trigger > 0.8) {
                stV = -stV;
                lrV = -lrV;
                hdV = -hdV;
                clbV = -clbV;
            }
        }

        intake.suck(ikV);
        drivebase.tankController(rightDrB, leftDrB);
        hood.calibrate(hdV);
        loader.load(lrV);
        shooter.shoot(stV);
        climber.climb(clbV);

        // Print the data to the table
        telemetry.addData("Test mode", testMode);
        telemetry.addData("Loader speed", lrV);
        telemetry.addData("Intake speed", ikV);
        telemetry.addData("Shooter angle rate", shooter.getVelocity());
        telemetry.addData("Hood speed", hdV);
        telemetry.addData("left DrB speed", leftDrB);
        telemetry.addData("Right DrB speed", rightDrB);
        telemetry.addData("Max DrB speed", MAX_SPEED);
        telemetry.addData("Climber speed", clbV);
        telemetry.addData("Pose x", odometry.getRobotPose().getX());
        telemetry.addData("Pose y", odometry.getRobotPose().getY());
        telemetry.addData("Heading", odometry.getRobotPose().getHeading());
        telemetry.update();
    }
}
