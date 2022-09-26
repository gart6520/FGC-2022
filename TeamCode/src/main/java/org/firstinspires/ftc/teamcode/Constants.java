package org.firstinspires.ftc.teamcode;


// All calculation must be performed in SI units besides odometry
// For measuring the angle, we must use the radian
public final class Constants {
    public static final class ODOMETRY {
        public static final double INIT_X = 0;
        public static final double INIT_Y = 0;
        public static final double INIT_THETA = 0;

        public static final double ROBOT_WIDTH = 0.45 * 39.3701;
        public static final double ROBOT_LENGTH = 0.48 * 39.3701;
        public static final double ARROW_LENGTH = 2;

        // lf, lb, rf, rb, tracking arrow
        public static final double[] ANGLE_DIRECTIONS =
                {
                        Math.atan2(ROBOT_WIDTH, ROBOT_LENGTH),
                        Math.atan2(ROBOT_WIDTH, ROBOT_LENGTH) + Math.PI / 2,
                        -Math.atan2(ROBOT_WIDTH, ROBOT_LENGTH),
                        -(Math.atan2(ROBOT_WIDTH, ROBOT_LENGTH) + Math.PI / 2),
                        0
                };
        public static final double[] WHEEL_TRANSITIONS =
                {
                        Math.hypot(ROBOT_WIDTH / 2, ROBOT_LENGTH / 2),
                        Math.hypot(ROBOT_WIDTH / 2, ROBOT_LENGTH / 2),
                        Math.hypot(ROBOT_WIDTH / 2, ROBOT_LENGTH / 2),
                        Math.hypot(ROBOT_WIDTH / 2, ROBOT_LENGTH / 2),
                        ARROW_LENGTH
                };
    }

    public static final class SHOOT {
        public static final double KP = 1.3;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double MAX_INTERGRAL = 0;
        public static final double MIN_INTERGRAL = 1;
        public static final double VELOCITY_TOLERANCE = 0.05;
        public static final double POSITION_TOLERANCE = 0.05;
    }

    public static final class ROTATION {
        public static final double ROTATE_KP = 1.3;
        public static final double ROTATE_KI = 0;
        public static final double ROTATE_KD = 0;
        public static final double TOLERANCE = 0.05;
        public static final double INTERGRAL_MIN = 0;
        public static final double INTERGRAL_MAX = 1;
    }

    public static final class PROJECTILE_MOTION {
        public static final double ANGLE_RATE_ERROR = 0;
        public static final double THETA_ERROR = 0;
        public static final double R1 = 0.03;
        public static final double GRAVITATIONAL_ACCEL = 9.81;
        public static final double SINK_HEIGHT = 2.5;
        public static final double SINK_X = 4.12; //3.0023
        public static final double SINK_Y = 4; //3.5
        public static final double SINK_RADIUS = 2 * Math.sqrt(3) / 5;
    }

    public static final class HOOD_CONST {
        public static final double HD_KP = 1.3;
        public static final double HD_KI = 0;
        public static final double HD_KD = 0;

        public static final double P_TOLERANCE = 0.08;
        public static final double V_TOLERANCE = 0.05;
    }

}
