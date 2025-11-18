package frc.team5115;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.List;

public final class Constants {
    private static final boolean isReplay = false;
    public static final Mode currentMode =
            RobotBase.isReal() ? Mode.REAL : (isReplay ? Mode.REPLAY : Mode.SIM);

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final boolean SINGLE_MODE = true;

    public static final double SLOW_MODE_SPEED = 0.4;

    public static final byte PNEUMATIC_HUB_ID = 2;
    public static final byte OUTTAKE_FORWARD_CHANNEL = 9;
    public static final byte OUTTAKE_REVERSE_CHANNEL = 10;

    public static final byte ARM_MOTOR_ID = 11;

    public static final byte INTAKE_MOTOR_ID = 12;
    public static final double INTAKE_SPEED = 0.15;
    public static final double INTAKE_VOMIT_SPEED = -0.22;
    public static final double INTAKE_XFER_SPEED = -Math.PI / 20;

    public static final byte NET_ACTUATOR_1_ID = 9;
    public static final byte NET_ACTUATOR_2_ID = 10;

    public static final byte LUNITE_SENSOR = 1;
    public static final byte LUNITE_SENSOR2 = 2;
    public static final byte LUNITE_SENSOR3 = 3;

    public static final byte LED_STRIP_PWM_ID = 0;

    public static final double LOOP_PERIOD_SECS = 0.02;

    public static final double SENSOR_FILTER_TIME =
            0.2; // Amount of time to wait while the sensor is true before starting the xfer routine

    public static class SwerveConstants {
        public static final byte FRONT_LEFT_DRIVE_ID = 6;
        public static final byte FRONT_RIGHT_DRIVE_ID = 4;
        public static final byte BACK_LEFT_DRIVE_ID = 10;
        public static final byte BACK_RIGHT_DRIVE_ID = 8;

        public static final byte FRONT_LEFT_TURN_ID = 5;
        public static final byte FRONT_RIGHT_TURN_ID = 3;
        public static final byte BACK_LEFT_TURN_ID = 9;
        public static final byte BACK_RIGHT_TURN_ID = 7;

        private static RobotConfig ROBOT_CONFIG = null;

        public static RobotConfig getRobotConfig() {
            if (ROBOT_CONFIG == null) {
                try {
                    ROBOT_CONFIG = RobotConfig.fromGUISettings();
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            return ROBOT_CONFIG;
        }

        public static final double MAX_LINEAR_SPEED = 5; // meters per second

        private static final double TRACK_WIDTH = Units.inchesToMeters(26.25);
        public static final double TRACK_WIDTH_X = TRACK_WIDTH;
        public static final double TRACK_WIDTH_Y = TRACK_WIDTH;
        public static final double DRIVE_BASE_RADIUS =
                Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
        public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.5);

        // Required for inverse kinematics. +x is forward, +y is left
        // The module order, as with everywhere else, is FL, FR, BL, BR
        public static final Translation2d[] MODULE_TRANSLATIONS =
                new Translation2d[] {
                    new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
                    new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / -2.0),
                    new Translation2d(TRACK_WIDTH_X / -2.0, TRACK_WIDTH_Y / 2.0),
                    new Translation2d(TRACK_WIDTH_X / -2.0, TRACK_WIDTH_Y / -2.0)
                };

        public static final Rotation2d FRONT_LEFT_ANGULAR_OFFSET = Rotation2d.fromDegrees(270);
        public static final Rotation2d FRONT_RIGHT_ANGULAR_OFFSET = Rotation2d.fromDegrees(0);
        public static final Rotation2d BACK_LEFT_ANGULAR_OFFSET = Rotation2d.fromDegrees(180);
        public static final Rotation2d BACK_RIGHT_ANGULAR_OFFSET = Rotation2d.fromDegrees(90);

        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear
        // 15 teeth on the bevel pinion, 13 teeth on the driving motor
        public static final double DrivingMotorReduction = (45.0 * 22.0) / (13.0 * 15.0);

        public static final int DrivingMotorAutoCurrentLimit = 60; // amp
        public static final int DrivingMotorTeleopCurrentLimit = 50; // amps, lower than in auto
        public static final int TurningMotorCurrentLimit = 20; // amps
    }

    public static class AutoConstants {
        private static final List<Pose2d> leftScoringPoses = new ArrayList<Pose2d>();
        private static final List<Pose2d> rightScoringPoses = new ArrayList<Pose2d>();
        private static final List<Pose2d> centerScoringPoses = new ArrayList<Pose2d>();

        public enum Side {
            LEFT(leftScoringPoses),
            RIGHT(rightScoringPoses),
            CENTER(centerScoringPoses);

            public final List<Pose2d> poses;

            Side(final List<Pose2d> poses) {
                this.poses = poses;
            }
        }
    }

    public static class VisionConstants {

        // private static AprilTagFieldLayout loadReefOnlyFieldLayout() {
        //     try {
        //         // throw new IOException();
        //         return new AprilTagFieldLayout(
        //                 Filesystem.getDeployDirectory().getAbsolutePath()
        //                         + File.separatorChar
        //                         + "reef_only.json");
        //     } catch (IOException e) {
        //         e.printStackTrace();
        //         return loadFullField();
        //     }
        // }

        private static AprilTagFieldLayout loadFullField() {
            return AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        }

        public static final AprilTagFieldLayout FIELD_LAYOUT = loadFullField();

        // Camera sim values
        public static final int WIDTH_PX = 1280;
        public static final int HEIGHT_PX = 720;
        public static final double DIAG_FOV_DEGREES = 90;
        public static final double AVG_ERR_PX = 1;
        public static final double STD_DEV_ERR_PX = 0;
        public static final double FPS = 30;
        public static final double AVG_LATENCY_MS = 30;
        public static final double STD_DEV_LATENCY_MS = 10;

        // Pose filtering values
        public static final double distanceThreshold = 1.5; // meters
        public static final double angleThreshold = 10.0; // degrees
        public static final double zTranslationThreshold = 0.15; // meters
        public static final double ambiguityThreshold = 0.5;
        // every tag beyond seeing two tags gives us an extra meter of trusted distance
        public static final double multiTagDistanceFactor = 1.0;
    }
}
