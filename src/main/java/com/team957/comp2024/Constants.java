package com.team957.comp2024;

import com.team957.lib.controllers.feedback.PID.PIDConstants;
import com.team957.lib.util.GearRatioHelper;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {
    public static final class MiscConstants {
        public static final double SATURATION_VOLTAGE = 12;

        public static final Alliance DEFAULT_ALLIANCE = Alliance.Blue;

        public static final double NOMINAL_LOOP_TIME_SECONDS = 0.01; // 10 ms

        public static final double LOOP_WATCHDOG_TRIGGER_SECONDS = 0.030; // 30 ms

        public static final double HIGH_CAN_UTIL_THRESHOLD = .8;

        public static final int PRACTICE_BOT_JUMPER_CHANNEL = 0;
    }

    public static final class LEDConstants {
        public static final int TOTAL_PIXELS = 60;

        public static final int FULL_RED_RGB = 0;
        public static final int FULL_GREEN_RGB = 0;
        public static final int FULL_BLUE_RGB = 0;
    }

    public static final class SwerveConstants {
        public static final double DRIVE_CURRENT_LIMIT = 40;
        public static final double STEER_CURRENT_LIMIT = 20;

        public static final boolean INITIAL_BRAKE_MODE_ACTIVE = false;
        // for the sake of the people positioning the robot!!

        public static final double STEER_PLANT_KS = 0;
        public static final double STEER_PLANT_KV = 0.1; // wild guess
        public static final double STEER_PLANT_KA = 0.1;

        public static final double DRIVE_PLANT_KS = 0;
        public static final double DRIVE_PLANT_KV = 0.1;
        public static final double DRIVE_PLANT_KA = 0.1;

        public static final double STEER_GEARING = 42.62;
        public static final double DRIVE_GEARING = 4.71;

        public static final GearRatioHelper DRIVE_GEARING_HELPER =
                new GearRatioHelper(1, DRIVE_GEARING);

        public static final DCMotor STEER_MOTOR = DCMotor.getNeo550(1);
        public static final DCMotor DRIVE_MOTOR = DCMotor.getNEO(1);

        public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.5);

        public static final double MAX_WHEEL_SPEED_METERS_PER_SECOND = 8;

        public static final double a = .3302;
        public static final double b = .254;

        public static final Translation2d FRONT_LEFT_TRANSLATION = new Translation2d(a, b);
        public static final Translation2d FRONT_RIGHT_TRANSLATION = new Translation2d(a, -b);
        public static final Translation2d BACK_RIGHT_TRANSLATION = new Translation2d(-a, -b);
        public static final Translation2d BACK_LEFT_TRANSLATION = new Translation2d(-a, b);

        public static final SwerveDriveKinematics KINEMATICS =
                new SwerveDriveKinematics(
                        FRONT_LEFT_TRANSLATION,
                        FRONT_RIGHT_TRANSLATION,
                        BACK_RIGHT_TRANSLATION,
                        BACK_LEFT_TRANSLATION);

        public static final double ONBOARD_DRIVE_CONTROLLER_KP = 0.00004;
        public static final double ONBOARD_DRIVE_CONTROLLER_KI = 0;
        public static final double ONBOARD_DRIVE_CONTROLLER_KD = 0;
        public static final double ONBOARD_DRIVE_CONTROLLER_KFF = 0.000175;

        public static final double ONBOARD_STEER_CONTROLLER_KP = 2.5;
        public static final double ONBOARD_STEER_CONTROLLER_KI = 0;
        public static final double ONBOARD_STEER_CONTROLLER_KD = 0;

        // the offset is the "raw" value reported when the module is at the "zero"
        // position

        public static final int FRONT_LEFT_DRIVE_CANID = 7;
        public static final int FRONT_LEFT_STEER_CANID = 8;
        public static final double FRONT_LEFT_STEER_OFFSET_RADIANS = 2.555 + (Math.PI / 2);
        public static final boolean FRONT_LEFT_DRIVE_INVERTED = false;
        public static final double PRACTICE_FRONT_LEFT_STEER_OFFSET_RADIANS = -5.9 + (Math.PI / 4);

        public static final int FRONT_RIGHT_DRIVE_CANID = 1;
        public static final int FRONT_RIGHT_STEER_CANID = 2;
        public static final double FRONT_RIGHT_STEER_OFFSET_RADIANS = 3.169;
        public static final boolean FRONT_RIGHT_DRIVE_INVERTED = true;
        public static final double PRACTICE_FRONT_RIGHT_STEER_OFFSET_RADIANS = 1.58;

        public static final int BACK_LEFT_DRIVE_CANID = 5;
        public static final int BACK_LEFT_STEER_CANID = 6;
        public static final double BACK_LEFT_STEER_OFFSET_RADIANS = 0.478;
        public static final boolean BACK_LEFT_DRIVE_INVERTED = false;
        public static final double PRACTICE_BACK_LEFT_STEER_OFFSET_RADIANS = 6.27;

        public static final int BACK_RIGHT_DRIVE_CANID = 3;
        public static final int BACK_RIGHT_STEER_CANID = 4;
        public static final double BACK_RIGHT_STEER_OFFSET_RADIANS = 0.73 + (Math.PI / 2);
        public static final boolean BACK_RIGHT_DRIVE_INVERTED = true;
        public static final double PRACTICE_BACK_RIGHT_STEER_OFFSET_RADIANS = .63 + (Math.PI / 2);
    }

    public static final class ShooterConstants {
        public static final int LEFT_CANID = 10;
        public static final int RIGHT_CANID = 9;

        public static final int CURRENT_LIMIT = 30;
        public static final DCMotor SHOOTER_MOTOR = DCMotor.getNEO(1);
        public static final double SHOOTER_REDUCTION = 1;

        public static final double IDLE_CONTROL_EFFORT_VOLTS = -2;
        public static final double SUBWOOFER_CONTROL_EFFORT_VOLTS = -12;

        public static final boolean leftMotorInverted = true;
        public static final boolean rightMotorInverted = false;
        public static final boolean leftEncoderInverted = false;
        public static final boolean rightEncoderInverted = true;
    }

    public static final class AutoConstants {
        public static final PIDConstants LINEAR_PATHFINDING_GAINS = new PIDConstants(10, 0, 0);
        public static final PIDConstants ROTATIONAL_PATHFINDING_GAINS = new PIDConstants(10, 0, 0);

        public static final PIDConstants LINEAR_GAINS_NO_FF = new PIDConstants(3, 0, 0);
        public static final PIDConstants ROTATIONAL_GAINS_NO_FF = new PIDConstants(3, 0, 0);

        public static final double DEFAULT_PIVOT_DELAY_SECONDS = 0.5;

        public static final double PROFILE_OVERRUN_TOLERANCE_SECONDS = 0.4;
    }

    public static final class IMUConstants {
        public static final boolean YAW_AXIS_INVERTED = true;
        // this should NOT be applied to the navx.getRotation2D(), that's already offset
        // this is for the angular rate

        // the value reported as "unoffset" when the robot is zeroed
        public static final double YAW_OFFSET = -Math.PI;
    }

    public static final class PDHConstants {
        public static final int PDH_CAN_ID = 30;

        public static final boolean STARTING_SWITCHABLE_CHANNEL_STATE = false;
    }

    public static final class IntakePivotConstants {
        public static final double MIN_ANGLE_RADIANS = Units.degreesToRadians(-20);
        public static final double MAX_ANGLE_RADIANS = Units.degreesToRadians(160);

        public static final double PLANT_KS = 0;
        public static final double PLANT_KV = 0.00001; // .92
        public static final double PLANT_KA = 0.000001; // 0.01
        public static final double PLANT_KG = 0.3;

        public static final DCMotor DRIVE_MOTOR = DCMotor.getNEO(1);

        public static final GearRatioHelper GEARING_HELPER = new GearRatioHelper(1, 45);

        public static final double PIVOT_TO_TIP_METERS = Units.inchesToMeters(14);

        public static final double FLOOR_INTAKE_ANGLE_RADIANS = Units.degreesToRadians(327.5);
        public static final double STOW_INTAKE_ANGLE_RADIANS = Units.degreesToRadians(120);
        public static final double HANDOFF_INTAKE_ANGLE_RADIANS = Units.degreesToRadians(158.428);
        public static final double AMP_INTAKE_ANGLE_RADIANS = Units.degreesToRadians(70);

        public static final int INTAKE_PIVOT_MOTOR_CANID = 11;

        public static final int INTAKE_PIVOT_CURRENT_LIMIT_AMPS = 40;

        public static final boolean INTAKE_PIVOT_MOTOR_INVERTED = false;

        public static final double INTAKE_PIVOT_PROFILE_CONTROL_EFFORT = 3;

        // reported angle when the pivot is at "zero" (straight ahead)
        public static final double INTAKE_PIVOT_OFFSET_RADIANS = -.1;

        public static final PIDConstants PID_CONSTANTS = new PIDConstants(2.5, 0, 0);
    }

    public static final class BoxClimberConstants {
        public static final double STANDARD_VOLTAGE = 6;
        // CAN ID not set
        public static final int MOTOR_CANID = 13;
        public static final int CURRENT_LIMIT = 20;
        // positive = going up. negative = going down.
        public static final boolean MOTOR_INVERTED = false;

        public static final double IDLE_VOLTAGE = -4;
    }

    public static final class OIConstants {
        public static final int DRIVER_PORT = 0;
    }

    public static final class WinchConstants {
        // CAN ID not set
        public static final int MOTOR_CANID = 14;
        public static final double STANDARD_VOLTAGE = 12;
        public static final int CURRENT_LIMIT = 40;
        // positive = going up. negative = going down.
        public static final boolean MOTOR_INVERTED = false;
    }

    public static final class IntakeRollerConstants {
        public static final int ROLLER_CANID = 12;
        public static final int CURRENT_LIMIT = 30;
        public static final boolean ROLLER_INVERTED = false;

        public static final double FLOOR_INTAKE_VOLTAGE = 6;
        public static final double SHOOTER_HANDOFF_VOLTAGE = -9;
        public static final double AMP_SHOT_VOLTAGE = -4;
        public static final double SLOW_RUN_VOLTAGE = 2;
        public static final double CENTERING_VOLTAGE = .5;

        public static final int TOF_CANID = 0;

        public static final double TOF_TIMING_BUDGET_MS = 30;

        public static final double TOF_NOTE_PRESENT_THRESHOLD_METERS = .45;

        public static final double TOF_NOTE_PRESENT_DEBOUNCE_SECONDS = 0.1;

        public static final double SIM_MOCK_OUTTAKE_DELAY_SECONDS = 0.5;
        public static final double SIM_MOCK_INTAKE_DELAY_SECONDS = 0.5;

        public static final double CENTERING_REVERSAL_HYSTERESIS_METERS = .01; // 10mm

        public static final double CENTERING_PULSE_VOLTAGE = 2;
    }

    public static final class AlertConstants {
        public static final double OVERCURRENT_THRESHOLD_AMPS = 120;

        public static final double HIGH_RESISTANCE_THRESHOLD_OHMS = 20.0 / 1000;
        // 20 mOhms

        public static final double LOW_VOLTAGE_THRESHOLD = 10.5;
    }

    public static final class VisionConstants {

        public static final boolean VISION_POSE_ESTIMATION_ENABLED = false;

        public static final double LL_FOV_DEGREES = (29.8 * 2);
        public static final double LL_FOV_PIXELS = 320;

        public static final double NOTE_WIDTH = 14; // 0.3556 METERS

        public static final Vector<N3> STATE_STDS =
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
        public static final Vector<N3> VISION_STDS =
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30));

        public static final Transform3d LL1_TO_CENTER =
                new Transform3d(
                        new Translation3d(-Units.inchesToMeters(3), 0, Units.inchesToMeters(24)),
                        new Rotation3d(0, 0, 0));
        public static final Transform3d LL2_TO_CENTER =
                new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

        public static final double TARGET_AREA_CUTOFF = 0.5; // PERCENT OF SCREEN //TODO: FIND VAL
        public static final double TARGET_TX_CUTOFF = 24; // DEGREES
        public static final double TARGET_THOR_CUTOFF = 70; // PIXELS

        public static final double TRACKING_KP = 5.5;
        public static final double TRACKING_MIN_COMMAND = 0.1;
        public static final double TRACKING_STOP_THRESHOLD = 0.025; // RADIANS
        public static final double MIN_COMMAND_TRESHOLD = 0.02; // RADIANS
    }

    public static final class OtfPathingConstants {
        public static final Pose2d OTF_SPEAKER_POSE_BLUE = new Pose2d(1.6, 5.5, new Rotation2d());
        public static final Pose2d OTF_SPEAKER_POSE_RED =
                new Pose2d(15, 5.5, new Rotation2d(Math.PI));

        public static final Pose2d OTF_AMP_POSE_BLUE =
                new Pose2d(1.9, 7.6, new Rotation2d(Math.PI / 2));
        public static final Pose2d OTF_AMP_POSE_RED =
                new Pose2d(14.75, 7.6, new Rotation2d(-Math.PI / 2));
    }

    public static final class SequencingConstants {
        // coordinatedSubwooferShot
        public static final double CENTERING_NOTE_DURATION = .05;
        public static final double UNTIL_SHOOT_DELAY = .6;
        public static final double SHOOT_DURATION = .5;

        // coordinatedFloorIntake
        public static final double ROLLER_DELAY = .1;
    }
}
