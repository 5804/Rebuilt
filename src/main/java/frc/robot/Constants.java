package frc.robot;

import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.*;

public final class Constants {
    public final class ElevatorConstants {
        public static final int ELEVATOR_MOTOR_ID = 53;
        public static final double ELEVATOR_SPEED = -.3;

    }

    public final class IndexerConstants {
        public static final int INDEXER_MOTOR_ID = 52;
        public static final double INDEXER_SPEED = .3; // .6

    }

    public final class ShooterConstants {
        public static final int LEFT_SHOOTER_MOTOR_ID = 55;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 56;
        public static double SHOOTER_SPEED = 60;
        public static double SHOOTER_WHEEL_DIAMETER = 4.0;
        public static double kP = 0.3;
        public static double kI = 0;
        public static double kD = 0;
        public static double kS = 0;
        public static double kV = 0.11;
        public static double kA = 0;
    }

    public final class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 51;
        public static final double INTAKE_SPEED = -0.5;
    }

    public final class TurretConstants {
        public static final int TURRET_MOTOR_ID = 54;
        public static double kP = 14;
        public static double kI = 0;
        public static double kD = 0.35;
        public static double kS = 0;
        public static double kV = 0.117 / (2 * Math.PI);
        public static double kA = 0;
    }

    public final class DriveConstants {
        public static final double DRIVE_DEADBAND = 0.20;
        public static final double ANGLE_DEADBAND = 0.15;
        public static final double MAX_SPEED = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    }

    public final class ClimberConstants {
        public static final int LEFT_CLIMBER_MOTOR_ID = 62;
        public static final int RIGHT_CLIMBER_MOTOR_ID = 5;
        public static final int LINEAR_ACTUATOR_MOTOR = 61;
        public static final double ACTUATOR_DELTA_POS = -3.035;
        public static final double ACTUATOR_POS_TOLERANCE = .1;

        public static final double visionOrthogonalSpeedScale = 2.25;
        public static final double visionRotationalSpeedScale = 0.95;
        public static final double climberOffsetMagnitudeX = .1;
        public static final double climberOffsetMagnitudeY = .1;
    }

    public final class GlobalConstants {
        public static final double[][] hubPositions = { { 4.625594, 4 }, { 11.665394, 4 } };
    }
}
