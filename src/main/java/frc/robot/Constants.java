package frc.robot;

import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.*;


public final class Constants {
    public final class ElevatorConstants {
        public static final int ELEVATOR_MOTOR_ID = 53;
        public static final double ELEVATOR_SPEED = -1;
        
    }
    public final class IndexerConstants {
        public static final int INDEXER_MOTOR_ID = 52;
        public static final double INDEXER_SPEED = -.3; // If going faster than .3, make sure wheels don't break robot
        
    }
    public final class ShooterConstants {
        public static final int LEFT_SHOOTER_MOTOR_ID = 55;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 56;
        public static double SHOOTER_SPEED = 60;
        public static double SHOOTER_WHEEL_DIAMETER = 4.0;
    }
    public final class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 51;
        public static final double INTAKE_SPEED = 0.5;
    }
    public final class TurretConstants {
        public static final int TURRET_MOTOR_ID = 54;
    }
    public final class DriveConstants {
        public static final double DRIVE_DEADBAND = 0.20;
        public static final double ANGLE_DEADBAND = 0.15;
        public static final double MAX_SPEED = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocit
    }
    public final class ClimberConstants {
        public static final int LEFT_CLIMBER_MOTOR_ID = 99;
        public static final int RIGHT_CLIMBER_MOTOR_ID = 99;
    }
}
