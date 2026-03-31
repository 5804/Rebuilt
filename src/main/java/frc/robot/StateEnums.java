package frc.robot;

public final class StateEnums {
    public final class ShooterState {
        public static boolean Running = false;
        public static boolean Reversing = false;
        public static boolean ReachedSpeed = false;
        public static boolean Manual = false;
    }

    public final class ElevatorState {
        public static boolean Running = false;
        public static boolean Reversing = false;
        public static boolean Unjamming = false;
        public static boolean ReachedSpeed = false;
        public static boolean Manual = false;
    }

    public final class IndexerState {
        public static boolean Running = false;
        public static boolean Reversing = false;
        public static boolean Unjamming = false;
        public static boolean ReachedSpeed = false;
        public static boolean Manual = false;
    }

    public final class IntakeState {
        public static boolean Running = false;
        public static boolean Reversing = false;
    }

    public final class TurretState {
        public static boolean Aiming = false;
    }
}
