package frc.robot;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TurretMath {

    public static double turretAngle = 0; // Degrees
    public static double turretRPS = 0; // Revolutions per second

    public double targetY = 4.034663;
    public double targetX;
    public double targetZ = 1.75;

    // Hub Height = 1.75 Meters

    // Shooter offset from robot center (meters)
    private static final double OFFSET_X = 0.0;
    private static final double OFFSET_Y = -0.25;
    private static final double OFFSET_Z = 0.7112;

    private static final double LAUNCH_ANGLE = Math.toRadians(15.0);
    private static final double COS_ANGLE = Math.cos(LAUNCH_ANGLE);
    private static final double TAN_ANGLE = Math.tan(LAUNCH_ANGLE);

    // 30 Degree Hood
    // private double distanceToAirtime(double distance) {
    // return 0.128428 * distance + 0.453662;
    // }
    // private double distanceToRPS(double distance) {
    // return 3.89912 * distance + 35.10389;
    // }

    private double distanceToAirtime(double distance) {
        return 0.128428 * distance + 0.453662;
    }

    private double distanceToRPS(double distance) {
        return 3.89912 * distance + 35.10389;
    }

    private double distanceToExitVelocity(double distance, double dz) {
        double denominator = 2 * Math.pow(COS_ANGLE, 2) * (distance * TAN_ANGLE - dz);
        if (denominator <= 0)
            return Double.NaN;
        return Math.sqrt((9.81 * Math.pow(distance, 2)) / denominator);
    }

    public void calculateTurretMath(
            double robot_x, double robot_y, double robot_angle,
            double robotVel_x, double robotVel_y) {

        double fieldVel_x = robotVel_x * Math.cos(robot_angle) - robotVel_y * Math.sin(robot_angle);
        double fieldVel_y = robotVel_x * Math.sin(robot_angle) + robotVel_y * Math.cos(robot_angle);

        double shooter_x = robot_x + OFFSET_X * Math.cos(robot_angle) - OFFSET_Y * Math.sin(robot_angle);
        double shooter_y = robot_y + OFFSET_Y * Math.cos(robot_angle) + OFFSET_X * Math.sin(robot_angle);

        double distance = Math.sqrt(Math.pow(shooter_x - targetX, 2) + Math.pow(shooter_y - targetY, 2));
        double airtime = distanceToAirtime(distance);

        double new_targetX = targetX - fieldVel_x * airtime;
        double new_targetY = targetY - fieldVel_y * airtime;

        double actualDistance = Math.sqrt(Math.pow(shooter_x - new_targetX, 2) + Math.pow(shooter_y - new_targetY, 2));

        turretAngle = Math.toDegrees(Math.atan2(new_targetY - shooter_y, new_targetX - shooter_x));
        turretRPS = distanceToRPS(actualDistance);
    }

    public void calculate3DTurretMath(
            double robot_x, double robot_y, double robot_angle,
            double robotVel_x, double robotVel_y) {

        double fieldVel_x = robotVel_x * Math.cos(robot_angle) - robotVel_y * Math.sin(robot_angle);
        double fieldVel_y = robotVel_x * Math.sin(robot_angle) + robotVel_y * Math.cos(robot_angle);

        double shooter_x = robot_x + OFFSET_X * Math.cos(robot_angle) - OFFSET_Y * Math.sin(robot_angle);
        double shooter_y = robot_y + OFFSET_Y * Math.cos(robot_angle) + OFFSET_X * Math.sin(robot_angle);

        double dz = targetZ - OFFSET_Z;
        double distance = Math.sqrt(Math.pow(shooter_x - targetX, 2) + Math.pow(shooter_y - targetY, 2));
        double airtime = distanceToAirtime(distance);

        double new_targetX = targetX - fieldVel_x * airtime;
        double new_targetY = targetY - fieldVel_y * airtime;

        double actualDistance = Math.sqrt(Math.pow(shooter_x - new_targetX, 2) + Math.pow(shooter_y - new_targetY, 2));

        double hubDz = 1.75 - OFFSET_Z;
        turretAngle = Math.toDegrees(Math.atan2(new_targetY - shooter_y, new_targetX - shooter_x));
        turretRPS = distanceToRPS(actualDistance) *
                (distanceToExitVelocity(actualDistance, dz) /
                        distanceToExitVelocity(actualDistance, hubDz));
    }

    double[][] redZones = { { 11.665394, 3.8, 1.75 }, { 13.5/* 15.0 */, 6.0, 0 }, { 13.5/* 15.0 */, 2.0, 0 } };
    double[][] blueZones = { { 4.875594, 3.8, 1.75 }, { 3/* 1.5 */, 2.0, 0 }, { 3/* 1.5 */, 6.0, 0 } };

    public void calculateTarget(boolean isRedAlliance, CommandSwerveDrivetrain drivetrain) {
        double[][] zones = isRedAlliance ? redZones : blueZones;
        double x = drivetrain.getState().Pose.getX();
        double y = drivetrain.getState().Pose.getY();

        int zoneIndex;
        if (isRedAlliance && x > 12.0 || !isRedAlliance && x < 4.6)
            zoneIndex = 0;
        else if (y >= 4)
            zoneIndex = isRedAlliance ? 1 : 2;
        else
            zoneIndex = isRedAlliance ? 2 : 1;

        targetX = zones[zoneIndex][0];
        targetY = zones[zoneIndex][1];
        targetZ = zones[zoneIndex][2];
    }
}
