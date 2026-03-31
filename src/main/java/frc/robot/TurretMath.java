package frc.robot;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TurretMath {

    public static double turretAngle = 0;
    public static double turretRPS = 0;

    public static double targetY = 4.034663;
    public static double targetX;
    public static double targetZ = 1.75;

    private static final double OFFSET_X = -0.25;
    private static final double OFFSET_Y = 0.0;
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

    private static double distanceToAirtime(double distance) {
        return 0.16808 * distance + 0.479526;
    }

    private static double distanceToRPS(double distance) {
        return 6.13132 * distance + 26.94506;
    }

    private double distanceToExitVelocity(double distance, double dz) {
        double denominator = 2 * Math.pow(COS_ANGLE, 2) * (distance * TAN_ANGLE - dz);
        if (denominator <= 0)
            return 0;
        return Math.sqrt((9.81 * Math.pow(distance, 2)) / denominator);
    }

    public static void calculateTurretMath(
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

        double hubDz = 2 - OFFSET_Z;
        turretAngle = Math.toDegrees(Math.atan2(new_targetY - shooter_y, new_targetX - shooter_x));
        turretRPS = distanceToRPS(actualDistance) *
                (distanceToExitVelocity(actualDistance, dz) /
                        distanceToExitVelocity(actualDistance, hubDz));
    }

    public static double[][] redZones = { { 11.665394, 4.0, 1.75 }, { 15.4/* 15.0 */, 6.5, 0 },
            { 15.4/* 15.0 */, 1.7, 0 } };
    public static double[][] blueZones = { { 4.875594, 4.0, 1.75 }, { 1.1/* 1.5 */, 1.7, 0 }, // move this stuff to constants later (im lazy rn)
            { 1.1/* 1.5 */, 6.5, 0 } };

    public static void calculateTarget(boolean isRedAlliance, CommandSwerveDrivetrain drivetrain) {
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