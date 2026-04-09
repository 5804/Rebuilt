package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurretMath {

    // Outputs
    public static double turretAngle = 0; // Degrees (FIELD RELATIVE)
    public static double turretRPS = 0; // Revolutions per second

    // Inputs
    public double targetX;
    public double targetY = 4.034663;
    public double targetZ = 1.75; // Where you want the ball to go

    // Constants
    private static final double OFFSET_X = -0.25;
    private static final double OFFSET_Y = 0.0;
    private static final double OFFSET_Z = 0.7112; // Turret's FIELD RELATIVE offset from the center of the robot

    private static final double TURRET_ANGLE = 22.5; // Degrees from vertical (0 = straight up)
    private static final double MAX_RPS = 100; // Our shooter motors' maximum RPS

    private static final double TESTED_Z = 1.75; // The target's z used during our equation testing

    private static final int ITERATIONS = 3; // Iterations of the airtime and new target calculation
    private static final double DEADBAND = .05; // Maximum velocity where we treat velocity as 0 to avoid unnecessary
                                                // noise

    // Math Constants
    private static final double LAUNCH_ANGLE = Math.toRadians(90 - TURRET_ANGLE); // Degrees from horizontal (0 =
                                                                                  // forward)
    private static final double COS_ANGLE = Math.cos(LAUNCH_ANGLE); // Cosine of launch angle
    private static final double TAN_ANGLE = Math.tan(LAUNCH_ANGLE); // Tangent of launch angle

    private static final double OFFSET_RADIUS = Math.hypot(OFFSET_X, OFFSET_Y); // Radius of shooter from the center of
                                                                                // the robot
    private static final double OFFSET_ANGLE_INITIAL = Math.atan2(OFFSET_Y, OFFSET_X); // Initial angle of the shooter

    private static final double TESTED_DZ = TESTED_Z - OFFSET_Z; // Difference in tested target z and shooter z

    // 30 Degree Hood
    // private double distanceToAirtime(double distance) {
    // return 0.128428 * distance + 0.453662;
    // }
    // private double distanceToRPS(double distance) {
    // return 3.89912 * distance + 35.10389;
    // }

    // Equations
    private double distanceToAirtime(double distance) { // EMPIRICAL
        return 0.16808 * distance + 0.479526; // Tested data points and fit equation (currently based on 22.5 degree
                                              // turret angle)
    }

    private double distanceToRPS(double distance) { // EMPIRICAL
        return 6.13132 * distance + 26.94506; // Tested data points and fit equation (currently based on 22.5 degree
                                              // turret angle)
    }

    private double distanceToExitVelocity(double distance, double dz) { // THEORETICAL
        double denominator = 2 * Math.pow(COS_ANGLE, 2) * (distance * TAN_ANGLE - dz);
        if (denominator <= 0)
            return 0;
        return Math.sqrt((9.81 * Math.pow(distance, 2)) / denominator);
    }// Pure physics equation to give exit velocity based on distance and difference
     // in z (hit the point on the way down)

    // CURRENTLY WORKS TURRET MATH. USE IF 3D BREAKS

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

    // PLS WORK THIS TIME

    public void calculate3DTurretMath(double robot_x /* meters, FIELD REL */, double robot_y /* meters, FIELD REL */,
            double robot_angle /* radians, FIELD REL */, double robotVel_x /* meters per second, ROBOT REL */,
            double robotVel_y /* meters per second, ROBOT REL */, double robotVel_angle /*radians per second, ROBOT REL */) {

        double fieldVel_x = robotVel_x * Math.cos(robot_angle) - robotVel_y * Math.sin(robot_angle);
        double fieldVel_y = robotVel_x * Math.sin(robot_angle) + robotVel_y * Math.cos(robot_angle); // Converts robot's local velocity to field velocity

        double shooter_x = robot_x + OFFSET_X * Math.cos(robot_angle) - OFFSET_Y * Math.sin(robot_angle);
        double shooter_y = robot_y + OFFSET_Y * Math.cos(robot_angle) + OFFSET_X * Math.sin(robot_angle); // Gets the shooters actual field position

        double offsetAngle = OFFSET_ANGLE_INITIAL + robot_angle;
        double tangentialVel_x = -robotVel_angle * OFFSET_RADIUS * Math.sin(offsetAngle);
        double tangentialVel_y = robotVel_angle * OFFSET_RADIUS * Math.cos(offsetAngle); // Gets tangential velocity of shooter for while spinning (NOT FIELD RELATIVE)

        double totalVel_x = fieldVel_x; //Add the tagential velocities when fixed
        double totalVel_y = fieldVel_y; // Calculates the shooters final velocity by adding field velocity and tangential velocity

        if (Math.hypot(totalVel_x, totalVel_y) < DEADBAND) {
            totalVel_x = 0;
            totalVel_y = 0;
        } // If velocity basically 0, treat as such to avoid noise (deadband)

        double dz = targetZ - OFFSET_Z; // Difference in the target's z and the turret's z

        double aimDistance = Math.hypot((shooter_x - targetX), (shooter_y - targetY)); // Initial distance to target
        double aimX = targetX;
        double aimY = targetY;

        for (int i = 0; i < ITERATIONS; i++) { // Iterates because airtime depends on distance, which is dependent on
                                               // target position, which is dependent on airtime. This is so we get
                                               // closer and closer to the actual target we should aim for.
            double airtime = distanceToAirtime(aimDistance); // Gets airtime of currently best know distance
            aimX = targetX - totalVel_x * airtime;
            aimY = targetY - totalVel_y * airtime; // Gets new target position based on the calculated airtime and
                                                   // velocity
            aimDistance = Math.hypot((shooter_x - aimX), (shooter_y - aimY)); // Sets new distance to target
        }

        double vRequired = distanceToExitVelocity(aimDistance, dz); // The exit velocity required to reach the desired z
                                                                    // at the given distance
        double vTested = distanceToExitVelocity(aimDistance, TESTED_DZ); // The exit velocity required to reach the
                                                                         // tested z at the given distance

        turretAngle = Math.toDegrees(Math.atan2(aimY - shooter_y, aimX - shooter_x)); // Simple Math.atan2 to get angle
                                                                                      // to aimed target
        turretRPS = (vTested > 0) // Divide by 0 check
                ? distanceToRPS(aimDistance) * (vRequired / vTested) // Scales RPS by how much harder/easier it is to
                                                                     // reach this Z vs the tested Z at the same
                                                                     // distance
                : distanceToRPS(aimDistance); // If vTested is 0, just give the distanceToRPS of the distance

        turretRPS = Math.max(0, Math.min(turretRPS, MAX_RPS)); // Clamps so we don't over ask of the turret motors

        if (Double.isNaN(turretAngle) || Double.isNaN(turretRPS)) {
            turretAngle = 0;
            turretRPS = 50;
        } // Final check for NaN and sets to 0 if found
    }

    // Variables for target calculation
    public static double[][] REDZONES = {
            { 11.915394, 4.034663, 1.75 } /* Red Hub */,
            { 13.5, 6.0, 0 } /* Red Left Side Passing */,
            { 13.5, 2.0, 0 } /* Red Right Side Passing */ };
    public static double[][] BLUEZONES = {
            { 4.625594, 4.034663, 1.75 } /* Blue Hub */,
            { 3.0, 6.0, 0 } /* Blue Right Side Passing */,
            { 3.0, 2.0, 0 } /* Blue Left Side Passing */ };
    private static final double RED_NEUTRAL_LINE_X = 12.0; // X of the line between the red and the neutral zones
    private static final double BLUE_NEUTRAL_LINE_X = 4.6; // X of the line between the blue and the neutral zones
    private static final double MIDLINE_Y = 4; // Y of the mid-line

    // Variables for SPECIFIC target calculation
    private static final double[] RED_HUB = { 11.915394, 4.034663 };
    private static final double[] BLUE_HUB = { 4.625594, 4.034663 };
    private static final double[] RED_X_VALS = { 12.55, 13.7, 14.85, 16 }; // Top to Bottom
    private static final double[] BLUE_X_VALS = { 4, 2.8333, 1.666, 0.5 }; // Top to Bottom
    private static final double[] Y_VALS = { 0.5, 2.5, 4, 5.5, 7.5 }; // Left to Right from RED SIDE (reversed order for
                                                                      // Blue Side)
    private static final double HUB_Z = 1.75;
    private static final double PASSING_Z = .10;

    public void calculateTarget(boolean isRedAlliance, double x, double y) {
        double[][] zones = isRedAlliance ? REDZONES : BLUEZONES; // Sets zones based on which alliance we're on

        int zoneIndex; // Sets the zone we want to aim for (Hub, Passing Left, Passing Right)
        if (isRedAlliance && x > RED_NEUTRAL_LINE_X || !isRedAlliance && x < BLUE_NEUTRAL_LINE_X) {
            zoneIndex = 0;
        } else if (y >= MIDLINE_Y) {
            zoneIndex = 1;
        } else {
            zoneIndex = 2;
        }

        targetX = zones[zoneIndex][0]; // Sets the target X
        targetY = zones[zoneIndex][1]; // Sets the target Y
        targetZ = zones[zoneIndex][2]; // Sets the target Z
    }

    public void calculateSpecificTarget(boolean isRedAlliance, double x, int zoneButtonIndex /* 1-20 */) {
        if (isRedAlliance && x > RED_NEUTRAL_LINE_X) {
            targetX = RED_HUB[0];
            targetY = RED_HUB[1];
            targetZ = HUB_Z;
        } else if (!isRedAlliance && x < BLUE_NEUTRAL_LINE_X) {
            targetX = BLUE_HUB[0];
            targetY = BLUE_HUB[1];
            targetZ = HUB_Z;
        } else {
            int buttonX = (int) Math.floor((zoneButtonIndex - 1) / 4.0);
            int buttonY = (zoneButtonIndex - 1) % 4;
            targetZ = PASSING_Z;
            if (isRedAlliance) {
                targetX = (buttonY == 0 && buttonX > 0 && buttonX < 4) ? 13.0 : RED_X_VALS[buttonY];
                targetY = Y_VALS[buttonX];
            } else {
                targetX = (buttonY == 0 && buttonX > 0 && buttonX < 4) ? 3.55 : BLUE_X_VALS[buttonY];
                targetY = Y_VALS[4 - buttonX];
            }
        }
        SmartDashboard.putString("Current Turret Target",
                "Zone: " + zoneButtonIndex + " (" + targetX + ", " + targetY + ", " + targetZ + ")");
    }
}