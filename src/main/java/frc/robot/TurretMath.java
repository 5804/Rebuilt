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

    private double distanceToAirtime(double distance) {
        return 0.128428 * distance + 0.453662;
    }

    private double distanceToRPS(double distance) {
        return 3.89912 * distance + (35.10389-.25);
    }

    private double distanceToExitVelocity(double distance, double dz) {
        double denominator = 2 * Math.pow(COS_ANGLE, 2) * (distance * TAN_ANGLE - dz);
        if (denominator <= 0)
            return 0;
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

        double hubDz = 2 - OFFSET_Z;
        turretAngle = Math.toDegrees(Math.atan2(new_targetY - shooter_y, new_targetX - shooter_x));
        turretRPS = distanceToRPS(actualDistance) *
                (distanceToExitVelocity(actualDistance, dz) /
                        distanceToExitVelocity(actualDistance, hubDz));
    }

    double[][] redZones = { { 11.665394, 4.0, 1.75 }, { 15.4/* 15.0 */, 6.5, 0 }, { 15.4/* 15.0 */, 1.7, 0 } };
    double[][] blueZones = { { 4.875594, 4.0, 1.75 }, { 1.1/* 1.5 */, 1.7, 0 }, { 1.1/* 1.5 */, 6.5, 0 } };
    
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
// //All comments written by Hunt because he was bored and so when he has to explain he can reference his own comments ;)

// public class TurretMath {

//     //Outputs
//     public double turretAngle = 0; // Degrees
//     public double turretRPS = 0; // Revolutions per second

//     //Inputs
//     public double targetX;
//     public double targetY = 4.034663;
//     public double targetZ = 1.75; //Where you want the ball to go

//     private static final double OFFSET_X = 0.0;
//     private static final double OFFSET_Y = -0.25;
//     private static final double OFFSET_Z = 0.7112;    //Turret's LOCAL offset from the center of the robot

//     private static final double TURRET_ANGLE = 30; // Degrees from vertical (0 = straight up)
//     private static final double MAX_RPS = 90; //Our shooter motors' maximum RPS

//     private static final double TESTED_Z = 1.75; //The target's z used during our equation testing

//     private static final int ITERATIONS = 3; //Iterations of the airtime and new target calculation
//     private static final double DEADBAND = .05; //Maximum velocity where we treat velocity as 0 to avoid unnecessary noise

//     //Constants
//     private static final double LAUNCH_ANGLE = Math.toRadians(90-TURRET_ANGLE); // Degrees from horizontal (0 = forward)
//     private static final double COS_ANGLE = Math.cos(LAUNCH_ANGLE); //Cosine of launch angle
//     private static final double TAN_ANGLE = Math.tan(LAUNCH_ANGLE); //Tangent of launch angle

//     private static final double OFFSET_RADIUS = Math.hypot(OFFSET_X, OFFSET_Y); //Radius of shooter from the center of the robot
//     private static final double OFFSET_ANGLE_INITIAL = Math.atan2(OFFSET_Y, OFFSET_X); //Initial angle of the shooter

//     private static final double TESTED_DZ = TESTED_Z - OFFSET_Z; //Difference in tested target z and shooter z

//     //Equations
//     private double distanceToAirtime(double distance) { //EMPIRICAL
//         return 0.157202 * distance + 0.523216; //Tested data points and fit equation (currently based on 30 degree turret angle)
//     }

//     private double distanceToRPS(double distance) { //EMPIRICAL
//         return 0.399341 * Math.pow(distance, 2) + 1.63185 * distance + 38.60187; //Tested data points and fit equation (currently based on 30 degree turret angle)
//     }

//     private double distanceToExitVelocity(double distance, double dz) { //THEORETICAL
//         double denominator = 2 * Math.pow(COS_ANGLE, 2) * (distance * TAN_ANGLE - dz);
//         if (denominator <= 0)
//             return 0;
//         return Math.sqrt((9.81 * Math.pow(distance, 2)) / denominator); //Pure physics equation to give exit velocity based on distance and difference in z (hit the point on the way down)
//     }


//     public void calculateTurretMath(double robot_x, double robot_y, double robot_angle, double robotVel_x, double robotVel_y) { //Old equation, but keeping it until we successfully test 3DMath

//         double fieldVel_x = robotVel_x * Math.cos(robot_angle) - robotVel_y * Math.sin(robot_angle);
//         double fieldVel_y = robotVel_x * Math.sin(robot_angle) + robotVel_y * Math.cos(robot_angle);

//         double shooter_x = robot_x + OFFSET_X * Math.cos(robot_angle) - OFFSET_Y * Math.sin(robot_angle);
//         double shooter_y = robot_y + OFFSET_Y * Math.cos(robot_angle) + OFFSET_X * Math.sin(robot_angle);

//         double aimDistance = Math.hypot((shooter_x - targetX), (shooter_y - targetY));
//         double aimX = targetX;
//         double aimY = targetY;

//         for (int i = 0; i < ITERATIONS; i++) {
//             double airtime = distanceToAirtime(aimDistance);
//             aimX = targetX - fieldVel_x * airtime;
//             aimY = targetY - fieldVel_y * airtime;
//             aimDistance = Math.hypot((shooter_x - aimX), (shooter_y - aimY));
//         }


//         turretAngle = Math.toDegrees(Math.atan2(aimY - shooter_y, aimX - shooter_x));
//         turretRPS = distanceToRPS(aimDistance);
//     }

//     public void calculate3DTurretMath(double robot_x /*meters*/, double robot_y /*meters*/, double robot_angle /*radians*/, double robotVel_x /*meters per second*/, double robotVel_y /*meters per second*/, double robotVel_angle /*radians per second*/) {
        
//         double fieldVel_x = robotVel_x * Math.cos(robot_angle) - robotVel_y * Math.sin(robot_angle);
//         double fieldVel_y = robotVel_x * Math.sin(robot_angle) + robotVel_y * Math.cos(robot_angle); //Converts robot's local velocity to field velocity

//         double shooter_x = robot_x + OFFSET_X * Math.cos(robot_angle) - OFFSET_Y * Math.sin(robot_angle);
//         double shooter_y = robot_y + OFFSET_Y * Math.cos(robot_angle) + OFFSET_X * Math.sin(robot_angle); //Gets the shooters actual field position

//         double offsetAngle = OFFSET_ANGLE_INITIAL + robot_angle;
//         double tangentialVel_x = -robotVel_angle * OFFSET_RADIUS * Math.sin(offsetAngle);
//         double tangentialVel_y =  robotVel_angle * OFFSET_RADIUS * Math.cos(offsetAngle); //Gets tangential velocity of shooter for while spinning

//         double totalVel_x = fieldVel_x + tangentialVel_x;
//         double totalVel_y = fieldVel_y + tangentialVel_y; //Calculates the shooters final velocity by adding field velocity and tangential velocity

//         if (Math.hypot(totalVel_x, totalVel_y) < DEADBAND) {
//             totalVel_x = 0;
//             totalVel_y = 0;
//         } //If velocity basically 0, treat as such to avoid noise (deadband)

//         double dz = targetZ - OFFSET_Z; //Difference in the target's z and the turret's z

//         double aimDistance = Math.hypot((shooter_x - targetX), (shooter_y - targetY)); //Initial distance to target
//         double aimX = targetX;
//         double aimY = targetY;

//         for (int i = 0; i < ITERATIONS; i++) { //Iterates because airtime depends on distance, which is dependent on target position, which is dependent on airtime. This is so we get closer and closer to the actual target we should aim for.
//             double airtime = distanceToAirtime(aimDistance); //Gets airtime of currently best know distance
//             aimX = targetX - totalVel_x * airtime;
//             aimY = targetY - totalVel_y * airtime; //Gets new target position based on the calculated airtime and velocity
//             aimDistance = Math.hypot((shooter_x - aimX), (shooter_y - aimY)); //Sets new distance to target
//         }

//         double vRequired = distanceToExitVelocity(aimDistance, dz); //The exit velocity required to reach the desired z at the given distance
//         double vTested   = distanceToExitVelocity(aimDistance, TESTED_DZ); //The exit velocity required to reach the tested z at the given distance

//         turretAngle = Math.toDegrees(Math.atan2(aimY - shooter_y, aimX - shooter_x)); //Simple Math.atan2 to get angle to aimed target
//         turretRPS = (vTested > 0) //Divide by 0 check
//                 ? distanceToRPS(aimDistance) * (vRequired / vTested) // Scales RPS by how much harder/easier it is to reach this Z vs the tested Z at the same distance
//                 : distanceToRPS(aimDistance);  //If vTested is 0, just give the distanceToRPS of the distance

//         turretRPS = Math.max(0, Math.min(turretRPS, MAX_RPS)); //Clamps so we don't over ask of the turret motors

//         if (Double.isNaN(turretAngle) || Double.isNaN(turretRPS)) {
//             turretAngle = 0;
//             turretRPS = 0;
//         } //Final check for NaN and sets to 0 if found
//     }

//     //Variables for target calculation
//     private static final double[][] REDZONES = { { 11.915394, 4.034663, 1.75 } /*Red Hub*/,{ 13.5, 6.0, 0 } /*Red Left Side Passing*/, { 13.5, 2.0, 0 } /*Red Right Side Passing*/}; //List of zones for Red Alliance
//     private static final double[][] BLUEZONES = { { 4.625594, 4.034663, 1.75 } /*Blue Hub*/, { 3.0, 6.0, 0 } /*Blue Right Side Passing*/, { 3.0, 2.0, 0 } /*Blue Left Side Passing*/}; //List of zones for Blue Alliance
//     private static final double RED_NEUTRAL_LINE_X = 12.0; //X of the line between the red and the neutral zones
//     private static final double BLUE_NEUTRAL_LINE_X = 4.6; //X of the line between the blue and the neutral zones
//     private static final double MIDLINE_Y = 4; //Y of the mid-line

//     public void calculateTarget(boolean isRedAlliance, double x, double y) {
//         double[][] zones = isRedAlliance ? REDZONES : BLUEZONES; //Sets zones based on which alliance we're on

//         int zoneIndex; //Sets the zone we want to aim for (Hub, Passing Left, Passing Right)
//         if (isRedAlliance && x > RED_NEUTRAL_LINE_X || !isRedAlliance && x < BLUE_NEUTRAL_LINE_X) {
//             zoneIndex = 0;
//         } else if (y >= MIDLINE_Y) {
//             zoneIndex = 1;
//         } else {
//             zoneIndex = 2;
//         }

//         targetX = zones[zoneIndex][0]; //Sets the target X
//         targetY = zones[zoneIndex][1]; //Sets the target Y
//         targetZ = zones[zoneIndex][2]; //Sets the target Z
//     }
// }