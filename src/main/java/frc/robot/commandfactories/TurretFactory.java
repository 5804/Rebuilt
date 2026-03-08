package frc.robot.commandfactories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;

public class TurretFactory {
    CommandSwerveDrivetrain drivetrain;
    Turret turret;
    boolean isRedAlliance;

    double motorYawOffset;
    double odometryX;
    double odometryY;
    double odometryRot;

    public TurretFactory(CommandSwerveDrivetrain drivetrain, Turret turret, boolean isRedAlliance) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.isRedAlliance = isRedAlliance;

    }
 
   double[] turretOffset = {0.25, 0}; // (forward, side)

    public Command aimTurretHub() {
    return Commands.run(() -> {

        Pose2d robotPose = drivetrain.getState().Pose;

        Pose2d turretPose = robotPose.transformBy(
            new Transform2d(
                new Translation2d(turretOffset[0], turretOffset[1]),
                new Rotation2d()
            )
        );

        turret.setYaw(
            -(robotPose.getRotation().getDegrees() - 90)
            + turret.getMotorYawOffset(
                turretPose.getX(),
                turretPose.getY(),
                isRedAlliance
            )
        );

    }, turret);
}

    public double getMotorYawOffset() {
        return turret.getMotorYawOffset(
                drivetrain.getState().Pose.getX(),
                drivetrain.getState().Pose.getY(),
                isRedAlliance
        );
    }
}