package frc.robot.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;

public class ClimberFactory {
    CommandSwerveDrivetrain drivetrain;
    Turret turret;
    Climber climber;

    public Command alignClimber(String limelight) {
        String limelightName = "limelight-" + limelight; // need to implement something that makes it so 1) we ony use tag data from tags 16 (red) or 32 (blue), and so limelight uses the limelight that see said tag
        return drivetrain.applyRequest(() -> {
            double[] targetPose = LimelightHelpers.getTargetPose_RobotSpace(limelightName);
            boolean hasTarget = LimelightHelpers.getTV(limelightName);

            return RobotContainer.driveRobotCentric
                    .withVelocityX(!hasTarget ? 0
                            : (targetPose[0] - Constants.ClimberConstants.climberOffsetMagnitudeX)
                                    * Constants.ClimberConstants.visionOrthogonalSpeedScale)
                    .withVelocityY(!hasTarget ? 0
                            : (targetPose[1] - Constants.ClimberConstants.climberOffsetMagnitudeY)
                                    * Constants.ClimberConstants.visionOrthogonalSpeedScale)
                    .withRotationalRate(!hasTarget ? 0
                            : ((Math.PI - Math.abs(targetPose[5])) * Math.signum(targetPose[5])) * -1
                                    * Constants.ClimberConstants.visionRotationalSpeedScale);
        });
    }

    public ClimberFactory(CommandSwerveDrivetrain d, Turret t, Climber c) { 
        this.drivetrain = d;
        this.turret = t;
        this.climber = c;
     }

     public Command prepareClimbSettings() {
        return Commands.runOnce(() -> { turret.setYaw(0); } )
        .andThen(() -> {
            RobotContainer.rotationEnabled = 0;
            RobotContainer.speedMultiplier = .1;
        });
     }

     public Command resetClimbSettings() {
        return Commands.runOnce(() -> {
            RobotContainer.rotationEnabled = 1;
            RobotContainer.speedMultiplier = 1;
        } );
     }

     public Command setClimbPos() { return Commands.run(() -> { alignClimber("Front"); }); }
}
