package frc.robot.commandfactories;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ScoringFactory {
  Shooter shooter;
  Elevator elevator;
  Indexer indexer;
  CommandSwerveDrivetrain drivetrain;
  boolean isRedAlliance;


  public ScoringFactory(Shooter s, Elevator e, Indexer i, CommandSwerveDrivetrain d, boolean a) {
    this.shooter = s;
    this.elevator = e;
    this.indexer = i;
    this.drivetrain = d;
    this.isRedAlliance = a;
  }

  double[] turretOffset = {0.25, 0};
  public double getRps() {

    Pose2d robotPose = drivetrain.getState().Pose;

        Pose2d turretPose = robotPose.transformBy(
            new Transform2d(
                new Translation2d(turretOffset[0], turretOffset[1]),
                new Rotation2d()
            )
        );

    double hubY = 4.034663;
    double hubX;

    if (isRedAlliance) {
      hubX = 11.915394; // Red
    } else {
      hubX = 4.625594; // Blue
    }

    double turretOffsetX = hubX - turretPose.getX();
    double turretOffsetY = hubY - turretPose.getY();

    double distFromHub = Math.sqrt((Math.pow(turretOffsetX, 2) + Math.pow(turretOffsetY, 2)));
    double rps = -.337118 * distFromHub * distFromHub + 7.85399 * distFromHub + 26.39938;
    return rps;
  }

  public Command runShooter() {
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    return Commands.run(() -> {
      shooter.leftShooterMotor.setControl(m_request.withVelocity(getRps()).withFeedForward(.5)); // .5
      if (shooter.leftShooterMotor.getVelocity().getValueAsDouble() > getRps()-(getRps() * 0.03)) { // 97
        elevator.elevatorMotor.set(-getRps()/100);
        indexer.indexerMotor.set(Constants.IndexerConstants.INDEXER_SPEED);
      } else {
        elevator.elevatorMotor.set(0);
        indexer.indexerMotor.set(0);
      }
    }, shooter, elevator, indexer);
  }

  public Command stopShooter() {
    return Commands.run(() -> {
      shooter.leftShooterMotor.set(0);
      elevator.elevatorMotor.set(0);
      indexer.indexerMotor.set(0);
    }, shooter, elevator, indexer);
  }
}