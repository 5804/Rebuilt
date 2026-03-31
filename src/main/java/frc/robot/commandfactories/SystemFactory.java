package frc.robot.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.StateEnums.*;

public class SystemFactory extends SubsystemBase {
  Shooter shooter;
  Elevator elevator;
  Indexer indexer;
  Intake intake;
  CommandSwerveDrivetrain drivetrain;

  public SystemFactory(Shooter s, Elevator e, Indexer i, Intake in, CommandSwerveDrivetrain d) {
    this.shooter = s;
    this.elevator = e;
    this.indexer = i;
    this.intake = in;
    this.drivetrain = d;
  }

  public Command runScore() {
    return Commands.runOnce(() -> {
      if (ShooterState.Running) {
        elevator.runElevator(false);
        indexer.runIndexer(false);
      }
    }, indexer, elevator );
  }

  public Command stopScore() {
    return Commands.runOnce(() -> {
      elevator.stopElevator();
      indexer.stopIndexer();
    }, indexer, elevator);
  }

  public Command runOuttake() {
    return Commands.runOnce(() -> {
      elevator.reverseElevator(false);
      indexer.reverseIndexer(false);
      intake.reverseIntake();
    }, elevator, indexer, intake);
  }

  public Command stopOuttake() {
    return Commands.runOnce(() -> {
      elevator.stopElevator();
      indexer.stopIndexer();
      intake.stopIntake();
    }, elevator, indexer, intake);
  }

  public Command reverseSystem() {
    return Commands.runOnce(() -> {
      shooter.reverseShooter(false);
      elevator.reverseElevator(false);
      indexer.reverseIndexer(false);
      intake.reverseIntake();
    }, shooter, elevator, indexer, intake);
  }

  public Command stopSystem() {
    return Commands.runOnce(() -> {
      shooter.stopShooter();
      elevator.stopElevator();
      indexer.stopIndexer();
      intake.stopIntake();
    }, shooter, elevator, indexer, intake);
  }
}