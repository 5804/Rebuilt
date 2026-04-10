package frc.robot.commandfactories;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.TurretMath;

public class ScoringFactory extends SubsystemBase {
  Shooter shooter;
  Elevator elevator;
  Indexer indexer;
  CommandSwerveDrivetrain drivetrain;
  boolean isRedAlliance;
  TurretMath turretMath;

  boolean reachedSpeed = false;
  public boolean isShooting = false;
  public boolean isReversing = false;
  boolean elevatorToSpeed = false;
  boolean elevatorUnjamming = false;
  boolean indexerUnjamming = false;
  boolean agitatorUnjamming = false;


  final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  public ScoringFactory(Shooter s, Elevator e, Indexer i, CommandSwerveDrivetrain d, boolean a, TurretMath t) {
    this.shooter = s;
    this.elevator = e;
    this.indexer = i;
    this.drivetrain = d;
    this.isRedAlliance = a;
    this.turretMath = t;
  }

  double elvPosInit = 0;
  double indPosInit = 0;
  double agiPosInit = 0;

  double unjamRot = 6;
  double agitatorUnjamPercentThreshold = .1;
  double indexerUnjamPercentThreshold = .05;



  @Override
  public void periodic() {
    if (isShooting) {
      double rps = TurretMath.turretRPS;
      SmartDashboard.putNumber("Shooter Velocity (RPS)", rps);
      shooter.leftShooterMotor.setControl(m_request.withVelocity(rps).withFeedForward(.5));

      if (reachedSpeed || shooter.leftShooterMotor.getVelocity().getValueAsDouble() > rps - (rps * 0.03)) {
        // if (reachedSpeed || shooter.leftShooterMotor.getVelocity().getValueAsDouble()
        // > .1) {
        reachedSpeed = true;

        double elevatorVel = Math.abs(elevator.elevatorMotor.getVelocity().getValueAsDouble());
        double targetElevatorRps = rps / 100;
        double indexerVel = Math.abs(indexer.indexerMotor.getVelocity().getValueAsDouble());
        double agitatorVel = Math.abs(indexer.agitatorMotor.getVelocity().getValueAsDouble());
        double targetIndexerRps = Math.abs(Constants.IndexerConstants.INDEXER_SPEED);
        double targetAgitatorSpeed = Math.abs(Constants.IndexerConstants.AGITATOR_SPEED);


        // // Are we unjamming the elavator?
        // if (elevatorUnjamming) {
        //   // Has it done a the correct # of rotation?
        //   if (Math.abs(elvPosInit - elevator.elevatorMotor.getPosition().getValueAsDouble()) >= unjamRot) {
        //     elvPosInit = 0;
        //     elevatorUnjamming = false;
        //     elevatorToSpeed = false;
        //   }
        // } else {
        //   // Is the elevator currently jamming
        //   if (elevatorVel < targetElevatorRps * unjamPercentThreshold) {
        //     elvPosInit = elevator.elevatorMotor.getPosition().getValueAsDouble();
        //     elevatorUnjamming = true;
        //   }
        // }

        //UNJAMMING CODE
        // Are we unjamming the indexer?
        if (indexerUnjamming) {
          // Has it done a the correct # of rotation?
          if (Math.abs(indPosInit - indexer.indexerMotor.getPosition().getValueAsDouble()) >= unjamRot) {
            indPosInit = 0;
            indexerUnjamming = false;
          }
        } else {
          // Is the indexer currently jamming
          if (indexerVel < targetIndexerRps * targetAgitatorSpeed) {
            indPosInit = indexer.indexerMotor.getPosition().getValueAsDouble();
            indexerUnjamming = true;
          }
        }

        // Are we unjamming the agitator?
        if (agitatorUnjamming) {
          // Has it done a the correct # of rotation?
          if (Math.abs(agiPosInit - indexer.agitatorMotor.getPosition().getValueAsDouble()) >= unjamRot) {
            agiPosInit = 0;
            agitatorUnjamming = false;
          }
        } else {
          // Is the indexer currently jamming
          if (agitatorVel < targetAgitatorSpeed * agitatorUnjamPercentThreshold) {
            agiPosInit = indexer.agitatorMotor.getPosition().getValueAsDouble();
            agitatorUnjamming = true;
          }
        }

        elevator.elevatorMotor.set(-rps / 100);

        // Unjam
        if (indexerUnjamming || agitatorUnjamming) {
          indexer.indexerMotor.set(-rps / 100);
          indexer.agitatorMotor.set(rps / 200);
          // elevator.elevatorMotor.set(-rps / 400);
        } else {
          indexer.indexerMotor.set(rps / 150);
          indexer.agitatorMotor.set(-rps / 100);
          // elevator.elevatorMotor.set(-rps / 50);
        }

      }
    } else if (isReversing) {
      shooter.leftShooterMotor.set(-.5);
      elevator.elevatorMotor.set(-.5);
      indexer.indexerMotor.set(-.5);
      indexer.agitatorMotor.set(-.5);
    }
  }

  public Command runShooter() {
    return Commands.runOnce(() -> {
      reachedSpeed = false;
      isReversing = false;
      isShooting = true;
      indexer.scoring = true;
    }, shooter, elevator, indexer);
  }

  public Command reverseSystem() {
    return Commands.runOnce(() -> {
      reachedSpeed = false;
      isReversing = true;
      isShooting = false;
      indexer.scoring = true;
    }, shooter, elevator, indexer);
  }

  public Command stopShooter() {
    return Commands.runOnce(() -> {
      reachedSpeed = false;
      elevatorToSpeed = false;
      elevatorUnjamming = false;
      indexerUnjamming = false;
      isShooting = false;
      indexer.scoring = false;
      isReversing = false;
      shooter.leftShooterMotor.set(0);
      elevator.elevatorMotor.set(0);
      indexer.indexerMotor.set(0);
      indexer.agitatorMotor.set(0);
    }, shooter, elevator, indexer);
  }
}