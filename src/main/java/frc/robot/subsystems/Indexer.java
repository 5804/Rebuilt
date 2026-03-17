package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  public TalonFX indexerMotor;

  public Indexer() { indexerMotor = new TalonFX(Constants.IndexerConstants.INDEXER_MOTOR_ID); }

  public Command runIndexer() { return run(() -> { indexerMotor.set(Constants.IndexerConstants.INDEXER_SPEED); }); }

  public Command reverseIndexer() { return run(() -> { indexerMotor.set(-Constants.IndexerConstants.INDEXER_SPEED); }); }

  public Command stopIndexer() { return run(() -> { indexerMotor.set(0); }); }
}