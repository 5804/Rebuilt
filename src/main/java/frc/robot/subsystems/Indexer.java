package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.StateEnums.IndexerState;

public class Indexer extends SubsystemBase {
  public TalonFX indexerMotor;

  public Indexer() {
    indexerMotor = new TalonFX(IndexerConstants.INDEXER_MOTOR_ID);

    var indexerMotorConfig = new TalonFXConfiguration();
    indexerMotorConfig
        .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(15).withSupplyCurrentLimitEnable(true));
    indexerMotor.getConfigurator().apply(indexerMotorConfig);
  }

  public Command runIndexer(boolean manual) {
    return Commands.runOnce(() -> {
      IndexerState.Reversing = false;
      IndexerState.Unjamming = false;
      IndexerState.ReachedSpeed = false;
      IndexerState.Manual = manual;
      IndexerState.Running = true;
    }, this);
  }

  public Command reverseIndexer(boolean manual) {
    return Commands.runOnce(() -> {
      IndexerState.Running = false;
      IndexerState.Unjamming = false;
      IndexerState.ReachedSpeed = false;
      IndexerState.Manual = manual;
      IndexerState.Reversing = true;
    }, this);
  }

  public Command stopIndexer() {
    return Commands.runOnce(() -> {
      IndexerState.Running = false;
      IndexerState.Reversing = false;
      IndexerState.Unjamming = false;
      IndexerState.ReachedSpeed = false;
      IndexerState.Manual = false;
      indexerMotor.set(0);
    }, this);
  }
}