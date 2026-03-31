package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.StateEnums.ElevatorState;

public class Elevator extends SubsystemBase {
  public TalonFX elevatorMotor;
  public TalonFX tempIndexMotor;
  public boolean isRunning = false;
  public boolean isReversing = false;
  public double speed = 0;

  public Elevator() {
    elevatorMotor = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR_ID);

    var elevatorMotorConfig = new TalonFXConfiguration();
    elevatorMotorConfig
        .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(40).withSupplyCurrentLimitEnable(true));
    elevatorMotor.getConfigurator().apply(elevatorMotorConfig);
  }

  public Command runElevator(boolean manual) {
    return Commands.runOnce(() -> {
      ElevatorState.Reversing = false;
      ElevatorState.Unjamming = false;
      ElevatorState.ReachedSpeed = false;
      ElevatorState.Manual = manual;
      ElevatorState.Running = true;
    }, this);
  }

  public Command reverseElevator(boolean manual) {
    return Commands.runOnce(() -> {
      ElevatorState.Running = false;
      ElevatorState.Unjamming = false;
      ElevatorState.ReachedSpeed = false;
      ElevatorState.Manual = manual;
      ElevatorState.Reversing = true;
    }, this);
  }

  public Command stopElevator() {
    return Commands.runOnce(() -> {
      ElevatorState.Running = false;
      ElevatorState.Reversing = false;
      ElevatorState.Unjamming = false;
      ElevatorState.ReachedSpeed = false;
      ElevatorState.Manual = false;
      elevatorMotor.set(0);
    }, this);
  }
}