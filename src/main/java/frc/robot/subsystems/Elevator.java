package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  public TalonFX elevatorMotor;
  public TalonFX tempIndexMotor;
  public boolean isRunning = false;
  public boolean isReversing = false;
  public double speed = 0;
  public double defaultSupplyLimit = 30;
  public double previousSupplyLimit = -1;
  public TalonFXConfiguration elevatorMotorConfig;

  public Elevator() {
    elevatorMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_MOTOR_ID);
    
    elevatorMotorConfig = new TalonFXConfiguration();
    elevatorMotor.getConfigurator().apply(elevatorMotorConfig);
    SmartDashboard.putNumber("ElevatorSupplyLimit", defaultSupplyLimit);
  }

  @Override
  public void periodic() {
    if (isRunning) elevatorMotor.set(speed);
    else if (isReversing) elevatorMotor.set(-speed);
    double elevatorSupplyLimit = SmartDashboard.getNumber("ElevatorSupplyLimit", defaultSupplyLimit); // The elevator motors will not stop if the current limit is set to 0
    if (previousSupplyLimit != elevatorSupplyLimit) {
        elevatorMotorConfig.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(elevatorSupplyLimit).withSupplyCurrentLimitEnable(true));
        elevatorMotor.getConfigurator().apply(elevatorMotorConfig);
        previousSupplyLimit = elevatorSupplyLimit;
    }
  }

  public Command runElevator(double speed) { return Commands.runOnce(() -> { isRunning = true; isReversing = false; this.speed = speed; }, this); }
  public Command reverseElevator(double speed) { return Commands.runOnce(() -> { isReversing = true; isRunning = false; this.speed = speed; }, this); }
  public Command stopElevator() { return Commands.runOnce(() -> { isRunning = false; isReversing = false; elevatorMotor.set(0); }, this); }
}