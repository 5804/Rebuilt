package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  public TalonFX elevatorMotor;
  public TalonFX tempIndexMotor;
  public boolean isRunning = false;
  public boolean isReversing = false;

  public Elevator() { elevatorMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_MOTOR_ID); }

  @Override
  public void periodic() {
    if (isRunning) elevatorMotor.set(Constants.ElevatorConstants.ELEVATOR_SPEED);
    else if (isReversing) elevatorMotor.set(-Constants.ElevatorConstants.ELEVATOR_SPEED);
  }

  public Command runElevator() { return Commands.runOnce(() -> { isRunning = true; isReversing = false; }, this); }
  public Command reverseElevator() { return Commands.runOnce(() -> { isReversing = true; isRunning = false; }, this); }
  public Command stopElevator() { return Commands.runOnce(() -> { isRunning = false; elevatorMotor.set(0); }, this); }
}