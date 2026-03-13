package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  public TalonFX elevatorMotor;

  public Elevator() { 
    elevatorMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_MOTOR_ID); 
  
  }

  public Command runElevator() { return run(() -> { elevatorMotor.set(Constants.ElevatorConstants.ELEVATOR_SPEED); }); }

  public Command stopElevator() { return run(() -> { elevatorMotor.set(0); }); }
}