package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  public TalonFX elevatorMotor;
  public TalonFXS tempIndexMotor; // temp

  // public Elevator() { elevatorMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_MOTOR_ID); } // uncomment when index runs off elevator

  public Elevator() {
    elevatorMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_MOTOR_ID);
    tempIndexMotor = new TalonFXS(0);
    TalonFXSConfiguration indexConfig = new TalonFXSConfiguration();
    indexConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    tempIndexMotor.getConfigurator().apply(indexConfig);
  } // delete this constructor when indexer changed to run off elevator

  public Command runElevator() { return run(() -> { elevatorMotor.set(Constants.ElevatorConstants.ELEVATOR_SPEED); }); }

  public Command stopElevator() { return run(() -> { elevatorMotor.set(0); }); }
}