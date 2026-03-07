package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public TalonFX elevatorMotor;
  public Elevator() {
       elevatorMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_MOTOR_ID);
       
  }

  public Command runElevator() {
    return run(() -> {
        elevatorMotor.set(Constants.ElevatorConstants.ELEVATOR_SPEED);
      // Code to run the elevator motor at a certain speed
    });
  }

  public Command stopElevator() {
    return run(() -> {
        elevatorMotor.set(0);
      // Code to stop the elevator motor
    });
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}