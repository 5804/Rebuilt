package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.Constants;


public class Shooter extends SubsystemBase {
  public TalonFX rightShooterMotor;
  public TalonFX leftShooterMotor;
  /** Creates a new ExampleSubsystem. */
  public TalonFX shooterMotor;
  public Shooter() {
    rightShooterMotor = new TalonFX(Constants.ShooterConstants.RIGHT_SHOOTER_MOTOR_ID);
    leftShooterMotor = new TalonFX(Constants.ShooterConstants.LEFT_SHOOTER_MOTOR_ID);
    
    rightShooterMotor.setControl(new Follower(Constants.ShooterConstants.LEFT_SHOOTER_MOTOR_ID, MotorAlignmentValue.Opposed));

    var rightShooterMotorConfig = new TalonFXConfiguration();
    var leftShooterMotorConfig = new TalonFXConfiguration();

    var rightShooterMotorSlot0Configs = rightShooterMotorConfig.Slot0;
    var leftShooterMotorSlot0Configs = leftShooterMotorConfig.Slot0;

    leftShooterMotorSlot0Configs.kS = 0;//0.1;
    leftShooterMotorSlot0Configs.kV = .11; // 0.15;
    leftShooterMotorSlot0Configs.kP = .3;
    leftShooterMotorSlot0Configs.kI = 0;
    leftShooterMotorSlot0Configs.kD = 0;
    
    leftShooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rightShooterMotor.getConfigurator().apply(rightShooterMotorConfig);
    leftShooterMotor.getConfigurator().apply(leftShooterMotorConfig);
  }

  public Command runShooter() {
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    double bot_dist = 100;
    double shoot_set = (bot_dist * .68 + 13)/2 + 6;

    return run(() -> {
      leftShooterMotor.setControl(m_request.withVelocity(bot_dist).withFeedForward(.5)); //.5
    });
  }

  public Command stopShooter() {
    return run(() -> {
        leftShooterMotor.set(0);
      // Code to stop the indexer motor
    });
  }

  public Command increaseShooterSpeed(){
    return run(() -> {
        Constants.ShooterConstants.SHOOTER_SPEED +=0.1;    
    });
  }
  
  public Command decreaseShooterSpeed(){
    return run(() -> {
        Constants.ShooterConstants.SHOOTER_SPEED -=0.1;    
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