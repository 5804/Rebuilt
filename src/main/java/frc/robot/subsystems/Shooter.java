package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Constants;
import frc.robot.TurretMath;

public class Shooter extends SubsystemBase {
  public TalonFX rightShooterMotor;
  public TalonFX leftShooterMotor;
  public boolean isRunning = false;
  public boolean isReversing = false;
  public boolean reachedSpeed = false;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  public Shooter() {
    rightShooterMotor = new TalonFX(Constants.ShooterConstants.RIGHT_SHOOTER_MOTOR_ID);
    leftShooterMotor = new TalonFX(Constants.ShooterConstants.LEFT_SHOOTER_MOTOR_ID);

    rightShooterMotor.setControl(new Follower(Constants.ShooterConstants.LEFT_SHOOTER_MOTOR_ID, MotorAlignmentValue.Opposed));

    var rightShooterMotorConfig = new TalonFXConfiguration();
    var leftShooterMotorConfig = new TalonFXConfiguration();

    var leftShooterMotorSlot0Configs = leftShooterMotorConfig.Slot0;
    leftShooterMotorSlot0Configs.kS = 0;
    leftShooterMotorSlot0Configs.kV = .11;
    leftShooterMotorSlot0Configs.kP = .3;
    leftShooterMotorSlot0Configs.kI = 0;
    leftShooterMotorSlot0Configs.kD = 0;

    leftShooterMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    //rightShooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    leftShooterMotorConfig.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(40).withSupplyCurrentLimitEnable(true));
    rightShooterMotorConfig.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(40).withSupplyCurrentLimitEnable(true));
    rightShooterMotor.getConfigurator().apply(rightShooterMotorConfig);
    leftShooterMotor.getConfigurator().apply(leftShooterMotorConfig);
  }

  @Override
  public void periodic() {
    if (leftShooterMotor.getVelocity().getValueAsDouble() > TurretMath.turretRPS - (TurretMath.turretRPS * 0.03)) this.reachedSpeed = true;
    if (isRunning) {
      leftShooterMotor.setControl(velocityRequest.withVelocity(TurretMath.turretRPS).withFeedForward(.5));
    } else if (isReversing) {
      leftShooterMotor.setControl(velocityRequest.withVelocity(-TurretMath.turretRPS).withFeedForward(.5));
    }
  }

  public Command runShooter() { return runOnce(() -> { isRunning = true;  isReversing = false; }); }
  public Command reverseShooter() { return runOnce(() -> { isRunning = false; isReversing = true; }); }
  public Command stopShooter() { return runOnce(() -> { this.reachedSpeed = false; isRunning = false; isReversing = false; leftShooterMotor.set(0); }); }

  public Command increaseShooterSpeed() { return run(() -> Constants.ShooterConstants.SHOOTER_SPEED += 0.1); }
  public Command decreaseShooterSpeed() { return run(() -> Constants.ShooterConstants.SHOOTER_SPEED -= 0.1); }
}