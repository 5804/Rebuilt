package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public TalonFXConfiguration leftShooterMotorConfig;
  public TalonFXConfiguration rightShooterMotorConfig;
  public double defaultSupplyLimit = 40;
  public double previousSupplyLimit = -1;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  public Shooter() {
    rightShooterMotor = new TalonFX(Constants.ShooterConstants.RIGHT_SHOOTER_MOTOR_ID);
    leftShooterMotor = new TalonFX(Constants.ShooterConstants.LEFT_SHOOTER_MOTOR_ID);

    rightShooterMotor.setControl(new Follower(Constants.ShooterConstants.LEFT_SHOOTER_MOTOR_ID, MotorAlignmentValue.Opposed));

    rightShooterMotorConfig = new TalonFXConfiguration();
    leftShooterMotorConfig = new TalonFXConfiguration();

    var leftShooterMotorSlot0Configs = leftShooterMotorConfig.Slot0;
    leftShooterMotorSlot0Configs.kS = 0;
    leftShooterMotorSlot0Configs.kV = .11;
    leftShooterMotorSlot0Configs.kP = .3;
    leftShooterMotorSlot0Configs.kI = 0;
    leftShooterMotorSlot0Configs.kD = 0;

    leftShooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rightShooterMotor.getConfigurator().apply(rightShooterMotorConfig);
    leftShooterMotor.getConfigurator().apply(leftShooterMotorConfig);
    SmartDashboard.putNumber("ShooterSupplyLimit", defaultSupplyLimit);
  }

  @Override
  public void periodic() {
    if (isRunning) {
      leftShooterMotor.setControl(velocityRequest.withVelocity(TurretMath.turretRPS).withFeedForward(.5));
    } else if (isReversing) {
      leftShooterMotor.setControl(velocityRequest.withVelocity(-TurretMath.turretRPS).withFeedForward(.5));
    }
    double shooterSupplyLimit = SmartDashboard.getNumber("ShooterSupplyLimit", defaultSupplyLimit); // The Shooter motors will not stop if the current limit is set to 0
    if (previousSupplyLimit != shooterSupplyLimit) {
        leftShooterMotorConfig.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(shooterSupplyLimit).withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLowerLimit(shooterSupplyLimit / 2)
        .withSupplyCurrentLowerTime(0.25));
        rightShooterMotorConfig.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(shooterSupplyLimit).withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLowerLimit(shooterSupplyLimit / 2)
        .withSupplyCurrentLowerTime(0.25));
        leftShooterMotor.getConfigurator().apply(leftShooterMotorConfig);
        rightShooterMotor.getConfigurator().apply(rightShooterMotorConfig);
        previousSupplyLimit = shooterSupplyLimit;
    }
  }

  public Command runShooter() { return runOnce(() -> { isRunning = true;  isReversing = false; }); }
  public Command reverseShooter() { return runOnce(() -> { isRunning = false; isReversing = true; }); }
  public Command stopShooter() { return runOnce(() -> { isRunning = false; isReversing = false; leftShooterMotor.set(0); }); }

  public Command increaseShooterSpeed() { return run(() -> Constants.ShooterConstants.SHOOTER_SPEED += 0.1); }
  public Command decreaseShooterSpeed() { return run(() -> Constants.ShooterConstants.SHOOTER_SPEED -= 0.1); }
}