package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TurretMath;
import frc.robot.Constants.TurretConstants;
import frc.robot.StateEnums.TurretState;

import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;

public class Turret extends SubsystemBase {
  public TalonFX yawMotor = new TalonFX(Constants.TurretConstants.TURRET_MOTOR_ID);
  public Pose2d turretPose;
  public TurretMath turretMath;
  public boolean isRedAlliance;
  public CommandSwerveDrivetrain drivetrain;
  public boolean isAiming = false;

  public Turret(CommandSwerveDrivetrain d, TurretMath t, boolean a) {
    this.drivetrain = d;
    this.turretMath = t;
    this.isRedAlliance = a;
    TurretMath.calculateTarget(isRedAlliance, drivetrain);

    var talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = TurretConstants.kP;
    slot0Configs.kI = TurretConstants.kI;
    slot0Configs.kD = TurretConstants.kD;
    slot0Configs.kS = TurretConstants.kS;
    slot0Configs.kV = TurretConstants.kV;
    slot0Configs.kA = TurretConstants.kA;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 100;
    motionMagicConfigs.MotionMagicAcceleration = 1000;
    motionMagicConfigs.MotionMagicJerk = 10000;

    yawMotor.getConfigurator().apply(talonFXConfigs);
    yawMotor.setPosition(0 / 360);
  }

  final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private double TURRET_GEAR_RATIO = 6.4;

  private double normalizeAngle(double deg) { return MathUtil.inputModulus(deg, -180.0, 180.0); }

  public void setYaw(double angleDeg) { yawMotor.setControl(new MotionMagicExpoVoltage(TURRET_GEAR_RATIO * normalizeAngle(angleDeg) / 360)); }

  public Command setYawCommand(double angleDeg) {
    double normalized = normalizeAngle(angleDeg);
    double targetAngle = TURRET_GEAR_RATIO * normalized / 360;
    return run(() -> yawMotor.setControl(new MotionMagicExpoVoltage(targetAngle)));
  }

  public Command aimTurret() { return Commands.runOnce(() -> TurretState.Aiming = true, this); }
  public Command stopAiming() { return Commands.runOnce(() -> { TurretState.Aiming = false; setYaw(0); }, this); }

}