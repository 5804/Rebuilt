// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climber;

public class RobotContainer {

    private final Joystick joystick = new Joystick(0);

    public int climbTriggerHeld = 0;
    public Climber climber = new Climber(() -> {
        return -1 * joystick.getRawAxis(1) * climbTriggerHeld;
    });


    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() { configureBindings(); }

    private void configureBindings() {
        

        
        // Climber Commands (Temp)

        // joystick.povRight().whileTrue(climber.extendActuator());
        // joystick.povLeft().whileTrue(climber.retractActuator());
          
        climber.setDefaultCommand(climber.setClimberSpeed());
        Trigger joystickTrigger = new Trigger(() -> { return joystick.getTrigger(); });
        
        joystickTrigger.whileTrue(new InstantCommand(() -> { climbTriggerHeld = 1; }));
        joystickTrigger.whileFalse(new InstantCommand(() -> { climbTriggerHeld = 0; }));

    }

    public Command getAutonomousCommand() { return autoChooser.getSelected(); }
}