// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.GoToNoteCommand;
import frc.robot.commands.RotateToNote;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    private final CommandJoystick leftJoystick = new CommandJoystick(0);
    private final CommandJoystick rightJoystick = new CommandJoystick(1);

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    swerveSubsystem.setDefaultCommand(
      swerveSubsystem.driveCommand(
        () -> deadband(leftJoystick.getY(), 0.1) * -1.0,
        () -> deadband(leftJoystick.getX(), 0.1) * -1.0,
        () -> deadband(rightJoystick.getX(), 0.1) * -1.0 * 0.5)
    );

    RotateToNote rotate = new RotateToNote(swerveSubsystem);
    rightJoystick.button(11).whileTrue(rotate);
    leftJoystick.button(1).whileTrue(new GoToNoteCommand(() -> deadband(leftJoystick.getX(), 0.1) * -1.0, () -> deadband(leftJoystick.getY(), 0.1) * -1.0, swerveSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private static double deadband(double input, double deadband) {
    double slope = 1 / (1-deadband); // m = rise/run
    double offset = 1 - slope; // b = y - mx
    if (input < 0.0) {
        return Math.abs(input) > deadband? (-1 * (slope * Math.abs(input) + offset)) : 0.0;
    } else if (input > 0.0) {
        return Math.abs(input) > deadband? (slope * Math.abs(input) + offset): 0.0;
    } else {
        return 0.0;
    }
  }

}
