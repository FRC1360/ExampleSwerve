// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class GoToNoteCommand extends Command {
  SwerveSubsystem swerve;
  PIDController PIDController = new PIDController( 0.005, 0, 0);
  double yaw;
  DoubleSupplier xSpeed, ySpeed;
  
  public GoToNoteCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult result = swerve.photonCamera.getLatestResult();

    if(result.hasTargets()){
      yaw = result.getBestTarget().getYaw();
      SmartDashboard.putNumber("YAW!", yaw);
      double output = PIDController.calculate(yaw, 0.0);
      swerve.drive(-Math.sqrt(Math.pow(xSpeed.getAsDouble(), 2)+Math.pow(ySpeed.getAsDouble(), 2)), 0, output, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
