package frc.robot.subsystems;


import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase{

    private final SwerveDrive swerveDrive;

    private StructPublisher<Pose2d> odometryPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("SwervePose", Pose2d.struct).publish();

    
    
    public SwerveSubsystem() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            swerveDrive = new SwerveParser(
                new File(Filesystem.getDeployDirectory(), "swerve")
            ).createSwerveDrive(4.2);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(true);
        swerveDrive.setChassisDiscretization(true, 0.02);

        
    }

    public void drive(double x, double y, double omega) {
        /*ChassisSpeeds continuousSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, swerveDrive.getOdometryHeading());
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(continuousSpeeds, 0.02);
        swerveDrive.drive(discreteSpeeds, false, new Translation2d());*/
        Translation2d translation = new Translation2d(
            x * swerveDrive.getMaximumVelocity(),
            y * swerveDrive.getMaximumVelocity());

        double rotation = omega * swerveDrive.getMaximumAngularVelocity();
        swerveDrive.drive(translation,
            rotation,
            true,
            false
        );
        Logger.recordOutput("DriveTranslation", translation);
        Logger.recordOutput("DriveRotation", rotation);
    }

    public Command driveCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier omegaSpeed) {
        return run(
            () -> {
                drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), omegaSpeed.getAsDouble());
                
            }
        );
    }



    @Override
    public void periodic() {
        SmartDashboard.putNumber("Module 0 Velocity", swerveDrive.getModules()[0].getDriveMotor().getVelocity());
        SmartDashboard.putNumber("Module 1 Velocity", swerveDrive.getModules()[1].getDriveMotor().getVelocity());
        SmartDashboard.putNumber("Module 2 Velocity", swerveDrive.getModules()[2].getDriveMotor().getVelocity());
        SmartDashboard.putNumber("Module 3 Velocity", swerveDrive.getModules()[3].getDriveMotor().getVelocity());
        Logger.recordOutput("Swerve stuff", swerveDrive.getPose());
    }
    
}
