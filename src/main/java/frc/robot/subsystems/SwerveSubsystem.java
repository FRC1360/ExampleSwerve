package frc.robot.subsystems;


import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.EstimatedRobotPose;

public class SwerveSubsystem extends SubsystemBase{

    private final SwerveDrive swerveDrive;
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private final PhotonCamera photonCamera = new PhotonCamera("Logitech_Camera_Brandon");

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
        swerveDrive.drive(new Translation2d(
            x * swerveDrive.getMaximumVelocity(),
            y * swerveDrive.getMaximumVelocity()),
            omega * swerveDrive.getMaximumAngularVelocity(),
            true,
            false
        );
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
        odometryPublisher.set(swerveDrive.getPose());

        double targetYaw = 0.0;
        var results = photonCamera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 0) {
                        targetYaw = target.getYaw();
                    }
                }
            }
        }

        SmartDashboard.putNumber("target_pitch", targetYaw);
    }
    
}
