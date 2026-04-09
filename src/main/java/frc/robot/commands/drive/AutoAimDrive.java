package frc.robot.commands.drive;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.drive.PoseEstimator;
import frc.robot.subsystems.drive.SwerveDrive;

public class AutoAimDrive extends Command {

    private final SwerveDrive swerveSys;
    private final PoseEstimator poseEstimator;

    private final Supplier<Double> xSupplier;
    private final Supplier<Double> ySupplier;

    private boolean isRed;

    public AutoAimDrive(
        SwerveDrive swerveSys,
        PoseEstimator poseEstimator,
        Supplier<Double> xSupplier,
        Supplier<Double> ySupplier
    ) {
        this.swerveSys = swerveSys;
        this.poseEstimator = poseEstimator;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        addRequirements(swerveSys);
    }

    @Override
    public void initialize() {
        isRed = DriverStation.getAlliance()
            .map(a -> a == DriverStation.Alliance.Red)
            .orElse(false);

       
        swerveSys.resetAutoAim(
            poseEstimator.getPose().getRotation().getDegrees()
        );
    }

    @Override
    public void execute() {

        Pose2d pose = poseEstimator.getPose();

        double currentDeg = pose.getRotation().getDegrees();

        double targetDeg;

       
        if (isRed) {
            targetDeg = TurretConstants.targetPoseRed.getTranslation()
                .minus(pose.getTranslation())
                .getAngle().getDegrees() + 180.0;
        } else {
            targetDeg = TurretConstants.targetPoseBlue.getTranslation()
                .minus(pose.getTranslation())
                .getAngle().getDegrees() + 180.0;
        }

        double omegaDeg = swerveSys.calculateAutoAimOmegaDeg(currentDeg, targetDeg);

        swerveSys.enableAutoAimDeg(omegaDeg);

       
        swerveSys.driveFieldRelative(
            xSupplier.get(),
            ySupplier.get(),
            0.0, // ignored because of override
            pose.getRotation()
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerveSys.disableAutoAim();
    }
}