package frc.robot.commands.drive;

import java.util.Optional;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.drive.PoseEstimator;
import frc.robot.subsystems.drive.SwerveDrive;

public class AimToHubCmd extends Command {

    /**
     * Command to allow for driver input in teleop
     */
    private final SwerveDrive swerveSys;
    private final PoseEstimator poseEstimator;
    private final ProfiledPIDController thetaController;

    // private final ProfiledPIDController aimController;

    /**
     * Joysticks return DoubleSuppliers when the get methods are called
     * This is so that joystick getter methods can be passed in as a parameter but will continuously update, 
     * versus using a double which would only update when the constructor is called
     */
   

    /**
     * Constructs a new ArcadeDriveCmd.
     * 
     * <p>ArcadeDriveCmd is used to control the swerve drive base with arcade drive.
     * 
     */
    public AimToHubCmd(SwerveDrive swerveSys, PoseEstimator poseEstimator) {
        this.swerveSys = swerveSys;
        this.poseEstimator = poseEstimator;

         thetaController = new ProfiledPIDController(
            Constants.SwerveDriveConstants.autoAimkP, 0.0, Constants.SwerveDriveConstants.autoAimkD,
            new Constraints(
                Constants.SwerveDriveConstants.autoAimOmegaMaxRadPerSec,
                Constants.SwerveDriveConstants.autoAimOmegaDotMaxDegPerSecSq));

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveSys, poseEstimator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            swerveSys.setOmegaOverrideRadPerSec(Optional.of(
                thetaController.calculate(
                    poseEstimator.getPose().getRotation().getDegrees(),    
                    TurretConstants.targetPoseBlue.getTranslation()
                    .minus(poseEstimator.getPose().getTranslation())
                    .getAngle().getDegrees())));
        } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
           swerveSys.setOmegaOverrideRadPerSec(Optional.of(
                thetaController.calculate(
                    poseEstimator.getPose().getRotation().getDegrees(),    
                    TurretConstants.targetPoseBlue.getTranslation()
                    .minus(poseEstimator.getPose().getTranslation())
                    .getAngle().getDegrees())));
        } else {
          swerveSys.setOmegaOverrideRadPerSec(Optional.of(
                thetaController.calculate(
                    poseEstimator.getPose().getRotation().getDegrees(),    
                    TurretConstants.targetPoseBlue.getTranslation()
                    .minus(poseEstimator.getPose().getTranslation())
                    .getAngle().getDegrees())));
        }
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveSys.setOmegaOverrideRadPerSec(Optional.empty());
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}