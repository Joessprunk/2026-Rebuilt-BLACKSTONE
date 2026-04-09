// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.StartIntaking;
import frc.robot.commands.StartManualShooting;
import frc.robot.commands.StartPassing;
import frc.robot.commands.StartShooting;
import frc.robot.commands.StartShootingAuto;
import frc.robot.commands.StartStealing;
import frc.robot.commands.StartVomiting;
import frc.robot.commands.StopIntaking;
import frc.robot.commands.StopManualShooting;
import frc.robot.commands.StopPassing;
import frc.robot.commands.StopShooting;
import frc.robot.commands.drive.AimToHubCmd;
import frc.robot.commands.drive.AimToPassCmd;
import frc.robot.commands.drive.ArcadeDriveCmd;
import frc.robot.commands.drive.AutoAimDrive;
import frc.robot.commands.drive.AutoPassDrive;
import frc.robot.commands.drive.LockCmd;
import frc.robot.commands.indexer.SetFloorRollerRPM;
import frc.robot.commands.indexer.SetTowerRollerRPM;
import frc.robot.commands.intake.ResetPivotAngle;
import frc.robot.commands.intake.SetTargetPivotAngle;
import frc.robot.commands.intake.SetIntakeRollerRPM;
import frc.robot.commands.turret.DecrementHoodOffset;
import frc.robot.commands.turret.DecrementFlywheelOffset;
import frc.robot.commands.turret.IncrementHoodOffset;
import frc.robot.commands.turret.IncrementFlywheelOffset;
import frc.robot.commands.turret.SetManualFlywheelRPM;
import frc.robot.commands.turret.SetManualHoodAngle;
import frc.robot.commands.turret.StopManualFlywheelRPM;
import frc.robot.subsystems.IndexerSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.TurretSys;
import frc.robot.subsystems.drive.PoseEstimator;
import frc.robot.subsystems.drive.SwerveDrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final SwerveDrive swerveDrive = new SwerveDrive();
	private final PoseEstimator poseEstimator = new PoseEstimator(swerveDrive);
	private final IntakeSys intakeSys = new IntakeSys();
	private final IndexerSys indexerSys = new IndexerSys();
	private final TurretSys turretSys = new TurretSys(poseEstimator);

	private final CommandXboxController driverController = new CommandXboxController(
			ControllerConstants.kDriverControllerPort);
	private final CommandXboxController operatorController = new CommandXboxController(
			ControllerConstants.kOperatorControllerPort);

	private final SendableChooser<Command> autoChooser;
	private RobotConfig config;

	public RobotContainer() {

		// register named commands
		NamedCommands.registerCommand("StartIntaking", new StartIntaking(intakeSys, indexerSys));
		NamedCommands.registerCommand("StopIntaking", new StopIntaking(intakeSys, indexerSys));
		NamedCommands.registerCommand("StartShooting", new StartShootingAuto(turretSys, indexerSys, intakeSys));
		NamedCommands.registerCommand("StopShooting", new StopShooting(turretSys, indexerSys, intakeSys));
		NamedCommands.registerCommand("ManualShoot", new StartManualShooting(turretSys, indexerSys, intakeSys));
		// configure autobuilder
		try {
			config = RobotConfig.fromGUISettings();
		} catch (Exception e) {
			e.printStackTrace();
		}

		AutoBuilder.configure(
				poseEstimator::getPose,
				poseEstimator::resetPose,
				swerveDrive::getRobotRelativeSpeeds,
				(chassisSpeeds, feedforward) -> swerveDrive.driveRobotRelative(chassisSpeeds),
				new PPHolonomicDriveController(
						new PIDConstants(SwerveDriveConstants.autoTranslationKp,
								SwerveDriveConstants.autoTranslationKd),
						new PIDConstants(SwerveDriveConstants.autoRotationKp, SwerveDriveConstants.autoRotationKd)),
				config,
				() -> {
					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
				},
				swerveDrive);

		// create test autos
		//new PathPlannerAuto("SquigglePathTest");
		//new PathPlannerAuto("TranslationTest");
		//new PathPlannerAuto("TurningWhileMovingTest");
		

		// create competition autos
		//new PathPlannerAuto("LoadingStation");
		new PathPlannerAuto("Left");
		new PathPlannerAuto("Right");
		new PathPlannerAuto("OtherLeft");
		

		// build auto chooser
		autoChooser = AutoBuilder.buildAutoChooser("Do Nothing");

		// send auto chooser to dashboard
		SmartDashboard.putData("auto chooser", autoChooser);

		// Configure the trigger bindings
		configureBindings();
	}

	private void configureBindings() {
		// driver controls for competition
		swerveDrive.setDefaultCommand(new ArcadeDriveCmd(
			() -> MathUtil.applyDeadband(driverController.getLeftY(), ControllerConstants.joystickDeadband),
			() -> MathUtil.applyDeadband(driverController.getLeftX(), ControllerConstants.joystickDeadband),
			() -> MathUtil.applyDeadband(driverController.getRightX(), ControllerConstants.joystickDeadband),
			true,
			swerveDrive,
			poseEstimator));

		if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
			driverController.y().whileTrue(
   		 new AutoAimDrive(
      		  swerveDrive,
       		 poseEstimator,
       		 () -> driverController.getLeftY(),
        	 () -> driverController.getLeftX())
    	);
		} else {
			driverController.y().whileTrue(
    	 new AutoAimDrive(
        	swerveDrive,
        	poseEstimator,
        	() -> -driverController.getLeftY(),
        	() -> -driverController.getLeftX())
    	);
		}
		
if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
			driverController.a().whileTrue(
    new AutoPassDrive(
        swerveDrive,
        poseEstimator,
        () -> driverController.getLeftY(),
        () -> driverController.getLeftX())
    );
		} else {
			driverController.a().whileTrue(
    new AutoPassDrive(
        swerveDrive,
        poseEstimator,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX())
    );
		}
		
		driverController.start().onTrue(Commands.runOnce(() -> poseEstimator.resetHeading(), poseEstimator));
    
			driverController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, ControllerConstants.triggerPressedThreshold)
			.onTrue(new StartIntaking(intakeSys, indexerSys))
			.onFalse(new StopIntaking(intakeSys, indexerSys));

			driverController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, ControllerConstants.triggerPressedThreshold)
			.onTrue(new StartShooting(turretSys, indexerSys, intakeSys /*SwerveDrive swerveSys, PoseEstimator poseEstimator*/))
			.onFalse(new StopShooting(turretSys, indexerSys, intakeSys));

			driverController.rightBumper()
			.onTrue(new StartVomiting(turretSys, indexerSys, intakeSys))
			.onFalse(new StopManualShooting(turretSys, indexerSys, intakeSys));
			driverController.leftBumper()
			.onTrue(new StartPassing(turretSys, indexerSys /*swerveDrive, poseEstimator*/))
			.onFalse(new StopPassing(turretSys, indexerSys, intakeSys));

			driverController.b()
			.onTrue(new StartStealing(turretSys, indexerSys, intakeSys))
			.onFalse(new StopManualShooting(turretSys, indexerSys, intakeSys));

			// driverController.b().onTrue(new SetTargetPivotAngle(intakeSys, 40.0));
			// driverController.a().onTrue(new SetTargetPivotAngle(intakeSys, IntakeConstants.intakingPivotAngle));

			// driverController.x().onTrue(new AimToHubCmd(swerveDrive, poseEstimator));
			// driverController.y().onTrue(new AimToPassCmd(swerveDrive, poseEstimator));


			// for tuning
			// driverController.povUp().onTrue(new IncrementFlywheelOffset(turretSys));
			// driverController.povDown().onTrue(new DecrementFlywheelOffset(turretSys));
			// driverController.povLeft().onTrue(new DecrementHoodOffset(turretSys));
			// driverController.povRight().onTrue(new IncrementHoodOffset(turretSys));

			
			

		



		// // operator bindings for competition
		 operatorController.povUp().onTrue(new IncrementFlywheelOffset(turretSys));
		 operatorController.povDown().onTrue(new DecrementFlywheelOffset(turretSys));
		  operatorController.povRight().onTrue(new IncrementHoodOffset(turretSys));
		 operatorController.povLeft().onTrue(new DecrementHoodOffset(turretSys));

		 operatorController.back().onTrue(new ResetPivotAngle(intakeSys));

		 operatorController.b().onTrue(new SetTargetPivotAngle(intakeSys, 40.0));
		 operatorController.a().onTrue(new SetTargetPivotAngle(intakeSys, IntakeConstants.intakingPivotAngle));
		 operatorController.y().onTrue(new SetTargetPivotAngle(intakeSys, 0.0));

		 operatorController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, ControllerConstants.triggerPressedThreshold)
			.onTrue(new StartManualShooting(turretSys, indexerSys, intakeSys /*SwerveDrive swerveSys, PoseEstimator poseEstimator*/))
			.onFalse(new StopManualShooting(turretSys, indexerSys, intakeSys));


		 

		 //operatorController.y().onTrue(new ToggleIsPassing(turretSys));
		
		// operatorController.start().onTrue(new ToggleIsPassing(turretSys));
		

		// operatorController

	

		// // binding commands for swerve sysID
		// // driverController.a().onTrue(swerveDrive.driveSysIdDynamicForward());
		// // driverController.b().onTrue(swerveDrive.driveSysIdDynamicReverse());
		// // driverController.x().onTrue(swerveDrive.driveSysIdQuasistaticForward());
		// // driverController.y().onTrue(swerveDrive.driveSysIdQuasistaticReverse());

		// // binding commands for turret sysID
		// // driverController.a().onTrue(turretSys.sysIdDynamicForward());
		// // driverController.b().onTrue(turretSys.sysIdDynamicReverse());
		// // driverController.x().onTrue(turretSys.sysIdQuasistaticForward());
		// // driverController.y().onTrue(turretSys.sysIdQuasistaticReverse());

		

		// flywheel RPM control bindings for testing
		// operatorController.x().onTrue(new SetManualFlywheelRPM(turretSys, 0.0));
		// operatorController.b().onTrue(new SetManualFlywheelRPM(turretSys, 2500.0));
		// operatorController.a().onTrue(new SetManualFlywheelRPM(turretSys, 2000.0));
		// operatorController.y().onTrue(new SetManualFlywheelRPM(turretSys, 3000.0));

		// hood angle control bindings for testing
		// operatorController.x().onTrue(new SetManualHoodAngle(turretSys, 0.0));
		// operatorController.b().onTrue(new SetManualHoodAngle(turretSys, 20.0));
		// operatorController.a().onTrue(new SetManualHoodAngle(turretSys, 10.0));
		// operatorController.y().onTrue(new SetManualHoodAngle(turretSys, 30.0));

		// Indexer RPM control bindings for testing
		// operatorController.a().onTrue(new SetTowerRollerRPM(indexerSys, 100.0));
		// operatorController.b().onTrue(new SetTowerRollerRPM(indexerSys, 500.0));
		// operatorController.x().onTrue(new SetTowerRollerRPM(indexerSys, 0.0));
		// operatorController.y().onTrue(new SetTowerRollerRPM(indexerSys, 1000.0));

		// Indexer floor roller RPM control bindings for testing
        // operatorController.a().onTrue(new SetFloorRollerRPM(indexerSys, 750.0));
		// operatorController.b().onTrue(new SetFloorRollerRPM(indexerSys, 500.0));
		// operatorController.x().onTrue(new SetFloorRollerRPM(indexerSys, 0.0));
		// operatorController.y().onTrue(new SetFloorRollerRPM(indexerSys, 1000.0));
		// operatorController.start().onTrue(new SetTargetPivotAngle(intakeSys, 30));

		// intake roller RPM control bindings for testing
		// operatorController.x().onTrue(new SetIntakeRollerRPM(intakeSys, 0.0));
		// operatorController.b().onTrue(new SetIntakeRollerRPM(intakeSys, 2000.0));
		// operatorController.y().onTrue(new SetIntakeRollerRPM(intakeSys, 5000.0));
		// operatorController.a().onTrue(new SetIntakeRollerRPM(intakeSys, 500.0));

		// // intake pivot position control bindings for testing
		// operatorController.a().onTrue(new SetTargetPivotAngle(intakeSys, 50.0));
		// operatorController.b().onTrue(new SetTargetPivotAngle(intakeSys, 80.0));
		// operatorController.x().onTrue(new SetTargetPivotAngle(intakeSys, 0.0));
		// operatorController.y().onTrue(new SetTargetPivotAngle(intakeSys,120.0));
		// operatorController.start().onTrue(new ResetPivotAngle(intakeSys));

		
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
		// return new ExampleCommand(null);
		// return new Command() {
	};

	public void updateDashboard() {
		// drive base
		// SmartDashboard.putNumber("FL CANcoder",
		// swerveDrive.getCanCoderAngles()[0].getDegrees());
		// SmartDashboard.putNumber("FR CANcoder",
		// swerveDrive.getCanCoderAngles()[1].getDegrees());
		// SmartDashboard.putNumber("BL CANcoder",
		// swerveDrive.getCanCoderAngles()[2].getDegrees());
		// SmartDashboard.putNumber("BR CANcoder",
		// swerveDrive.getCanCoderAngles()[3].getDegrees());
		// SmartDashboard.putNumber("field relative speed",
		// swerveDrive.getRobotVelocity());

		// pose info from pose estimator
		SmartDashboard.putNumber("pos-x", poseEstimator.getPose().getX());
		SmartDashboard.putNumber("pos-y", poseEstimator.getPose().getY());
		SmartDashboard.putNumber("pos-rot", poseEstimator.getHeading().getDegrees());
		// SmartDashboard.putNumber("gyro heading",
		// swerveDrive.getHeading().getDegrees());

		// turret azimuth info
		// SmartDashboard.putNumber("manual azimuth angle rads",
		// turretSys.getAzimuthManualTargetDeg());
		
		//SmartDashboard.putNumber("target azimuth angle rad", turretSys.calculateTargetAzimuthAngleShooting());
		// SmartDashboard.putNumberArray("turret pose", new double[] {
		// 		turretSys.getTurretPose().getTranslation().getX(),
		// 		turretSys.getTurretPose().getTranslation().getY() });
		SmartDashboard.putBoolean("is Aiming", turretSys.getIsAiming());
		SmartDashboard.putBoolean("is Passing", turretSys.getIsPassing());
		

		// turret flywheel
		SmartDashboard.putNumber("flywheel current RPM", turretSys.getFlywheelRPM());
		SmartDashboard.putNumber("flywheel target RPM", turretSys.calculateTargetFlywheelRPM());
		SmartDashboard.putNumber("Hood Target Angle", turretSys.calculateTargetHoodAngle());
		SmartDashboard.putNumber("flywheel manual rpm", turretSys.getManualFlywheelRPM());
		SmartDashboard.putNumber("distance to target", turretSys.calculateDistanceToTarget());
		SmartDashboard.putBoolean("is Firing", turretSys.getIsFiring());
		SmartDashboard.putNumber("flywheel offset RPM", turretSys.getFlywheelOffsetRPM());
		SmartDashboard.putBoolean("is at RPM", turretSys.isAtSpeed());
		SmartDashboard.putNumber("current hood angle", turretSys.getCurrentHoodAngleRad());
		// indexer info
		SmartDashboard.putNumber("tower RPM", indexerSys.getTowerRollerRPM());
		SmartDashboard.putNumber("floor RPM", indexerSys.getFloorRollerRPM());
		SmartDashboard.putNumber("hood Offset", turretSys.getHoodOffsetDeg());

		// intake info
		SmartDashboard.putNumber("actuator position angle", intakeSys.getPivotAngle());
		SmartDashboard.putNumber("roller RPM", intakeSys.getRollerRPM());

		// climber info
		

		// field
		SmartDashboard.putData("robot field", poseEstimator.getField());
		// SmartDashboard.putData("turret field", turretSys.getTurretField());
	}
}