// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.turret.SetManualFlywheelRPM;
import frc.robot.commands.turret.SetManualHoodAngle;
import frc.robot.commands.turret.StartAiming;
import frc.robot.commands.turret.StartFlywheelAndHood;
import frc.robot.subsystems.IndexerSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.TurretSys;
import frc.robot.subsystems.drive.PoseEstimator;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.drive.AimToHubCmd;
import frc.robot.commands.indexer.SetFloorRollerRPM;
import frc.robot.commands.indexer.SetTowerRollerRPM;
import frc.robot.commands.intake.SetTargetPivotAngle;
import frc.robot.commands.intake.SetIntakeRollerRPM;

/** An example command that uses an example subsystem. */
public class StartStealing extends SequentialCommandGroup {

  public StartStealing(TurretSys turretSys, IndexerSys indexerSys, IntakeSys intakeSys /*SwerveDrive swerveSys, PoseEstimator poseEstimator*/) {
    super(
       new SetManualFlywheelRPM(turretSys, 2600),
        new SetManualHoodAngle(turretSys, 27.5),
        new WaitCommand(1.0),
        new SetTowerRollerRPM(indexerSys, IndexerConstants.towerRollerShootingRPM),
        new SetFloorRollerRPM(indexerSys, IndexerConstants.floorRollerShootingRPM),
        new SetIntakeRollerRPM(intakeSys, IntakeConstants.RollerShootingRPM),
        new WaitCommand(1.0),
        new SetTargetPivotAngle(intakeSys, Constants.IntakeConstants.PivotBufferPositionAngle)
    );
  }
}