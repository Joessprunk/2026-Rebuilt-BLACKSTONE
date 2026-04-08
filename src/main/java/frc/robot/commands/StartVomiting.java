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
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.indexer.SetFloorRollerRPM;
import frc.robot.commands.indexer.SetTowerRollerRPM;
import frc.robot.commands.intake.SetTargetPivotAngle;
import frc.robot.commands.intake.SetIntakeRollerRPM;

/** An example command that uses an example subsystem. */
public class StartVomiting extends SequentialCommandGroup {

  public StartVomiting(TurretSys turretSys, IndexerSys indexerSys, IntakeSys intakeSys) {
    super(
        // new LockCmd(swerveSys),
        new SetManualFlywheelRPM(turretSys, 1700),
        new SetManualHoodAngle(turretSys, 30),
        new WaitCommand(0.1),
        new SetTowerRollerRPM(indexerSys, IndexerConstants.towerRollerVomitingRPM),
        new SetFloorRollerRPM(indexerSys, IndexerConstants.floorRollerVomitingRPM),
        new SetIntakeRollerRPM(intakeSys, IntakeConstants.RollerVomitingRPM),
        new WaitCommand(0.1), // 2.0 works idk why others dont...
        new SetTargetPivotAngle(intakeSys, 10)

        
        

    );
  }
}