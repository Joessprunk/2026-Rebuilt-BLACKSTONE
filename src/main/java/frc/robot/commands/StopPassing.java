// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.turret.StopAiming;
import frc.robot.commands.turret.StopFlywheelAndHood;
import frc.robot.commands.turret.ToggleIsPassingFalse;
import frc.robot.subsystems.IndexerSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.TurretSys;

import frc.robot.commands.indexer.SetFloorRollerRPM;
import frc.robot.commands.indexer.SetTowerRollerRPM;
import frc.robot.commands.intake.SetIntakeRollerRPM;

/** An example command that uses an example subsystem. */
public class StopPassing extends SequentialCommandGroup {

  public StopPassing(TurretSys turretSys, IndexerSys indexerSys, IntakeSys intakeSys) {
    super(
        new ToggleIsPassingFalse(turretSys), 
       // new StopAiming(turretSys), MAKE DRIVE CHASSIS BASED
        new StopFlywheelAndHood(turretSys),
        new SetFloorRollerRPM(indexerSys, 0.0),
        new SetTowerRollerRPM(indexerSys, 0.0),
        new SetIntakeRollerRPM(intakeSys, 0.0)
    );
  }
}