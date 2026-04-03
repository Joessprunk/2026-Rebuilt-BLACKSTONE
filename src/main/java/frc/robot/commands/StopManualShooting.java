// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.turret.StopAiming;
import frc.robot.commands.turret.StopFlywheel;
import frc.robot.commands.turret.StopManualFlywheelRPM;
import frc.robot.commands.turret.StopManualHoodAngle;
import frc.robot.subsystems.IndexerSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.TurretSys;

import frc.robot.commands.indexer.SetFloorRollerRPM;
import frc.robot.commands.indexer.SetTowerRollerRPM;
import frc.robot.commands.intake.SetIntakeRollerRPM;

/** An example command that uses an example subsystem. */
public class StopManualShooting extends SequentialCommandGroup {

  public StopManualShooting(TurretSys turretSys, IndexerSys indexerSys, IntakeSys intakeSys) {
    super(
        new StopManualFlywheelRPM(turretSys),
        new StopManualHoodAngle(turretSys),
        new SetFloorRollerRPM(indexerSys, 0.0),
        new SetTowerRollerRPM(indexerSys, 0.0),
        new SetIntakeRollerRPM(intakeSys, 0.0)
    );
  }
}