// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.commands.intake.SetIntakeRollerRPM;
import frc.robot.commands.intake.SetTargetPivotAngle;

/** An example command that uses an example subsystem. */
public class StopIntaking extends SequentialCommandGroup {
  public StopIntaking(IntakeSys intakeSys, IndexerSys indexerSys) {
    super(
        new SetIntakeRollerRPM(intakeSys, 0),
        new SetTargetPivotAngle(intakeSys, Constants.IntakeConstants.PivotBufferPositionAngle)
    );
  }
}