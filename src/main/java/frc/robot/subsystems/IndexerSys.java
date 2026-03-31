// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.IndexerConstants;

public class IndexerSys extends SubsystemBase {

  private final SparkFlex leftFloorRollerMtr;
  private final SparkFlex rightFloorRollerMtr;
  private final SparkFlex towerRollerMtr;

  private final RelativeEncoder leftFloorRollerEnc;
  private final RelativeEncoder rightFloorRollerEnc;
  private final RelativeEncoder towerRollerEnc;

  private final SparkClosedLoopController leftFloorRollerPID;
  private final SparkClosedLoopController rightFloorRollerPID;
  private final SparkClosedLoopController towerRollerPID;

  /** Creates a new ExampleSubsystem. */
  @SuppressWarnings("removal")
  public IndexerSys() {

    leftFloorRollerMtr = new SparkFlex(CANDevices.leftFloorRollerMtrID, MotorType.kBrushless);
    SparkFlexConfig leftFloorRollerSparkFlexConfig = new SparkFlexConfig();
    leftFloorRollerEnc = leftFloorRollerMtr.getEncoder();
    leftFloorRollerPID = leftFloorRollerMtr.getClosedLoopController();

    rightFloorRollerMtr = new SparkFlex(CANDevices.rightFloorRollerMtrID, MotorType.kBrushless);
    SparkFlexConfig rightFloorRollerSparkFlexConfig = new SparkFlexConfig();
    rightFloorRollerEnc = rightFloorRollerMtr.getEncoder();
    rightFloorRollerPID = rightFloorRollerMtr.getClosedLoopController();

    towerRollerMtr = new SparkFlex(CANDevices.towerRollerMtrID, MotorType.kBrushless);
    SparkFlexConfig towerRollerSparkFlexConfig = new SparkFlexConfig();
    towerRollerEnc = towerRollerMtr.getEncoder();
    towerRollerPID = towerRollerMtr.getClosedLoopController();

    
    towerRollerSparkFlexConfig.inverted(false);
    rightFloorRollerSparkFlexConfig.inverted(true);
    leftFloorRollerSparkFlexConfig.inverted(false);

    towerRollerSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
    rightFloorRollerSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
    leftFloorRollerSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

    towerRollerSparkFlexConfig.smartCurrentLimit(IndexerConstants.maxTowerRollerCurrentAmps);
    leftFloorRollerSparkFlexConfig.smartCurrentLimit(IndexerConstants.maxLeftFloorRollerCurrentAmps);
    rightFloorRollerSparkFlexConfig.smartCurrentLimit(IndexerConstants.maxRightFloorRollerCurrentAmps);

    towerRollerSparkFlexConfig.voltageCompensation(10);
    leftFloorRollerSparkFlexConfig.voltageCompensation(10);
    rightFloorRollerSparkFlexConfig.voltageCompensation(10);


    towerRollerSparkFlexConfig.softLimit.forwardSoftLimitEnabled(false);
    towerRollerSparkFlexConfig.softLimit.reverseSoftLimitEnabled(false);
    leftFloorRollerSparkFlexConfig.softLimit.forwardSoftLimitEnabled(false);
    leftFloorRollerSparkFlexConfig.softLimit.reverseSoftLimitEnabled(false);
    rightFloorRollerSparkFlexConfig.softLimit.forwardSoftLimitEnabled(false);
    rightFloorRollerSparkFlexConfig.softLimit.reverseSoftLimitEnabled(false);

    towerRollerSparkFlexConfig.encoder.positionConversionFactor(IndexerConstants.towerRollerPositionConversionFactor);
    towerRollerSparkFlexConfig.encoder.velocityConversionFactor(IndexerConstants.towerRollerVelocityConversionFactor);

    leftFloorRollerSparkFlexConfig.encoder.positionConversionFactor(IndexerConstants.FloorPositionConversionFactor);
    leftFloorRollerSparkFlexConfig.encoder.velocityConversionFactor(IndexerConstants.FloorVelocityConversionFactor);

    rightFloorRollerSparkFlexConfig.encoder.positionConversionFactor(IndexerConstants.FloorPositionConversionFactor);
    rightFloorRollerSparkFlexConfig.encoder.velocityConversionFactor(IndexerConstants.FloorVelocityConversionFactor);

    towerRollerSparkFlexConfig.closedLoop
      .p(IndexerConstants.towerRollerP)
      .d(IndexerConstants.towerRollerD)
      .feedForward.kS(IndexerConstants.towerRollerkS)
      .kV(IndexerConstants.towerRollerkV);

    leftFloorRollerSparkFlexConfig.closedLoop
      .p(IndexerConstants.FloorRollerP)
      .d(IndexerConstants.FloorRollerD);

    rightFloorRollerSparkFlexConfig.closedLoop
      .p(IndexerConstants.FloorRollerP)
      .d(IndexerConstants.FloorRollerD);

    towerRollerMtr.configure(
      towerRollerSparkFlexConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    leftFloorRollerMtr.configure(
      leftFloorRollerSparkFlexConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    rightFloorRollerMtr.configure(
      rightFloorRollerSparkFlexConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
  }

  public void setTargetFloorRollerRPM(double targetFloorRollerRPM) {
     if (targetFloorRollerRPM != 0.0) {
      leftFloorRollerPID.setSetpoint(targetFloorRollerRPM, ControlType.kVelocity);
      rightFloorRollerPID.setSetpoint(targetFloorRollerRPM, ControlType.kVelocity);
    } else {
      leftFloorRollerMtr.stopMotor();
      rightFloorRollerMtr.stopMotor();
     }
  }

  public void setTargetTowerRollerRPM(double targetTowerRollerRPM) {
    if (targetTowerRollerRPM != 0.0) {
      towerRollerPID.setSetpoint(targetTowerRollerRPM, ControlType.kVelocity);
    } else {
      towerRollerMtr.stopMotor();
    }
  }

  public double getFloorRollerRPM() {
    return (leftFloorRollerEnc.getVelocity() + rightFloorRollerEnc.getVelocity()) / 2.0;
  }

  public double getTowerRollerRPM() {
    return towerRollerEnc.getVelocity();
  }
}