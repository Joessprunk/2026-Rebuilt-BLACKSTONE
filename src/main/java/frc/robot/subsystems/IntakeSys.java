// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.IntakeConstants;

public class IntakeSys extends SubsystemBase {

  private final SparkFlex leftRollerMtr;
  private final SparkFlex intakePivotMtr;
  private final SparkFlex rightRollerMtr;

  private final RelativeEncoder leftRollerEnc;
  private final RelativeEncoder rightRollerEnc;
  private final RelativeEncoder intakePivotEnc;

  private final ProfiledPIDController intakePivotPID;
  //private final SparkClosedLoopController intakePivotPID;
  private final SparkClosedLoopController leftRollerPID;
  private final SparkClosedLoopController rightRollerPID;

 

  @SuppressWarnings("removal")
  public IntakeSys() {
    intakePivotMtr = new SparkFlex(CANDevices.intakePivotMtrID, MotorType.kBrushless);
    SparkFlexConfig intakePivotMtrSparkFlexConfig = new SparkFlexConfig();
    //intakePivotPID = intakePivotMtr.getClosedLoopController();
     intakePivotPID = new ProfiledPIDController(IntakeConstants.intakePivotP, 0.0, IntakeConstants.intakePivotD, 
     new TrapezoidProfile.Constraints(IntakeConstants.pivotMaxVelocityDegreesPerSec, IntakeConstants.pivotMaxAccelDegreesPerSec));
    intakePivotEnc = intakePivotMtr.getEncoder();

   

    leftRollerMtr = new SparkFlex(CANDevices.leftRollerMtrID, MotorType.kBrushless);
    SparkFlexConfig leftRollerMtrSparkFlexConfig = new SparkFlexConfig();
    leftRollerPID = leftRollerMtr.getClosedLoopController();
    leftRollerEnc = leftRollerMtr.getEncoder();

    rightRollerMtr = new SparkFlex(CANDevices.rightRollerMtrID, MotorType.kBrushless);
    SparkFlexConfig rightRollerMtrSparkFlexConfig = new SparkFlexConfig();
    rightRollerPID = rightRollerMtr.getClosedLoopController();
    rightRollerEnc = rightRollerMtr.getEncoder();

    intakePivotMtrSparkFlexConfig.inverted(true);
    leftRollerMtrSparkFlexConfig.inverted(false);
    rightRollerMtrSparkFlexConfig.inverted(true);

    intakePivotMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
    leftRollerMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
    rightRollerMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
    intakePivotMtrSparkFlexConfig.smartCurrentLimit(IntakeConstants.maxPivotCurrentAmps);
    leftRollerMtrSparkFlexConfig.smartCurrentLimit(IntakeConstants.maxRollerCurrentAmps);
    rightRollerMtrSparkFlexConfig.smartCurrentLimit(IntakeConstants.maxRollerCurrentAmps);

    intakePivotMtrSparkFlexConfig.voltageCompensation(10);
    leftRollerMtrSparkFlexConfig.voltageCompensation(10);
    rightRollerMtrSparkFlexConfig.voltageCompensation(10);
    intakePivotMtrSparkFlexConfig.softLimit.forwardSoftLimitEnabled(false);
    intakePivotMtrSparkFlexConfig.softLimit.reverseSoftLimitEnabled(false);

    intakePivotMtrSparkFlexConfig.softLimit.forwardSoftLimitEnabled(true);
    intakePivotMtrSparkFlexConfig.softLimit.reverseSoftLimitEnabled(true);
    intakePivotMtrSparkFlexConfig.softLimit.forwardSoftLimit(IntakeConstants.intakePivotMaxAngle);
    intakePivotMtrSparkFlexConfig.softLimit.reverseSoftLimit(IntakeConstants.intakePivotMinAngle);

    rightRollerMtrSparkFlexConfig.softLimit.forwardSoftLimitEnabled(false);
    rightRollerMtrSparkFlexConfig.softLimit.reverseSoftLimitEnabled(false);
  
    leftRollerMtrSparkFlexConfig.softLimit.forwardSoftLimitEnabled(false);
    leftRollerMtrSparkFlexConfig.softLimit.reverseSoftLimitEnabled(false);

    intakePivotMtrSparkFlexConfig.encoder.positionConversionFactor(IntakeConstants.intakePivotPositionConversionFactor);
    intakePivotMtrSparkFlexConfig.encoder.velocityConversionFactor(IntakeConstants.intakePivotVelocityConversionFactor);

    leftRollerMtrSparkFlexConfig.encoder.positionConversionFactor(IntakeConstants.rollerPositionConversionFactor);
    leftRollerMtrSparkFlexConfig.encoder.velocityConversionFactor(IntakeConstants.rollerVelocityConversionFactor);

    rightRollerMtrSparkFlexConfig.encoder.positionConversionFactor(IntakeConstants.rollerPositionConversionFactor);
    rightRollerMtrSparkFlexConfig.encoder.velocityConversionFactor(IntakeConstants.rollerVelocityConversionFactor);


    intakePivotMtrSparkFlexConfig.closedLoop
      .p(IntakeConstants.intakePivotP)
      .d(IntakeConstants.intakePivotD)
      .feedForward
      .kS(IntakeConstants.intakePivotkS)
      .kV(IntakeConstants.intakePivotkV);

    rightRollerMtrSparkFlexConfig.closedLoop
      .p(IntakeConstants.RollerP)
      .d(IntakeConstants.RollerD).feedForward
      .kS(IntakeConstants.RollerkS)
      .kV(IntakeConstants.RollerkV);
    
      leftRollerMtrSparkFlexConfig.closedLoop
      .p(IntakeConstants.RollerP)
      .d(IntakeConstants.RollerD)
      .feedForward
      .kS(IntakeConstants.RollerkS)
      .kV(IntakeConstants.RollerkV);

    intakePivotMtr.configure(
      intakePivotMtrSparkFlexConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    leftRollerMtr.configure(
      leftRollerMtrSparkFlexConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    rightRollerMtr.configure(
      rightRollerMtrSparkFlexConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  public void periodic() {
     if(DriverStation.isDisabled()){
      intakePivotPID.setGoal(getPivotAngle());
     }else{
     intakePivotMtr.set(intakePivotPID.calculate(getPivotAngle()));
     }
  }

  public void setTargetPivotAngle(double targetAngle) {
      intakePivotPID.setGoal(targetAngle);
      //intakePivotPID.setSetpoint(targetAngle, ControlType.kPosition);
  }

  public double getPivotAngle() {
    return (intakePivotEnc.getPosition());
  }

  public void resetPivotAngle () {
    intakePivotEnc.setPosition(0.0);
  }
  public void setTargetRollerRPM(double targetRollerRPM) {
    if (targetRollerRPM != 0.0) {
      leftRollerPID.setSetpoint(targetRollerRPM, ControlType.kVelocity);
      rightRollerPID.setSetpoint(targetRollerRPM, ControlType.kVelocity);

    } else {
      leftRollerMtr.stopMotor();
      rightRollerMtr.stopMotor();
    }
  }

  public double getRollerRPM() {
    return (leftRollerEnc.getVelocity()+rightRollerEnc.getVelocity()) / 2.0;
  }
}