// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANDevices;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.drive.PoseEstimator;

public class TurretSys extends SubsystemBase {
  private final SparkFlex leftFlyWheelMtr;
  private final SparkFlex rightFlyWheelMtr;
  private final SparkFlex hoodMtr;

  private final RelativeEncoder hoodEnc;
  private final RelativeEncoder leftFlyWheelEnc;
  private final RelativeEncoder rightFlyWheelEnc;

  private final SparkClosedLoopController leftFlyWheelPID;
  private final SparkClosedLoopController rightFlyWheelPID;
  private final SparkClosedLoopController hoodPID;
  private final PoseEstimator poseEstimator;

  private Double manualHoodAngle = null;
  private Double manualFlywheelRPM = 0.0;
  private boolean isAiming = false;
  private boolean isFiring = false;
  private boolean isPassing = false;
  private double flywheelOffsetRPM = 0.0;
  private double aziumthOffsetDeg = 0.0;
  // private final SysIdRoutine sysIdRoutine;
  private Field2d field = new Field2d();

  @SuppressWarnings("removal")
  public TurretSys(PoseEstimator poseEstimator) {

    this.poseEstimator = poseEstimator;

    leftFlyWheelMtr = new SparkFlex(CANDevices.leftFlyWheelMtrID, MotorType.kBrushless);
    SparkFlexConfig leftFlyWheelMtrSparkFlexConfig = new SparkFlexConfig();
    leftFlyWheelPID = leftFlyWheelMtr.getClosedLoopController();
    leftFlyWheelEnc = leftFlyWheelMtr.getEncoder();

    rightFlyWheelMtr = new SparkFlex(CANDevices.rightFlyWheelMtrID, MotorType.kBrushless);
    SparkFlexConfig rightFlyWheelMtrSparkFlexConfig = new SparkFlexConfig();
    rightFlyWheelPID = rightFlyWheelMtr.getClosedLoopController();
    rightFlyWheelEnc = rightFlyWheelMtr.getEncoder();

    hoodMtr = new SparkFlex(CANDevices.hoodMtrID, MotorType.kBrushless);
    SparkFlexConfig hoodMtrSparkFlexConfig = new SparkFlexConfig();
    hoodPID = hoodMtr.getClosedLoopController();
    hoodEnc = hoodMtr.getEncoder();

    hoodMtrSparkFlexConfig.encoder
        .positionConversionFactor(TurretConstants.hoodPositionConversionFactorDeg);
    hoodMtrSparkFlexConfig.encoder
        .velocityConversionFactor(TurretConstants.hoodVelocityConversionFactorDeg);

    leftFlyWheelMtrSparkFlexConfig.encoder
        .positionConversionFactor(TurretConstants.flyWheelPositionConversionFactorRot);
    leftFlyWheelMtrSparkFlexConfig.encoder
        .velocityConversionFactor(TurretConstants.flyWheelVelocityConversionFactorRPM);
    rightFlyWheelMtrSparkFlexConfig.encoder
        .positionConversionFactor(TurretConstants.flyWheelPositionConversionFactorRot);
    rightFlyWheelMtrSparkFlexConfig.encoder
        .velocityConversionFactor(TurretConstants.flyWheelVelocityConversionFactorRPM);

    hoodMtrSparkFlexConfig.inverted(true);
    leftFlyWheelMtrSparkFlexConfig.inverted(true);
    rightFlyWheelMtrSparkFlexConfig.inverted(false);

    hoodMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
    leftFlyWheelMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
    rightFlyWheelMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);

    hoodMtrSparkFlexConfig.smartCurrentLimit(TurretConstants.maxHoodCurrentAmps);
    leftFlyWheelMtrSparkFlexConfig.smartCurrentLimit(TurretConstants.maxFlyWheelCurrentAmps);
    rightFlyWheelMtrSparkFlexConfig.smartCurrentLimit(TurretConstants.maxFlyWheelCurrentAmps);

    hoodMtrSparkFlexConfig.voltageCompensation(12);
    leftFlyWheelMtrSparkFlexConfig.voltageCompensation(10);
    rightFlyWheelMtrSparkFlexConfig.voltageCompensation(10);

    hoodMtrSparkFlexConfig.softLimit.forwardSoftLimitEnabled(true);
    hoodMtrSparkFlexConfig.softLimit.reverseSoftLimitEnabled(true);
    hoodMtrSparkFlexConfig.softLimit.forwardSoftLimit(TurretConstants.maximumHoodAngleDeg);
    hoodMtrSparkFlexConfig.softLimit.reverseSoftLimit(TurretConstants.minimumHoodAngleDeg);

    leftFlyWheelMtrSparkFlexConfig.softLimit.forwardSoftLimitEnabled(false);
    leftFlyWheelMtrSparkFlexConfig.softLimit.reverseSoftLimitEnabled(false);
    rightFlyWheelMtrSparkFlexConfig.softLimit.forwardSoftLimitEnabled(false);
    rightFlyWheelMtrSparkFlexConfig.softLimit.reverseSoftLimitEnabled(false);

    rightFlyWheelMtrSparkFlexConfig.closedLoop
        .p(TurretConstants.flywheelkP)
        .d(TurretConstants.flywheelkD).feedForward
        .kS(TurretConstants.flywheelkS)
        .kV(TurretConstants.flywheelkV);

    leftFlyWheelMtrSparkFlexConfig.closedLoop
        .p(TurretConstants.flywheelkP)
        .d(TurretConstants.flywheelkD).feedForward
        .kS(TurretConstants.flywheelkS)
        .kV(TurretConstants.flywheelkV);

    hoodMtrSparkFlexConfig.closedLoop
        .p(TurretConstants.hoodP)
        .d(TurretConstants.hoodD);
        // .feedForward
        // .kS(TurretConstants.hoodkS)
        // .kV(TurretConstants.hoodkV)
        // .kA(TurretConstants.hoodkA);
    leftFlyWheelMtr.configure(leftFlyWheelMtrSparkFlexConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    rightFlyWheelMtr.configure(rightFlyWheelMtrSparkFlexConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    hoodMtr.configure(hoodMtrSparkFlexConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // // initialzing sysID routines
    // sysIdRoutine = new SysIdRoutine(
    // new SysIdRoutine.Config(
    // null, // default ramp rate
    // Volts.of(12), // default step voltage
    // Time.ofBaseUnits(10, Seconds) // default timeout)
    // ),
    // new SysIdRoutine.Mechanism(
    // (voltage) -> {
    // setAzimuthVoltage(voltage.in(Volts));
    // },
    // (log) -> {
    // Voltage voltage = Volts.of(getCharacterizationVoltage());
    // Angle angle = Angle.ofBaseUnits(azimuthEnc.getPosition(), Radian);
    // AngularVelocity angularVelocity =
    // AngularVelocity.ofBaseUnits(azimuthEnc.getVelocity(), RadiansPerSecond);
    // log.motor("azimuth")
    // .voltage(voltage)
    // .angularPosition(angle)
    // .angularVelocity(angularVelocity);
    // },
    // this));
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      hoodPID.setSetpoint(0.0, ControlType.kPosition);
    } else if (isAiming
        && !isPassing
        && calculateTargetAzimuthAngleShooting() <= Units.degreesToRadians(TurretConstants.maximumHoodAngleDeg)
        && calculateTargetAzimuthAngleShooting() >= Units.degreesToRadians(TurretConstants.minimumHoodAngleDeg)) {
      hoodPID.setSetpoint(calculateTargetAzimuthAngleShooting(), ControlType.kPosition);
    } else if (isAiming
        && isPassing
        && calculateTargetAzimuthAnglePassing() <= Units.degreesToRadians(TurretConstants.maximumHoodAngleDeg)
        && calculateTargetAzimuthAnglePassing() >= Units.degreesToRadians(TurretConstants.minimumHoodAngleDeg)) {
      hoodPID.setSetpoint(calculateTargetAzimuthAnglePassing(), ControlType.kPosition);
    } else if (manualHoodAngle != null) {
      hoodPID.setSetpoint(Units.degreesToRadians(manualHoodAngle), ControlType.kPosition);
    } else {
      hoodPID.setSetpoint(TurretConstants.hoodDefaultSetpointRad, ControlType.kPosition);
    }

    if (isFiring) {
      setFlywheelRPM(calculateTargetFlywheelRPM());
    } else if (manualFlywheelRPM != null) {
      setFlywheelRPM(manualFlywheelRPM + flywheelOffsetRPM);
    } else {
      leftFlyWheelMtr.stopMotor();
      rightFlyWheelMtr.stopMotor();
    }
  }

  public void setManualHoodAngle(Double manualHoodAngle) {
    this.manualHoodAngle = manualHoodAngle;
  }

  public void setManualFlywheelRPM(Double TargetFlywheelRPM) {
    this.manualFlywheelRPM = TargetFlywheelRPM;
  }

  public double getManualFlywheelRPM() {
    if (manualFlywheelRPM != null) {
      return manualFlywheelRPM;
    } else {
      return 0.0;
    }
  }

  public void setIsFiring(boolean isFiring) {
    this.isFiring = isFiring;
  }

  public boolean getIsFiring() {
    return isFiring;
  }

  public void setIsAiming(boolean isAiming) {
    this.isAiming = isAiming;
  }

  public boolean getIsAiming() {
    return isAiming;
  }

  public void setIsPassing(boolean isPassing) {
    this.isPassing = isPassing;
  }

  public boolean getIsPassing() {
    return isPassing;
  }

  public boolean isAtSpeed() {
    return Math.abs(getFlywheelRPM() - calculateTargetFlywheelRPM())
        - 30.0 <= TurretConstants.flywheelErrorToleranceRPM;
  }

  public Pose2d calculateTurretPose() {
    return poseEstimator.getPose().transformBy(TurretConstants.robotToTurret);
  }

  public double calculateTargetAzimuthAngleShooting() {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
      return TurretConstants.targetPoseBlue.getTranslation()
          .minus(calculateTurretPose().getTranslation())
          .getAngle()
          .minus(new Rotation2d(poseEstimator.getPose().getRotation().getRadians())).getRadians()
          + Units.degreesToRadians(aziumthOffsetDeg);
    } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      return TurretConstants.targetPoseRed.getTranslation()
          .minus(calculateTurretPose().getTranslation())
          .getAngle()
          .minus(new Rotation2d(poseEstimator.getPose().getRotation().getRadians())).getRadians()
          + Units.degreesToRadians(aziumthOffsetDeg);
    } else {
      return TurretConstants.targetPoseBlue.getTranslation()
          .minus(calculateTurretPose().getTranslation())
          .getAngle()
          .minus(new Rotation2d(poseEstimator.getPose().getRotation().getRadians())).getRadians()
          + Units.degreesToRadians(aziumthOffsetDeg);
    }
  }

  public double calculateTargetAzimuthAnglePassing() {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
      return TurretConstants.passingPoseBlue.getTranslation()
          .minus(calculateTurretPose().getTranslation())
          .getAngle()
          .minus(new Rotation2d(poseEstimator.getPose().getRotation().getRadians())).getRadians()
          + Units.degreesToRadians(aziumthOffsetDeg);
    } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      return TurretConstants.passingPoseRed.getTranslation()
          .minus(calculateTurretPose().getTranslation())
          .getAngle()
          .minus(new Rotation2d(poseEstimator.getPose().getRotation().getRadians())).getRadians()
          + Units.degreesToRadians(aziumthOffsetDeg);
    } else {
      return TurretConstants.passingPoseBlue.getTranslation()
          .minus(calculateTurretPose().getTranslation())
          .getAngle()
          .minus(new Rotation2d(poseEstimator.getPose().getRotation().getRadians())).getRadians()
          + Units.degreesToRadians(aziumthOffsetDeg);
    }
  }

  public double calculateDistanceToTarget() {
    if (isPassing) {
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
        return calculateTurretPose().getTranslation().getDistance(TurretConstants.passingPoseBlue.getTranslation());
      } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
        return calculateTurretPose().getTranslation().getDistance(TurretConstants.passingPoseRed.getTranslation());
      } else {
        return calculateTurretPose().getTranslation().getDistance(TurretConstants.passingPoseBlue.getTranslation());
      }
    } else {
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
        return calculateTurretPose().getTranslation().getDistance(TurretConstants.targetPoseBlue.getTranslation());
      } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
        return calculateTurretPose().getTranslation().getDistance(TurretConstants.targetPoseRed.getTranslation());
      } else {
        return calculateTurretPose().getTranslation().getDistance(TurretConstants.targetPoseBlue.getTranslation());
      }
    }
  }

  public double incrementFlywheelOffsetRPM() {
    flywheelOffsetRPM += TurretConstants.flywheelOffsetRPMIncrement;
    return flywheelOffsetRPM;
  }

  public double decrementFlywheelOffsetRPM() {
    flywheelOffsetRPM -= TurretConstants.flywheelOffsetRPMIncrement;
    return flywheelOffsetRPM;
  }

  public double incrementHoodOffsetDeg() {
    aziumthOffsetDeg += TurretConstants.azimuthOffsetIncrementDeg;
    return aziumthOffsetDeg;
  }

  public double decrementHoodOffsetDeg() {
    aziumthOffsetDeg -= TurretConstants.azimuthOffsetIncrementDeg;
    return aziumthOffsetDeg;
  }

  public double calculateTargetFlywheelRPM() {
    return flywheelOffsetRPM +
        TurretConstants.zerothDegreeFitConstant +
        TurretConstants.firstDegreeFitConstant * calculateDistanceToTarget() +
        TurretConstants.secondDegreeFitConstant * Math.pow(calculateDistanceToTarget(), 2);
  }

  // public boolean isOnTarget() {
  //   if (isPassing) {
      
  //   } else {
      
  //   }
  // }

  public void setFlywheelRPM(double targetRPM) {
    leftFlyWheelPID.setSetpoint(targetRPM, ControlType.kVelocity);
    rightFlyWheelPID.setSetpoint(targetRPM, ControlType.kVelocity);
  }

  public double getFlywheelRPM() {
    return (leftFlyWheelEnc.getVelocity() + rightFlyWheelEnc.getVelocity()) / 2.0;
  }

  public double getFlywheelOffsetRPM() {
    return (flywheelOffsetRPM);
  }

  public double getHoodOffsetDeg() {
    return aziumthOffsetDeg;
  }

  public double getCurrentHoodAngleRad() {
    return hoodEnc.getPosition();
  }

  public double getHoodManualTargetDeg() {
    if (manualHoodAngle != null) {
      return manualHoodAngle;
    } else {
      return 0.0;
    }
  }

  public Pose2d getTurretPose() {
    return calculateTurretPose();
  }

  public Field2d getTurretField() {
    field.setRobotPose(getTurretPose());
    return field;
  }

  // // for sysID charachterization only
  // public void setAzimuthVoltage(double voltage) {
  // azimuthMtr.setVoltage(voltage);
  // }

  // public double getCharacterizationVoltage() {
  // return azimuthMtr.getAppliedOutput() * RobotController.getBatteryVoltage();
  // }

  // public Command sysIdQuasistaticForward() {
  // return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
  // }

  // public Command sysIdQuasistaticReverse() {
  // return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
  // }

  // public Command sysIdDynamicForward() {
  // return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
  // }

  // public Command sysIdDynamicReverse() {
  // return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
  // }
}