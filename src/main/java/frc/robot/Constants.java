// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class RobotConstants {
        // Set to the current the weight of the robot, including the battery and
        // bumpers.
        public static final double massKg = 52.16;

        // Set the frame dimensions of the robot.
        public static final double robotWidthMeters = Units.inchesToMeters(27.0);
        public static final double robotLengthMeters = Units.inchesToMeters(27.0);

        // Moment of inertia of a uniform-mass slab with the axis of rotation centered
        // and perpendicular to the slab
        // This should be a reasonable approximation of the robot's MOI
        public static final double momentOfInertiaKgMetersSq = massKg
                * (Math.pow(robotWidthMeters, 2) + Math.pow(robotLengthMeters, 2)) / 12;
    }

    public static class ControllerConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        // change deadband based on controller drift
        public static final double joystickDeadband = 0.12;
        public static final double triggerPressedThreshold = 0.35;
    }

    public static class CANDevices {
        public static final int pigeonID = 14;

        public static final int frModuleCANCoderID = 2;
        public static final int frModuleDriveMtrID = 6;
        public static final int frModuleSteerMtrID = 10;

        public static final int brModuleCANCoderID = 3;
        public static final int brModuleDriveMtrID = 7;
        public static final int brModuleSteerMtrID = 11;

        public static final int flModuleCANCoderID = 4;
        public static final int flModuleDriveMtrID = 8;
        public static final int flModuleSteerMtrID = 12;

        public static final int blModuleCANCoderID = 5;
        public static final int blModuleDriveMtrID = 9;
        public static final int blModuleSteerMtrID = 13;

        public static final int intakePivotMtrID = 15;
        public static final int leftRollerMtrID = 16;
        public static final int rightRollerMtrID = 17;

        public static final int towerRollerMtrID = 19;
        public static final int leftFloorRollerMtrID = 20;
        public static final int rightFloorRollerMtrID = 21;



        
        public static final int leftFlyWheelMtrID = 23;
        public static final int rightFlyWheelMtrID = 24;

        public static final int hoodMtrID = 22;


    }

    public static class SwerveModuleConstants {
        // Tune the below PID and FF values using the SysID routines.
        public static final double driveKp = 0.18;
        public static final double driveKd = 0.0;

        public static final double steerKp = 0.45;
        public static final double steerKd = 0.15;

        public static final double driveKsVolts = 1.11607;
        public static final double driveKvVoltSecsPerMeter = 2.9583;
        public static final double driveKaVoldSecsPerMeterSq = 0.0;

        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(driveKsVolts,
                driveKvVoltSecsPerMeter, driveKaVoldSecsPerMeterSq);

        // Change this value depending on your breakers and the current usage of
        // the rest of your robot.
        public static final int driveMtrCurrentLimitAmps = 65;
        public static final int steerMtrCurrentLimitAmps = 30;

        // Change this number based on actual wheel diamter.
        public static final double wheelRadiusMeters = Units.inchesToMeters(1.9);

        // Set this value to the coefficient of friction of your wheels.
        public static final double wheelCoefficientOfFriction = 1.5;

        public static final double driveGearReduction = (16.0 / 54.0) * (32.0 / 25.0) * (15.0 / 30.0);

        public static final double driveMetersPerEncRev = driveGearReduction * 2.0 * wheelRadiusMeters * Math.PI;

        public static final double driveMetersPerSecPerEncRPM = driveMetersPerEncRev / 60.0;

        public static final double steerGearReduction = 1.0 / 26.0;

        public static final double steerRadiansPerEncRev = steerGearReduction * 2.0 * Math.PI;

        public static final double steerRadiansPerSecPerEncRPM = steerRadiansPerEncRev / 60.0;

        public static final double driveFreeSpeedMetersPerSec = Units.feetToMeters(22.5);

        public static final double driveFreeSpeedRadPerSec = driveFreeSpeedMetersPerSec / wheelRadiusMeters;

        public static final double driveNominalOperatingVoltage = 12.4;
        public static final double driveStallTorqueNewtonMeters = 3.6 / driveGearReduction; // Motor's stall torque
                                                                                            // times gear ratio
        public static final double driveStallCurrentAmps = 211.0;
        public static final double driveFreeCurrentAmps = 3.6;

        // uncomment once pathplannerlib is released
        public static final ModuleConfig moduleConfig = new ModuleConfig(
                wheelRadiusMeters, SwerveDriveConstants.maxAttainableSpeedMetersPerSec, wheelCoefficientOfFriction,
                new DCMotor(driveNominalOperatingVoltage, driveStallTorqueNewtonMeters, driveStallCurrentAmps,
                        driveFreeCurrentAmps, driveFreeSpeedRadPerSec, 1),
                driveMtrCurrentLimitAmps, 1);
    }

    public static class SwerveDriveConstants {
        // set these offsets based on module's zero position
        public static final Rotation2d flModuleOffset = Rotation2d.fromDegrees(-109.5); // 162.5
        public static final Rotation2d frModuleOffset = Rotation2d.fromDegrees(57.4); // 122.0
        public static final Rotation2d blModuleOffset = Rotation2d.fromDegrees(106.1); // -151.6
        public static final Rotation2d brModuleOffset = Rotation2d.fromDegrees(-30.4); // -102.8

        // Set these dimensions for the distance between the center of each wheel.
        // NOTE: these values are different from the robot's overall dimenstions.
        public static final double chassisTrackLengthMeters = Units.inchesToMeters(21.75); // 27 inch frame
        public static final double chassisTrackWidthMeters = Units.inchesToMeters(21.75); // 27 inch frame

        public static final double chassisRadiusMeters = Math.hypot(chassisTrackLengthMeters, chassisTrackWidthMeters);

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(chassisTrackWidthMeters / 2.0, chassisTrackLengthMeters / 2.0), // front left
                new Translation2d(chassisTrackWidthMeters / 2.0, -chassisTrackLengthMeters / 2.0), // front right
                new Translation2d(-chassisTrackWidthMeters / 2.0, chassisTrackLengthMeters / 2.0), // back left
                new Translation2d(-chassisTrackWidthMeters / 2.0, -chassisTrackLengthMeters / 2.0) // back right
        );

        // Tune these values based on actual robot performaance.
        public static final double maxAttainableSpeedMetersPerSec = Units.feetToMeters(16.5);
        public static final double maxAttainableRotationRadPerSec = 5.5;

        public static final double skewCompensationRatioOmegaPerTheta = 0.1;

        // Tune the below PID values
        public static final double autoTranslationKp = 5.0;
        public static final double autoTranslationKd = 0.0;

        public static final double autoRotationKp = 6.0;
        public static final double autoRotationKd = 0.0;

        public static final double autoAimkP = 10.9;
        public static final double autoAimkD = 0.5;
        public static final double autoAimTurnSpeedRadPerSec = 2.0 * Math.PI;
        public static final double autoAimTurnAccelRadPerSecSq = 3.0 * Math.PI;
        public static final double autoAimToleranceRad = 0.005;
    }

    public class VisionConstants {
        public static final String limelightOneName = "limelight-front";

        public static final String limelightTwoName = "limelight-back";

        public static final double odomTranslationStdDevMeters = 0.05;
        public static final double odomRotationStdDevRad = Units.degreesToRadians(0.25);

        public static final double visionTranslationStdDevMeters = 0.35;
        public static final double visionRotationStdDevRad = Units.degreesToRadians(30.0);

        public static final double angularVelocityDegPerSecThreshold = 120.0;
    }

    public class IntakeConstants {
        public static final int maxRollerCurrentAmps = 45;
        public static final int maxPivotCurrentAmps = 50;

        public static final double ActuatorPulleyToothCount = 18.0;
        public static final double actuatorPositionConversionFactor = 0.95; // Units.metersToInches(ActuatorPulleyToothCount
                                                                            // * 5.0) / 5.0; // convertInches(tooth *
                                                                            // pitch) / ratio
        public static final double actuatorVelocityConversionFactor = 0.95 / 60;// Units.metersToInches(ActuatorPulleyToothCount
        // * 5.0) / 5.0 /60.0;

        public static final double rollerPositionConversionFactor = 1.0;
        public static final double rollerVelocityConversionFactor = 1.0;

        public static final double intakePivotMinAngle = -10.0;
        public static final double intakePivotMaxAngle = 150.0;

        public static final double intakingPivotAngle = 120.0;
        public static final double PivotBufferPositionAngle = 90.0;

        public static final double intakePivotP = 0.01; // 0.0010;
        public static final double intakePivotD = 0.0; // 0.0002;

        public static final double pivotMaxAccelDegreesPerSec = 90.0;
        public static final double pivotMaxVelocityDegreesPerSec = 70.0;
        

        public static final double RollerP = 0.00008;// 0.015;
        public static final double RollerD = 0.00004;
        public static final double RollerkS = 0.35;// 2.5;
        public static final double RollerkV = 0.00179;   // 0.00175;
        
        public static final double RollerIntakingRPM = 4000.0;
        public static final double RollerShootingRPM = 2500.0;

        public static final double intakePivotPositionConversionFactor = 360.0 / 25.0;
        public static final double intakePivotVelocityConversionFactor = 360.0 / 25.0;
        public static final double stowedPivotAngle = 0;
        
        
        

        // public static final double manualActuatorAdjustmentSpeed = 0.4;
    }

    public class TurretConstants {
        
        public static final int maxFlyWheelCurrentAmps = 75;

        public static final double flywheelkP = 0.002;//0.00021;// 0.0003
        public static final double flywheelkD = 0.001;
        public static final double flywheelkS = 0.3; // increment voltage setpoint until the flywheel moves to find this
                                                     // value
        public static final double flywheelkV = 0.00177;// 0.00215; // calculated from ReCalc
        

        
        public static final double azimuthErrorToleranceDeg = 0.25;
        public static final double HoodDefaultSetpointDeg = 0.0;
        public static final double HoodOffsetIncrementDeg = 0.5;

        public static final double flyWheelPositionConversionFactorRot = 1.0;
        public static final double flyWheelVelocityConversionFactorRPM = 1.0;

        public static final double zerothDegreeFitConstant = 1166.0;
        public static final double firstDegreeFitConstant = 308.0;
        public static final double secondDegreeFitConstant = -6.09;

        public static final double flywheelOffsetRPMIncrement = 30.0;

        public static final Pose2d targetPoseBlue = new Pose2d(4.62, 4.04, null);
        public static final Pose2d targetPoseRed = new Pose2d(11.915, 4.035, null);

        public static final Pose2d passingPoseBlue = new Pose2d(2.0, 7.0, null);
        public static final Pose2d passingPoseRed = new Pose2d(14.0, 7.0, null);

        public static final double turretOffsetXInches = -1.5; // -5.75
        public static final double turretOffsetYInches = -5.75; // 1.5

        public static final Transform2d robotToTurret = new Transform2d(
                new Translation2d(Units.inchesToMeters(turretOffsetXInches), Units.inchesToMeters(turretOffsetYInches)),
                new Rotation2d());

        public static final double flywheelErrorToleranceRPM = 30.0;
        public static final double hoodPositionConversionFactorDeg = (360.0 * 15.0) / (188.0 * 5.0); 
        public static final double hoodVelocityConversionFactorDeg = (360.0 * 15.0) / (188.0 * 5.0);
        public static final int maxHoodCurrentAmps = 20;
        public static final double maximumHoodAngleDeg = 40.0;
        public static final double minimumHoodAngleDeg = -4.0;
        public static final double hoodP= 1.015;
        public static final double hoodD = 0.3;
        public static final double hoodkS = 0.2;
        public static final double hoodkG = 0.01;
        public static final double hoodDefaultSetpointAngleDeg = 0.0;
    }

    
    public class IndexerConstants {
        public static final int maxTowerRollerCurrentAmps = 30;
        public static final int maxLeftFloorRollerCurrentAmps = 30;//Make the same
        public static final int maxRightFloorRollerCurrentAmps = 30;

        public static final double towerRollerP = 0.0002;
        public static final double towerRollerD = 0.004;
        public static final double towerRollerkS = 0.2;
        public static final double towerRollerkV = 0.0088;

        public static final double FloorRollerP = 0.000;
        public static final double FloorRollerD = 0.000;
        public static final double FloorRollerkS = 0.3;
        public static final double FloorRollerkV = 0.0089;

        public static final double towerRollerPositionConversionFactor = 1.0 / 5.0;
        public static final double towerRollerVelocityConversionFactor = 1.0 / 5.0 ;

        public static final double FloorPositionConversionFactor = 1.0 / 5.0;
        public static final double FloorVelocityConversionFactor = 1.0 / 5.0;

        public static final double towerRollerShootingRPM = 200.0;
        public static final double floorRollerShootingRPM = 250.0;
        
    }
}