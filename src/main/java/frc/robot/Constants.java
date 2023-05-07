// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.FieldConstants.ChargeStation;
//import frc.robot.Constants.FieldConstants.RedChargeStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double kJoystickDeadband = 0.08;
    public static final double kJoystickSlewRate = 4.0;
    public static final String TurnPIDConstants = null;
    public static final double kpTurnRio = 0.1;
    public static final double kiTurnRio = 0.1;
    public static final double kdTurnRio = 0.1;

    public static final class General {
        public static final double kNominalBatteryVoltage = 12.0;
        public static final int kCANConfigTimeout = 400;
        public static final int kTalonCANStatusFastRateInMs = 10;
        public static final int kTalonCANStatusSlowRateInMs = 150;
        public static final int kTalonCANStatusVerySlowRateInMs = 255;  // 255 is max configurable period for CTRE CAN frames
        public static final int kTalonCANControlPrimaryRateInMs = 20;
        public static final int kTalonCANControlFollowerRateInMs = 40;
        public static final int kCANRetryCount = 10;
    }

    public static final class Swerve {

        // Controls the settings. If this is set to false, then it means we're using the Swerve Bot. If set to true, then the settings
        // should be for the competition robot.
        // THIS SHOULD BE SET TO TRUE BY DEFAULT IN THE MAIN BRANCH.
        // DO NOT SET IT TO FALSE AND CHECK IT IN TO MAIN!
        private static final boolean isCompetitionBot = true;

        // always ensure gyro is counter-clock-wise positive
        public static final boolean invertGyro = true;

        // drivetrain constants
        public static final double kTrackWidth = Units.inchesToMeters(22.50);
        public static final double kWheelBase = Units.inchesToMeters(22.50);
        public static final double kWheelDiameter = Units.inchesToMeters(4.00);
        public static final double kWheelCircumference = kWheelDiameter * Math.PI;

        // open loop (percent output) ramp probably better handled with our slew rate limiter
        public static final double kOpenLoopRamp = 0.0;
        public static final double kClosedLoopRamp = 0.0;

        // SDS Swerve Module MK3 fast drive gearing
        public static final double kDriveGearRatio = (6.86 / 1.0);
        public static final double kAngleGearRatio = (12.8 / 1.0);

        public static final double kDriveEncoderTicksPerRev = 2048.0 * kDriveGearRatio;  // 2048 ticks for Falcon 500
        public static final double kDriveEncoderDistancePerPulse =
            (kWheelDiameter * Math.PI) / (double) kDriveEncoderTicksPerRev;
        
        public static final double kAngleEncoderTicksPerRev = 2048.0 * kAngleGearRatio;
        public static final double kAngleEncoderTicksPerDegree = kAngleEncoderTicksPerRev / 360.0;
        public static final double kAngleEncoderDegreesPerTick = 360.0 / kAngleEncoderTicksPerRev;

        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

        // swerve current limiting
        public static final boolean kAngleEnableCurrentLimit = true;
        public static final int kAngleCurrentLimit = 20;
        public static final int kAngleThresholdCurrent = 30;
        public static final double kAngleThresholdDuration = 0.2;

        public static final boolean kDriveEnableCurrentLimit = true;
        public static final int kDriveCurrentLimit = 40;
        public static final int kDriveThresholdCurrent = 60;
        public static final double kDriveThresholdDuration = 0.5;

        // motion magic config values
        // 90 degree turn is 6553 ticks with 12.8 gear ratio.  Full output of Falcon with no load is ~6300rpm.
        // If we target 60% output, result is ~63 motor spindle rotations per second (~6.3 rotations per 100ms,
        // or 12902 ticks/100ms).
        // Acceleration is units/100ms per second, so multiply by 20 to reach specified velocity in 50ms
        // note that angleKP needs to be high enough to reach the specified velocity
        public static final double kMotionCruiseVelocity = 12902;
        public static final double kMotionAcceleration = kMotionCruiseVelocity * 20;
        public static final int kMotionCurveStrength = 0;
        // when motion magic is used, what wheel angle delta is acceptable before adding drive power
        public static final double kAngleDeltaForDrive = 30.0;
        public static final double kAngleNeutralDeadband = 0.01;

        // angle motor PID values
        public static final double angleKP = 0.8;
        public static final double angleKI = 0.0;
        public static final double angleKD = 8.0;
        public static final double angleKF = 0.0;

        // angle motor PID values for simulation
        public static final double angleKPSim = 0.40;
        public static final double angleKISim = 0.0;
        public static final double angleKDSim = 0.00;
        public static final double angleKFSim = 0.0;

        // angle motor peak output
        public static final double maxSteerOutput = 1.0;

        // drive motor PID values
        public static final double driveKP = 0.10;//0.16;  // from sysid tool
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        // drive motor characterization values
        // use robot characterization tool (sysid) to determine these
        // it installs as part of WPILib suite. Also at https://github.com/wpilibsuite/sysid
        // also see https://www.chiefdelphi.com/t/applying-sysid-feedforward-to-an-sds-swerve-mk4/398455/8
        public static final double driveKS = 0.2;
        public static final double driveKV = 2.0;
        public static final double driveKA = 0.1;

        // max swerve speed and rotation
        public static final double maxSpeed = 4.5;          // max speed we want the robot to go in open loop mode
        public static final double maxFalconSpeed = 4.5;    // meters per second, theoretical max is 16.2 ft/s or 4.9 m/s
        public static final double maxAngularVelocity = 1.5 * (2 * Math.PI); // 1.5 rotations/s (radians)

        // angle and drive motor inverts
        public static final boolean invertDriveMotor = isCompetitionBot ? true : false;
        public static final boolean invertAngleMotor = false;

        // angle encoder invert
        public static final boolean canCoderInvert = false;

        // slow mode factor
        public static final double slowModeFactor = 1.0/4;


        // module specific constants

        // Modules on each side should have wheels consistent with each other.
        // In other words, bevel gear should be facing the same direction across all modules.
        // Adjust invertDriveMotor if forward/backward are reversed.

        // front left module - module 0
        public static final class Module0 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 20;
            public static final double angleOffset = isCompetitionBot ? 261.0 : 344.97;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, invertDriveMotor);
        }

        // front right module - module 1
        public static final class Module1 {
            public static final int driveMotorID = 12;
            public static final int angleMotorID = 13;
            public static final int canCoderID = 21;
            public static final double angleOffset = isCompetitionBot ? 75.80 : 256.11;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, isCompetitionBot ? invertDriveMotor : !invertDriveMotor);
        }
        
        // rear left module - module 2
        public static final class Module2 {
            public static final int driveMotorID = 14;
            public static final int angleMotorID = 15;
            public static final int canCoderID = 22;
            public static final double angleOffset = isCompetitionBot ? 222.8 : 139.92;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, invertDriveMotor);
        }

        // rear right module - module 3
        public static final class Module3 {
            public static final int driveMotorID = 16;
            public static final int angleMotorID = 17;
            public static final int canCoderID = 23;
            public static final double angleOffset = isCompetitionBot ? 13.53 : 144.23;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, isCompetitionBot ? invertDriveMotor : !invertDriveMotor);
        }


        public static final class LimelightConstants {
            //initial value from testing nonTrig get distance that convertsfrom the double to inches
            public static final double distanceConversion = 0.0342;

            //angle of the limelight after its been mounted on robot ---- may change depending on desing
            public static final double limelightAngle = 0.0;

            

        }
    }

    public static final class AutoConstants {
        // For a value of 1.0, the holonomic drive controller will add an additional meter per second for every meter of error
        public static final double kPXController = 2.0;//5;
        public static final double kPYController = 2.0;//5;
        public static final double kPThetaController = 3;//1;
        // Auto balance threshold
        public static final double kOffBalanceAngle = 3.0;
    }

    public static final class FieldConstants {
        public static final double length = 16.54;
        public static final double width = 8.02;

        //to test if the robot's y value is too high (partially / whole robot in the charge station)
        /*
        public static final class AisleWithoutChargeStation { //only if we're driving from random spot to the scorint point, we're using this
            public static final double topTopY = 5.0;
            public static final double topBottomY = ChargeStation.topY;
            public static final double bottomTopY = ChargeStation.bottomY;
            public static final double bottomBottomY = 0;
        }
        */
        public static final class ChargeStation { //based on blue team
            public static final double topY = 4.32;
            public static final double bottomY = 1.18;
            public static final double midY = (topY + bottomY) / 2;
            public static final double leftX = 2.57;
            public static final double rightX = 5.2;
            public static final double midX = (leftX + rightX) / 2;

            //the bound value to test if robot's in the charge station
            public static final double inTopY = topY - 0.65;
            public static final double inBottomY = bottomY + 0.65;
            public static final double inLeftX = leftX + 0.65;
            public static final double inRightX = rightX - 0.65;
        }

        public static final class LoadingZone { //based on blue team
            public static final double topY = 7.62;
            public static final double bottomY = 6.1;
            public static final double rightX = 15.87;
            public static final double leftX = 12.87;
        }
    }

    public static final class TeamsPositionsConstants {
        //for blue team's scoring positions
        public static final HashMap<Integer, Pose2d> kBlueScoringPosition = new HashMap<Integer, Pose2d>() {{
            for (int i=1; i<=9; i++) {
                //for blue, first scoring point has x position of 2, y position of 0.5, distance between each score point is 0.56
                put(i, new Pose2d(1.7, 0.5 + (i-1) * 0.56, new Rotation2d(0))); //need change
            }
        }};

        //for red team's scoring positions
        public static final HashMap<Integer, Pose2d> kRedScoringPosition = new HashMap<Integer, Pose2d>() {{
            for (int i=1; i<=9; i++) {
                //for red, first scoring point has x position of 14.73, y position of 4.98, distance between each score point is 0.56
                put(i, new Pose2d(14.68, 4.98 - (i-1) * 0.56, new Rotation2d(Math.PI)));
            }
        }};

        //blue poses around charge station
        public static final Translation2d[] bluePoseRight = {
            new Translation2d(ChargeStation.rightX + 0.18, ChargeStation.topY + 0.18), //upper right (away from goals) of the charging pad
            new Translation2d(ChargeStation.leftX - 0.18, ChargeStation.topY + 0.18) //upper left (goal side) of the charging pad
        };
        public static final Translation2d[] bluePoseLeft = {
            new Translation2d(ChargeStation.rightX + 0.18, ChargeStation.bottomY - 0.18), //lower right of the charging pad
            new Translation2d(ChargeStation.leftX - 0.18, ChargeStation.bottomY - 0.18) //lower left of chargins station
        };

        public static final Translation2d[] specialPosePoints = {
            new Translation2d(12.86, 6.14), //midpoint for loading zone (when robot's in loading zone)
            new Translation2d(2.58, FieldConstants.ChargeStation.midY) //midpoint for charging station (when robot's on charging station)
        };
    }

    public static final class IDConstants {
        public static final double pivotMotor1ID = 4;
        public static final double pivotMotor2ID = 5;
        //Might be wrong

        //this needs to be set on actual robot
        public static final int kIntakeArmMotorID = 27;
        public static final int kIntakeCanCoderID = 26;

        public static final int kIntakeSpinnerMotorID = 8;

        public static final int kClawMotorID = 9;
        public static final int kFlapdexerMotorID = 2;

        public static final int kArmPivotCanCoderID = 25;
    }

    public static final class IntakeConstants {
        public static final int kIntakeSpinCurrentLimit = 20;
        public static final double kIntakeSpinForwardSpeed = -0.3;
        public static final double kIntakeShooterSpeedLow = 0.2;
        public static final double kIntakeShooterSpeedMed = 0.6;
        public static final double kIntakeShooterSpeedHigh = 1.0;

        public static final double kP = 0.03;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        // range is ~180 degrees
        public static final double kIntakeUpSensorDegrees = 233;//-3164;
        public static final double kIntakeDownSensorDegrees = 19;//-705;
        public static final double kIntakeArmPositionTolerance = 5;

        public static final double kIntakeArmEncoderOffset = 110;
        public static final double kIntakeArmUpSpeed = 0.65;
        public static final double kIntakeArmDownSpeed = 0.6;

        public static final double kDangerZoneLowerBound = 93;
        public static final double kDangerZoneUpperBound = 229;
       
        public static final double kPIDLoopTolerance = 5;
    }

    public static final class FlapdexerConstants {
        public static final double kflapperSpeed = 0.6;
        public static final double kGearRatio = 81; 
    }

    public static final class ClawConstants {

        public static final double kP = 0.03;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kTolerance = 3.0;

        // Motor runtime in seconds to ensure open or closed
        public static final double kClawRunTime = 1.0; 
        public static final double kClawMotorMaxSpeed = 0.6;
    }

    public static final class ArmPivotConstants {
        public static final double kArmFullFowardSensorDegree = -70;
        public static final double kArmRestSensorDegree = -285;
        public static final double kGrabItemSensorDegree = -355;

        public static final double kArmPivotSpeed = 0.35;
        
        public static final int kArmFirstMotorID = 4;
        public static final int kArmSecondMotorID = 5;

        public static final double kHighShelfLevelSensorDegree = -129;
        public static final double kMiddleShelfLevelSensorDegree = -108;
        public static final double kHighConeLevelSensorDegree = -129;
        public static final double kMiddleConeLevelSensorDegree = -108;

        public static final double kDangerZoneLowerBound = -348;
        public static final double kDangerZoneUpperBound = -295;
    }
}
