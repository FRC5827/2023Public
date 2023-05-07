// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

interface ConfigFactoryDefaultInterface {
    ErrorCode configFactoryDefault(int timeout);
}

interface SetTalonStatusFramePeriod {
    ErrorCode setStatusFramePeriod(StatusFrameEnhanced frame, int periodMs, int timeoutMs);
}

interface SetTalonControlFramePeriod {
    ErrorCode setControlFramePeriod(ControlFrame frame, int periodMs);
}

interface SetCANCoderStatusFramePeriod {
    ErrorCode setStatusFramePeriod(CANCoderStatusFrame frame, int periodMs, int timeoutMs);
}

public final class CTREConfigs {
    public final TalonFXConfiguration swerveAngleFXConfig;
    public final TalonFXConfiguration swerveDriveFXConfig;
    public final CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs() {
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        // Swerve Angle Motor Configurations
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
                Constants.Swerve.kAngleEnableCurrentLimit,
                Constants.Swerve.kAngleCurrentLimit,
                Constants.Swerve.kAngleThresholdCurrent,
                Constants.Swerve.kAngleThresholdDuration);

        if (RobotBase.isReal()) {
            swerveAngleFXConfig.slot0.kP = Constants.Swerve.angleKP;
            swerveAngleFXConfig.slot0.kI = Constants.Swerve.angleKI;
            swerveAngleFXConfig.slot0.kD = Constants.Swerve.angleKD;
            swerveAngleFXConfig.slot0.kF = Constants.Swerve.angleKF;
        } else {
            swerveAngleFXConfig.slot0.kP = Constants.Swerve.angleKPSim;
            swerveAngleFXConfig.slot0.kI = Constants.Swerve.angleKISim;
            swerveAngleFXConfig.slot0.kD = Constants.Swerve.angleKDSim;
            swerveAngleFXConfig.slot0.kF = Constants.Swerve.angleKFSim;
        }

        swerveAngleFXConfig.neutralDeadband = Constants.Swerve.kAngleNeutralDeadband;
        swerveAngleFXConfig.motionCruiseVelocity = Constants.Swerve.kMotionCruiseVelocity;
        swerveAngleFXConfig.motionAcceleration = Constants.Swerve.kMotionAcceleration;
        swerveAngleFXConfig.motionCurveStrength = Constants.Swerve.kMotionCurveStrength;

        swerveAngleFXConfig.peakOutputForward = Constants.Swerve.maxSteerOutput;
        swerveAngleFXConfig.peakOutputReverse = -Constants.Swerve.maxSteerOutput;

        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

        // Swerve Drive Motor Configuration
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
                Constants.Swerve.kDriveEnableCurrentLimit,
                Constants.Swerve.kDriveCurrentLimit,
                Constants.Swerve.kDriveThresholdCurrent,
                Constants.Swerve.kDriveThresholdDuration);

        swerveDriveFXConfig.slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.slot0.kD = Constants.Swerve.driveKD;
        swerveDriveFXConfig.slot0.kF = Constants.Swerve.driveKF;
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = Constants.Swerve.kOpenLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.Swerve.kClosedLoopRamp;
        swerveDriveFXConfig.neutralDeadband = .001;

        // Swerve CANCoder Configuration
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

    }

    public static void checkCtreError(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            DriverStation.reportError(String.format("%s: %s", message, errorCode.toString()), false);
        }
    }

    public static void configTalonFactoryDefault(BaseTalon talon, String message) {
        ConfigFactoryDefaultInterface iface = talon::configFactoryDefault;

        for (int i = 0; i < Constants.General.kCANRetryCount; i++) {
            ErrorCode error = iface.configFactoryDefault(Constants.General.kCANConfigTimeout);
            if (error == ErrorCode.OK) {
                break;
            } else {
                System.out.println(message + ", try # " + i + ": " + error.toString());
            }
        }
    }

    public static void setTalonStatusFramePeriod(BaseTalon talon, StatusFrameEnhanced statusFrame, int period,
            String message) {
        // don't increase CAN status frame period if in simulation, as it doesn't work
        // (causes weird voltage quantization)
        if (RobotBase.isSimulation()) {
            return;
        }

        SetTalonStatusFramePeriod iface = talon::setStatusFramePeriod;

        for (int i = 0; i < Constants.General.kCANRetryCount; i++) {
            ErrorCode error = iface.setStatusFramePeriod(statusFrame, period, Constants.General.kCANConfigTimeout);
            if (error == ErrorCode.OK) {
                break;
            } else {
                System.out.println(message + ", try # " + i + ": " + error.toString());
            }
        }
    }

    public static void setTalonControlFramePeriod(BaseTalon talon, ControlFrame controlFrame, int period,
            String message) {
        // don't increase CAN status frame period if in simulation, as it doesn't work
        // (causes weird voltage quantization)
        if (RobotBase.isSimulation()) {
            return;
        }

        SetTalonControlFramePeriod iface = talon::setControlFramePeriod;

        for (int i = 0; i < Constants.General.kCANRetryCount; i++) {
            ErrorCode error = iface.setControlFramePeriod(controlFrame, period);
            if (error == ErrorCode.OK) {
                break;
            } else {
                System.out.println(message + ", try # " + i + ": " + error.toString());
            }
        }
    }

    public static void configCANCoderFactoryDefault(CANCoder canCoder, String message) {
        ConfigFactoryDefaultInterface iface = canCoder::configFactoryDefault;

        for (int i = 0; i < Constants.General.kCANRetryCount; i++) {
            ErrorCode error = iface.configFactoryDefault(Constants.General.kCANConfigTimeout);
            if (error == ErrorCode.OK) {
                break;
            } else {
                System.out.println(message + ", try # " + i + ": " + error.toString());
            }
        }
    }

    public static void setCANCoderStatusFramePeriod(CANCoder canCoder, CANCoderStatusFrame statusFrame, int period,
            String message) {
        // don't increase CAN status frame period if in simulation, as it doesn't work
        // (causes weird voltage quantization)
        if (RobotBase.isSimulation()) {
            return;
        }

        SetCANCoderStatusFramePeriod iface = canCoder::setStatusFramePeriod;

        for (int i = 0; i < Constants.General.kCANRetryCount; i++) {
            ErrorCode error = iface.setStatusFramePeriod(statusFrame, period, Constants.General.kCANConfigTimeout);
            if (error == ErrorCode.OK) {
                break;
            } else {
                System.out.println(message + ", try # " + i + ": " + error.toString());
            }
        }
    }
}
