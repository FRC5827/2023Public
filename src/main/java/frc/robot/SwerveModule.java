// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import frc.lib.util.SwerveModuleConstants;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

public final class SwerveModule implements Sendable {
    public final int m_number;

    private final double m_angleOffset;
    private final WPI_TalonFX m_angleMotor;
    private final WPI_TalonFX m_driveMotor;
    private final WPI_CANCoder m_angleEncoder;

    private double m_lastAngleInTicks;

    // for simulation
    private final TalonFXSimCollection m_angleMotorSim;
    private final TalonFXSimCollection m_driveMotorSim;
    private final FlywheelSim m_angleWheelSim;
    private final FlywheelSim m_driveWheelSim;

    private final SimpleMotorFeedforward m_feedForward =
            new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        m_number = moduleNumber;
        if (RobotBase.isReal()) {
            m_angleOffset = moduleConstants.m_angleOffset;
        }
        else {
            m_angleOffset = 0.0;
        }
        
        // Angle Encoder Config
        m_angleEncoder = new WPI_CANCoder(moduleConstants.m_cancoderID);
        configAngleEncoder();

        // Angle Motor Config
        m_angleMotor = new WPI_TalonFX(moduleConstants.m_angleMotorID);
        configAngleMotor();

        // Drive Motor Config
        m_driveMotor = new WPI_TalonFX(moduleConstants.m_driveMotorID);
        configDriveMotor(moduleConstants.m_invertDriveMotor);

        m_lastAngleInTicks = m_angleMotor.getSelectedSensorPosition();

        if (RobotBase.isSimulation()) {
            // Object for simulated inputs into Talon.
            m_angleMotorSim = m_angleMotor.getSimCollection();
            m_driveMotorSim = m_driveMotor.getSimCollection();

            // 2 inches (radius of MK3 billet wheel) is .0508 meters
            // .42 lbs (mass of MK3 billet wheel with tread) plus gear is likely ~ .25kg
            // moment of intertia is then ~ .25 * .0508^2 = 0.00064516
            m_angleWheelSim = new FlywheelSim(DCMotor.getFalcon500(1), Constants.Swerve.kAngleGearRatio, 0.00064516);
            m_driveWheelSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(
                    // convert kV/kA to radians/sec based on wheel size
                    Constants.Swerve.driveKV / (1 / (Constants.Swerve.kWheelDiameter / 2)),
                    Constants.Swerve.driveKA / (1 / (Constants.Swerve.kWheelDiameter / 2))),
                DCMotor.getFalcon500(1),
                Constants.Swerve.kDriveGearRatio
            );
        }
        else
        {
            m_angleMotorSim = null;
            m_driveMotorSim = null;
            m_angleWheelSim = null;
            m_driveWheelSim = null;
        }
      
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        boolean bUseLastVelocity = false;
        double velocity = 0.0;
        double percentOutput = 0.0;

        if (m_angleMotor.hasResetOccurred()) {
            System.out.println("Angle motor reset detected! Recalibrating angle and setting status frame rates.");
            configAngleMotor();
            return;
        }

        double currentTicks = m_angleMotor.getSelectedSensorPosition();
        Rotation2d currentRotation = Rotation2d.fromDegrees(currentTicks * Constants.Swerve.kAngleEncoderDegreesPerTick);
        //SmartDashboard.putNumber("module " + m_number + " angle actual", currentTicks);

        desiredState = SwerveModuleState.optimize(desiredState, currentRotation);

        // find the rotation difference between current module angle and new calculated angle
        Rotation2d rotationDelta = desiredState.angle.minus(currentRotation);

        // find the new setpoint position in ticks based on the difference
        double deltaTicks = rotationDelta.getDegrees() * Constants.Swerve.kAngleEncoderTicksPerDegree;
        double desiredTicks = currentTicks + deltaTicks;
        double angle = desiredTicks;

        // prevent rotating the module angle if speed is less then 1%.  Avoids jittering.
        if (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
        {
            // Angle state at no velocity is likely the home position (0 degrees), 
            // and we aren't setting the angle since velocity
            // is too low.  Thus use last angle state.
            angle = m_lastAngleInTicks;
            rotationDelta = Rotation2d.fromDegrees(0.0);
        }

        // uncomment below and comment line after if switching from motion magic to straight PID
        //m_angleMotor.set(TalonFXControlMode.Position, angle); 
        m_angleMotor.set(TalonFXControlMode.MotionMagic, angle);
        //SmartDashboard.putNumber("module " + m_number + " angle set", angle);
        m_lastAngleInTicks = angle;

        // wait for angle to be within tolerance before setting new drive motor output
        if (!(Math.abs(rotationDelta.getDegrees()) <= (Constants.Swerve.kAngleDeltaForDrive))) {
            bUseLastVelocity = true;
        }

        // if angle was not close enough, do not set new output value on drive motor
        if (!bUseLastVelocity) {
            if (isOpenLoop) {
                // percentOutput used for open loop, velocity for closed loop
                percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxFalconSpeed;
                m_driveMotor.set(TalonFXControlMode.PercentOutput, percentOutput);
            }
            else {
                // Talon/Falcon API expects velocity in encoder unit change / 100ms, so convert meters/s to units/100ms.
                // Some platforms (such as RoboRio's ARM Cortex-A9) are slower for div than mul, so multiply instead.
                // Java compiler may be smart enough to convert divide to multiply for this constant, but use mul anyway.
                velocity = (desiredState.speedMetersPerSecond / Constants.Swerve.kDriveEncoderDistancePerPulse) * 0.1d;
                m_driveMotor.set(TalonFXControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    m_feedForward.calculate(desiredState.speedMetersPerSecond) / Constants.General.kNominalBatteryVoltage);
                //m_driveMotor.set(TalonFXControlMode.Velocity, velocity);
                SmartDashboard.putNumber("FF " + m_number, m_feedForward.calculate(desiredState.speedMetersPerSecond));
                SmartDashboard.putNumber("Desired Speed " + m_number, desiredState.speedMetersPerSecond);
            }
        }
    }

    // called if angle motor is reset (such as from brownout)
    private void resetToAbsolute() {
        double absolutePosition = (getCanCoderPosition() - m_angleOffset) * Constants.Swerve.kAngleEncoderTicksPerDegree;
        CTREConfigs.checkCtreError(
            m_angleMotor.setSelectedSensorPosition(absolutePosition, 0, Constants.General.kCANConfigTimeout), 
            "Error setting angle Talon " + m_number + " sensor position");
    }

    private void configAngleEncoder() {        
        CTREConfigs.configCANCoderFactoryDefault(m_angleEncoder, "Error configuring CANcoder " + m_number + " default settings");
        CTREConfigs.setCANCoderStatusFramePeriod(m_angleEncoder, CANCoderStatusFrame.SensorData, 
            Constants.General.kTalonCANStatusSlowRateInMs, "Error setting CANCoder " + m_number + " SensorData period");
        CTREConfigs.checkCtreError(m_angleEncoder.configAllSettings(Robot.m_ctreConfigs.swerveCanCoderConfig,
            Constants.General.kCANConfigTimeout), "Error configuring CANCoder " + m_number + " configuration");
    }

    private void configAngleMotor() {
        CTREConfigs.configTalonFactoryDefault(m_angleMotor, "Error configuring angle Talon " + m_number + " default settings");

        // reduce CAN usage since we don't need general status data as frequently
        CTREConfigs.setTalonStatusFramePeriod(m_angleMotor, StatusFrameEnhanced.Status_1_General,
            Constants.General.kTalonCANStatusVerySlowRateInMs - (15 * m_number),
            "Error setting angle Talon " + m_number + " status_1 period");

        CTREConfigs.checkCtreError(m_angleMotor.configAllSettings(Robot.m_ctreConfigs.swerveAngleFXConfig,
            Constants.General.kCANConfigTimeout), "Error configuring angle Talon " + m_number + " configuration");
        CTREConfigs.checkCtreError(m_angleMotor.configVoltageCompSaturation(Constants.General.kNominalBatteryVoltage,
            Constants.General.kCANConfigTimeout), "Error configuring angle Talon " + m_number + " voltage comp saturation"); 
        m_angleMotor.setInverted(Constants.Swerve.invertAngleMotor);
        m_angleMotor.setNeutralMode(NeutralMode.Brake);
        // toggle flag indicating device was reset (false for subsequent checks, unless reset actually occurred)
        m_angleMotor.hasResetOccurred(); 
        
        // don't enable voltage compensation if in simulation, as it doesn't work
        if (RobotBase.isReal()) {
            m_angleMotor.enableVoltageCompensation(true);
        }

        resetToAbsolute();
    }

    private void configDriveMotor(boolean invertDriveMotor) {  
        if(Robot.isSimulation()) {
            invertDriveMotor = false;
        }
        CTREConfigs.configTalonFactoryDefault(m_driveMotor, "Error configuring drive Talon " + m_number + " default settings");

        // reduce CAN usage since we don't need general status data as frequently
        CTREConfigs.setTalonStatusFramePeriod(m_driveMotor, StatusFrameEnhanced.Status_1_General,
            Constants.General.kTalonCANStatusVerySlowRateInMs - (15 * m_number),
                "Error setting drive Talon " + m_number + " status_1 period");
        // increase CAN status frames for sensor position for better accuracy
        CTREConfigs.setTalonStatusFramePeriod(m_driveMotor, StatusFrameEnhanced.Status_2_Feedback0,
            Constants.General.kTalonCANStatusFastRateInMs, "Error setting drive Talon " + m_number + " status_2 period");

        CTREConfigs.checkCtreError(m_driveMotor.configAllSettings(Robot.m_ctreConfigs.swerveDriveFXConfig,
            Constants.General.kCANConfigTimeout), "Error configuring drive Talon " + m_number + " configuration");
        CTREConfigs.checkCtreError(m_angleMotor.configVoltageCompSaturation(Constants.General.kNominalBatteryVoltage,
            Constants.General.kCANConfigTimeout), "Error configuring drive Talon " + m_number + " voltage comp saturation"); 
        m_driveMotor.setInverted(invertDriveMotor);
        m_driveMotor.setNeutralMode(NeutralMode.Brake);
        // toggle flag indicating device was reset (false for subsequent checks, unless reset actually occurred)
        m_driveMotor.hasResetOccurred();

        // don't enable voltage compensation if in simulation, as it doesn't work
        if (RobotBase.isReal()) {
            m_driveMotor.enableVoltageCompensation(true);
        }

        CTREConfigs.checkCtreError(m_driveMotor.setSelectedSensorPosition(0, 0, Constants.General.kCANConfigTimeout),
            "Error setting drive Talon " + m_number + " sensor position");

        //m_driveMotor.configVelocityMeasurementWindow(32);
        //m_driveMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_20Ms);
    
    }

    public double getCanCoderPosition() {
        return m_angleEncoder.getAbsolutePosition();
    }

    public double getDriveEncoderDistanceMeters() {
        return m_driveMotor.getSelectedSensorPosition() * Constants.Swerve.kDriveEncoderDistancePerPulse;
    }


    /**
     * Used to set brake or coast neutral mode for angle and drive motors.
     * If coast mode is set, also reset module to zero angle position.
     * 
     * @param mode if true, enable brake mode
     */
    public void setBrakeMode(boolean brake) {
        NeutralMode mode;

        if (brake) {
            mode = NeutralMode.Brake;
        }
        else {
            mode = NeutralMode.Coast;
        }

        m_driveMotor.setNeutralMode(mode);
        m_angleMotor.setNeutralMode(mode);
    }

    /**
     * Moves the angle of the module to the zero position
     * and updates the last angle
     */
    public void zeroModuleAngle() {
        m_angleMotor.set(TalonFXControlMode.MotionMagic, 0.0);
        m_lastAngleInTicks = 0.0;
    }

    /**
     * Returns the module's drive motor velocity in meters per second and angle in degrees
     * 
     * @return a SwerveModuleState instance
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), Rotation2d.fromDegrees(getAngle()));
    }

    /**
     * Returns the drive motor velocity in meters per second
     * 
     * @return velocity in m/s
     */
    public double getVelocity() {
        // multiply by 10 as CTRE motor controllers provides velocity in units per 100ms, so convert to units/second
        return (10.0 * m_driveMotor.getSelectedSensorVelocity() * Constants.Swerve.kDriveEncoderDistancePerPulse);
    }

    /**
     * Returns the angle of the module in degrees
     * @return angle of module in degrees
     */
    public double getAngle() {
        return (m_angleMotor.getSelectedSensorPosition() * Constants.Swerve.kAngleEncoderDegreesPerTick);
    }
    
    /**
     * Resets drive motor encoder to zero.  Call when resetting odometry position.
     */
    public void resetDriveEncoder() {
        CTREConfigs.checkCtreError(m_driveMotor.setSelectedSensorPosition(0, 0, Constants.General.kCANConfigTimeout),
            "Error setting drive Talon " + m_number + " sensor position");
    }

    public void simulationPeriodic(double dt) {
        double angleMotorVoltage = m_angleMotor.getMotorOutputVoltage();
        double driveMotorVoltage = m_driveMotor.getMotorOutputVoltage();

        m_angleWheelSim.setInputVoltage(angleMotorVoltage);
        //SmartDashboard.putNumber("module " + m_number + " sim angle volts", angleMotorVoltage);
        m_driveWheelSim.setInputVoltage(driveMotorVoltage);
        //SmartDashboard.putNumber("module " + m_number + " sim drive volts", driveMotorVoltage);

        m_angleWheelSim.update(dt);
        m_driveWheelSim.update(dt);

        double angleVelInTicks = m_angleWheelSim.getAngularVelocityRPM() * 2048 / 600 * Constants.Swerve.kAngleGearRatio;
        double driveVelInTicks = m_driveWheelSim.getAngularVelocityRPM() * 2048 / 600 * Constants.Swerve.kDriveGearRatio;

        //SmartDashboard.putNumber("module " + m_number + " sim angle wheel RPM", m_angleWheelSim.getAngularVelocityRPM());
        //SmartDashboard.putNumber("module " + m_number + " sim drive wheel RPM", m_driveWheelSim.getAngularVelocityRPM());

        m_angleMotorSim.setIntegratedSensorVelocity((int)angleVelInTicks);
        m_driveMotorSim.setIntegratedSensorVelocity((int)driveVelInTicks);

        m_angleMotorSim.addIntegratedSensorPosition((int)(angleVelInTicks * 10 * dt));
        m_driveMotorSim.addIntegratedSensorPosition((int)(driveVelInTicks * 10 * dt));

        m_angleMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        m_driveMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveModule");
        //builder.addDoubleProperty("driveMotorVoltage", () -> m_angleMotor.getMotorOutputVoltage(), null);
        //builder.addDoubleProperty("driveMotorCurrent", () -> m_angleMotor.getStatorCurrent(), null);
        //builder.addDoubleProperty("angleMotorVoltage", () -> m_angleMotor.getMotorOutputVoltage(), null);
        //builder.addDoubleProperty("angleMotorCurrent", () -> m_angleMotor.getStatorCurrent(), null);
        //builder.addDoubleProperty("measuredVelocity", this::getVelocity, null);
        //builder.addDoubleProperty("measuredAngle(falcon)", this::getAngle, null);
        builder.addDoubleProperty("measuredAngle(cancoder)", () -> m_angleEncoder.getAbsolutePosition(), null);
        //builder.addDoubleProperty("targetVelocity", () -> m_targetVelocity, null);
        //builder.addDoubleProperty("targetAngle", () -> m_targetAngle, null);
    }
}
