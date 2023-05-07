// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.General;
import frc.robot.Constants.IDConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;


public final class IntakeSpinner extends SubsystemBase {
    // Motor controllers used to move and change Intake's position
    private final CANSparkMax m_intakeSpinMotor = new CANSparkMax(IDConstants.kIntakeSpinnerMotorID, MotorType.kBrushless); //neo, but it uses the same control so it's fine for now


    /** Creates a new IntakeSpinner. */
    public IntakeSpinner() {
        m_intakeSpinMotor.restoreFactoryDefaults();
        m_intakeSpinMotor.setCANTimeout(General.kCANConfigTimeout);
        m_intakeSpinMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, Constants.General.kTalonCANStatusSlowRateInMs);
        m_intakeSpinMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, Constants.General.kTalonCANStatusSlowRateInMs);
        m_intakeSpinMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, Constants.General.kTalonCANStatusVerySlowRateInMs);
        m_intakeSpinMotor.enableVoltageCompensation(Constants.General.kNominalBatteryVoltage);
        m_intakeSpinMotor.setSmartCurrentLimit(Constants.IntakeConstants.kIntakeSpinCurrentLimit);
        m_intakeSpinMotor.setIdleMode(IdleMode.kCoast);
        m_intakeSpinMotor.setInverted(false);
        m_intakeSpinMotor.getEncoder().setPositionConversionFactor(360.0);

    }

    public void spin(double speed) {
        SmartDashboard.putNumber("intake spin output", speed);
        m_intakeSpinMotor.set(speed);
    }
}
