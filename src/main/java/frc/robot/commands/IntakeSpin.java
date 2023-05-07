// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSpinner;
import frc.robot.subsystems.UpdateButtonBoardInputs;

public class IntakeSpin extends CommandBase {

    private final IntakeSpinner m_intakeSpinner;
    private final boolean m_bIsForward;
    private final UpdateButtonBoardInputs m_buttonBoardInputs;
    private final NetworkTableEntry m_intakeSpeedEntry = NetworkTableInstance.getDefault().getEntry("intakeSpeed");

    /** Creates a new IntakeSpin. */
    public IntakeSpin(IntakeSpinner intake, boolean bIsForward, UpdateButtonBoardInputs buttonBoard) {
        m_intakeSpinner = intake;
        m_bIsForward = bIsForward;
        m_buttonBoardInputs = buttonBoard;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_intakeSpinner);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double speed = m_bIsForward ?
                IntakeConstants.kIntakeSpinForwardSpeed :
                m_buttonBoardInputs.getIntakeShooterSpeed();
        m_intakeSpeedEntry.setNumber(speed);
        m_intakeSpinner.spin(m_intakeSpeedEntry.getDouble(0));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intakeSpeedEntry.setNumber(0.0);
        m_intakeSpinner.spin(m_intakeSpeedEntry.getDouble(0));        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
