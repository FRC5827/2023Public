package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeArm.IntakeState;

public final class IntakeUpDown extends CommandBase {

    private final IntakeState m_targetState;
    private final IntakeArm m_intakeArm;

    public IntakeUpDown(IntakeArm intakeArm, IntakeState target) {
        m_targetState = target;
        m_intakeArm = intakeArm;
        addRequirements(m_intakeArm);
    }

    @Override
    public void initialize() {
        m_intakeArm.setIntakePIDSetpoint(m_targetState.getValue());
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeArm.setIntakePIDSetpoint(m_intakeArm.getIntakeEncoderPosition());
    }

    @Override
    public boolean isFinished() {
        return m_intakeArm.atSetpoint();
    }
}