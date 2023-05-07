package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawState;

public final class ClawCommand extends CommandBase {

    private final Claw m_claw;
    private final ClawState m_clawState;

    public ClawCommand(Claw claw, ClawState state) {
        m_claw = claw;
        m_clawState = state;
        addRequirements(m_claw);
    }

    @Override
    public void initialize() {
        m_claw.operate(m_clawState, true);
    }

    @Override
    public void end(boolean interrupted) {
        m_claw.operate(ClawState.HOLD, false);
    }

    @Override
    public boolean isFinished() {
        return m_claw.isFinished();
    }
}