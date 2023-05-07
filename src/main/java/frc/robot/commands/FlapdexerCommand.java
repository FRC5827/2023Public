package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.FlapdexerSubsystem;

public final class FlapdexerCommand extends CommandBase {
    
    private final FlapdexerSubsystem m_flapdexer;
    
    public FlapdexerCommand(FlapdexerSubsystem flapdexer) {
        m_flapdexer = flapdexer;
        addRequirements(m_flapdexer);
    }

    @Override
    public void initialize() {
        m_flapdexer.startFlap();
    }

    @Override
    public void end(boolean interrupted) {
        m_flapdexer.stopFlap();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
