package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.UpdateButtonBoardInputs;

public final class AligntoScorePoint extends CommandBase {
    private final UpdateButtonBoardInputs m_buttonBoardInputs;
    private final SwerveDrive m_swerve;
    private final Limelight m_Limelight;
    private Command m_command;

    public AligntoScorePoint(UpdateButtonBoardInputs buttonBoardInputs, SwerveDrive swerve, Limelight limelight) {
        m_buttonBoardInputs = buttonBoardInputs;
        m_swerve = swerve;
        m_Limelight = limelight;
    }
    
    @Override
    public void initialize() {
        // determine if we're doing AlignToScorePeg or AlignToAprilTag
        int target = m_buttonBoardInputs.getTarget();

        if (target == 2 || target == 5 || target == 8) {
            m_command = new AlignToAprilTag(m_Limelight, m_swerve);
        } else {
            m_command = new AlignToScorePeg(m_Limelight, m_swerve);
        }

        m_command.initialize();
    }
    
    @Override
    public void execute() {
        m_command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        m_command.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return m_command.isFinished();
    }
}
