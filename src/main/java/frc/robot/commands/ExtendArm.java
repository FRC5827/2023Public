package frc.robot.commands;

import frc.robot.subsystems.Pneumatics;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public final class ExtendArm extends CommandBase {
    private final Pneumatics m_pneumatics;

    public ExtendArm(Pneumatics pneumatics) {
        m_pneumatics = pneumatics;
        addRequirements(pneumatics);
    }
    
    @Override
    public void initialize() {
        m_pneumatics.extend();
        SmartDashboard.putBoolean("Extending: ", true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
