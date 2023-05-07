package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public final class RetractArm extends CommandBase{
    
    private final Pneumatics m_pneumatics;

    public RetractArm(Pneumatics pneumatics) {
        m_pneumatics = pneumatics;
        addRequirements(pneumatics);
    }

    @Override
    public void initialize() {
        m_pneumatics.retract();
        SmartDashboard.putBoolean("Extending", false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
