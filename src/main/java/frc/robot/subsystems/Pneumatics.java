package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Pneumatics extends SubsystemBase {
    private final DoubleSolenoid m_solenoid;
    
    public Pneumatics(DoubleSolenoid solenoid) {
        this.m_solenoid = solenoid;
    }

    public void extend() {
        m_solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        m_solenoid.set(DoubleSolenoid.Value.kReverse);
    }
}
