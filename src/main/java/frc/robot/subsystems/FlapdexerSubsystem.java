package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FlapdexerSubsystem extends SubsystemBase {
    //creating motor object
    private final WPI_TalonFX m_flapdexerMotor = new WPI_TalonFX(Constants.IDConstants.kFlapdexerMotorID);
    private double m_accumulatorValue = 0;
    private boolean m_run = false;

    public void startFlap() {
        this.startFlap(Constants.FlapdexerConstants.kflapperSpeed);
    }
    
    private void startFlap(double flapdexerSpeed) {
        m_run = true;
        //sets motors to a set speed, decided by constants
        m_flapdexerMotor.set(TalonFXControlMode.PercentOutput, flapdexerSpeed);
        SmartDashboard.putNumber("flapdexer motor output", flapdexerSpeed);
    }
    
    public void stopFlap() {
        m_run = false;
    }

    @Override
    public void periodic() {
        // 2048 for ticks per rotation, 5 to change time to 20 ms instead of 100 ms
        m_accumulatorValue += ((m_flapdexerMotor.getSelectedSensorVelocity()/2048.0)/5)/Constants.FlapdexerConstants.kGearRatio;
        if(!m_run) {
            if(m_accumulatorValue % 1 >= 0.82 && m_accumulatorValue % 1 <= 0.87) {
                m_flapdexerMotor.set(TalonFXControlMode.PercentOutput, 0.0);
                SmartDashboard.putNumber("flapdexer motor output", 0.0);
            }
        }
        SmartDashboard.putNumber("flapdexer accumulator value", m_accumulatorValue);
    }
}
