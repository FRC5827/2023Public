package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;

public final class AlignToAprilTag extends CommandBase {
    private final Limelight m_limelight;
    private final SwerveDrive m_swerveDrive;
    private final PIDController m_controller;

    public AlignToAprilTag(Limelight limelight, SwerveDrive swerveDrive) {
        m_limelight = limelight;
        m_swerveDrive = swerveDrive;
        m_controller = new PIDController(-0.1, 0, -0.002);
        
        m_controller.disableContinuousInput();
        m_controller.setSetpoint(0.0);
        m_controller.setTolerance(0.1);
        addRequirements(m_swerveDrive);
    }

    @Override
    public void initialize() {
        m_limelight.switchPipeline(0);
        //start = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        m_limelight.switchPipeline(0);
        if(m_limelight.getTX() != 0 && m_limelight.getPipeline()==0) {
            double xDrive = m_controller.calculate(m_limelight.getTX());
            /*double xDrive = 0;
            if(-m_limelight.getTX()>0) {
                xDrive = -0.2;
            } else if(-m_limelight.getTX()<0) {
                xDrive = 0.2;
            }
            xDrive *= Math.ceil(Math.abs(m_limelight.getTX())/5);*/
            //System.out.println(m_controller.calculate(-m_limelight.getTX()));
            if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                xDrive *= -1;
            }
            //System.out.println(Timer.getFPGATimestamp()-start + "\t" + -m_limelight.getTX()+ "\t" + xDrive);
            m_swerveDrive.drive(0, xDrive, 0, true, false, true);
        } else {
            m_swerveDrive.drive(0, 0, 0, true, false, true);
            m_controller.calculate(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_limelight.switchPipeline(0);
        m_swerveDrive.drive(0, 0, 0, false, false, false);
    }
    
}
