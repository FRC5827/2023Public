package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public final class AutoBalance extends CommandBase {
    private final SwerveDrive m_swerveDrive;
    private final AHRS m_gyro;

    public AutoBalance(SwerveDrive swerveDrive, AHRS gyro) {
        m_gyro = gyro;
        m_swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        //get the current tiltness from gyroscope
        double xAngleDegree = m_gyro.getPitch();
        double yAngleDegree = m_gyro.getRoll();
        //balance mode of x and y direction
        boolean xBalanceOn = (Math.abs(xAngleDegree) > Constants.AutoConstants.kOffBalanceAngle);
        boolean yBalanceOn = (Math.abs(yAngleDegree) > Constants.AutoConstants.kOffBalanceAngle);

        //reset axes values to 0
        double xAxis = 0;
        double yAxis = 0;

        //update axes values
        if (xBalanceOn) {
            double pitchAngleRadians = xAngleDegree * (Math.PI / 180.0);
            xAxis = Math.sin(pitchAngleRadians) * 1.8;
        }
        if (yBalanceOn) {
            double rollAngleRadians = yAngleDegree * (Math.PI / 180.0);
            yAxis = Math.sin(rollAngleRadians) * 1.8;
        }
        
        if (!xBalanceOn && !yBalanceOn) {
            m_swerveDrive.setBrakeMode(true);
        } else {
            m_swerveDrive.drive(xAxis, yAxis, 0.0, false, false, true); 
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.drive(0, 0, 0, false, false, false);
    }
}