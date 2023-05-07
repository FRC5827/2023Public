package frc.robot.commands;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public abstract class GeneratedPath extends CommandBase {
    protected Command m_command;

    /**
     * Creates a path point at location (setX, setY) and sets the heading to go in the direction of (newX, newY) from (oldX, oldY)
     * @param setX
     * @param setY
     * @param newX
     * @param newY
     * @param oldX
     * @param oldY
     * @param rotation
     * @return
     */
    public PathPoint createPathPoint(double setX, double setY, double newX, double newY, double oldX, double oldY, Rotation2d rotation) {
        if(DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            newX = Constants.FieldConstants.length - newX;
            oldX = Constants.FieldConstants.length - oldX;
            setX = Constants.FieldConstants.length - setX;
        }
        return new PathPoint(new Translation2d(setX, setY), new Rotation2d(newX-oldX, newY-oldY), rotation);
    }

    /**
     * Creates a path point at location (newX, newY) and sets the heading to go in the direction of (newX, newY) from (oldX, oldY)
     * @param newX
     * @param newY
     * @param oldX
     * @param oldY
     * @param rotation
     * @return
     */
    public PathPoint createPathPoint(double newX, double newY, double oldX, double oldY, Rotation2d rotation) {
        return createPathPoint(newX, newY, newX, newY, oldX, oldY, rotation);
    }

    /**
     * Creates a path point at location (oldX, oldY) and sets the heading to go in the direction of (newX, newY) from (oldX, oldY)
     * @param newX
     * @param newY
     * @param oldX
     * @param oldY
     * @param rotation
     * @return
     */
    public PathPoint createStartPathPoint(double oldX, double oldY, double newX, double newY, Rotation2d rotation) {
        return createPathPoint(oldX, oldY, newX, newY, oldX, oldY, rotation);
    }

    public abstract Command generateCommand();

    @Override
    public void initialize() {
        m_command = generateCommand();
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
