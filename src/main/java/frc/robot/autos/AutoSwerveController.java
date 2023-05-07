package frc.robot.autos;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class AutoSwerveController extends CommandBase {

    private final Timer m_timer = new Timer();
    private final PathPlannerTrajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final SwerveDriveKinematics m_kinematics;
    private final HolonomicDriveController m_controller;
    private final Consumer<SwerveModuleState[]> m_outputModuleStates;

    @SuppressWarnings("ParameterName")
    public AutoSwerveController(
            PathPlannerTrajectory trajectory,
            Supplier<Pose2d> pose,
            SwerveDriveKinematics kinematics,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            Consumer<SwerveModuleState[]> outputModuleStates,
            Subsystem... requirements) {
        m_trajectory = trajectory;
        m_pose = pose;
        m_kinematics = kinematics;

        m_controller = new HolonomicDriveController(
                xController,
                yController,
                thetaController);

        m_outputModuleStates = outputModuleStates;

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    @SuppressWarnings("LocalVariableName")
    public void execute() {
        double curTime = m_timer.get();
        var desiredState = (PathPlannerState) m_trajectory.sample(curTime);
    
        var desiredPose = desiredState.poseMeters;
        var currentPose = m_pose.get();

        double errorX = desiredPose.getX() - currentPose.getX();
        double errorY = desiredPose.getY() - currentPose.getY();
        double errorTheta= desiredPose.getRotation().getRadians() - currentPose.getRotation().getRadians(); 

        SmartDashboard.putNumber("errorX", errorX);
        SmartDashboard.putNumber("errorY", errorY);
        SmartDashboard.putNumber("errorTheta", errorTheta);
        SmartDashboard.putNumber("currentX", currentPose.getX());
        SmartDashboard.putNumber("desiredX", desiredPose.getX());
        SmartDashboard.putNumber("desiredY", desiredPose.getY());
        SmartDashboard.putNumber("currentY", currentPose.getY());
        SmartDashboard.putNumber("currentT", currentPose.getRotation().getRadians());
        SmartDashboard.putNumber("desiredT", desiredPose.getRotation().getRadians());
        SmartDashboard.putNumber("desiredV", desiredState.velocityMetersPerSecond);
       
        

        var targetChassisSpeeds = m_controller.calculate(currentPose, desiredState, desiredState.holonomicRotation);
        var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

        m_outputModuleStates.accept(targetModuleStates);
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
    
}
