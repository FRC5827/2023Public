package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmPivotConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public final class ArmPivot extends SubsystemBase {

    public enum ArmZone {
        BelowDanger(), InDanger(), AboveDanger();
    }

    private final WPI_TalonFX m_pivotMotor1 = new WPI_TalonFX (ArmPivotConstants.kArmFirstMotorID);
    private final WPI_TalonFX m_pivotMotor2 = new WPI_TalonFX(ArmPivotConstants.kArmSecondMotorID);
    private final CANCoder m_canCoder = new CANCoder(Constants.IDConstants.kArmPivotCanCoderID);
    
    private final DangerZoneAccessor m_accessor;

    private final PIDController m_controller;

    public ArmPivot(DangerZone dangerZone) {
        this.m_accessor = new DangerZoneAccessor(this, dangerZone, ArmPivotConstants.kDangerZoneLowerBound, ArmPivotConstants.kDangerZoneUpperBound);
        m_controller = new PIDController(0.015, 0, 0);
        SmartDashboard.putData(m_controller);
        m_controller.setSetpoint(this.getArmCancoderPosition());
        m_controller.setTolerance(5);
        m_controller.disableContinuousInput();    
    }

    public void resetCancoder() {
        m_canCoder.setPositionToAbsolute();
    }

    // Sets the speed of both motors so they turn together.
    // Clamps them to the max speed.
    public void setArmPivotSpeed(double speed) {
        double clampedSpeed = MathUtil.clamp(speed, -ArmPivotConstants.kArmPivotSpeed, ArmPivotConstants.kArmPivotSpeed);
        
        m_pivotMotor1.set(ControlMode.PercentOutput, -clampedSpeed);
        m_pivotMotor2.set(ControlMode.PercentOutput, clampedSpeed);
        
        SmartDashboard.putNumber("arm pivot output", clampedSpeed);
    }

    // This makes the absolute position a continuous run from -54 to 238.
    private double getArmCancoderPosition() {
        double pos = -m_canCoder.getAbsolutePosition();
        
        return pos;
    }

    public ArmZone getArmZone() {
        double position = getArmCancoderPosition();

        if (position > ArmPivotConstants.kDangerZoneUpperBound) {
            return ArmZone.AboveDanger;
        } else if (position < ArmPivotConstants.kDangerZoneLowerBound) {
            return ArmZone.BelowDanger;
        }

        return ArmZone.InDanger;
    }

    @Override
    public void periodic() {
        double position = this.getArmCancoderPosition();

        if (!m_accessor.checkAndTryEnter(position)) {
            // Not safe to enter right now, set setpoint to current position
            m_controller.setSetpoint(position);
        }
        else {
            double power = m_controller.calculate(position);
            setArmPivotSpeed(power);    
        }

        SmartDashboard.putNumber("arm pivot cancoder", position);
        SmartDashboard.putBoolean("Arm at set point", m_controller.atSetpoint());
        SmartDashboard.putNumber("Arm Pivot Setpoint", m_controller.getSetpoint());
    }

    public void setRequiredPivotPoint(double degrees) {
        m_controller.setSetpoint(degrees);
    }

    public boolean isFinished() {
        return m_controller.atSetpoint();
    }

    public void holdAtCurrentPosition() {
        m_controller.setSetpoint(this.getArmCancoderPosition());
    }
}
