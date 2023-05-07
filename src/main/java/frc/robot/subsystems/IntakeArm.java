package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public final class IntakeArm extends SubsystemBase {

    private final WPI_TalonFX m_intakeArmMotor = new WPI_TalonFX (Constants.IDConstants.kIntakeArmMotorID);
    private final CANCoder m_canCoder = new CANCoder(Constants.IDConstants.kIntakeCanCoderID);
    private final PIDController m_PIDController;

    private final DangerZoneAccessor m_accessor;
   
    public enum IntakeState {
        IntakeUp(Constants.IntakeConstants.kIntakeUpSensorDegrees),
        IntakeDown(Constants.IntakeConstants.kIntakeDownSensorDegrees);

        private final double value;

        IntakeState(double value) {
            this.value = value;
        }

        public double getValue() {
            return this.value;
        }
    }

    public IntakeArm(DangerZone dangerzone) {
        this.m_accessor = new DangerZoneAccessor(this, dangerzone, Constants.IntakeConstants.kDangerZoneLowerBound, Constants.IntakeConstants.kDangerZoneUpperBound);
        m_PIDController = new PIDController(Constants.IntakeConstants.kP, Constants.IntakeConstants.kI, Constants.IntakeConstants.kD);
        SmartDashboard.putData("intakeUpDownPID", m_PIDController);
        m_PIDController.setSetpoint(getIntakeEncoderPosition());
        m_PIDController.setTolerance(Constants.IntakeConstants.kPIDLoopTolerance);
        m_PIDController.disableContinuousInput();
    }

    public void resetCancoder() {
        m_canCoder.setPositionToAbsolute();
    }

    public void setIntakePIDSetpoint(double point) {
        m_PIDController.setSetpoint(point);
    }

    public double getIntakeEncoderPosition() {
        double position = m_canCoder.getAbsolutePosition() + Constants.IntakeConstants.kIntakeArmEncoderOffset;
        position = MathUtil.inputModulus(position, 0, 360);
        SmartDashboard.putNumber("/Intake Encoder/Position", position);
        return position;
    }

    public boolean atSetpoint() {
        return m_PIDController.atSetpoint();
    }

    /**
     * 
     * @return value from built in encoder on first motor
     */
    @Override
    public void periodic() {
        // force PID loop value because it keeps changing to 1.0
        m_PIDController.setP(Constants.IntakeConstants.kP); // TODO: This seems a bit iffy
        double position = getIntakeEncoderPosition();
        SmartDashboard.putNumber("intake encoder position", position);

        if (!m_accessor.checkAndTryEnter(position)) {
            // Not safe to enter right now, set setpoint to current position
            m_PIDController.setSetpoint(position);
            return;
        }
        
        double calculatedSpeed = -m_PIDController.calculate(position);

        calculatedSpeed = MathUtil.clamp(calculatedSpeed, -Constants.IntakeConstants.kIntakeArmUpSpeed,
                                                           Constants.IntakeConstants.kIntakeArmDownSpeed);

        SmartDashboard.putNumber("intake PID output", calculatedSpeed);
        SmartDashboard.putNumber("intake PID setpoint", m_PIDController.getSetpoint());
        SmartDashboard.putNumber("intake PID kP", m_PIDController.getP());
        SmartDashboard.putNumber("intake PID tolerance", m_PIDController.getPositionTolerance());

        m_intakeArmMotor.set(ControlMode.PercentOutput, calculatedSpeed);
    }
}
