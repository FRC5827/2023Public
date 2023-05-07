package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.UpdateButtonBoardInputs;

/**
 * Command to update the required arm position. Utilizes the ArmPivot subsystem that
 * has it's own constantly running PID controller.
 */
public final class SetPivotPosition extends CommandBase {

    private final ArmPivot m_pivot;
    private final DoubleSupplier m_supplier;

    // Constructor
    public SetPivotPosition(ArmPivot pivot, double targetDegrees) {
        this(pivot, () -> targetDegrees);
    }

    public SetPivotPosition(ArmPivot pivot, UpdateButtonBoardInputs inputs) {
        this(pivot, () -> inputs.getPivotAngle());
    }

    public SetPivotPosition(ArmPivot pivot, DoubleSupplier supplier) {
        m_pivot = pivot;
        m_supplier = supplier;
        addRequirements(m_pivot);        
    }

    @Override
    public void initialize() {
        m_pivot.setRequiredPivotPoint(m_supplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_pivot.holdAtCurrentPosition();
    }

    @Override
    public boolean isFinished() {
        return m_pivot.isFinished();
    }
}