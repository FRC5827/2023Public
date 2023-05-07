package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.subsystems.IntakeArm.IntakeState;

public class AutonScore extends SequentialCommandGroup {
    public AutonScore(IntakeArm intakeArm, Claw claw, ArmPivot armPivot, Pneumatics pneumatics) {
        addCommands(
            new ParallelCommandGroup(
                        new IntakeUpDown(intakeArm, IntakeState.IntakeDown),
                        new ClawCommand(claw, ClawState.CLOSE)
                    ),
                    // move arm to position
                    //new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kArmRestSensorDegree),
                    new SetPivotPosition(armPivot, Constants.ArmPivotConstants.kHighShelfLevelSensorDegree),
                    new ExtendArm(pneumatics),
                    new WaitCommand(1),
    
                    // drop the piece
                    new ClawCommand(claw, ClawState.OPEN),

                    //return arm to rest
                    new RetractArm(pneumatics),
                    new WaitCommand(0.5),
                    new SetPivotPosition(armPivot, Constants.ArmPivotConstants.kArmRestSensorDegree)
        );
    }
}
