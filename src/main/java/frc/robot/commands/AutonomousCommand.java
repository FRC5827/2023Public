// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * A complex auto command sequence.
 */
public final class AutonomousCommand extends SequentialCommandGroup {
  /**
   * Creates a new AutonomousCommand.
   *
   * @param commandList The ordered list of commands to execute
   */
  public AutonomousCommand(List<Command> commandList) {
      for (Command command : commandList) {
      addCommands(command);
    }
  }

  public void Cancel() {
      this.cancel();
  }

}
