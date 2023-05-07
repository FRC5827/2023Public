package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer.ButtonBoardButton;
import frc.robot.commands.FlapdexerCommand;

public final class UpdateButtonBoardInputs extends SubsystemBase
{
   private final GenericHID m_buttonBoard;
   private final Map<ButtonBoardButton, Integer> m_buttonMap;
   private final FlapdexerSubsystem m_flapdexer;
   private final SwerveDrive m_swerveDrive;

   // for scoring position, and the low / mid / high scoring
   private int m_scoringTarget = 1; // set default target to 1
   private double m_scoringSelectionNumber = 0;

   // for flapdexer
   private final FlapdexerCommand m_flapdexerCommand;

   // for claw
   private boolean m_isCube;

   private boolean m_isFieldRelativeFirstTime = true;
   private boolean m_isFieldRelativeButtonPressed = false;

   public UpdateButtonBoardInputs(
      GenericHID buttonBoard, Map<ButtonBoardButton, Integer> buttonMap,
      FlapdexerSubsystem flapdexer,
      SwerveDrive swerveDrive) {
      m_buttonBoard = buttonBoard;
      m_buttonMap = buttonMap;
      m_flapdexer = flapdexer;
      m_swerveDrive = swerveDrive;

      m_flapdexerCommand = new FlapdexerCommand(m_flapdexer);
   }

   public int getTarget() {
      return m_scoringTarget;
   }

   public double getPivotAngle() {
      if (m_scoringSelectionNumber > 0.9) {
         return Constants.ArmPivotConstants.kHighShelfLevelSensorDegree; // high
      } else if (m_scoringSelectionNumber < -0.9) {
         return Constants.ArmPivotConstants.kArmFullFowardSensorDegree; // low
      }
      return Constants.ArmPivotConstants.kMiddleShelfLevelSensorDegree; // middle
   }

   public double getIntakeShooterSpeed() {
      if (m_scoringSelectionNumber > 0.9) {
         return Constants.IntakeConstants.kIntakeShooterSpeedHigh; // high
      } else if (m_scoringSelectionNumber < -0.9) {
         return Constants.IntakeConstants.kIntakeShooterSpeedLow; // low
      }
      return Constants.IntakeConstants.kIntakeShooterSpeedMed; // middle
   }

   @Override
   public void periodic() {
      // for scoring positions
      SmartDashboard.putNumber("Current Target", m_scoringTarget);
      for (ButtonBoardButton keyName : m_buttonMap.keySet()) {
         int scoringID = m_buttonMap.get(keyName);
         // Temporarily reassigning scoring id 9 to do field offset, instead of scoring position.
         // if (scoringID >= 1 && scoringID <= 9 && m_buttonBoard.getRawButtonPressed(scoringID)) {
         if (scoringID >= 1 && scoringID <= 8 && m_buttonBoard.getRawButtonPressed(scoringID)) {
               m_scoringTarget = scoringID;
         }
      }

      // for scoring state (low, middle, high)
      m_scoringSelectionNumber = m_buttonBoard.getRawAxis(1); // having axis ID of 1

      if (m_scoringSelectionNumber > 0.9) {
         SmartDashboard.putString("Pivot State", "High");
      } else if (m_scoringSelectionNumber < -0.9) {
         SmartDashboard.putString("Pivot State", "Low");
      } else {
         SmartDashboard.putString("Pivot State", "Middle");
      }

      // for flapdexer
      int flapdexerButtonID = m_buttonMap.get(ButtonBoardButton.button_flapdexer);
      boolean isFlapdexerPressed = m_buttonBoard.getRawButtonPressed(flapdexerButtonID); // only true at the instant it
                                                                                         // is pressed
      boolean isFlapdexerReleased = m_buttonBoard.getRawButtonReleased(flapdexerButtonID); // only true at the instant
                                                                                           // it is released

      if (isFlapdexerPressed) { // mimicking the structure of the method "whileTrue"
         m_flapdexerCommand.schedule(); // schedule this command when it's initially pressed
         SmartDashboard.putBoolean("Flapdexer Usage", true);
      } else if (isFlapdexerReleased) {
         m_flapdexerCommand.cancel(); // cancel this command when it's released
         SmartDashboard.putBoolean("Flapdexer Usage", false);
      }

      // for cube or cone
      int objectTypeButton = m_buttonMap.get(ButtonBoardButton.button_twelve);

      if (m_buttonBoard.getRawButtonPressed(objectTypeButton)) {
         m_isCube = true; //cube
         SmartDashboard.putString("Game Piece", "Cube");
      } else if (m_buttonBoard.getRawButtonReleased(objectTypeButton)) {
         m_isCube = false; //cone
         SmartDashboard.putString("Game Piece", "Cone");
      }

      boolean isFieldRelativeButtonPressed = m_buttonBoard.getRawButton(9);
      // Avoid accidentally resetting heading arbitrarily, if the button starts in the true state.
      if (!m_isFieldRelativeFirstTime) {
         if (m_isFieldRelativeButtonPressed != isFieldRelativeButtonPressed) {
            m_swerveDrive.zeroHeading();
         }
      }

      m_isFieldRelativeFirstTime = false;
      m_isFieldRelativeButtonPressed = isFieldRelativeButtonPressed;
   }
}
