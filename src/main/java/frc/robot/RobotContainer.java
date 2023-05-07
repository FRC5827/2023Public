// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.EnumMap;
import java.util.Map;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.TrajectoryLoader;
import frc.lib.util.TrajectoryLoader.Trajectories;
import frc.robot.Constants.ArmPivotConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutonScore;
import frc.robot.commands.ContinuousClawCommand;
import frc.robot.commands.EmptyCommand;
import frc.robot.commands.ExtendArm;
import frc.robot.subsystems.FlapdexerSubsystem;
import frc.robot.subsystems.IntakeArm;
import frc.robot.commands.IntakeSpin;
import frc.robot.commands.IntakeUpDown;
import frc.robot.commands.RetractArm;
import frc.robot.commands.SetPivotPosition;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DangerZone;
import frc.robot.subsystems.IntakeSpinner;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pneumatics;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.UpdateButtonBoardInputs;
import frc.robot.subsystems.ArmPivot.ArmZone;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.subsystems.IntakeArm.IntakeState;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    public enum ButtonBoardButton{
        button_one,button_two,button_three,button_four,button_five,
        button_six,button_seven,button_eight,button_nine,
        button_red_blue,button_flapdexer,button_twelve,button_thirteen
    }

    public enum States {
        Rest(), Grab(), Score(), Pickup(), Defense();

        public States getNextState() {
            switch (this) {
                case Pickup:
                    return Rest;

                default:
                case Rest:
                    return Grab;
                
                case Grab:
                    return Score;

                case Score:
                    return Pickup;
            }
        }

        public States getPreviousState() {
            switch (this) {
                default:
                case Rest:
                    return Pickup;
                    
                case Pickup:
                    return Score;
                
                case Score:
                    return Grab;

                case Grab:
                    return Rest;
            }
        }
    }

    private final EnumMap<States, Command> m_commands = new EnumMap<States, Command>(States.class);

    // Controllers
    private final XboxController m_driverInput = new XboxController(0);
    public final GenericHID m_buttonBoard;

    // Drive Controls
    private final int m_translationAxis = XboxController.Axis.kLeftY.value;
    private final int m_strafeAxis = XboxController.Axis.kLeftX.value;
    private final int m_rotationAxis = XboxController.Axis.kRightX.value;

    // Driver buttons
    private final Map<ButtonBoardButton, Integer > m_buttonMap = new EnumMap<>(ButtonBoardButton.class); 

    // Controller Definitions

    // Buttons (Y B X A)
    private final JoystickButton m_restMode = new JoystickButton(m_driverInput, XboxController.Button.kB.value);
    private final JoystickButton m_grabMode = new JoystickButton(m_driverInput, XboxController.Button.kX.value);
    private final JoystickButton m_scoreMode = new JoystickButton(m_driverInput, XboxController.Button.kA.value);
    private final JoystickButton m_intakeMode = new JoystickButton(m_driverInput, XboxController.Button.kY.value);

    // Back / Start (middle buttons)
    private final JoystickButton m_fieldRelativeBack = new JoystickButton(m_driverInput, XboxController.Button.kBack.value);
    private final JoystickButton m_updateFromLimelightStart = new JoystickButton(m_driverInput, XboxController.Button.kStart.value);

    // POV (d-pad)
    private final POVButton m_extendArm = new POVButton(m_driverInput, 0); // Up
    private final POVButton m_retractArm = new POVButton(m_driverInput, 180); // Down
    private final POVButton m_defenseMode = new POVButton(m_driverInput, 90); // right
    private final POVButton m_autoBalancedX = new POVButton(m_driverInput, 270); // left

    // Left / Right Bumpers
    private final JoystickButton m_clawButtonOpen = new JoystickButton(m_driverInput, XboxController.Button.kLeftBumper.value);
    private final JoystickButton m_clawButtonClose = new JoystickButton(m_driverInput, XboxController.Button.kRightBumper.value);

    // Left / Right Triggers
    private final Trigger m_shootFromIntake = new Trigger(() -> { return m_driverInput.getRightTriggerAxis() > 0.7;});
    private final Trigger m_intakeSpinTrigger = new Trigger(() -> { return m_driverInput.getLeftTriggerAxis() > 0.7;});
    
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
    private final DoubleSolenoid m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    // Danger zones
    private final DangerZone m_arm_and_intake_dangerzone = new DangerZone();

    // Subsystems
    private final Claw m_claw = new Claw();
    private final Limelight m_limelight = new Limelight();
    private final SwerveDrive m_swerveDrive = new SwerveDrive(m_limelight, m_gyro);
    private final UpdateButtonBoardInputs m_buttonBoardInputs;
    private final IntakeSpinner m_intakeSpinner = new IntakeSpinner();
    private final FlapdexerSubsystem m_flapdexerSubsystem = new FlapdexerSubsystem();
    private final Pneumatics m_pneumatics = new Pneumatics(m_solenoid);
    private final IntakeArm m_intakeArm = new IntakeArm(m_arm_and_intake_dangerzone);
    private final ArmPivot m_armPivot = new ArmPivot(m_arm_and_intake_dangerzone);

    // Swerve Drive configuration
    private boolean m_slowMode;
    private boolean m_fieldRelative;
    private final boolean m_openLoop;
    private final ShuffleboardTab m_driveShuffleboardTab;
    private final GenericEntry m_fieldRelativeStateShuffleboard;
    private final GenericEntry m_slowModeStateShuffleboard;

    // Auton selection
    private final SendableChooser<Double> m_autonomousDelay = new SendableChooser<Double>(); 
	private final SendableChooser<Trajectories> m_autoSelector = new SendableChooser<Trajectories>();
    private final SendableChooser<Boolean> m_autonomousWithMechanism = new SendableChooser<Boolean>();

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer() {
        //stuffs for Co Pilot
        m_buttonMap.put(ButtonBoardButton.button_one, 1);
        m_buttonMap.put(ButtonBoardButton.button_two, 2);
        m_buttonMap.put(ButtonBoardButton.button_three, 3);
        m_buttonMap.put(ButtonBoardButton.button_four, 4);
        m_buttonMap.put(ButtonBoardButton.button_five, 5);
        m_buttonMap.put(ButtonBoardButton.button_six, 6);
        m_buttonMap.put(ButtonBoardButton.button_seven, 7);
        m_buttonMap.put(ButtonBoardButton.button_eight, 8);
        m_buttonMap.put(ButtonBoardButton.button_nine, 9);
        m_buttonMap.put(ButtonBoardButton.button_red_blue, 10);
        m_buttonMap.put(ButtonBoardButton.button_flapdexer, 11);
        m_buttonMap.put(ButtonBoardButton.button_twelve, 12);
        m_buttonMap.put(ButtonBoardButton.button_thirteen, 13);

        m_buttonBoard = new GenericHID(1); //add copilot input

        m_buttonBoardInputs = new UpdateButtonBoardInputs(m_buttonBoard, m_buttonMap, m_flapdexerSubsystem, m_swerveDrive);

        // Set the scheduler to log Shuffleboard events for command initialize, interrupt, finish
        CommandScheduler.getInstance()
            .onCommandInitialize(
                command ->
                    Shuffleboard.addEventMarker(
                        "Command initialized", command.getName(), EventImportance.kNormal));
        CommandScheduler.getInstance()
            .onCommandInterrupt(
                command ->
                    Shuffleboard.addEventMarker(
                        "Command interrupted", command.getName(), EventImportance.kNormal));
        CommandScheduler.getInstance()
            .onCommandFinish(
                command ->
                    Shuffleboard.addEventMarker(
                        "Command finished", command.getName(), EventImportance.kNormal));

        
        m_slowMode = false;
        m_fieldRelative = true;
        m_openLoop = true;
                
        m_driveShuffleboardTab = Shuffleboard.getTab("Drivetrain");
        m_fieldRelativeStateShuffleboard = m_driveShuffleboardTab.add("Field Relative", m_fieldRelative)
                                                    .withPosition(0, 0)
                                                    .withSize(1, 1)
                                                    .withWidget(BuiltInWidgets.kToggleButton).getEntry();
        m_driveShuffleboardTab.add("FieldRelativeStatus", m_fieldRelative)
                                                    .withPosition(1, 0)
                                                    .withSize(1, 1)
                                                    .withWidget(BuiltInWidgets.kBooleanBox);
        m_slowModeStateShuffleboard = m_driveShuffleboardTab.add("Slow Mode", m_slowMode)
                                                    .withPosition(0, 1)
                                                    .withSize(1, 1)
                                                    .withWidget(BuiltInWidgets.kToggleButton).getEntry();
        m_driveShuffleboardTab.add("SlowModeStatus", m_slowMode)
                                                    .withPosition(1, 1)
                                                    .withSize(1, 1)
                                                    .withWidget(BuiltInWidgets.kBooleanBox);


        m_commands.put(States.Rest, createStateCommand(States.Rest));
        m_commands.put(States.Grab, createStateCommand(States.Grab));
        m_commands.put(States.Pickup, createStateCommand(States.Pickup));
        m_commands.put(States.Score, createStateCommand(States.Score));
        m_commands.put(States.Defense, createStateCommand(States.Defense));
                                                                                                
        // m_commands.put(States.Rest, createStateCommandNoArm(States.Rest));
        // m_commands.put(States.Grab, createStateCommandNoArm(States.Grab));
        // m_commands.put(States.Pickup, createStateCommandNoArm(States.Pickup));
        // m_commands.put(States.Score, createStateCommandNoArm(States.Score));
        // m_commands.put(States.Defense, createStateCommandNoArm(   States.Defense));
                                            
        // Configure the button bindings
        configureButtonBindings();

        m_swerveDrive.setDefaultCommand(
            new TeleopSwerve(m_swerveDrive, m_driverInput, m_translationAxis, m_strafeAxis, m_rotationAxis, m_openLoop));

        initChooser();

        TrajectoryLoader.loadTrajectories();
    }

    private void configureButtonBindings() {
        
        // Setup / configuration
        m_updateFromLimelightStart.toggleOnTrue(new InstantCommand(m_swerveDrive::updatePoseFromLimelight).ignoringDisable(true));
        m_fieldRelativeBack.onTrue(new InstantCommand(this::toggleFieldRelative));
        
        // Individual commands
        m_extendArm.onTrue(new ExtendArm(m_pneumatics) );
        m_retractArm.onTrue(new RetractArm(m_pneumatics));
 
        m_clawButtonOpen.whileTrue(new ContinuousClawCommand(m_claw, ClawState.OPEN));
        m_clawButtonClose.whileTrue(new ContinuousClawCommand(m_claw, ClawState.CLOSE));

        m_autoBalancedX.whileTrue(new AutoBalance(m_swerveDrive, m_gyro));

        m_shootFromIntake.whileTrue(new IntakeSpin(m_intakeSpinner, false, m_buttonBoardInputs));
        m_intakeSpinTrigger.whileTrue(new IntakeSpin(m_intakeSpinner, true, m_buttonBoardInputs));

        // Complex Commands
        m_restMode.whileTrue(m_commands.get(States.Rest));
        m_grabMode.whileTrue(m_commands.get(States.Grab));
        m_scoreMode.whileTrue(m_commands.get(States.Score));
        m_intakeMode.whileTrue(m_commands.get(States.Pickup));
        m_defenseMode.whileTrue(m_commands.get(States.Defense));
    }

    public void resetCancoders() {
        m_armPivot.resetCancoder();
        m_intakeArm.resetCancoder();
    }

    public void initChooser() {
        m_autoSelector.addOption("1 - Score",                    Trajectories._1_Score_P1);
        m_autoSelector.addOption("1 - Score and Move and Pickup",           Trajectories._1_ScoreMovePickup_P1);
        m_autoSelector.addOption("1 - Score and Move and Charge", Trajectories._1_ScoreMoveCharge_P1);
        m_autoSelector.addOption("2 - Score and Move and Pickup and Charge",Trajectories._2_ScoreMovePickupCharge_P1);
        m_autoSelector.addOption("2 - Score and Move and Charge", Trajectories._2_ScoreMoveCharge_P1);
        m_autoSelector.addOption("3 - Score and Move and Pickup", Trajectories._3_ScoreMovePickup_P1);
        m_autoSelector.addOption("3 - Score and Move and Pickup and Charge", Trajectories._3_ScoreMoveCharge_P1);
        m_autoSelector.addOption("Safe Mode", Trajectories._SafeMode);
        //new adjusted paths

        //m_autoSelector.addOption("3 - Score and Move and Pickup", Trajectories._3_ScoreMovePickup_P1);
		Shuffleboard.getTab("Autonomous").add("Autonomous sequence", m_autoSelector).withPosition(0, 0)
                            .withSize(2, 1).withWidget(BuiltInWidgets.kComboBoxChooser);
        m_autonomousDelay.setDefaultOption("0 Second Delay", 0.0);
        m_autonomousDelay.addOption("2 Second Delay",        2.0);
        m_autonomousDelay.addOption("5 Second Delay",        5.0);
        m_autonomousDelay.addOption("8 Second Delay",        8.0);
        m_autonomousDelay.addOption("10 Second Delay",       10.0);
        Shuffleboard.getTab("Autonomous").add("Autonomous sequence delay", m_autonomousDelay).withPosition(2, 0)
                            .withSize(2, 1).withWidget(BuiltInWidgets.kComboBoxChooser);

        //add option to select if we 
        Shuffleboard.getTab("Autonomous").add("Use Scoring Mechanism", m_autonomousWithMechanism).withPosition(4, 0)
                            .withSize(2, 1).withWidget(BuiltInWidgets.kComboBoxChooser);
        m_autonomousWithMechanism.setDefaultOption("True", true);
        m_autonomousWithMechanism.addOption("False", false);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * (this is the original "getAutonomousCommand")
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        boolean selected = m_autonomousWithMechanism.getSelected();

        if (selected) {
            return getAutonomousCommandDefault();
        }

        return getAutonomousCommandWithScoringCommandsInline();
    }

    /**
     * return the autonomous command using robot states
     *
     * @return the autonomous command with robot states
     */
    private Command getAutonomousCommandDefault() {
        Trajectories selected = m_autoSelector.getSelected();
        final PathPlannerTrajectory trajectory; 

        if(selected == null) {
            System.err.println("No auton command to run!");
            return new WaitCommand(0.0);
        }

		switch (selected) {
            case _SafeMode:
                return new InstantCommand();
            case _1_Score_P1:
                trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(TrajectoryLoader.getTrajectory(Trajectories._2_ScoreMoveCharge_P1), DriverStation.getAlliance());
                return new SequentialCommandGroup(
                    new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                        trajectory.getInitialPose(),
                        true)),

                    new AutonScore(m_intakeArm, m_claw, m_armPivot, m_pneumatics)
                );
            case _1_ScoreMovePickup_P1:
                trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(TrajectoryLoader.getTrajectory(selected), DriverStation.getAlliance());
                return new SequentialCommandGroup(
                    new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                        trajectory.getInitialPose(),
                        true)),

                    new AutonScore(m_intakeArm, m_claw, m_armPivot, m_pneumatics),
                    new ParallelCommandGroup(
                        this.getFollowPathCommand(trajectory),
                        new IntakeSpin(m_intakeSpinner, true, m_buttonBoardInputs))
                );
            case _2_ScoreMoveCharge_P1:
                trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(TrajectoryLoader.getTrajectory(selected), DriverStation.getAlliance());
                return new SequentialCommandGroup(
                    new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                        trajectory.getInitialPose(),
                        true)),

                        new AutonScore(m_intakeArm, m_claw, m_armPivot, m_pneumatics),

                    new ParallelDeadlineGroup(
                        this.getFollowPathCommand(trajectory),
                        new IntakeSpin(m_intakeSpinner, true, m_buttonBoardInputs)
                    ),
                
                    new AutoBalance(m_swerveDrive, m_gyro)
                );
            case _3_ScoreMovePickup_P1:
                trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(TrajectoryLoader.getTrajectory(selected), DriverStation.getAlliance());
                return new SequentialCommandGroup(
                    new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                        trajectory.getInitialPose(),
                        true)),

                        new AutonScore(m_intakeArm, m_claw, m_armPivot, m_pneumatics),
                    new ParallelCommandGroup(
                        this.getFollowPathCommand(trajectory),
                        new IntakeSpin(m_intakeSpinner, true, m_buttonBoardInputs))
                );

            case _1_ScoreMoveCharge_P1: 
                trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(TrajectoryLoader.getTrajectory(selected), DriverStation.getAlliance());
                return new SequentialCommandGroup(
                    new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                        trajectory.getInitialPose(),
                        true)),

                        new AutonScore(m_intakeArm, m_claw, m_armPivot, m_pneumatics),

                    new ParallelDeadlineGroup(
                        this.getFollowPathCommand(trajectory),
                        new IntakeSpin(m_intakeSpinner, true, m_buttonBoardInputs)
                    ),
            
                    new AutoBalance(m_swerveDrive, m_gyro)
                );

            case _3_ScoreMoveCharge_P1:
                trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(TrajectoryLoader.getTrajectory(selected), DriverStation.getAlliance());
                return new SequentialCommandGroup(
                    new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                        trajectory.getInitialPose(),
                        true)),

                        new AutonScore(m_intakeArm, m_claw, m_armPivot, m_pneumatics),
                    new ParallelDeadlineGroup(
                        this.getFollowPathCommand(trajectory),
                        new IntakeSpin(m_intakeSpinner, true, m_buttonBoardInputs)),
    
                    //auto balance
                    new AutoBalance(m_swerveDrive, m_gyro)
                );

            default:
                // nothing to do -- shouldn't ever happen
                System.err.println("No auton command to run!");
                return new WaitCommand(0.0);
        }

    }

    /**
     * return the autonomous commands without using robot states
     * 
     * @return autonomous command without using any robot states
     */
    private Command getAutonomousCommandWithScoringCommandsInline() {
        Trajectories selected = m_autoSelector.getSelected();
        final PathPlannerTrajectory trajectory;

        if(selected == null) {
            System.err.println("No auton command to run!");
            return new WaitCommand(0.0);
        }

		switch (selected) {
            case _SafeMode:
                return new InstantCommand();
            case _1_Score_P1:
                trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(TrajectoryLoader.getTrajectory(Trajectories._2_ScoreMoveCharge_P1), DriverStation.getAlliance());
                return new SequentialCommandGroup(
                    new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                        trajectory.getInitialPose(),
                        true))
                );
            case _1_ScoreMovePickup_P1:
                trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(TrajectoryLoader.getTrajectory(selected), DriverStation.getAlliance());
                return new SequentialCommandGroup(
                    new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                        trajectory.getInitialPose(),
                        true)),
                    
                    this.getFollowPathCommand(trajectory)
                );
            case _2_ScoreMoveCharge_P1:
                trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(TrajectoryLoader.getTrajectory(selected), DriverStation.getAlliance());
                return new SequentialCommandGroup(
                    new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                        trajectory.getInitialPose(),
                        true)),

                    this.getFollowPathCommand(trajectory),
                
                    new AutoBalance(m_swerveDrive, m_gyro)
                );
            case _3_ScoreMovePickup_P1:
                trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(TrajectoryLoader.getTrajectory(selected), DriverStation.getAlliance());
                return new SequentialCommandGroup(
                    new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                        trajectory.getInitialPose(),
                        true)),

                    this.getFollowPathCommand(trajectory)
                );

            case _1_ScoreMoveCharge_P1: 
                trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(TrajectoryLoader.getTrajectory(selected), DriverStation.getAlliance());
                return new SequentialCommandGroup(
                    new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                        trajectory.getInitialPose(),
                        true)),

                    this.getFollowPathCommand(trajectory),
            
                    new AutoBalance(m_swerveDrive, m_gyro)
                );

            case _3_ScoreMoveCharge_P1:
                trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(TrajectoryLoader.getTrajectory(selected), DriverStation.getAlliance());
                return new SequentialCommandGroup(
                    new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                        trajectory.getInitialPose(),
                        true)),

                    this.getFollowPathCommand(trajectory),
    
                    //auto balance
                    new AutoBalance(m_swerveDrive, m_gyro)
                );

            default:
                // nothing to do -- shouldn't ever happen
                System.err.println("No auton command to run!");
                return new WaitCommand(0.0);
        }
    }

    /**
     * Toggles field relative state in networktables via shuffleboard boolean
     * for use by other commands/subsystems
     */
    public void toggleFieldRelative() {
        this.m_fieldRelative = !this.m_fieldRelative;
        m_fieldRelativeStateShuffleboard.setBoolean(this.m_fieldRelative);
    }

    /**
     * Sets slow mode state in networktables via shuffleboard boolean
     * for use by other commands/subsystems
     */
    public void setSlowMode(boolean status) {
        this.m_slowMode = status;
        m_slowModeStateShuffleboard.setBoolean(this.m_slowMode);
    }

    /**
     * Used to set brake or neutral mode for motors on drive subsystem
     * 
     * @param mode if true, enable brake mode
     */
    public void setBrakeMode(boolean mode) {
        m_swerveDrive.setBrakeMode(mode);
    }

    /**
     * Used to reset gyro heading and pose/odometry.  Useful at beginning of autononmous period,
     * particularly if the robot has been sitting powered on for some time as gyro may have drifted.
     *  
     * @param pose to reset to
     */
    public void resetHeadingAndOdometry(Pose2d pose) {
        m_swerveDrive.resetOdometryAndAngleOffset(pose, true);
    }

    public Command getFollowPathCommand(PathPlannerTrajectory trajectory) {
        return new PPSwerveControllerCommand(
            trajectory, 
            m_swerveDrive::getPose, // Pose supplier
            Constants.Swerve.kSwerveKinematics, // SwerveDriveKinematics
            new PIDController(Constants.AutoConstants.kPXController, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(Constants.AutoConstants.kPXController, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(Constants.AutoConstants.kPThetaController, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            m_swerveDrive::setModuleStates, // Module states consumer
            m_swerveDrive
        );
    }

    private Command createStateCommandNoArm(States state) {
        switch(state) {
            case Rest:
                    new IntakeUpDown(m_intakeArm, IntakeState.IntakeUp);

            case Grab:
                new IntakeUpDown(m_intakeArm, IntakeState.IntakeDown);

            case Score:
                new IntakeUpDown(m_intakeArm, IntakeState.IntakeUp);

            case Pickup:
                return new IntakeUpDown(m_intakeArm, IntakeState.IntakeDown);

            default:
            case Defense:
                return new IntakeUpDown(m_intakeArm, IntakeState.IntakeUp);
        }
    }

    private Command createStateCommand(States state) {
        switch(state) {
            case Rest:
                return new SequentialCommandGroup(
                    // Always retract pneumatics first.
                    new SequentialCommandGroup(
                        new RetractArm(m_pneumatics),
                        new WaitCommand(0.5)
                        ),

                        // If arm is not above danger level (e.g. Defense), then first extend Intake. 
                    new ConditionalCommand(
                        new EmptyCommand(),
                        new IntakeUpDown(m_intakeArm, IntakeState.IntakeDown),
                        ()-> m_armPivot.getArmZone().equals(ArmZone.AboveDanger)),
                    // Move the arm to the rest position.
                    new SetPivotPosition(m_armPivot, ArmPivotConstants.kArmRestSensorDegree),
                    // Finally, move the intake up.
                    new IntakeUpDown(m_intakeArm, IntakeState.IntakeUp));

            case Grab:
                return new SequentialCommandGroup(
                    // Always retract the arm first.
                    new RetractArm(m_pneumatics), 
                    new ConditionalCommand(
                        // If the arm is above the danger level, then ensure that the arm is at the rest position.
                        new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kArmRestSensorDegree),
                        new EmptyCommand(), 
                        ()-> m_armPivot.getArmZone().equals(ArmZone.AboveDanger)),
                    // extend the intake
                    new IntakeUpDown(m_intakeArm, IntakeState.IntakeDown),
                    // lower the arm to the grab position.
                    new SetPivotPosition(m_armPivot, ArmPivotConstants.kGrabItemSensorDegree)
                );

            case Score:
                return new SequentialCommandGroup(
                    // If the arm is below danger position, then extend the intake. and move the arm to the rest position.
                    new ConditionalCommand(
                        new EmptyCommand(), 
                        new IntakeUpDown(m_intakeArm, IntakeState.IntakeDown),
                        ()-> m_armPivot.getArmZone().equals(ArmZone.AboveDanger)),
                    // Move the arm to the score position.
                    new SetPivotPosition(m_armPivot, m_buttonBoardInputs)
                );

            case Pickup:
                return new SequentialCommandGroup(
                    // Always retract arm first.
                    new RetractArm(m_pneumatics),
                    new ConditionalCommand(
                        // If the arm is above danger level, then move the arm to the rest position, then lower the intake.
                        new SequentialCommandGroup(
                            new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kArmRestSensorDegree),
                            new IntakeUpDown(m_intakeArm, IntakeState.IntakeDown)
                        ), 
                        // Otherwise (e.g. defense position) lower the intake first, and then move the arm.
                        new SequentialCommandGroup(
                            new IntakeUpDown(m_intakeArm, IntakeState.IntakeDown),
                            new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kArmRestSensorDegree)
                        ),
                        ()-> m_armPivot.getArmZone().equals(ArmZone.AboveDanger))
                );

            default:
            case Defense:
                return new SequentialCommandGroup(
                    // Always retract the arm first.
                    new RetractArm(m_pneumatics),              
                    new ConditionalCommand(
                        // If the arm is above the danger level, then move the arm to the rest position, then lower the intake.
                        new SequentialCommandGroup(
                            new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kArmRestSensorDegree),
                            new IntakeUpDown(m_intakeArm, IntakeState.IntakeDown)),
                        new EmptyCommand(), 
                        ()-> m_armPivot.getArmZone().equals(ArmZone.AboveDanger)),
                        // Move the arm to the grab position
                        new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kGrabItemSensorDegree),
                        // Finally, retract the intake.
                        new IntakeUpDown(m_intakeArm, IntakeState.IntakeUp)
                );
        }
    }
}

/*
 * 
            System.err.println("No auton command to run!");
            return new WaitCommand(0.0);
        }

		switch (selected) {
            case _1_Score_P1:
                trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(TrajectoryLoader.getTrajectory(Trajectories._2_ScoreMoveCharge_P1), DriverStation.getAlliance());
                return new InstantCommand(() -> System.out.println("after claw open")

                    // new ParallelCommandGroup(
                    //     new IntakeUpDown(m_intakeArm, IntakeState.IntakeDown),
                    //     new ClawCommand(m_claw, ClawState.CLOSE)
                    // ),
                    // new InstantCommand(() -> System.out.println("after parallelcommandgroup1")),
                    // // move arm to position
                    // new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kArmRestSensorDegree),
                    // new InstantCommand(() -> System.out.println("after armrest")),
                    // new ParallelCommandGroup(
                    //     new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kHighShelfLevelSensorDegree)
                    // ),
                    // new InstantCommand(() -> System.out.println("before extend arm")),
                    // new ExtendArm(m_pneumatics),

                    // new InstantCommand(() -> System.out.println("after extend arm")),
                    // new WaitCommand(1),

                    // // drop the piece
                    // new ClawCommand(m_claw, ClawState.OPEN),
                    // new InstantCommand(() -> System.out.println("after claw open")),
                    // new RetractArm(m_pneumatics),

                    // new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kArmRestSensorDegree),
                    // new InstantCommand(() -> System.out.println("after return to rest"))
                );
            case _1_ScoreMovePickup_P1:
                trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(TrajectoryLoader.getTrajectory(selected), DriverStation.getAlliance());
                return new SequentialCommandGroup(
                    new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                    trajectory.getInitialPose(),
                    true)),

                    // new ParallelCommandGroup(
                    //     new IntakeUpDown(m_intakeArm, IntakeState.IntakeDown),
                    //     new ClawCommand(m_claw, ClawState.CLOSE)
                    // ),
                    // new InstantCommand(() -> System.out.println("after parallelcommandgroup1")),
                    // // move arm to position
                    // new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kArmRestSensorDegree),
                    // new InstantCommand(() -> System.out.println("after armrest")),
                    // new ParallelCommandGroup(
                    //     new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kHighShelfLevelSensorDegree)
                    // ),
                    // new InstantCommand(() -> System.out.println("before extend arm")),
                    // new ExtendArm(m_pneumatics),

                    // new InstantCommand(() -> System.out.println("after extend arm")),
                    // new WaitCommand(1),

                    // // drop the piece
                    // new ClawCommand(m_claw, ClawState.OPEN),
                    // new InstantCommand(() -> System.out.println("after claw open")),
                    // new RetractArm(m_pneumatics),

                    // new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kArmRestSensorDegree),
                    // new InstantCommand(() -> System.out.println("after return to rest")),
                    // new ParallelCommandGroup(
                        this.getFollowPathCommand(trajectory)//,
                        //new IntakeSpin(m_intakeSpinner, true, m_buttonBoardInputs))
                );
            case _2_ScoreMoveCharge_P1:
                trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(TrajectoryLoader.getTrajectory(selected), DriverStation.getAlliance());
                return new SequentialCommandGroup(
                    new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                        trajectory.getInitialPose(),
                        true)),

                    // new ParallelCommandGroup(
                    //     new IntakeUpDown(m_intakeArm, IntakeState.IntakeDown),
                    //     new ClawCommand(m_claw, ClawState.CLOSE)
                    // ),
                    // new InstantCommand(() -> System.out.println("after parallelcommandgroup1")),
                    // // move arm to position
                    // new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kArmRestSensorDegree),
                    // new InstantCommand(() -> System.out.println("after armrest")),
                    // new ParallelCommandGroup(
                    //     new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kHighShelfLevelSensorDegree)
                    // ),
                    // new InstantCommand(() -> System.out.println("before extend arm")),
                    // new ExtendArm(m_pneumatics),
    
                    // new InstantCommand(() -> System.out.println("after extend arm")),
                    // new WaitCommand(1),
    
                    // // drop the piece
                    // new ClawCommand(m_claw, ClawState.OPEN),
                    // new InstantCommand(() -> System.out.println("after claw open")),

                    // new ParallelCommandGroup(
                    //     new RetractArm(m_pneumatics),
                    //     new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kArmRestSensorDegree)
                    // ),
                    // new InstantCommand(() -> System.out.println("after return to rest")),

                    // new ParallelDeadlineGroup(
                        this.getFollowPathCommand(trajectory),
                        //new IntakeSpin(m_intakeSpinner, true, m_buttonBoardInputs)
                    //),
                
                    new AutoBalance(m_swerveDrive, m_gyro)
                );
            case _3_ScoreMovePickup_P1:
                trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(TrajectoryLoader.getTrajectory(selected), DriverStation.getAlliance());
                return new SequentialCommandGroup(
                    new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                        trajectory.getInitialPose(),
                        true)),

                    new ParallelCommandGroup(
                        new IntakeUpDown(m_intakeArm, IntakeState.IntakeDown),
                        new ClawCommand(m_claw, ClawState.CLOSE)
                    ),
                    new InstantCommand(() -> System.out.println("after parallelcommandgroup1")),
                    // move arm to position
                    new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kArmRestSensorDegree),
                    new InstantCommand(() -> System.out.println("after armrest")),
                    new ParallelCommandGroup(
                        new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kHighShelfLevelSensorDegree)
                    ),
                    new InstantCommand(() -> System.out.println("before extend arm")),
                    new ExtendArm(m_pneumatics),
    
                    new InstantCommand(() -> System.out.println("after extend arm")),
                    new WaitCommand(1),
    
                    // drop the piece
                    new ClawCommand(m_claw, ClawState.OPEN),
                    new InstantCommand(() -> System.out.println("after claw open")),

                    new ParallelCommandGroup(
                        new RetractArm(m_pneumatics),
                        new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kArmRestSensorDegree)
                    ),
                    new InstantCommand(() -> System.out.println("after return to rest")),
                    new ParallelCommandGroup(
                        this.getFollowPathCommand(trajectory),
                        new IntakeSpin(m_intakeSpinner, true, m_buttonBoardInputs))
                );

            //new added autonomous
            case _1_ScoreMoveCharge_P1: 
                trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(TrajectoryLoader.getTrajectory(selected), DriverStation.getAlliance());
                return new SequentialCommandGroup(
                    new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                        trajectory.getInitialPose(),
                        true)),

                    new ParallelCommandGroup(
                        new IntakeUpDown(m_intakeArm, IntakeState.IntakeDown),
                        new ClawCommand(m_claw, ClawState.CLOSE)
                    ),
                    new InstantCommand(() -> System.out.println("after parallelcommandgroup1")),
                    // move arm to position
                    new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kArmRestSensorDegree),
                    new InstantCommand(() -> System.out.println("after armrest")),
                    new ParallelCommandGroup(
                        new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kHighShelfLevelSensorDegree)
                    ),
                    new InstantCommand(() -> System.out.println("before extend arm")),
                    new ExtendArm(m_pneumatics),
    
                    new InstantCommand(() -> System.out.println("after extend arm")),
                    new WaitCommand(1),
    
                    // drop the piece
                    new ClawCommand(m_claw, ClawState.OPEN),
                    new InstantCommand(() -> System.out.println("after claw open")),

                    new ParallelCommandGroup(
                        new RetractArm(m_pneumatics),
                        new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kArmRestSensorDegree)
                    ),
                    new InstantCommand(() -> System.out.println("after return to rest")),
                    new ParallelDeadlineGroup(
                        this.getFollowPathCommand(trajectory),
                        new IntakeSpin(m_intakeSpinner, true, m_buttonBoardInputs)
                    ),
            
                    new AutoBalance(m_swerveDrive, m_gyro)
                );

            case _3_ScoreMoveCharge_P1:
                trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(TrajectoryLoader.getTrajectory(selected), DriverStation.getAlliance());
                return new SequentialCommandGroup(
                    new InstantCommand(() -> m_swerveDrive.resetOdometryAndAngleOffset(
                        trajectory.getInitialPose(),
                        true)),

                    new ParallelCommandGroup(
                        new IntakeUpDown(m_intakeArm, IntakeState.IntakeDown),
                        new ClawCommand(m_claw, ClawState.CLOSE)
                    ),
                    new InstantCommand(() -> System.out.println("after parallelcommandgroup1")),
                    // move arm to position
                    new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kArmRestSensorDegree),
                    new InstantCommand(() -> System.out.println("after armrest")),
                    new ParallelCommandGroup(
                        new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kHighShelfLevelSensorDegree)
                    ),
                    new InstantCommand(() -> System.out.println("before extend arm")),
                    new ExtendArm(m_pneumatics),
    
                    new InstantCommand(() -> System.out.println("after extend arm")),
                    new WaitCommand(1),
    
                    // drop the piece
                    new ClawCommand(m_claw, ClawState.OPEN),
                    new InstantCommand(() -> System.out.println("after claw open")),

                    new ParallelCommandGroup(
                        new RetractArm(m_pneumatics),
                        new SetPivotPosition(m_armPivot, Constants.ArmPivotConstants.kArmRestSensorDegree)
                    ),
                    new InstantCommand(() -> System.out.println("after return to rest")),
                    new ParallelDeadlineGroup(
                        this.getFollowPathCommand(trajectory),
                        new IntakeSpin(m_intakeSpinner, true, m_buttonBoardInputs)),
    
                
                    //auto balance
                    new AutoBalance(m_swerveDrive, m_gyro)
                );

            default:
                // nothing to do -- shouldn't ever happen
                System.err.println("No auton command to run!");
                return new WaitCommand(0.0);
        }

    }

 */