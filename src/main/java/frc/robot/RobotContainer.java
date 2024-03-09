// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Arm arm = new Arm();
  private final Climber climber = new Climber();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandGenericHID opStick =
      new CommandGenericHID(OperatorConstants.opStickPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -Math.pow(MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.kDriveDeadband), 3),
                -Math.pow(MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.kDriveDeadband), 3),
                -Math.pow(MathUtil.applyDeadband(m_driverController.getRightX(), OperatorConstants.kDriveDeadband), 3),
                true, true),
            m_robotDrive));
            
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driverController.rightBumper().whileTrue(m_robotDrive.run(m_robotDrive :: setX));
        

    m_driverController.y().onTrue(m_robotDrive.runOnce((m_robotDrive :: zeroHeading)));
        
      opStick.axisGreaterThan(OperatorConstants.intakeAxis, OperatorConstants.intakeTheshhold).and(new Trigger (shooter::sensorOff)) .onTrue(shooter.holdCommand(ShooterConstants.holdFwd)
          .andThen(intake.runIf(.3, () -> {
            var a = arm.atBottom();
            //SmartDashboard.putBoolean("Arm test", a);
            return a;})))
        .or(
      opStick.axisLessThan(OperatorConstants.intakeAxis, -OperatorConstants.intakeTheshhold) .onTrue(shooter.holdCommand(ShooterConstants.holdRvs)
          .andThen(intake.runIf(-.3, () -> {
            var a = arm.atBottom();
            //SmartDashboard.putBoolean("Arm test", a);
            return a;})))
        )
                                            .onFalse(shooter.holdCommand(0) // possible bug !!! Line 85 may counteract it
                                            .andThen(intake.runcommand(0)));
    
    /* y button: shooter shoot */
      opStick.button(1)
                  .onTrue(shooter.shootCommand(1)
                      .andThen( new WaitUntilCommand(shooter::shootFastEnough), 
                                shooter.holdCommand(ShooterConstants.holdFwd))) // possible bug !!! Line 79 may counteract it
                  .onFalse(shooter.shootCommand(0)
                      .andThen(shooter.holdCommand(0)));
      opStick.button(5)  .onTrue(shooter.shootCommand(1))
                                .onFalse(shooter.shootCommand(0));
      opStick.button(6)  .onTrue(shooter.holdCommand(ShooterConstants.holdFwd))
                                .onFalse(shooter.holdCommand(0));
      /* a button: arm up */
      opStick.button(2) .onTrue(arm.upCmd(true));
      opStick.button(7) .onTrue(arm.upCmd(false));

      /* x button: release climber */
      opStick.button(3) .and(opStick.button(4).negate()) .onTrue(climber.topOrBottomCommand(true))
        .or (
      /* b button: retract climber and stop */
      opStick.button(4) .and(opStick.button(3).negate()) .onTrue(climber.topOrBottomCommand(false)) 
        )
        .or (
      opStick.button(3) .and(opStick.button(4)) .onTrue(climber.windDownCommand())
        )
                          .onFalse(climber.stayPutCommand());

      new Trigger(() -> climber.currentHigh(true))
                  .onTrue(climber.runOnce(() -> climber.stayPut1(true)));
      new Trigger(() -> climber.currentHigh(false))
                  .onTrue(climber.runOnce(() -> climber.stayPut1(false)));

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}
