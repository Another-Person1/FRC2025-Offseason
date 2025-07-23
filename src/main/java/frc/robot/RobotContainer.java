// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

//import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.subsystems.RampSubsystem;
import frc.robot.subsystems.TuskSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/kraken-navarch"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driveController.getLeftY() * -1,
                                                                () -> driveController.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> -driveController.getRightX())
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(false);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driveController::getRightX,
                                                                                             driveController::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driveController.getLeftY(),
                                                                        () -> -driveController.getLeftX())
                                                                    .withControllerRotationAxis(() -> driveController.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driveController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driveController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
  private final RampSubsystem rampSubsystem = RampSubsystem.getInstance();
  private final EndEffectorSubsystem endEffector = EndEffectorSubsystem.getInstance();
  private final TuskSubsystem tuskSubsystem = TuskSubsystem.getInstance();
  private final LightingSubsystem lightingSubsystem = LightingSubsystem.getInstance();

  private static final double CONVEYOR_INTAKE_SPEED = 0.1;
  private static double CONVEYOR_EJECT_SPEED = -0.2;
  private static final double RAMP_SPEED = 0.35;
  private static boolean speedCutOff = false;

  private SendableChooser<Command> autoChooser;

  
  public RobotContainer()
  { 
    registerAutos();
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }


  private void registerAutos() {
        registerNamedCommands();

        autoChooser = new SendableChooser<>();

        autoChooser.setDefaultOption("Middle Barge to Reef G", drivebase.getAutonomousCommand("Middle Barge to Reef G"));
        // autoChooser.addOption("Testy", drivebase.getAutonomousCommand("Testy", true));
        // autoChooser.addOption("Curvy", drivebase.getAutonomousCommand("Curvy", true));
        // autoChooser.addOption("Start 1 to F to A Three Coral", drivebase.getAutonomousCommand("Start 1 to F to A Three Coral", true));
        // autoChooser.addOption("Start 1 to F Three Coral", drivebase.getAutonomousCommand("Start 1 to F Three Coral", true));
        // autoChooser.addOption("Start 1 to F Two Coral", drivebase.getAutonomousCommand("Start 1 to F Two Coral", true));
        // autoChooser.addOption("Start 1 to F One Coral", drivebase.getAutonomousCommand("Start 1 to F One Coral", true));
        // autoChooser.addOption("Start 1 to E to F Three Coral", drivebase.getAutonomousCommand("Start 1 to E to F Three Coral", true));
        // autoChooser.addOption("Start 1 to E to F Two Coral", drivebase.getAutonomousCommand("Start 1 to E to F Two Coral", true));
        // autoChooser.addOption("Start 1 to E One Coral", drivebase.getAutonomousCommand("Start 1 to E One Coral", true));
        
        //All of section 3 autos
        // autoChooser.addOption("3-c", drivebase.getAutonomousCommand("3-c", true));
        autoChooser.addOption("ProcessorSide 3 Coral -- 3-c-y-b-y-b", drivebase.getAutonomousCommand("ProcessorSide 3 Coral -- 3-c-y-b-y-b"));
        autoChooser.addOption("BargeSide 3 Coral -- 3-e-x-f-x-f", drivebase.getAutonomousCommand("BargeSide 3 Coral -- 3-e-x-f-x-f"));
        autoChooser.addOption("ProcessorSide 2 coral -- 3-c-y-b", drivebase.getAutonomousCommand("ProcessorSide 2 coral -- 3-c-y-b"));
        autoChooser.addOption("BargeSide 2 coral -- 1-e-x-f", drivebase.getAutonomousCommand("BargeSide 2 coral -- 1-e-x-f"));
        autoChooser.addOption("3-c-y-b-y-b", drivebase.getAutonomousCommand("3-c-y-b-y-b"));
        autoChooser.addOption("1-e-x-f-x-f", drivebase.getAutonomousCommand("1-e-x-f-x-f"));
        autoChooser.addOption("Movement 1-e-x-f-x-f", drivebase.getAutonomousCommand("Movement 1-e-x-f-x-f"));
        // autoChooser.addOption("3-c-y-b-y-b", drivebase.getAutonomousCommand("3-c-y-b-y-b", true));
        // autoChooser.addOption("3-b", drivebase.getAutonomousCommand("3-b", true));
        // autoChooser.addOption("3-b-y-b", drivebase.getAutonomousCommand("3-b-y-b", true));
        // autoChooser.addOption("3-b-y-b-y-b", drivebase.getAutonomousCommand("3-b-y-b-y-b", true));
        // autoChooser.addOption("3-b-y-b-y-a", drivebase.getAutonomousCommand("3-b-y-b-y-a", true));
        
        autoChooser.addOption("Shoot Ests!", drivebase.getAutonomousCommand("Shoot Ests!"));
        autoChooser.addOption("2-d One Coral", drivebase.getAutonomousCommand("2-d One Coral"));
        autoChooser.addOption("testStationPath", drivebase.getAutonomousCommand("test-pathing"));
        autoChooser.addOption("Nothing", drivebase.getAutonomousCommand("Nothing"));
        autoChooser.addOption("ProcessorSide 4 Coral", drivebase.getAutonomousCommand("ProcessorSide 4 Coral"));
        //autoChooser.addOption("2-d-auto", drivebase.getAutonomousCommand("2-d-auto", false));
        SmartDashboard.putData("AutoPaths", autoChooser);
    }

    private void registerNamedCommands() {

        // Example
        NamedCommands.registerCommand("elevatorGround", new ElevatorDownAuto(elevator, endEffector, tuskSubsystem, CONVEYOR_EJECT_SPEED));
        NamedCommands.registerCommand("elevatorGroundFromIntake", new ElevatorDownFromIntake(elevator, endEffector, tuskSubsystem, CONVEYOR_EJECT_SPEED));
        NamedCommands.registerCommand("elevatorLow", new L2Command(elevator, endEffector, CONVEYOR_EJECT_SPEED));
        NamedCommands.registerCommand("elevatorMid", new L3Command(elevator, endEffector, CONVEYOR_EJECT_SPEED));
        NamedCommands.registerCommand("elevatorMidNoShoot", new L3Command(elevator, endEffector, 0));
        NamedCommands.registerCommand("elevatorHigh", new L4Command(elevator, endEffector, CONVEYOR_EJECT_SPEED-0.15));
        NamedCommands.registerCommand("elevatorHighNoShoot", new L4Command(elevator, endEffector, 0));
        NamedCommands.registerCommand("elevatorHighDunk", new L4CommandDunk(elevator, endEffector, CONVEYOR_EJECT_SPEED-0.15));
        NamedCommands.registerCommand("elevatorLowAlgae", new AlgaeLow(elevator, endEffector, tuskSubsystem, CONVEYOR_EJECT_SPEED));
        NamedCommands.registerCommand("elevatorHighAlgae", new AlgaeHigh(elevator, endEffector, tuskSubsystem, CONVEYOR_EJECT_SPEED));
        NamedCommands.registerCommand("pivotUp", endEffector.setPivotPositionCommand(Constants.EndEffectorConstants.PivotPosition.UP));
        NamedCommands.registerCommand("pivotDown", endEffector.setPivotPositionCommand(Constants.EndEffectorConstants.PivotPosition.DOWN));
        NamedCommands.registerCommand("pivotDunk", new InstantCommand(() -> endEffector.setPivotPositionCommand(Constants.EndEffectorConstants.PivotPosition.DUNK)));
        NamedCommands.registerCommand("pivotAlgae", new InstantCommand(() -> endEffector.setPivotPositionCommand(Constants.EndEffectorConstants.PivotPosition.ALG)));
        NamedCommands.registerCommand("conveyorIntake", new InstantCommand(() -> endEffector.setConveyorSpeedCommand(CONVEYOR_INTAKE_SPEED)));
        NamedCommands.registerCommand("conveyorEject", endEffector.setConveyorSpeedCommand(CONVEYOR_EJECT_SPEED-0.2).withTimeout(0.17));
        NamedCommands.registerCommand("conveyorEjectNoTimeout", endEffector.setConveyorSpeedCommand(CONVEYOR_EJECT_SPEED-0.2).withTimeout(2));
        NamedCommands.registerCommand("conveyorStop", new InstantCommand(() -> endEffector.stopConveyorCommand()));
        NamedCommands.registerCommand("tuskUp", new InstantCommand(() -> tuskSubsystem.setPivotPositionCommand(Constants.TuskConstants.PivotPosition.UP)));
        NamedCommands.registerCommand("tuskDown", new InstantCommand(() -> tuskSubsystem.setPivotPositionCommand(Constants.TuskConstants.PivotPosition.DOWN)));
        NamedCommands.registerCommand("rampIntake", new InstantCommand(() -> rampSubsystem.run(RAMP_SPEED)));
        NamedCommands.registerCommand("rampReverse", new InstantCommand(() -> rampSubsystem.run(-RAMP_SPEED)));
        NamedCommands.registerCommand("rampStop", new InstantCommand(() -> rampSubsystem.run(0)));
        NamedCommands.registerCommand("intake", new AutoIntake(rampSubsystem, endEffector, elevator, CONVEYOR_EJECT_SPEED, RAMP_SPEED).withTimeout(2));
        NamedCommands.registerCommand("stopIntake", new InstantCommand(() -> rampSubsystem.run(0)).alongWith(new InstantCommand(() -> endEffector.stopConveyorCommand())).raceWith(new WaitCommand(0.01)));
        // NamedCommands.registerCommand("StopLimelight", drivebase.LimelightStatus(false));
        // NamedCommands.registerCommand("StartLimelight", drivebase.LimelightStatus(true));

        // NamedCommands.registerCommand("AutoAlignLeft", new InstantCommand(() -> {
        //     if(LimelightHelpers.getTV("") == true) {
        //         var cmd = AutoBuilder.followPath(drivebase.GoLeft(1));
        //         cmd.schedule();}   
        //     }            
        // ));
        // NamedCommands.registerCommand("AutoAlignRight", new InstantCommand(() -> {
        //     if(LimelightHelpers.getTV("") == true) {
        //           var cmd = AutoBuilder.followPath(drivebase.GoRight(1));
        //           cmd.schedule();}
        //     }
        // ));7

    }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // driveFieldOrientedAnglularVelocity
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driveController.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driveController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driveController.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

//      driveController.b().whileTrue(
//          drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driveController.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driveController.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driveController.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driveController.back().whileTrue(drivebase.centerModulesCommand());
      driveController.leftBumper().onTrue(Commands.none());
      driveController.rightBumper().onTrue(Commands.none());
    } else
    {
      driveController.y().onTrue(new InstantCommand(() -> speedCutOff = !speedCutOff));
      driveController.leftBumper().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driveController.start().whileTrue(Commands.none());
      driveController.back().whileTrue(Commands.none());
      driveController.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driveController.rightBumper().onTrue(Commands.none());
      driveController.b().whileTrue(endEffector.setConveyorSpeedCommand(-0.12));
      //operatorController.a().onTrue(elevator.moveToPosition(ElevatorPosition.GROUND));
      operatorController.y().onTrue(new L3Command(elevator, endEffector, 0));
      operatorController.x().onTrue(new L2Command(elevator, endEffector, 0));
      operatorController.b().onTrue(new L4Command(elevator, endEffector, 0));
      // Algae - press joystick inwards
      operatorController.button(9).onTrue(new AlgaeLow(elevator, endEffector, tuskSubsystem, 0));
      operatorController.button(10).onTrue(new AlgaeHigh(elevator, endEffector, tuskSubsystem, 0));

      // EndEffector Pivot Controls
      operatorController.povUp().onTrue(endEffector.setPivotPositionCommand(Constants.EndEffectorConstants.PivotPosition.UP));
      // operatorController.povUp().onTrue(elevator.moveToPosition(ElevatorPosition.INTAKE));
      operatorController.povDown().onTrue(endEffector.setPivotPositionCommand(Constants.EndEffectorConstants.PivotPosition.DOWN));
      //operatorController.povRight().onTrue(endEffector.setPivotPositionCommand(PivotPosition.DUNK));
      driveController.start().onTrue(elevator.resetEncoderCommand());
      operatorController.povLeft().onTrue(endEffector.setPivotPositionCommand(Constants.EndEffectorConstants.PivotPosition.ALG));
      operatorController.povRight().onTrue(endEffector.setPivotPositionCommand(Constants.EndEffectorConstants.PivotPosition.GROUNDALG));
      // operatorController.povRight().onTrue(new InstantCommand(() -> endEffector.test()));
              
      // Conveyor Controls (using triggers)
      operatorController.rightTrigger().whileTrue(endEffector.setConveyorSpeedCommand(CONVEYOR_INTAKE_SPEED+0.3));
      operatorController.leftTrigger().whileTrue(endEffector.setConveyorSpeedCommand(elevator::getElevatorPosition));
      

      // operatorController.rightBumper().whileTrue(endEffector.run(0.05));
      // operatorController.leftBumper().whileTrue(endEffector.run(-0.1));
      //operatorController.povRight().onTrue(endEffector.resetEncoderCommand());
      operatorController.back().onTrue(elevator.resetEncoderCommand());

      // Ramp Controls (using bumpers)
      operatorController.rightBumper().whileTrue(endEffector.setConveyorSpeedCommand(CONVEYOR_EJECT_SPEED).until(() -> endEffector.StopWithSensor()));
      operatorController.rightBumper().whileTrue(rampSubsystem.run(RAMP_SPEED)); // .until(() -> endEffector.StopWithSensor())
      operatorController.leftBumper().whileTrue(rampSubsystem.runSeparately(-RAMP_SPEED/1.5, -0.2));
      driveController.leftTrigger().whileTrue(rampSubsystem.runSeparately(0, 1));
      driveController.rightTrigger().whileTrue(rampSubsystem.runSeparately(0, -1));


      endEffector.setDefaultCommand(endEffector.stopConveyorCommand());
        // endEffector.setDefaultCommand(
        //     new ManualPivot(
        //         endEffector, 
        //         () -> -operatorController.getLeftY()
        //     )
        // );
      rampSubsystem.setDefaultCommand(rampSubsystem.run(0));
      // Manual elevator control
      elevator.setDefaultCommand(
        new ManualElevator(elevator, () -> -operatorController.getRightY())
      );

      lightingSubsystem.setDefaultCommand(
          new Lighting(
              lightingSubsystem, 
              () -> endEffector.AnalogOutput()
          )
      );


    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
