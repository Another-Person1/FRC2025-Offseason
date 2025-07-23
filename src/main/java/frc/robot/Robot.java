// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.TargetCorner;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.List;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  private XboxController controller = new XboxController(0);

  private PhotonCamera camera = new PhotonCamera("center");

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
  "swerve/kraken-navarch"));


  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }
  }

 /**
 * This function is called periodically during operator control.
 */
@Override
public void teleopPeriodic() {
    // Calculate drivetrain commands from Joystick values
    double forward = -controller.getLeftY() * Constants.DrivebaseConstants.kMaxLinearSpeed;
    double strafe = -controller.getLeftX() * Constants.DrivebaseConstants.kMaxLinearSpeed;
    double turn = -controller.getRightX() * Constants.DrivebaseConstants.kMaxAngularSpeed;

    // Read in relevant data from the Camera
    boolean targetVisible = false;
    Transform3d cameraToTarget = null;

    // Get all the results from the camera, which may include multiple frames of data.
    var results = camera.getAllUnreadResults();

    if (!results.isEmpty()) {
        // Camera processed a new frame since last
        var result = results.get(results.size() - 1);

        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
                if (target.getFiducialId() == 7) {
                    // Found Tag 7, record its information
                    cameraToTarget = target.getBestCameraToTarget();
                    targetVisible = true;
                    SmartDashboard.putNumber("AprilTag ID", target.getFiducialId());
                    break; // Exit loop after finding tag 7
                }
            }
        } else {
            SmartDashboard.putNumber("AprilTag ID", 0);
        }
    } else {
        SmartDashboard.putNumber("AprilTag ID", 0);
    }

    // Auto-align when requested
    if (controller.getAButton() && targetVisible && cameraToTarget != null) {
        System.out.println("Auto aligning to AprilTag ID 7...");

        // Get the yaw (rotation around Z-axis) of the camera-to-target transform
        double yaw = cameraToTarget.getRotation().getZ(); // In radians

        // Apply a proportional controller to turn toward the target
        double turnError = -yaw; // Negative to correct toward target
        turn = turnError * Constants.VisionConstants.VISION_TURN_kP * Constants.DrivebaseConstants.kMaxAngularSpeed;

        // Clamp the turn speed to avoid overshooting
        turn = Math.max(-Constants.DrivebaseConstants.kMaxAngularSpeed, 
                        Math.min(turn, Constants.DrivebaseConstants.kMaxAngularSpeed));

        // Optional: Add camera-to-robot offset if camera is not at robot's center
        // Transform3d robotToCamera = new Transform3d(...); // Define camera offset
        // Transform3d robotToTarget = cameraToTarget.plus(robotToCamera.inverse());
        // Use robotToTarget.getRotation().getZ() for yaw if offset is applied
    }

    // Command drivetrain motors based on target speeds
    var chassisSpeeds = new ChassisSpeeds(forward, strafe, turn);
    drivebase.drive(chassisSpeeds);

    // Put debug information to the dashboard
    SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
    if (cameraToTarget != null) {
        SmartDashboard.putNumber("Target Yaw (deg)", Math.toDegrees(cameraToTarget.getRotation().getZ()));
    }
}

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
}