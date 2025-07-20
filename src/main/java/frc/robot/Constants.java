// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kMaxLinearSpeed = 3; // meters per second
  }
  public static class VisionConstants {

    public static final double VISION_TURN_kP = 0.02; // Example value, adjust as needed

  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.05;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }


    public static class RampConstants {
        public static final int RampID = 43;
        public static final int StarWheelID = 44;
    }

    public static final class TagCoods {
        public TagCoods(double midX, double midY, double rightX, double rightY, double leftX, double leftY, Rotation2d angle) {
            LeftX = leftX;
            LeftY = leftY;
            RightX = rightX;
            RightY = rightY;
            BotAngle = angle;
            MidX = midX;
            MidY = midY;
        }
        public double LeftX = -1;
        public double LeftY = -1;
        public double RightX = -1;
        public double RightY = -1;
        public Rotation2d BotAngle;
        public double MidX = -1;
        public double MidY = -1;
    }

    public static final class AutoMoveConstants {
        public static final double a = 0.13; // 0.2 // 0.305 0.185
        public static final double b = 0.165; //0.17
        public static final double c = a /2;
        public static final double d = Math.sqrt(3) * (a/2);
        public static final double e = b / 2;
        public static final double f = Math.sqrt(3) * (b/2);
    }

    public static class ElevatorConstants {
        public static final int LIMIT_SWITCH_ID = 9;
        public static final int LEFT_MOTOR_ID = 41;
        public static final int RIGHT_MOTOR_ID = 42;
        public static final double SENSOR_TO_MECHANISM_RATIO = 3d/11d; // Units will be in inches
        public static final double MAX_HEIGHT = 60; // 60 inches

        // PID Constants
        public static final double kP = 0.7; // 0.675
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kIz = 0.0;
        public static final double kFF = 0.09;
        public static final double kG = 0;
        public static final double kA = 0.002;

        // Motion Magic Constants
        public static final double MAX_VELOCITY = 800; // 10 inches per second
        public static final double MAX_ACCELERATION = 2000; // 20 inches per second squared
        public static final double MAX_JERK = 4000; // 60 inches per second cubed
        public static final double ALLOWABLE_ERROR = 0.5; // 0.5 inches

        public static final int CURRENT_LIMIT = 120;
        public static enum ElevatorPosition {
            GROUND(0, -0.45),
            LOW(13.3, -0.12), // 11.41455078125, 7.984863
            MID(29.056641, -0.12), // 27.5 
            HIGH(55, -0.45), // 52
            LOWALG(27.1, -0.45), // 23.7724609375
            HIGHALG(42.4, -0.45); // 39.609130859375
            // INTAKE(1.7197265625); old elev pos for intake at centennial, unused with new mechanical changes

            private final double height;
            private final double speed;

            ElevatorPosition(double height, double speed) {
                this.height = height;
                this.speed = speed;
            }

            public double getHeight() {
                return height;
            }

            public double getSpeed() {
                return speed;
            }
        }
    }

    public static class EndEffectorConstants {
        public static final int PIVOT_ID = 23;
        public static final int CONVEYOR_ID = 20;
        public static final double SENSOR_TO_MECHANISM_RATIO = 14.0625;
        public static final double MAX_ANGLE = 0;
        public static final double MIN_ANGLE = -246;

        // PID Constants
        public static final double kP = 0.0285;//0.032;
        public static final double kI = 0;
        public static final double kD = 0.015;
        public static final double kIz = 0.0;
        public static final double kFF = 0;

        public static final double kP_CONVEYOR = 0.2;
        public static final double kI_CONVEYOR = 0.0;
        public static final double kD_CONVEYOR = 0.0;

        // MAX motion constants
        public static final double MAX_VELOCITY = 30000;
        public static final double MAX_ACCELERATION = MAX_VELOCITY*2;
        public static final double MAX_JERK = MAX_ACCELERATION*2;
        public static final double ALLOWABLE_ERROR = 1;//1
        // Old values (starting from floor)
        public static enum PivotPosition {
            UP(-245),
            DOWN(-190),
            L4DOWN(-180),
            DUNK(-122),
            ALG(-128.57),
            GROUNDALG(-117.5),
            PARALLELGROUND(-60);

            private final double angle;

            PivotPosition(double angle) {
                this.angle = angle;
            }

            public double getAngle() {
                return angle;
            }
        }

        // public static enum PivotPosition {
        //     UP(30),
        //     DOWN(55),
        //     DUNK(123),
        //     ALG(116.43);

        //     private final double angle;

        //     PivotPosition(double angle) {
        //         this.angle = angle;
        //     }

        //     public double getAngle() {
        //         return angle;
        //     }
        // }

        // Current Limiting Constants
        public static final int PIVOT_CURRENT_LIMIT = 120; //60;
        public static final int CONVEYOR_CURRENT_LIMIT = 60;


    }

    public static class TuskConstants {
        public static final int PIVOT_ID = 24;
        public static final double SENSOR_TO_MECHANISM_RATIO = 25;
        public static final double MAX_ANGLE = 0;
        public static final double MIN_ANGLE = -220;

        public static final double kP = 0.01;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kIz = 0.0;
        public static final double kFF = 0.0;

        public static final double MAX_VELOCITY = 20000;
        public static final double MAX_ACCELERATION = MAX_VELOCITY*1.5;
        public static final double MAX_JERK = MAX_ACCELERATION*3;
        public static final double ALLOWABLE_ERROR = 0.5;
        public static enum PivotPosition {
            UP(0),
            DOWN(126.78);

            private final double angle;

            PivotPosition(double angle) {
                this.angle = angle;
            }

            public double getAngle() {
                return angle;
            }
        }

        // Current Limiting Constants
        public static final int PIVOT_CURRENT_LIMIT = 60;
        public static final int CONVEYOR_CURRENT_LIMIT = 40;
    }
}

