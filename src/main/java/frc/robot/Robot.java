// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.ser.impl.FailingSerializer;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.opencv.calib3d.StereoBM;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  XboxController driverController = new XboxController(0);
  // AHRS gyro = new AHRS(SerialPort.Port.kMXP);
  CANSparkMax flDrive = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax flSteer = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax frDrive = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax frSteer = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax blDrive = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax blSteer = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax brDrive = new CANSparkMax(7, MotorType.kBrushless);
  CANSparkMax brSteer = new CANSparkMax(8, MotorType.kBrushless);
  DutyCycleEncoder blEncoder = new DutyCycleEncoder(0);
  DutyCycleEncoder flEncoder = new DutyCycleEncoder(1);
  DutyCycleEncoder frEncoder = new DutyCycleEncoder(2);
  DutyCycleEncoder brEncoder = new DutyCycleEncoder(3);
  SpeedControllerGroup driveMotors = new SpeedControllerGroup(flDrive, frDrive, blDrive, brDrive);
  SpeedControllerGroup steerMotors = new SpeedControllerGroup(flSteer, frSteer, blSteer, brSteer);
  double deadzone = 0.1;
  double zeroTolerance = 0.3;
  double steerGearReduction = 1000 / 8.16;
  double driveGearReduction = 1000 / 12.8;
  double x = 0;
  double rot = 0;
  double y = 0;
  double w1ca = 0;
  double w2ca = 0;
  double w3ca = 0;
  double w4ca = 0;
  double w1Angle;

  double counter = 0;

  double length = 0.47; // meters module center point to module center point
  double width = 0.47; // meters module center point to module center point
  double chassisRadius = Math.sqrt(Math.pow(length, 2) + Math.pow(width, 2)) / 2;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    flDrive.setInverted(false);
    flSteer.setInverted(false);
    frDrive.setInverted(false);
    frSteer.setInverted(false);
    blDrive.setInverted(false);
    blSteer.setInverted(false);
    brDrive.setInverted(false);
    brSteer.setInverted(false);
    brSteer.setIdleMode(CANSparkMax.IdleMode.kBrake);
    blSteer.setIdleMode(CANSparkMax.IdleMode.kBrake);
    frSteer.setIdleMode(CANSparkMax.IdleMode.kBrake);
    flSteer.setIdleMode(CANSparkMax.IdleMode.kBrake);
    counter = 0;

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // TODO get angle fron gyro
    if (Math.abs(driverController.getX(Hand.kLeft)) < deadzone) {
      x = 0;
    } else {
      x = driverController.getX(Hand.kLeft);
    }

    if (Math.abs(driverController.getY(Hand.kLeft)) < deadzone) {
      y = 0;
    } else {
      y = -driverController.getY(Hand.kLeft);
    }

    if (Math.abs(driverController.getX(Hand.kRight)) < deadzone) {
      rot = 0;
    } else {
      rot = driverController.getX(Hand.kRight);
    }

    double vx = x;// desired x speed
    double vy = y;// desired y speed
    double omega = rot;// desired rotation, clockwise pos in deg Check?

    double A = vx - omega * length / 2;
    double B = vx + omega * length / 2;
    double C = vy - omega * length / 2;
    double D = vy + omega * length / 2;

    double w1s = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2));
    double w2s = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
    double w3s = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));
    double w4s = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));

    double w1a = Math.atan2(B, C) * (180 / Math.PI);
    double w2a = Math.atan2(B, D) * (180 / Math.PI);
    double w3a = Math.atan2(A, D) * (180 / Math.PI);
    double w4a = Math.atan2(A, C) * (180 / Math.PI);

    double max = w1s;
    if (w2s > max)
      max = w2s;
    if (w3s > max)
      max = w3s;
    if (w4s > max)
      max = w4s;
    if (max > 1) {
      w1s = w1s / max;
      w2s = w2s / max;
      w3s = w3s / max;
      w4s = w4s / max;
    }
    double spinTolerance = 10;

    w1ca = -1 * (getPosition(frEncoder.get(), 267.4) - 180);
    w2ca = -1 * (getPosition(flEncoder.get(), 120.7) - 180);
    w3ca = -1 * (getPosition(blEncoder.get(), 64.7) - 180);
    w4ca = -1 * (getPosition(brEncoder.get(), 335.2) - 180);

    double w1d = w1ca - w1a;
    double w2d = w2ca - w2a;
    double w3d = w3ca - w3a;
    double w4d = w4ca - w4a;

    // System.out.println(getPosition(frEncoder.get(), 267.4) + "Encoder FL");
    // System.out.println(getPosition(flEncoder.get(), 120.7) + "Encoder FL");
    // System.out.println(getPosition(brEncoder.get(), 335.2) + "Encoder BR");
    // System.out.println(getPosition(blEncoder.get(), 64.7) + "Encoder BL");
    /*
     * if (counter == 10) { System.out.format("Desired " + "%.1f%n", w1a);
     * System.out.format("Desired Aswell " + "%.1f%n", w1Angle);
     * System.out.format("Current " + "%.1f%n", w1ca);
     * System.out.format("Difference " + "%.1f%n%n", w1d); System.out.println(" ");
     * counter = 0; } counter = counter + 1;
     */
    // frDrive.set(w1s);
    /*
     * if (w1d > spinTolerance) { if (Math.abs(w1a - w1ca) < 180) {
     * frSteer.set(-0.07); } else { frSteer.set(0.07); }
     * 
     * } else { frSteer.set(0); }
     */

    /*
     * if (w1ca > 90 && w1a < -90) { if (((Math.abs(w1ca - 360)) - w1a) < 90) {
     * frSteer.set(-0.07); } else { frSteer.set(0.07); } } else if (w1ca < -90 &&
     * w1a > 90) { if (((Math.abs(w1ca + 360)) - w1a) < 90) { frSteer.set(0.07); }
     * else { frSteer.set(-0.07); } } else { if (w1d < 0) { frSteer.set(-0.07); }
     * else { frSteer.set(0.07); } }
     */

    if (Math.abs(w1d) < spinTolerance) {
      frSteer.set(0);
    } else {
      if (Math.abs(w1d) > 90) {
        if (w1a >= 0) {
          w1a = w1a - 180;
        } else {
          w1a = w1a + 180;
        }
      }
      if (w1ca > 90 && w1a < -90) {
        if (((Math.abs(w1ca - 360)) - w1a) < 90) {
          frSteer.set(-0.07);
        } else {
          frSteer.set(0.07);
        }
      } else if (w1ca < -90 && w1a > 90) {
        if (((Math.abs(w1ca + 360)) - w1a) < 90) {
          frSteer.set(0.07);
        } else {
          frSteer.set(-0.07);
        }
      } else {
        if (w1d < 0) {
          frSteer.set(-0.07);
        } else {
          frSteer.set(0.07);
        }
      }
    }

    if (Math.abs(w2d) < spinTolerance) {
      flSteer.set(0);
    } else {
      if (Math.abs(w2d) > 90) {
        if (w2a >= 0) {
          w2a = w2a - 180;
        } else {
          w2a = w2a + 180;
        }
      }
      if (w2ca > 90 && w2a < -90) {
        if (((Math.abs(w2ca - 360)) - w2a) < 90) {
          flSteer.set(-0.07);
        } else {
          flSteer.set(0.07);
        }
      } else if (w2ca < -90 && w2a > 90) {
        if (((Math.abs(w2ca + 360)) - w2a) < 90) {
          flSteer.set(0.07);
        } else {
          flSteer.set(-0.07);
        }
      } else {
        if (w2d < 0) {
          flSteer.set(-0.07);
        } else {
          flSteer.set(0.07);
        }
      }
    }

    if (Math.abs(w3d) < spinTolerance) {
      blSteer.set(0);
    } else {
      if (Math.abs(w3d) > 90) {
        if (w3a >= 0) {
          w3a = w3a - 180;
        } else {
          w3a = w3a + 180;
        }
      }

      if (w3ca > 90 && w3a < -90) {
        if (((Math.abs(w3ca - 360)) - w3a) < 90) {
          blSteer.set(-0.07);
        } else {
          blSteer.set(0.07);
        }
      } else if (w3ca < -90 && w3a > 90) {
        if (((Math.abs(w3ca + 360)) - w3a) < 90) {
          blSteer.set(0.07);
        } else {
          blSteer.set(-0.07);
        }
      } else {
        if (w3d < 0) {
          blSteer.set(-0.07);
        } else {
          blSteer.set(0.07);
        }
      }
    }

    if (Math.abs(w4d) < spinTolerance) {
      brSteer.set(0);
    } else {
      if (Math.abs(w4d) > 90) {
        if (counter == 0) {
          System.out.println(w4a);
        }
        if (w4a >= 0) {
          w4a = w4a - 180;
          System.out.println("B");
        } else {
          w4a = w4a + 180;
          if (counter == 0) {
            System.out.println(w4a);
            System.out.println("A");
            counter = counter + 1;
          }
        }
      }
      if (w4ca > 90 && w4a < -90) {
        if (((Math.abs(w4ca - 360)) - w4a) < 90) {
          brSteer.set(-0.07);
        } else {
          brSteer.set(0.07);
        }
      } else if (w4ca < -90 && w4a > 90) {
        if (((Math.abs(w4ca + 360)) - w4a) < 90) {
          brSteer.set(0.07);
        } else {
          brSteer.set(-0.07);
        }
      } else {
        if (w4d < 0) {
          brSteer.set(-0.07);
        } else {
          brSteer.set(0.07);
        }
      }
    }

    frDrive.set(w1s / 3);

    flDrive.set(w2s / 3);

    blDrive.set(w3s / 3);

    brDrive.set(w4s / 3);

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  public double getPosition(double rawAngle, double offset) {
    double offsetRot = offset / 360;
    double angle = rawAngle - offsetRot;
    double angleDeg = (angle % 1) * 360;
    if (angleDeg < 0) {
      angleDeg = angleDeg + 360;
    }

    return angleDeg;
  }

  public double getRawPosition(double rawAngle, double offset) {
    double offsetRot = offset / 360;
    double angle = rawAngle - offsetRot;
    double angleRaw = (angle % 1);
    if (angleRaw < 0) {
      angleRaw = angleRaw + 1;
    }
    return angleRaw;
  }
  // Offsets
  // FR = 267.4
  // FL = 120.7
  // BR = 335.2
  // BL = 64.7

}
