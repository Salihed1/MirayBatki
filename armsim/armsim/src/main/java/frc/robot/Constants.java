// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
  public static final int kMotorPort = 1;
  public static final int kEncoderAChannel = 2;
  public static final int kEncoderBChannel = 3;
  public static final int kJoystickPort = 0;

  public static final String kArmPositionKey = "ArmPosition";
  public static final String kArmPKey = "ArmP";

  public static final double kDefaultArmKp = 50.0;
  public static final double kDefaultArmSetpointDegrees = 100.0;

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  //  = (2 * PI rads) / (4096 pulses)
  public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

  public static final double kArmReduction = 10;
  public static final double kArmMass = 5.0; 
  public static final double kArmLength = Units.inchesToMeters(20);
  public static final double kMinAngleRads = Units.degreesToRadians(-75);
  public static final double kMaxAngleRads = Units.degreesToRadians(255);
}
