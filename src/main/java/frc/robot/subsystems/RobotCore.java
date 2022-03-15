// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotCore extends SubsystemBase {

  private PneumaticHub pneumaticHub = new PneumaticHub();
  private PowerDistribution powerDistribution = new PowerDistribution();
  private UsbCamera driverCamera;
  /** Creates a new Pneumatics. */
  public RobotCore() {
    pneumaticHub.enableCompressorAnalog(65, 110);
    powerDistribution.clearStickyFaults();
    driverCamera = CameraServer.startAutomaticCapture();
    driverCamera.setVideoMode(PixelFormat.kYUYV, 160, 120, 20);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
