// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.CANifier.LEDChannel;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotCore extends SubsystemBase {

  private PneumaticHub pneumaticHub = new PneumaticHub();
  private PowerDistribution powerDistribution = new PowerDistribution();
  private CANifier robotCanifier = new CANifier(1);
  private UsbCamera driverCamera;
  /** Creates a new Pneumatics. */
  public RobotCore() {
    pneumaticHub.enableCompressorAnalog(65, 110);
    powerDistribution.clearStickyFaults();
    pneumaticHub.clearStickyFaults();
    
    robotCanifier.setLEDOutput(1, LEDChannel.LEDChannelC);
    robotCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_1_General, 255);
    robotCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 255);
    robotCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_3_PwmInputs0, 255);
    robotCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_4_PwmInputs1, 255);
    robotCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_5_PwmInputs2, 255);
    robotCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_6_PwmInputs3, 255);
    robotCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_8_Misc, 255);
    driverCamera = CameraServer.startAutomaticCapture();
    driverCamera.setVideoMode(PixelFormat.kYUYV, 160, 120, 20);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
