package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriverCameraSubsystem extends SubsystemBase {
    UsbCamera frontBumper;
    UsbCamera ShooterCam;
    NetworkTableEntry cameraSelection;
    int currentCameraIndex = 0;
    VideoSink cameraServer;
    UsbCamera[] allCameras;

    public DriverCameraSubsystem() {
        frontBumper = CameraServer.startAutomaticCapture(0); // 0 is placeholder
        ShooterCam = CameraServer.startAutomaticCapture(1); // 1 is placeholder
        cameraServer = CameraServer.getServer();
        allCameras = new UsbCamera[] { frontBumper, ShooterCam };
        frontBumper.setResolution(424, 240);
        frontBumper.setFPS(30);
        ShooterCam.setResolution(160, 120);
        ShooterCam.setFPS(20);
    }
}