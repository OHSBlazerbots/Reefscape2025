package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriverCameraSubsystem extends SubsystemBase {
    UsbCamera armCamera;
    NetworkTableEntry cameraSelection;
    int currentCameraIndex = 0;
    VideoSink cameraServer;
    UsbCamera[] allCameras;

    public DriverCameraSubsystem() {
        armCamera = CameraServer.startAutomaticCapture(0); // 0 is placeholder
        cameraServer = CameraServer.getServer();
        allCameras = new UsbCamera[] { armCamera, };
        armCamera.setResolution(424, 240);
        armCamera.setFPS(30);
    }
}