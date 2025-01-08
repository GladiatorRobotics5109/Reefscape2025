package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

import frc.robot.Constants.VisionConstants.PhotonCameraConfiguration;

public class VisionIOPhotonVision implements VisionIO {
    private PhotonCamera[] m_cameras;

    public VisionIOPhotonVision(PhotonCameraConfiguration[] cameras) {
        PhotonCamera[] photonCameras = new PhotonCamera[cameras.length];
        for (int i = 0; i < cameras.length; i++) {
            photonCameras[i] = new PhotonCamera(cameras[i].cameraName());
        }
    }
}
