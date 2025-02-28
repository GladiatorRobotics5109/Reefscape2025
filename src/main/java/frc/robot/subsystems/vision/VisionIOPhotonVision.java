package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.PhotonCameraConfiguration;

public class VisionIOPhotonVision implements VisionIO {
    private PhotonCamera m_camera;
    private PhotonPoseEstimator m_poseEstimator;

    public VisionIOPhotonVision(PhotonCameraConfiguration cameraConfigs) {
        m_camera = new PhotonCamera(cameraConfigs.cameraName());
        m_poseEstimator = new PhotonPoseEstimator(
            VisionConstants.kAprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            cameraConfigs.robotToCamera()
        );
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();

        for (PhotonPipelineResult result : results) {
            Optional<EstimatedRobotPose> estimatedPose = m_poseEstimator.update(
                result,
                m_camera.getCameraMatrix(),
                m_camera.getDistCoeffs()
            );
            if (estimatedPose.isEmpty()) {
                DriverStation.reportWarning("Failed to estimate vision position!", true);
                continue;
            }

            inputs.posees.add(estimatedPose.get().estimatedPose);
            inputs.timestamps.add(estimatedPose.get().timestampSeconds);
            for (PhotonTrackedTarget target : estimatedPose.get().targetsUsed) {
                Optional<Pose3d> pose = VisionConstants.kAprilTagFieldLayout.getTagPose(target.fiducialId);
                if (pose.isEmpty())
                    continue;
                inputs.targetsUsed.add(pose.get());
            }
        }
    }
}
