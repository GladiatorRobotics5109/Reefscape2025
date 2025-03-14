package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.PhotonCameraConfiguration;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Queue;

public class VisionIOPhotonVision implements VisionIO {
    private PhotonCamera m_camera;
    private PhotonPoseEstimator m_poseEstimator;

    private final Optional<Matrix<N3, N3>> m_cameraMatrix;
    private final Optional<Matrix<N8, N1>> m_distCoeffs;

    private final Queue<EstimatedRobotPose> m_scratchBuff;

    private final String m_cameraName;

    public VisionIOPhotonVision(PhotonCameraConfiguration cameraConfigs) {
        m_cameraName = cameraConfigs.cameraName();
        m_camera = new PhotonCamera(m_cameraName);
        m_poseEstimator = new PhotonPoseEstimator(
            VisionConstants.kAprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            cameraConfigs.robotToCamera()
        );

        m_cameraMatrix = m_camera.getCameraMatrix();
        m_distCoeffs = m_camera.getDistCoeffs();

        m_scratchBuff = new LinkedList<>();
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();

        for (PhotonPipelineResult result : results) {
            Optional<EstimatedRobotPose> estimatedPose = m_poseEstimator.update(
                result,
                m_cameraMatrix,
                m_distCoeffs
            );
            if (estimatedPose.isEmpty()) {
                continue;
            }

            m_scratchBuff.add(estimatedPose.get());
        }

        inputs.poses = m_scratchBuff.stream().map((pose) -> pose.estimatedPose).toArray(Pose3d[]::new);
        inputs.timestamps = m_scratchBuff.stream().mapToDouble((pose) -> pose.timestampSeconds).toArray();
        // inputs.targetsUsed = m_scratchBuff.stream().map(
        //     (pose) -> pose.targetsUsed.stream().map(
        //         (target) -> VisionConstants.kAprilTagFieldLayout.getTagPose(target.fiducialId).orElse(Pose3d.kZero)
        //     ).toArray(Pose3d[]::new)
        // ).toArray(Pose3d[][]::new);

        inputs.cameraName = m_cameraName;

        m_scratchBuff.clear();
    }
}
