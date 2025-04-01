
package frc.robot.classes;

import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.*; 
import frc.robot.utils.NCDebug;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/**
 * The Vision class handles getting and managing data from the PhotoVision system.
 * It is responsible for getting target data, selecting appropriate targets, and passing information to other subsystems.
 */
public class Vision {
	private static Vision instance;
  public final PhotonCamera front_camera, back_camera;
  private final PhotonPoseEstimator photonEstimatorFront, photonEstimatorBack;
  private Matrix<N3, N1> curStdDevsFront, curStdDevsBack;
  private double lastEstTimestampFront, lastEstTimestampBack = 0;
  private Pose2d m_visFrontPose = Pose2d.kZero;
  private Pose2d m_visBackPose = Pose2d.kZero;

  // Simulator
  private VisionSystemSim visionSim;

  public enum Tags {
    BLUE_SOURCE_RIGHT(1),
    BLUE_SOURCE_LEFT(2),
    BLUE_AMP(6),
    BLUE_SPEAKER_CENTER(7),
    BLUE_SPEAKER_SIDE(8),
    BLUE_STAGE_LEFT(15),
    BLUE_STAGE_RIGHT(16),
    BLUE_STAGE_CENTER(14),
    RED_SOURCE_RIGHT(9),
    RED_SOURCE_LEFT(10),
    RED_AMP(5),
    RED_SPEAKER_CENTER(4),
    RED_SPEAKER_SIDE(3),
    RED_STAGE_LEFT(11),
    RED_STAGE_RIGHT(12),
    RED_STAGE_CENTER(13);
    private final int id;
    Tags(int id) { this.id = id; }
    public int ID() { return this.id; }
  }
  public enum Targets {
    SPEAKER,
    AMP,
    STAGE_LEFT,
    STAGE_RIGHT,
    STAGE_CENTER,
    SOURCE;
  }

  /**
	 * Returns the instance of the class.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return instance of this class
	 */
  public static Vision getInstance() {
		if (instance == null)
			instance = new Vision();
		return instance;
	}

  public Vision() {
    front_camera = new PhotonCamera(VisionConstants.Front.kCameraName);
    photonEstimatorFront = new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.Front.kRobotToCam);
    // photonEstimatorFront = new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, front_camera, VisionConstants.Front.kRobotToCam);
    photonEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    Matrix<N3, N1> front_curStdDevs;

    back_camera = new PhotonCamera(VisionConstants.Back.kCameraName);
    photonEstimatorBack = new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.Back.kRobotToCam);
    // photonEstimatorBack = new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, back_camera, VisionConstants.Back.kRobotToCam);
    photonEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    Matrix<N3, N1> back_curStdDevs;

    // Simulation
    // if (Robot.isSimulation()) {
        // Create the vision system simulation which handles cameras and targets on the field.
        visionSim = new VisionSystemSim("main");
        // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
        visionSim.addAprilTags(VisionConstants.kTagLayout);
        // Create simulated camera properties. These can be set to mimic your actual camera.
        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(15);
        cameraProp.setAvgLatencyMs(50);
        cameraProp.setLatencyStdDevMs(15);
        // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
        // targets.
        PhotonCameraSim front_cameraSim = new PhotonCameraSim(front_camera, cameraProp);
        PhotonCameraSim back_cameraSim = new PhotonCameraSim(back_camera, cameraProp);
        // Add the simulated camera to view the targets on this simulated field.
        visionSim.addCamera(front_cameraSim, VisionConstants.Front.kRobotToCam);
        visionSim.addCamera(back_cameraSim,VisionConstants.Back.kRobotToCam);

        // front_cameraSim.enableDrawWireframe(true);
        // back_cameraSim.enableDrawWireframe(true);
    // }
    createDashboards();
  }

    public void createDashboards() {
    // ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    // driverTab.addString("LED Color", this::getColor)
    //   .withSize(5, 4)
    //   .withWidget("Single Color View")
    //   .withPosition(19, 0);  
		if(VisionConstants.debugDashboard) {
      ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
      ShuffleboardLayout dbgVisionList = debugTab.getLayout("Vision", BuiltInLayouts.kList)
        .withSize(4, 8)
        .withPosition(0, 0)
        .withProperties(Map.of("Label position", "LEFT"));
      dbgVisionList.addBoolean("Front Targets", () -> getLatestResult(front_camera).hasTargets())
        .withWidget("Boolean Box");
      dbgVisionList.addBoolean("Back Targets", () -> getLatestResult(back_camera).hasTargets())
        .withWidget("Boolean Box");
      dbgVisionList.addBoolean("Front Suppressed", () -> RobotContainer.drivetrain.isFrontVisionSuppressed())
        .withWidget("Boolean Box");
      dbgVisionList.addBoolean("Back Suppressed", () -> RobotContainer.drivetrain.isFrontVisionSuppressed())
        .withWidget("Boolean Box");
      dbgVisionList.addNumber("Front Vision Pose X", () -> NCDebug.General.roundDouble(getVisionPose("front").getX(),3));
      dbgVisionList.addNumber("Front Vision Pose Y", () -> NCDebug.General.roundDouble(getVisionPose("front").getY(),3));
      dbgVisionList.addNumber("Back Vision Pose X", () -> NCDebug.General.roundDouble(getVisionPose("back").getX(),3));
      dbgVisionList.addNumber("Back Vision Pose Y", () -> NCDebug.General.roundDouble(getVisionPose("back").getY(),3));
      dbgVisionList.addNumber("Robot Pose X", () -> NCDebug.General.roundDouble(RobotContainer.drivetrain.getBotPose().getX(),3));
      dbgVisionList.addNumber("Robot Pose Y", () -> NCDebug.General.roundDouble(RobotContainer.drivetrain.getBotPose().getY(),3));
    }
  }

  public Pose2d getVisionPose(String cam) {
    Optional<EstimatedRobotPose> estimatedPose;
    if(cam == "front") {
      estimatedPose = getFrontEstimatedGlobalPose();
      if (!estimatedPose.isEmpty()) {
        m_visFrontPose = estimatedPose.get().estimatedPose.toPose2d();
      }
      return m_visFrontPose;
    }
    estimatedPose = getBackEstimatedGlobalPose();
    if (!estimatedPose.isEmpty()) {
      m_visBackPose = estimatedPose.get().estimatedPose.toPose2d();
    }
    return m_visBackPose;
  }

  public PhotonPipelineResult getLatestResult(PhotonCamera camera) {
    return camera.getLatestResult();
  }

  public Optional<EstimatedRobotPose> getFrontEstimatedGlobalPose() {
    return getEstimatedGlobalPose(photonEstimatorFront, front_camera, curStdDevsFront); 
  }
  public Optional<EstimatedRobotPose> getBackEstimatedGlobalPose() {
    return getEstimatedGlobalPose(photonEstimatorBack, back_camera, curStdDevsBack); 
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPoseEstimator estimator, PhotonCamera camera, Matrix<N3, N1> stdDevs) {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      // estimator.setReferencePose(RobotContainer.drivetrain.getState().Pose);
      for (var change: camera.getAllUnreadResults()) {
        visionEst = estimator.update(change);
        updateEstimationStdDevs(visionEst, change.getTargets(), estimator, stdDevs);
        if (Robot.isSimulation()) {
          visionEst.ifPresentOrElse(
            est ->
              getSimDebugField()
                .getObject("VisionEstimation")
                .setPose(est.estimatedPose.toPose2d()),
            () -> {
              getSimDebugField().getObject("VisionEstimation").setPoses();
            }
          );
        }
      }
      return visionEst;
  }

      private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, PhotonPoseEstimator estimator, Matrix<N3, N1> stdDev) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            stdDev = VisionConstants.kSingleTagStdDevs;
        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                stdDev = VisionConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                stdDev = estStdDevs;
            }
        }
    }

  public Matrix<N3, N1> getFrontEstimationStdDevs(Pose2d pose) {
    return getEstimationStdDevs(pose, photonEstimatorFront, front_camera);
  }
  public Matrix<N3, N1> getBackEstimationStdDevs(Pose2d pose) {
    return getEstimationStdDevs(pose, photonEstimatorBack, back_camera);
  }


  /**
   * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, PhotonPoseEstimator estimator, PhotonCamera camera) {
      var estStdDevs = VisionConstants.kSingleTagStdDevs;
      var targets = getLatestResult(camera).getTargets();
      int numTags = 0;
      double avgDist = 0;
      for (var tgt : targets) {
          var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty()) continue;
          numTags++;
          avgDist +=
                  tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
      }
      if (numTags == 0) return estStdDevs;
      avgDist /= numTags;
      // Decrease std devs if multiple targets are visible
      if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
      // Increase std devs based on (average) distance
      if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

      return estStdDevs;
  }

  // ----- Simulation

  public void simulationPeriodic(Pose2d robotSimPose) {
      visionSim.update(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
      if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
      if (!Robot.isSimulation()) return null;
      return visionSim.getDebugField();
  }

  public void updateDashboard() {
    // Dashboard.Vision.setVisionRinglight(llresults.targetingResults.);
  }

  public void updateResults() {
    
  }

  /**
   * addVisionMeasurement fuses the Pose2d from the vision system into the robot pose
   * @param visionMeasurement
   * @param timestampSeconds
   */
	public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
		RobotContainer.drivetrain.addVisionMeasurement(visionMeasurement, timestampSeconds);
	}
	public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
		RobotContainer.drivetrain.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
	}

	/**
	 * Corrects the bot pose based on information from the vision system
	 */
  @SuppressWarnings({"unused"})
	public void correctPoseWithVision() {
		if(VisionConstants.kUseVisionForPose && VisionConstants.Front.kUseForPose) {
			var visionEstFront = RobotContainer.vision.getFrontEstimatedGlobalPose();
			visionEstFront.ifPresent(
				est -> {
					var estPose = est.estimatedPose.toPose2d();
					//workaround for remove camera to robot center
					// estPose = estPose.transformBy(new Transform2d(new Translation2d(-0.339,-0.250), new Rotation2d())); 
					// Change our trust in the measurement based on the tags we can see
					var estStdDevs = RobotContainer.vision.getFrontEstimationStdDevs(estPose);
          //For CTR, timestamp must be in correct timebase, use Utils.fpgaToCurrentTime(timestamp) to correct
					RobotContainer.drivetrain.addVisionMeasurement(estPose, Utils.fpgaToCurrentTime(est.timestampSeconds), estStdDevs);
				}
			);
		}
		if(VisionConstants.kUseVisionForPose && VisionConstants.Back.kUseForPose) {
			var visionEstBack = RobotContainer.vision.getBackEstimatedGlobalPose();
			visionEstBack.ifPresent(
				est -> {
					var estPose = est.estimatedPose.toPose2d();
					//workaround for remove camera to robot center
					// estPose = estPose.transformBy(new Transform2d(new Translation2d(0.44,0.0), new Rotation2d())); 
					// Change our trust in the measurement based on the tags we can see
					var estStdDevs = RobotContainer.vision.getBackEstimationStdDevs(estPose);
          //For CTR, timestamp must be in correct timebase, use Utils.fpgaToCurrentTime(timestamp) to correct
					RobotContainer.drivetrain.addVisionMeasurement(estPose, Utils.fpgaToCurrentTime(est.timestampSeconds), estStdDevs);
				}
			);
		}
	}

}
