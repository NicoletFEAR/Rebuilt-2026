package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import frc.robot.constants.TuskConstants.VisionConstants;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;

public class LimelightCamera {

    private String m_cameraName;

    private final Map<String, DoubleArrayEntry> doubleArrayEntries = new ConcurrentHashMap<>();

    private ObjectMapper mapper;

    public LimelightCamera(String cameraName) {
        m_cameraName = cameraName;
    }

    public void addPoseEstimateMegatag2(SwerveDrivePoseEstimator poseEstimator, ChassisSpeeds robotSpeeds, Rotation2d yaw) {

        PoseEstimate poseEstimate;

        poseEstimate = getBotPoseEstimate_wpiBlue_MegaTag2();

        Pose2d pose = poseEstimate.pose;
        boolean isPoseInField =
                // pose.getTranslation().getX() < Constants.kFieldTopRight.getX()
                // && pose.getTranslation().getY() < Constants.kFieldTopRight.getY()
                // &&
                !pose.getTranslation().equals(new Translation2d(0, 0));
        // boolean isWithinTolerance = pose.getTranslation()
        // .getDistance(poseEstimator.getEstimatedPosition().getTranslation()) <
        // VisionConstants.kOffsetTolerance;

        // Logger.recordOutput("Vision/" + m_cameraName + "/isPoseInField",
        // isPoseInField);
        // Logger.recordOutput("Vision/" + m_cameraName + "/isWithinTolerance",
        // isWithinTolerance);

        double tagAmountTrust = VisionConstants.kTargetAmountConstant * poseEstimate.tagCount;
        double speedTrust = Math.sqrt(
                Math.pow(robotSpeeds.vxMetersPerSecond, 2)
                        + Math.pow(robotSpeeds.vyMetersPerSecond, 2))
                * VisionConstants.kSpeedsConstant;
        double rotationTrust = robotSpeeds.omegaRadiansPerSecond * VisionConstants.kRotationsConstant;
        double distanceTrust = poseEstimate.avgTagDist * VisionConstants.kDistanceConstant;
        double areaTrust = poseEstimate.avgTagArea * VisionConstants.kAreaConstant;

        // Logger.recordOutput("Vision/" + m_cameraName + "/tagAmountTrust",
        // tagAmountTrust);
        // Logger.recordOutput("Vision/" + m_cameraName + "/speedTrust", speedTrust);
        // Logger.recordOutput("Vision/" + m_cameraName + "/rotationTrust",
        // rotationTrust);
        // Logger.recordOutput("Vision/" + m_cameraName + "/distanceTrust",
        // distanceTrust);
        // Logger.recordOutput("Vision/" + m_cameraName + "/areaTrust", areaTrust);

        double trust = Math.max(speedTrust + rotationTrust - tagAmountTrust + distanceTrust - areaTrust, 0.1);

        Matrix<N3, N1> poseMatrix = VecBuilder.fill(trust, trust, 999999999);

        if (isPoseInField) {
            poseEstimator.addVisionMeasurement(new Pose2d(pose.getTranslation(), yaw), poseEstimate.timestampSeconds,
                    poseMatrix);
            Logger.recordOutput("Vision/" + m_cameraName + "/Trust", trust);
        }
        // else {
        // Logger.recordOutput("Vision/" + m_cameraName + "/Trust", 0);
        // }
    }
    public void addPoseEstimateMegatag1(SwerveDrivePoseEstimator poseEstimator, ChassisSpeeds robotSpeeds) {

        PoseEstimate poseEstimate = getBotPoseEstimate_wpiBlue();
        
        Pose2d pose = poseEstimate.pose;
        boolean isPoseInField =
                // pose.getTranslation().getX() < Constants.kFieldTopRight.getX()
                // && pose.getTranslation().getY() < Constants.kFieldTopRight.getY()
                // &&
                !pose.getTranslation().equals(new Translation2d(0, 0));
        // boolean isWithinTolerance = pose.getTranslation()
        // .getDistance(poseEstimator.getEstimatedPosition().getTranslation()) <
        // VisionConstants.kOffsetTolerance;

        // Logger.recordOutput("Vision/" + m_cameraName + "/isPoseInField",
        // isPoseInField);
        // Logger.recordOutput("Vision/" + m_cameraName + "/isWithinTolerance",
        // isWithinTolerance);

        double tagAmountTrust = VisionConstants.kTargetAmountConstant * poseEstimate.tagCount;
        double speedTrust = Math.sqrt(
                Math.pow(robotSpeeds.vxMetersPerSecond, 2)
                        + Math.pow(robotSpeeds.vyMetersPerSecond, 2))
                * VisionConstants.kSpeedsConstant;
        double rotationTrust = robotSpeeds.omegaRadiansPerSecond * VisionConstants.kRotationsConstant;
        double distanceTrust = poseEstimate.avgTagDist * VisionConstants.kDistanceConstant;
        double areaTrust = poseEstimate.avgTagArea * VisionConstants.kAreaConstant;

        // Logger.recordOutput("Vision/" + m_cameraName + "/tagAmountTrust",
        // tagAmountTrust);
        // Logger.recordOutput("Vision/" + m_cameraName + "/speedTrust", speedTrust);
        // Logger.recordOutput("Vision/" + m_cameraName + "/rotationTrust",
        // rotationTrust);
        // Logger.recordOutput("Vision/" + m_cameraName + "/distanceTrust",
        // distanceTrust);
        // Logger.recordOutput("Vision/" + m_cameraName + "/areaTrust", areaTrust);

        double trust = Math.max(speedTrust + rotationTrust - tagAmountTrust + distanceTrust - areaTrust, 0.5);

        // Matrix<N3, N1> poseMatrix = VecBuilder.fill(trust, trust, trust);

        if (isPoseInField) {
            poseEstimator.addVisionMeasurement(pose, poseEstimate.timestampSeconds);
            Logger.recordOutput("Vision/" + m_cameraName + "/MT1 Trust", trust);
        }
    }

    public PoseEstimate getBotPoseEstimate_wpiBlue() {
        return getBotPoseEstimate("botpose_wpiblue", false);
    }

    public PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2() {
        return getBotPoseEstimate("botpose_orb_wpiblue", true);
    }
    // public PoseEstimate getBotPoseEstimate_wpiRed_MegaTag2() {
    // return getBotPoseEstimate("botpose_orb_wpired", true);
    // }

    /**
     * Gets the latest JSON results output and returns a LimelightResults object.
     * 
     * @param limelightName Name of the Limelight camera
     * @return LimelightResults object containing all current target data
     */
    public LimelightResults getLatestResults() {

        long start = System.nanoTime();
        LimelightResults results = new LimelightResults();
        if (mapper == null) {
            mapper = new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
        }

        try {
            results = mapper.readValue(getJSONDump(m_cameraName), LimelightResults.class);
        } catch (JsonProcessingException e) {
            results.error = "lljson error: " + e.getMessage();
        }

        long end = System.nanoTime();
        double millis = (end - start) * .000001;
        results.latency_jsonParse = millis;

        return results;
    }

    public void SetRobotOrientation(
            double yaw, double yawRate, double pitch, double pitchRate, double roll, double rollRate) {
        SetRobotOrientation_INTERNAL(yaw, yawRate, pitch, pitchRate, roll, rollRate, true);
    }

    private void SetRobotOrientation_INTERNAL(
            double yaw,
            double yawRate,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate,
            boolean flush) {

        double[] entries = new double[6];
        entries[0] = yaw;
        entries[1] = yawRate;
        entries[2] = pitch;
        entries[3] = pitchRate;
        entries[4] = roll;
        entries[5] = rollRate;
        setLimelightNTDoubleArray(m_cameraName, "robot_orientation_set", entries);
        if (flush) {
            Flush();
        }
    }

    public void setLimelightNTDoubleArray(String tableName, String entryName, double[] val) {
        getLimelightNTTableEntry(tableName, entryName).setDoubleArray(val);
    }

    public void Flush() {
        NetworkTableInstance.getDefault().flush();
    }

    public NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
        return getLimelightNTTable(tableName).getEntry(entryName);
    }

    public Pose2d toPose2D(double[] inData) {
        if (inData.length < 6) {
            return new Pose2d();
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
        return new Pose2d(tran2d, r2d);
    }

    /**
     * Takes a 6-length array of pose data and converts it to a Pose3d object.
     * Array format: [x, y, z, roll, pitch, yaw] where angles are in degrees.
     * 
     * @param inData Array containing pose data [x, y, z, roll, pitch, yaw]
     * @return Pose3d object representing the pose, or empty Pose3d if invalid data
     */
    public Pose3d toPose3D(double[] inData) {
        if (inData.length < 6) {
            // System.err.println("Bad LL 3D Pose Data!");
            return new Pose3d();
        }
        return new Pose3d(
                new Translation3d(inData[0], inData[1], inData[2]),
                new Rotation3d(Units.degreesToRadians(inData[3]), Units.degreesToRadians(inData[4]),
                        Units.degreesToRadians(inData[5])));
    }

    private double extractArrayEntry(double[] inData, int position) {
        if (inData.length < position + 1) {
            return 0;
        }
        return inData[position];
    }

    /**
     * Gets the full JSON results dump.
     * 
     * @param limelightName Name of the Limelight camera
     * @return JSON string containing all current results
     */
    public String getJSONDump(String limelightName) {
        return getLimelightNTString(limelightName, "json");
    }

    private String sanitizeName(String name) {
        if (name == "" || name == null) {
            return "limelight";
        }
        return name;
    }

    public NetworkTable getLimelightNTTable(String tableName) {
        return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
    }

    public DoubleArrayEntry getLimelightDoubleArrayEntry(String tableName, String entryName) {
        String key = tableName + "/" + entryName;
        return doubleArrayEntries.computeIfAbsent(
                key,
                k -> {
                    NetworkTable table = getLimelightNTTable(tableName);
                    return table.getDoubleArrayTopic(entryName).getEntry(new double[0]);
                });
    }

    private PoseEstimate getBotPoseEstimate(String entryName, boolean isMegaTag2) {
        DoubleArrayEntry poseEntry = getLimelightDoubleArrayEntry(m_cameraName, entryName);

        TimestampedDoubleArray tsValue = poseEntry.getAtomic();
        double[] poseArray = tsValue.value;
        long timestamp = tsValue.timestamp;

        if (poseArray.length == 0) {
            return null;
        }

        var pose = toPose2D(poseArray);
        double latency = extractArrayEntry(poseArray, 6);
        int tagCount = (int) extractArrayEntry(poseArray, 7);
        double tagSpan = extractArrayEntry(poseArray, 8);
        double tagDist = extractArrayEntry(poseArray, 9);
        double tagArea = extractArrayEntry(poseArray, 10);

        double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);

        RawFiducial[] rawFiducials = new RawFiducial[tagCount];
        int valsPerFiducial = 7;
        int expectedTotalVals = 11 + valsPerFiducial * tagCount;

        if (poseArray.length != expectedTotalVals) {
        } else {
            for (int i = 0; i < tagCount; i++) {
                int baseIndex = 11 + (i * valsPerFiducial);
                int id = (int) poseArray[baseIndex];
                double txnc = poseArray[baseIndex + 1];
                double tync = poseArray[baseIndex + 2];
                double ta = poseArray[baseIndex + 3];
                double distToCamera = poseArray[baseIndex + 4];
                double distToRobot = poseArray[baseIndex + 5];
                double ambiguity = poseArray[baseIndex + 6];
                rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
            }
        }

        return new PoseEstimate(
                pose,
                adjustedTimestamp,
                latency,
                tagCount,
                tagSpan,
                tagDist,
                tagArea,
                rawFiducials,
                isMegaTag2);
    }

    public class RawFiducial {
        public int id = 0;
        public double txnc = 0;
        public double tync = 0;
        public double ta = 0;
        public double distToCamera = 0;
        public double distToRobot = 0;
        public double ambiguity = 0;

        public RawFiducial(
                int id,
                double txnc,
                double tync,
                double ta,
                double distToCamera,
                double distToRobot,
                double ambiguity) {
            this.id = id;
            this.txnc = txnc;
            this.tync = tync;
            this.ta = ta;
            this.distToCamera = distToCamera;
            this.distToRobot = distToRobot;
            this.ambiguity = ambiguity;
        }
    }

    public class PoseEstimate {
        public Pose2d pose;
        public double timestampSeconds;
        public double latency;
        public int tagCount;
        public double tagSpan;
        public double avgTagDist;
        public double avgTagArea;

        public RawFiducial[] rawFiducials;
        public boolean isMegaTag2;

        public PoseEstimate() {
            this.pose = new Pose2d();
            this.timestampSeconds = 0;
            this.latency = 0;
            this.tagCount = 0;
            this.tagSpan = 0;
            this.avgTagDist = 0;
            this.avgTagArea = 0;
            this.rawFiducials = new RawFiducial[] {};
            this.isMegaTag2 = false;
        }

        public PoseEstimate(
                Pose2d pose,
                double timestampSeconds,
                double latency,
                int tagCount,
                double tagSpan,
                double avgTagDist,
                double avgTagArea,
                RawFiducial[] rawFiducials,
                boolean isMegaTag2) {

            this.pose = pose;
            this.timestampSeconds = timestampSeconds;
            this.latency = latency;
            this.tagCount = tagCount;
            this.tagSpan = tagSpan;
            this.avgTagDist = avgTagDist;
            this.avgTagArea = avgTagArea;
            this.rawFiducials = rawFiducials;
            this.isMegaTag2 = isMegaTag2;
        }
    }

    /**
     * Represents a Color/Retroreflective Target Result extracted from JSON Output
     */
    public class LimelightTarget_Retro {

        @JsonProperty("t6c_ts")
        private double[] cameraPose_TargetSpace;

        @JsonProperty("t6r_fs")
        private double[] robotPose_FieldSpace;

        @JsonProperty("t6r_ts")
        private double[] robotPose_TargetSpace;

        @JsonProperty("t6t_cs")
        private double[] targetPose_CameraSpace;

        @JsonProperty("t6t_rs")
        private double[] targetPose_RobotSpace;

        public Pose3d getCameraPose_TargetSpace() {
            return toPose3D(cameraPose_TargetSpace);
        }

        public Pose3d getRobotPose_FieldSpace() {
            return toPose3D(robotPose_FieldSpace);
        }

        public Pose3d getRobotPose_TargetSpace() {
            return toPose3D(robotPose_TargetSpace);
        }

        public Pose3d getTargetPose_CameraSpace() {
            return toPose3D(targetPose_CameraSpace);
        }

        public Pose3d getTargetPose_RobotSpace() {
            return toPose3D(targetPose_RobotSpace);
        }

        public Pose2d getCameraPose_TargetSpace2D() {
            return toPose2D(cameraPose_TargetSpace);
        }

        public Pose2d getRobotPose_FieldSpace2D() {
            return toPose2D(robotPose_FieldSpace);
        }

        public Pose2d getRobotPose_TargetSpace2D() {
            return toPose2D(robotPose_TargetSpace);
        }

        public Pose2d getTargetPose_CameraSpace2D() {
            return toPose2D(targetPose_CameraSpace);
        }

        public Pose2d getTargetPose_RobotSpace2D() {
            return toPose2D(targetPose_RobotSpace);
        }

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("typ")
        public double ty_pixels;

        @JsonProperty("tx_nocross")
        public double tx_nocrosshair;

        @JsonProperty("ty_nocross")
        public double ty_nocrosshair;

        @JsonProperty("ts")
        public double ts;

        public LimelightTarget_Retro() {
            cameraPose_TargetSpace = new double[6];
            robotPose_FieldSpace = new double[6];
            robotPose_TargetSpace = new double[6];
            targetPose_CameraSpace = new double[6];
            targetPose_RobotSpace = new double[6];
        }

    }

    /**
     * Represents an AprilTag/Fiducial Target Result extracted from JSON Output
     */
    public class LimelightTarget_Fiducial {

        @JsonProperty("fID")
        public double fiducialID;

        @JsonProperty("fam")
        public String fiducialFamily;

        @JsonProperty("t6c_ts")
        private double[] cameraPose_TargetSpace;

        @JsonProperty("t6r_fs")
        private double[] robotPose_FieldSpace;

        @JsonProperty("t6r_ts")
        private double[] robotPose_TargetSpace;

        @JsonProperty("t6t_cs")
        private double[] targetPose_CameraSpace;

        @JsonProperty("t6t_rs")
        private double[] targetPose_RobotSpace;

        public Pose3d getCameraPose_TargetSpace() {
            return toPose3D(cameraPose_TargetSpace);
        }

        public Pose3d getRobotPose_FieldSpace() {
            return toPose3D(robotPose_FieldSpace);
        }

        public Pose3d getRobotPose_TargetSpace() {
            return toPose3D(robotPose_TargetSpace);
        }

        public Pose3d getTargetPose_CameraSpace() {
            return toPose3D(targetPose_CameraSpace);
        }

        public Pose3d getTargetPose_RobotSpace() {
            return toPose3D(targetPose_RobotSpace);
        }

        public Pose2d getCameraPose_TargetSpace2D() {
            return toPose2D(cameraPose_TargetSpace);
        }

        public Pose2d getRobotPose_FieldSpace2D() {
            return toPose2D(robotPose_FieldSpace);
        }

        public Pose2d getRobotPose_TargetSpace2D() {
            return toPose2D(robotPose_TargetSpace);
        }

        public Pose2d getTargetPose_CameraSpace2D() {
            return toPose2D(targetPose_CameraSpace);
        }

        public Pose2d getTargetPose_RobotSpace2D() {
            return toPose2D(targetPose_RobotSpace);
        }

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("typ")
        public double ty_pixels;

        @JsonProperty("tx_nocross")
        public double tx_nocrosshair;

        @JsonProperty("ty_nocross")
        public double ty_nocrosshair;

        @JsonProperty("ts")
        public double ts;

        public LimelightTarget_Fiducial() {
            cameraPose_TargetSpace = new double[6];
            robotPose_FieldSpace = new double[6];
            robotPose_TargetSpace = new double[6];
            targetPose_CameraSpace = new double[6];
            targetPose_RobotSpace = new double[6];
        }
    }

    /**
     * Represents a Neural Classifier Pipeline Result extracted from JSON Output
     */
    public class LimelightTarget_Classifier {

        @JsonProperty("class")
        public String className;

        @JsonProperty("classID")
        public double classID;

        @JsonProperty("conf")
        public double confidence;

        @JsonProperty("zone")
        public double zone;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("typ")
        public double ty_pixels;

        public LimelightTarget_Classifier() {
        }
    }

    public String getLimelightNTString(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getString("");
    }

    /**
     * Represents a Neural Detector Pipeline Result extracted from JSON Output
     */
    public class LimelightTarget_Detector {

        @JsonProperty("class")
        public String className;

        @JsonProperty("classID")
        public double classID;

        @JsonProperty("conf")
        public double confidence;

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("typ")
        public double ty_pixels;

        @JsonProperty("tx_nocross")
        public double tx_nocrosshair;

        @JsonProperty("ty_nocross")
        public double ty_nocrosshair;

        public LimelightTarget_Detector() {
        }
    }

    /**
     * Limelight Results object, parsed from a Limelight's JSON results output.
     */
    public class LimelightResults {

        public String error;

        @JsonProperty("pID")
        public double pipelineID;

        @JsonProperty("tl")
        public double latency_pipeline;

        @JsonProperty("cl")
        public double latency_capture;

        public double latency_jsonParse;

        @JsonProperty("ts")
        public double timestamp_LIMELIGHT_publish;

        @JsonProperty("ts_rio")
        public double timestamp_RIOFPGA_capture;

        @JsonProperty("v")
        @JsonFormat(shape = Shape.NUMBER)
        public boolean valid;

        @JsonProperty("botpose")
        public double[] botpose;

        @JsonProperty("botpose_wpired")
        public double[] botpose_wpired;

        @JsonProperty("botpose_wpiblue")
        public double[] botpose_wpiblue;

        @JsonProperty("botpose_tagcount")
        public double botpose_tagcount;

        @JsonProperty("botpose_span")
        public double botpose_span;

        @JsonProperty("botpose_avgdist")
        public double botpose_avgdist;

        @JsonProperty("botpose_avgarea")
        public double botpose_avgarea;

        @JsonProperty("t6c_rs")
        public double[] camerapose_robotspace;

        public Pose3d getBotPose3d() {
            return toPose3D(botpose);
        }

        public Pose3d getBotPose3d_wpiRed() {
            return toPose3D(botpose_wpired);
        }

        public Pose3d getBotPose3d_wpiBlue() {
            return toPose3D(botpose_wpiblue);
        }

        public Pose2d getBotPose2d() {
            return toPose2D(botpose);
        }

        public Pose2d getBotPose2d_wpiRed() {
            return toPose2D(botpose_wpired);
        }

        public Pose2d getBotPose2d_wpiBlue() {
            return toPose2D(botpose_wpiblue);
        }

        @JsonProperty("Retro")
        public LimelightTarget_Retro[] targets_Retro;

        @JsonProperty("Fiducial")
        public LimelightTarget_Fiducial[] targets_Fiducials;

        @JsonProperty("Classifier")
        public LimelightTarget_Classifier[] targets_Classifier;

        @JsonProperty("Detector")
        public LimelightTarget_Detector[] targets_Detector;

        @JsonProperty("Barcode")
        public LimelightTarget_Barcode[] targets_Barcode;

        public LimelightResults() {
            botpose = new double[6];
            botpose_wpired = new double[6];
            botpose_wpiblue = new double[6];
            camerapose_robotspace = new double[6];
            targets_Retro = new LimelightTarget_Retro[0];
            targets_Fiducials = new LimelightTarget_Fiducial[0];
            targets_Classifier = new LimelightTarget_Classifier[0];
            targets_Detector = new LimelightTarget_Detector[0];
            targets_Barcode = new LimelightTarget_Barcode[0];

        }

    }

    /**
     * Represents a Barcode Target Result extracted from JSON Output
     */
    public class LimelightTarget_Barcode {

        /**
         * Barcode family type (e.g. "QR", "DataMatrix", etc.)
         */
        @JsonProperty("fam")
        public String family;

        /**
         * Gets the decoded data content of the barcode
         */
        @JsonProperty("data")
        public String data;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("typ")
        public double ty_pixels;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("tx_nocross")
        public double tx_nocrosshair;

        @JsonProperty("ty_nocross")
        public double ty_nocrosshair;

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("pts")
        public double[][] corners;

        public LimelightTarget_Barcode() {
        }

        public String getFamily() {
            return family;
        }
    }
}
