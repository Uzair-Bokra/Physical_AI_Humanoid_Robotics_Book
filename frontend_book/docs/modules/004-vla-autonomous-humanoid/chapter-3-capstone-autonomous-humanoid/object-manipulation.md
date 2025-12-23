# Object Identification and Manipulation in Autonomous Humanoid Systems

Object identification and manipulation represent critical capabilities for autonomous humanoid robots, enabling them to interact with the physical world in meaningful ways. This chapter explores the sophisticated systems required to identify, locate, and manipulate objects in real-world environments, with emphasis on the integration of perception, planning, and control technologies.

## Object Identification Architecture

### Multi-Modal Perception System

The object identification system employs a multi-modal approach combining various sensing technologies:

```
┌─────────────────────────────────────────────────────────────────┐
│                    PERCEPTION FUSION                            │
│  Camera • LiDAR • Depth Sensor • Tactile • Force Feedback      │
├─────────────────────────────────────────────────────────────────┤
│                    OBJECT DETECTION                            │
│  Instance Segmentation • Classification • Pose Estimation       │
├─────────────────────────────────────────────────────────────────┤
│                    OBJECT UNDERSTANDING                        │
│  Affordance Recognition • Attribute Extraction • Context        │
├─────────────────────────────────────────────────────────────────┤
│                    MANIPULATION PLANNING                        │
│  Grasp Planning • Trajectory Generation • Force Control         │
└─────────────────────────────────────────────────────────────────┘
```

### Perception Pipeline Components

The system integrates multiple perception components working in coordination:

#### Visual Perception
- **Function**: Processes RGB camera data for object detection and recognition
- **Inputs**: Camera images, depth maps
- **Outputs**: Object bounding boxes, 3D poses, semantic labels
- **Algorithms**: YOLO, Mask R-CNN, DeepLab variants

#### Depth Perception
- **Function**: Provides 3D information for object localization
- **Inputs**: Depth camera data, stereo vision
- **Outputs**: 3D point clouds, object poses
- **Algorithms**: PCL processing, surface reconstruction

#### Tactile Perception
- **Function**: Provides feedback during manipulation
- **Inputs**: Tactile sensors, force/torque readings
- **Outputs**: Contact information, force feedback
- **Algorithms**: Contact detection, slip detection

## Object Detection and Recognition

### Deep Learning-Based Detection

The system employs state-of-the-art deep learning for object detection:

```cpp
// Isaac ROS-based object detection system
class IsaacObjectDetector
{
public:
    IsaacObjectDetector()
    {
        // Initialize Isaac ROS perception pipeline
        initializeIsaacPerception();

        // Load object detection models
        detection_model_ = loadDetectionModel("detection_model.pt");
        segmentation_model_ = loadSegmentationModel("segmentation_model.pt");
        pose_estimator_ = loadPoseEstimator("pose_estimator.pt");
    }

    std::vector<ObjectDetection> detectObjects(const sensor_msgs::msg::Image::SharedPtr image_msg,
                                            const sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
    {
        // Convert ROS image to format expected by Isaac ROS
        auto image_tensor = convertImageToTensor(image_msg);

        // Run object detection
        auto detections = runDetectionModel(image_tensor);

        // Apply segmentation to refine object boundaries
        auto segmentations = runSegmentationModel(image_tensor, detections);

        // Estimate 3D poses from 2D detections and depth information
        auto poses = estimatePoses(detections, segmentations, camera_info);

        // Combine results into structured detections
        std::vector<ObjectDetection> results;
        for (size_t i = 0; i < detections.size(); ++i) {
            ObjectDetection detection;
            detection.class_name = detections[i].class_name;
            detection.confidence = detections[i].confidence;
            detection.bounding_box = detections[i].bbox;
            detection.mask = segmentations[i];
            detection.pose = poses[i];
            detection.attributes = extractAttributes(detection);

            results.push_back(detection);
        }

        return results;
    }

private:
    void initializeIsaacPerception()
    {
        // Initialize Isaac ROS perception components
        perception_pipeline_ = std::make_unique<IsaacPerceptionPipeline>();

        // Configure camera parameters
        camera_intrinsics_ = configureCameraIntrinsics();

        // Set detection parameters
        detection_params_.confidence_threshold = 0.7;
        detection_params_.nms_threshold = 0.4;
        detection_params_.max_detections = 50;
    }

    std::vector<Detection> runDetectionModel(const torch::Tensor& image_tensor)
    {
        // Run object detection model
        auto output = detection_model_->forward({image_tensor}).toTuple();

        // Extract detection results
        auto boxes = output->elements()[0].toTensor();
        auto scores = output->elements()[1].toTensor();
        auto labels = output->elements()[2].toTensor();

        std::vector<Detection> detections;
        for (int i = 0; i < boxes.size(0); ++i) {
            if (scores[i].item<float>() > detection_params_.confidence_threshold) {
                Detection det;
                det.bbox.xmin = boxes[i][0].item<float>();
                det.bbox.ymin = boxes[i][1].item<float>();
                det.bbox.xmax = boxes[i][2].item<float>();
                det.bbox.ymax = boxes[i][3].item<float>();
                det.confidence = scores[i].item<float>();
                det.class_id = labels[i].item<int64_t>();
                det.class_name = getClassLabel(det.class_id);

                detections.push_back(det);
            }
        }

        // Apply non-maximum suppression
        detections = applyNMS(detections, detection_params_.nms_threshold);

        return detections;
    }

    std::vector<SegmentationMask> runSegmentationModel(const torch::Tensor& image_tensor,
                                                    const std::vector<Detection>& detections)
    {
        // Run instance segmentation model
        auto masks = segmentation_model_->forward({image_tensor}).toTensor();

        std::vector<SegmentationMask> segmentations;
        for (const auto& detection : detections) {
            // Extract mask for this detection
            auto mask = extractMaskForDetection(masks, detection);
            segmentations.push_back(mask);
        }

        return segmentations;
    }

    std::vector<Pose3D> estimatePoses(const std::vector<Detection>& detections,
                                    const std::vector<SegmentationMask>& segmentations,
                                    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
    {
        std::vector<Pose3D> poses;

        for (size_t i = 0; i < detections.size(); ++i) {
            // Get 2D bounding box
            auto bbox = detections[i].bbox;

            // Use segmentation mask to get precise object boundary
            auto mask = segmentations[i];

            // Estimate 3D pose using PnP or template matching
            auto pose = estimateObjectPose(bbox, mask, camera_info);

            poses.push_back(pose);
        }

        return poses;
    }

    ObjectAttributes extractAttributes(const ObjectDetection& detection)
    {
        ObjectAttributes attributes;

        // Extract color information from the segmented region
        attributes.color = extractColorFromMask(detection.image, detection.mask);

        // Estimate size from bounding box and depth
        attributes.size = estimateSizeFromDepth(detection.bounding_box, detection.pose);

        // Determine material properties from appearance
        attributes.material = estimateMaterial(detection.image, detection.mask);

        // Estimate weight based on size and material
        attributes.weight = estimateWeight(attributes.size, attributes.material);

        return attributes;
    }

    torch::jit::script::Module detection_model_;
    torch::jit::script::Module segmentation_model_;
    torch::jit::script::Module pose_estimator_;

    IsaacPerceptionPipeline perception_pipeline_;
    CameraIntrinsics camera_intrinsics_;
    DetectionParameters detection_params_;
};
```

### 3D Object Pose Estimation

Accurate pose estimation is crucial for manipulation:

```cpp
// 3D pose estimation system
class PoseEstimator
{
public:
    Pose3D estimateObjectPose(const BoundingBox& bbox,
                            const SegmentationMask& mask,
                            const sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
    {
        // Get 2D keypoints from the object
        auto keypoints_2d = detectKeypoints(bbox, mask);

        // Get 3D model of the object
        auto model_3d = getModel3D(bbox.class_name);

        if (!model_3d.valid) {
            // Use template-based estimation
            return estimatePoseFromTemplate(bbox, camera_info);
        }

        // Use PnP (Perspective-n-Point) algorithm
        auto pose = solvePnP(keypoints_2d, model_3d, camera_info);

        return pose;
    }

private:
    std::vector<Point2D> detectKeypoints(const BoundingBox& bbox, const SegmentationMask& mask)
    {
        // Extract keypoints from the segmented object region
        cv::Mat object_region = extractRegionFromMask(bbox, mask);

        // Use SIFT, ORB, or learned keypoints
        std::vector<cv::KeyPoint> keypoints;
        cv::Ptr<cv::SIFT> detector = cv::SIFT::create();
        detector->detect(object_region, keypoints);

        // Convert to Point2D format
        std::vector<Point2D> points_2d;
        for (const auto& kp : keypoints) {
            points_2d.push_back({kp.pt.x + bbox.xmin, kp.pt.y + bbox.ymin});
        }

        return points_2d;
    }

    ObjectModel3D getModel3D(const std::string& class_name)
    {
        // Load precomputed 3D model for the object class
        static std::map<std::string, ObjectModel3D> model_cache;

        auto it = model_cache.find(class_name);
        if (it != model_cache.end()) {
            return it->second;
        }

        // Load model from database
        ObjectModel3D model = loadModel3D(class_name);
        model_cache[class_name] = model;

        return model;
    }

    Pose3D solvePnP(const std::vector<Point2D>& points_2d,
                   const ObjectModel3D& model_3d,
                   const sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
    {
        // Prepare 2D and 3D point vectors
        std::vector<cv::Point2f> cv_points_2d;
        std::vector<cv::Point3f> cv_points_3d;

        for (size_t i = 0; i < points_2d.size(); ++i) {
            cv_points_2d.push_back({points_2d[i].x, points_2d[i].y});
        }

        for (const auto& pt_3d : model_3d.keypoints) {
            cv_points_3d.push_back({pt_3d.x, pt_3d.y, pt_3d.z});
        }

        // Camera intrinsic matrix
        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
            camera_info->k[0], 0, camera_info->k[2],
            0, camera_info->k[4], camera_info->k[5],
            0, 0, 1);

        // Distortion coefficients
        cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);
        for (int i = 0; i < 4; ++i) {
            dist_coeffs.at<double>(i, 0) = camera_info->d[i];
        }

        // Solve PnP
        cv::Mat rvec, tvec;
        bool success = cv::solvePnP(cv_points_3d, cv_points_2d, camera_matrix, dist_coeffs, rvec, tvec);

        if (!success) {
            throw std::runtime_error("PnP solution failed");
        }

        // Convert to Pose3D format
        Pose3D pose;
        pose.translation.x = tvec.at<double>(0);
        pose.translation.y = tvec.at<double>(1);
        pose.translation.z = tvec.at<double>(2);

        // Convert rotation vector to quaternion
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);

        cv::Mat R = rotation_matrix;
        double w = sqrt(1.0 + R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2)) / 2.0;
        double x = (R.at<double>(2,1) - R.at<double>(1,2)) / (4 * w);
        double y = (R.at<double>(0,2) - R.at<double>(2,0)) / (4 * w);
        double z = (R.at<double>(1,0) - R.at<double>(0,1)) / (4 * w);

        pose.rotation.w = w;
        pose.rotation.x = x;
        pose.rotation.y = y;
        pose.rotation.z = z;

        return pose;
    }

    Pose3D estimatePoseFromTemplate(const BoundingBox& bbox,
                                  const sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
    {
        // Use template-based pose estimation for unknown objects
        Pose3D pose;

        // Estimate distance from size in image
        double avg_focal_length = (camera_info->k[0] + camera_info->k[4]) / 2.0;
        double object_size_real = ESTIMATED_OBJECT_SIZE; // meters
        double object_size_pixels = std::max(bbox.width, bbox.height); // pixels

        pose.translation.z = (avg_focal_length * object_size_real) / object_size_pixels;

        // Calculate X, Y from image coordinates
        double center_x = (bbox.xmin + bbox.xmax) / 2.0;
        double center_y = (bbox.ymin + bbox.ymax) / 2.0;

        pose.translation.x = (center_x - camera_info->k[2]) * pose.translation.z / camera_info->k[0];
        pose.translation.y = (center_y - camera_info->k[5]) * pose.translation.z / camera_info->k[4];

        // Default orientation (facing camera)
        pose.rotation.w = 1.0;
        pose.rotation.x = 0.0;
        pose.rotation.y = 0.0;
        pose.rotation.z = 0.0;

        return pose;
    }

    static constexpr double ESTIMATED_OBJECT_SIZE = 0.1; // 10cm average object size
};
```

## Object Affordance Recognition

### Affordance-Based Understanding

The system recognizes object affordances to understand potential interactions:

```cpp
// Affordance recognition system
class AffordanceRecognizer
{
public:
    ObjectAffordances recognizeAffordances(const ObjectDetection& detection)
    {
        ObjectAffordances affordances;

        // Determine affordances based on object class and attributes
        std::string object_class = detection.class_name;
        ObjectAttributes attributes = detection.attributes;

        // Define affordance rules
        if (object_class == "cup" || object_class == "glass") {
            affordances.push_back(AffordanceType::CONTAIN_LIQUID);
            affordances.push_back(AffordanceType::GRASP_HANDLE);
            affordances.push_back(AffordanceType::POUR);
        } else if (object_class == "bottle") {
            affordances.push_back(AffordanceType::CONTAIN_LIQUID);
            affordances.push_back(AffordanceType::GRASP_BODY);
            affordances.push_back(AffordanceType::POUR);
        } else if (object_class == "book") {
            affordances.push_back(AffordanceType::GRASP_EDGE);
            affordances.push_back(AffordanceType::PLACE_FLAT);
        } else if (object_class == "box") {
            affordances.push_back(AffordanceType::GRASP_CORNERS);
            affordances.push_back(AffordanceType::PLACE_ON_TOP);
        } else if (object_class == "mug") {
            affordances.push_back(AffordanceType::CONTAIN_LIQUID);
            affordances.push_back(AffordanceType::GRASP_HANDLE);
        }

        // Add affordances based on attributes
        if (attributes.size.volume < 0.001) { // Less than 1 liter
            affordances.push_back(AffordanceType::EASY_TO_LIFT);
        }

        if (attributes.material == "metal" || attributes.material == "plastic") {
            affordances.push_back(AffordanceType::WASHABLE);
        }

        // Filter affordances based on robot capabilities
        affordances = filterAffordancesByRobotCapabilities(affordances);

        return affordances;
    }

    GraspConfiguration suggestGrasp(const ObjectDetection& detection,
                                  const ObjectAffordances& affordances)
    {
        GraspConfiguration grasp;

        std::string object_class = detection.class_name;
        BoundingBox bbox = detection.bounding_box;
        Pose3D pose = detection.pose;

        // Determine best grasp based on object affordances
        if (hasAffordance(affordances, AffordanceType::GRASP_HANDLE)) {
            // Find handle location and suggest handle grasp
            auto handle_location = findHandleLocation(detection);
            grasp = suggestHandleGrasp(pose, handle_location);
        } else if (hasAffordance(affordances, AffordanceType::GRASP_CORNERS)) {
            // Suggest corner grasp for box-like objects
            grasp = suggestCornerGrasp(pose, bbox);
        } else if (hasAffordance(affordances, AffordanceType::GRASP_EDGE)) {
            // Suggest edge grasp for flat objects
            grasp = suggestEdgeGrasp(pose, bbox);
        } else {
            // Default grasp for unknown objects
            grasp = suggestTopGrasp(pose, bbox);
        }

        return grasp;
    }

private:
    bool hasAffordance(const ObjectAffordances& affordances, AffordanceType type)
    {
        return std::find(affordances.begin(), affordances.end(), type) != affordances.end();
    }

    GraspConfiguration suggestHandleGrasp(const Pose3D& object_pose, const Point3D& handle_location)
    {
        GraspConfiguration grasp;
        grasp.type = GraspType::HANDLE_GRASP;

        // Calculate grasp pose relative to handle
        grasp.pose = object_pose;
        grasp.pose.translation = handle_location; // Adjust to handle location

        // Orient gripper to grasp handle
        grasp.pose.rotation = calculateHandleGraspOrientation(handle_location, object_pose);

        // Set grasp parameters
        grasp.approach_direction = {0, 0, -1}; // Approach from above
        grasp.grasp_width = calculateGraspWidth(0.02); // 2cm for handle
        grasp.grasp_force = 5.0; // Moderate force for handle

        return grasp;
    }

    GraspConfiguration suggestCornerGrasp(const Pose3D& object_pose, const BoundingBox& bbox)
    {
        GraspConfiguration grasp;
        grasp.type = GraspType::CORNER_GRASP;

        // Calculate corner position
        double corner_x = object_pose.translation.x + (bbox.width / 2.0) * 0.8; // 80% of width
        double corner_y = object_pose.translation.y + (bbox.height / 2.0) * 0.8; // 80% of height
        double corner_z = object_pose.translation.z;

        grasp.pose.translation = {corner_x, corner_y, corner_z};
        grasp.pose.rotation = object_pose.rotation;

        // Orient gripper for corner grasp
        grasp.approach_direction = {0, 0, -1};
        grasp.grasp_width = calculateGraspWidth(bbox.width * 0.3); // 30% of object width
        grasp.grasp_force = 10.0; // Higher force for corner grasp

        return grasp;
    }

    GraspConfiguration suggestTopGrasp(const Pose3D& object_pose, const BoundingBox& bbox)
    {
        GraspConfiguration grasp;
        grasp.type = GraspType::TOP_GRASP;

        // Grasp from top center
        grasp.pose.translation = object_pose.translation;
        grasp.pose.translation.z += bbox.height / 2.0 + GRASP_APPROACH_DISTANCE;

        // Orient gripper for top grasp
        grasp.pose.rotation = object_pose.rotation;
        // Ensure gripper is oriented properly for top grasp
        grasp.pose.rotation = alignForTopGrasp(grasp.pose.rotation);

        grasp.approach_direction = {0, 0, -1};
        grasp.grasp_width = calculateGraspWidth(bbox.width * 0.7); // 70% of object width
        grasp.grasp_force = 8.0; // Moderate force for top grasp

        return grasp;
    }

    double calculateGraspWidth(double object_width)
    {
        // Calculate appropriate grasp width with safety margin
        double required_width = object_width * 1.2; // 20% safety margin
        return std::min(required_width, MAX_GRASP_WIDTH);
    }

    ObjectAffordances filterAffordancesByRobotCapabilities(const ObjectAffordances& affordances)
    {
        ObjectAffordances filtered;

        for (const auto& affordance : affordances) {
            if (robot_can_perform(affordance)) {
                filtered.push_back(affordance);
            }
        }

        return filtered;
    }

    bool robot_can_perform(AffordanceType affordance)
    {
        // Check if robot has capability to perform the affordance
        switch (affordance) {
            case AffordanceType::GRASP_HANDLE:
            case AffordanceType::GRASP_CORNERS:
            case AffordanceType::GRASP_EDGE:
                return robot_has_gripper();
            case AffordanceType::POUR:
                return robot_has_pouring_capability();
            case AffordanceType::CONTAIN_LIQUID:
                return robot_has_container_capability();
            default:
                return true;
        }
    }

    static constexpr double GRASP_APPROACH_DISTANCE = 0.05; // 5cm approach distance
    static constexpr double MAX_GRASP_WIDTH = 0.1; // 10cm max grasp width
};
```

## Grasp Planning and Execution

### Advanced Grasp Planning

The system plans grasps considering multiple factors:

```cpp
// Advanced grasp planning system
class GraspPlanner
{
public:
    GraspPlanner()
    {
        // Initialize grasp planning components
        grasp_generator_ = std::make_unique<GraspGenerator>();
        grasp_evaluator_ = std::make_unique<GraspEvaluator>();
        grasp_optimizer_ = std::make_unique<GraspOptimizer>();
    }

    std::vector<GraspConfiguration> planGrasps(const ObjectDetection& object,
                                             const RobotState& robot_state)
    {
        // Generate candidate grasps
        auto candidates = grasp_generator_->generateGrasps(object, robot_state);

        // Evaluate grasp quality
        auto evaluated = grasp_evaluator_->evaluateGrasps(candidates, object, robot_state);

        // Optimize grasp selection
        auto optimized = grasp_optimizer_->optimizeGrasps(evaluated, robot_state);

        return optimized;
    }

    GraspConfiguration selectBestGrasp(const std::vector<GraspConfiguration>& grasps,
                                     const ObjectDetection& object,
                                     const RobotState& robot_state)
    {
        if (grasps.empty()) {
            throw std::runtime_error("No valid grasps found");
        }

        // Score each grasp based on multiple criteria
        std::vector<GraspScore> scores;
        for (const auto& grasp : grasps) {
            GraspScore score = calculateGraspScore(grasp, object, robot_state);
            scores.push_back(score);
        }

        // Find grasp with highest score
        auto best_it = std::max_element(scores.begin(), scores.end(),
                                      [](const GraspScore& a, const GraspScore& b) {
                                          return a.total_score < b.total_score;
                                      });

        return grasps[std::distance(scores.begin(), best_it)];
    }

private:
    struct GraspScore {
        double stability_score;
        double accessibility_score;
        double safety_score;
        double efficiency_score;
        double total_score;

        GraspScore() : stability_score(0), accessibility_score(0),
                      safety_score(0), efficiency_score(0), total_score(0) {}
    };

    GraspScore calculateGraspScore(const GraspConfiguration& grasp,
                                 const ObjectDetection& object,
                                 const RobotState& robot_state)
    {
        GraspScore score;

        // Calculate stability score
        score.stability_score = calculateStabilityScore(grasp, object);

        // Calculate accessibility score
        score.accessibility_score = calculateAccessibilityScore(grasp, robot_state);

        // Calculate safety score
        score.safety_score = calculateSafetyScore(grasp, robot_state);

        // Calculate efficiency score
        score.efficiency_score = calculateEfficiencyScore(grasp, robot_state);

        // Combine scores with weights
        score.total_score = STABILITY_WEIGHT * score.stability_score +
                          ACCESSIBILITY_WEIGHT * score.accessibility_score +
                          SAFETY_WEIGHT * score.safety_score +
                          EFFICIENCY_WEIGHT * score.efficiency_score;

        return score;
    }

    double calculateStabilityScore(const GraspConfiguration& grasp, const ObjectDetection& object)
    {
        // Calculate grasp stability based on contact points and object properties
        double stability = 0.0;

        // Check if grasp points are on object surface
        if (!isGraspOnObjectSurface(grasp, object)) {
            return 0.0;
        }

        // Calculate grasp quality based on contact points distribution
        auto contact_points = calculateContactPoints(grasp);
        stability = calculateGraspQuality(contact_points, object.attributes);

        // Consider object weight and center of mass
        double weight_factor = std::max(0.1, 1.0 - (object.attributes.weight / MAX_GRASPABLE_WEIGHT));
        stability *= weight_factor;

        return std::min(1.0, stability);
    }

    double calculateAccessibilityScore(const GraspConfiguration& grasp, const RobotState& robot_state)
    {
        // Calculate how accessible the grasp pose is from current robot configuration
        double accessibility = 0.0;

        // Check if grasp pose is in reachable workspace
        if (!isInReachableWorkspace(grasp.pose, robot_state)) {
            return 0.0;
        }

        // Calculate inverse of distance to grasp pose
        double distance = calculateDistance(robot_state.end_effector_pose, grasp.pose);
        accessibility = 1.0 / (1.0 + distance);

        // Consider joint configuration quality
        auto joint_config = calculateJointConfigurationForPose(grasp.pose);
        double configuration_quality = calculateConfigurationQuality(joint_config);

        return (accessibility + configuration_quality) / 2.0;
    }

    double calculateSafetyScore(const GraspConfiguration& grasp, const RobotState& robot_state)
    {
        // Calculate safety of the grasp execution
        double safety = 1.0;

        // Check for collision with environment
        if (wouldCollideWithEnvironment(grasp, robot_state)) {
            safety = 0.0;
        }

        // Check for self-collision
        if (wouldSelfCollide(grasp, robot_state)) {
            safety = 0.0;
        }

        // Consider approach trajectory safety
        auto approach_trajectory = calculateApproachTrajectory(grasp, robot_state);
        if (hasCollisionInTrajectory(approach_trajectory)) {
            safety *= 0.5; // Reduce safety if trajectory has potential collisions
        }

        return safety;
    }

    double calculateEfficiencyScore(const GraspConfiguration& grasp, const RobotState& robot_state)
    {
        // Calculate efficiency of the grasp execution
        double efficiency = 0.0;

        // Consider trajectory length
        auto trajectory = calculateGraspTrajectory(grasp, robot_state);
        double trajectory_length = calculateTrajectoryLength(trajectory);
        efficiency = 1.0 / (1.0 + trajectory_length);

        // Consider execution time
        double execution_time = estimateGraspExecutionTime(grasp);
        efficiency += 1.0 / (1.0 + execution_time);

        return efficiency / 2.0; // Normalize
    }

    std::unique_ptr<GraspGenerator> grasp_generator_;
    std::unique_ptr<GraspEvaluator> grasp_evaluator_;
    std::unique_ptr<GraspOptimizer> grasp_optimizer_;

    static constexpr double STABILITY_WEIGHT = 0.4;
    static constexpr double ACCESSIBILITY_WEIGHT = 0.3;
    static constexpr double SAFETY_WEIGHT = 0.2;
    static constexpr double EFFICIENCY_WEIGHT = 0.1;
    static constexpr double MAX_GRASPABLE_WEIGHT = 2.0; // 2kg maximum
};
```

### Grasp Generator Implementation

The grasp generator creates candidate grasps:

```cpp
// Grasp generation system
class GraspGenerator
{
public:
    std::vector<GraspConfiguration> generateGrasps(const ObjectDetection& object,
                                                 const RobotState& robot_state)
    {
        std::vector<GraspConfiguration> grasps;

        // Generate grasps based on object affordances
        auto affordances = recognizeObjectAffordances(object);

        for (const auto& affordance : affordances) {
            auto affordance_grasps = generateGraspsForAffordance(object, affordance, robot_state);
            grasps.insert(grasps.end(), affordance_grasps.begin(), affordance_grasps.end());
        }

        // Generate generic grasps for unknown objects
        if (grasps.empty()) {
            auto generic_grasps = generateGenericGrasps(object, robot_state);
            grasps.insert(grasps.end(), generic_grasps.begin(), generic_grasps.end());
        }

        return grasps;
    }

private:
    std::vector<GraspConfiguration> generateGraspsForAffordance(const ObjectDetection& object,
                                                              AffordanceType affordance,
                                                              const RobotState& robot_state)
    {
        std::vector<GraspConfiguration> grasps;

        switch (affordance) {
            case AffordanceType::GRASP_HANDLE:
                grasps = generateHandleGrasps(object, robot_state);
                break;
            case AffordanceType::GRASP_CORNERS:
                grasps = generateCornerGrasps(object, robot_state);
                break;
            case AffordanceType::GRASP_EDGE:
                grasps = generateEdgeGrasps(object, robot_state);
                break;
            default:
                // Generate general grasps
                grasps = generateGeneralGrasps(object, robot_state);
                break;
        }

        return grasps;
    }

    std::vector<GraspConfiguration> generateHandleGrasps(const ObjectDetection& object,
                                                       const RobotState& robot_state)
    {
        std::vector<GraspConfiguration> grasps;

        // Find handle location from object segmentation
        auto handle_points = findHandlePoints(object);

        for (const auto& handle_point : handle_points) {
            // Generate multiple grasp orientations around the handle
            for (double angle = 0; angle < 2 * M_PI; angle += M_PI_4) {
                GraspConfiguration grasp;
                grasp.type = GraspType::HANDLE_GRASP;

                // Position at handle location
                grasp.pose.translation = handle_point.position;

                // Orient gripper to grasp handle
                grasp.pose.rotation = calculateHandleOrientation(handle_point.normal, angle);

                // Set grasp parameters
                grasp.approach_direction = handle_point.normal;
                grasp.grasp_width = calculateHandleGraspWidth(object.attributes);
                grasp.grasp_force = 5.0;

                // Check if grasp is feasible
                if (isGraspFeasible(grasp, object, robot_state)) {
                    grasps.push_back(grasp);
                }
            }
        }

        return grasps;
    }

    std::vector<GraspConfiguration> generateCornerGrasps(const ObjectDetection& object,
                                                       const RobotState& robot_state)
    {
        std::vector<GraspConfiguration> grasps;

        // Find corner points of the object
        auto corner_points = findObjectCorners(object);

        for (const auto& corner : corner_points) {
            // Generate corner grasp from different directions
            std::vector<Point3D> approach_directions = {
                {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
            };

            for (const auto& approach_dir : approach_directions) {
                GraspConfiguration grasp;
                grasp.type = GraspType::CORNER_GRASP;

                // Position at corner with approach offset
                grasp.pose.translation = corner;
                grasp.pose.translation.x += approach_dir.x * CORNER_APPROACH_DISTANCE;
                grasp.pose.translation.y += approach_dir.y * CORNER_APPROACH_DISTANCE;
                grasp.pose.translation.z += approach_dir.z * CORNER_APPROACH_DISTANCE;

                // Orient gripper to approach from the specified direction
                grasp.pose.rotation = calculateApproachOrientation(approach_dir);

                grasp.approach_direction = approach_dir;
                grasp.grasp_width = calculateCornerGraspWidth(object.attributes);
                grasp.grasp_force = 10.0;

                if (isGraspFeasible(grasp, object, robot_state)) {
                    grasps.push_back(grasp);
                }
            }
        }

        return grasps;
    }

    std::vector<GraspConfiguration> generateEdgeGrasps(const ObjectDetection& object,
                                                     const RobotState& robot_state)
    {
        std::vector<GraspConfiguration> grasps;

        // Find edge points of the object
        auto edge_points = findObjectEdges(object);

        for (const auto& edge_point : edge_points) {
            // Generate edge grasps along the edge
            for (double offset = -0.02; offset <= 0.02; offset += 0.01) {
                GraspConfiguration grasp;
                grasp.type = GraspType::EDGE_GRASP;

                // Position along the edge
                grasp.pose.translation = edge_point.position;
                grasp.pose.translation.x += edge_point.tangent.x * offset;
                grasp.pose.translation.y += edge_point.tangent.y * offset;
                grasp.pose.translation.z += edge_point.tangent.z * offset;

                // Orient gripper perpendicular to the edge
                grasp.pose.rotation = calculateEdgeGraspOrientation(edge_point.normal, edge_point.tangent);

                grasp.approach_direction = edge_point.normal;
                grasp.grasp_width = calculateEdgeGraspWidth(object.attributes);
                grasp.grasp_force = 8.0;

                if (isGraspFeasible(grasp, object, robot_state)) {
                    grasps.push_back(grasp);
                }
            }
        }

        return grasps;
    }

    bool isGraspFeasible(const GraspConfiguration& grasp,
                        const ObjectDetection& object,
                        const RobotState& robot_state)
    {
        // Check if grasp is geometrically feasible
        if (!isInWorkspace(grasp.pose)) {
            return false;
        }

        // Check if grasp pose is reachable
        if (!canReachPose(grasp.pose, robot_state)) {
            return false;
        }

        // Check if grasp parameters are within limits
        if (grasp.grasp_width > MAX_GRIPPER_WIDTH) {
            return false;
        }

        if (object.attributes.weight > MAX_GRASPABLE_WEIGHT) {
            return false;
        }

        return true;
    }

    static constexpr double CORNER_APPROACH_DISTANCE = 0.05; // 5cm
    static constexpr double MAX_GRIPPER_WIDTH = 0.1; // 10cm
    static constexpr double MAX_GRASPABLE_WEIGHT = 2.0; // 2kg
};
```

## Manipulation Execution and Control

### Grasp Execution Framework

The system executes grasps with precise control:

```cpp
// Grasp execution framework
class GraspExecutionSystem
{
public:
    GraspExecutionSystem(rclcpp::Node* node)
    {
        // Initialize manipulation action clients
        gripper_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
            node, "gripper_controller/gripper_cmd");

        arm_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            node, "arm_controller/follow_joint_trajectory");

        // Initialize force/torque sensors
        ft_sensor_subscriber_ = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "wrench", 10,
            std::bind(&GraspExecutionSystem::ftCallback, this, std::placeholders::_1));

        // Initialize tactile sensors
        tactile_subscriber_ = node->create_subscription<sensor_msgs::msg::JointState>(
            "tactile_sensors", 10,
            std::bind(&GraspExecutionSystem::tactileCallback, this, std::placeholders::_1));
    }

    ExecutionResult executeGrasp(const GraspConfiguration& grasp_config,
                               const ObjectDetection& target_object)
    {
        ExecutionResult result;

        try {
            // Move to pre-grasp position
            result = movePreGrasp(grasp_config);
            if (!result.success) {
                return result;
            }

            // Execute grasp approach
            result = executeApproach(grasp_config);
            if (!result.success) {
                return result;
            }

            // Close gripper
            result = closeGripper(grasp_config.grasp_force, target_object.attributes);
            if (!result.success) {
                return result;
            }

            // Verify grasp success
            result = verifyGraspSuccess(target_object);
            if (!result.success) {
                // Attempt recovery if grasp failed
                result = attemptGraspRecovery(grasp_config, target_object);
            }

            // Lift object if grasp was successful
            if (result.success) {
                result = liftObject();
            }

        } catch (const std::exception& e) {
            result.success = false;
            result.error_message = std::string("Grasp execution failed: ") + e.what();
        }

        return result;
    }

private:
    ExecutionResult movePreGrasp(const GraspConfiguration& grasp_config)
    {
        ExecutionResult result;

        // Calculate pre-grasp pose (approach distance above grasp pose)
        auto pre_grasp_pose = grasp_config.pose;
        pre_grasp_pose.translation.z += PRE_GRASP_DISTANCE;

        // Plan and execute trajectory to pre-grasp pose
        auto trajectory = planTrajectoryToPose(pre_grasp_pose);

        if (trajectory.empty()) {
            result.success = false;
            result.error_message = "Could not plan trajectory to pre-grasp pose";
            return result;
        }

        // Execute trajectory
        auto trajectory_future = arm_client_->async_send_goal(trajectory);
        if (trajectory_future.wait_for(std::chrono::seconds(30)) == std::future_status::timeout) {
            result.success = false;
            result.error_message = "Pre-grasp trajectory execution timed out";
            return result;
        }

        auto trajectory_handle = trajectory_future.get();
        if (!trajectory_handle) {
            result.success = false;
            result.error_message = "Pre-grasp trajectory was rejected";
            return result;
        }

        // Wait for trajectory completion
        auto result_future = arm_client_->async_get_result(trajectory_handle);
        if (result_future.wait_for(std::chrono::seconds(60)) == std::future_status::timeout) {
            result.success = false;
            result.error_message = "Pre-grasp trajectory result timed out";
            return result;
        }

        auto trajectory_result = result_future.get();
        result.success = trajectory_result.result->success;
        result.error_message = trajectory_result.result->error_string;

        return result;
    }

    ExecutionResult executeApproach(const GraspConfiguration& grasp_config)
    {
        ExecutionResult result;

        // Plan trajectory from pre-grasp to grasp pose
        auto approach_trajectory = planApproachTrajectory(grasp_config);

        // Execute with force control to handle contact
        auto trajectory_future = arm_client_->async_send_goal(approach_trajectory);
        auto trajectory_handle = trajectory_future.get();

        if (!trajectory_handle) {
            result.success = false;
            result.error_message = "Approach trajectory was rejected";
            return result;
        }

        // Monitor force during approach to detect contact
        auto start_time = std::chrono::steady_clock::now();
        auto timeout_time = start_time + std::chrono::seconds(10);

        while (std::chrono::steady_clock::now() < timeout_time) {
            // Check force/torque readings
            if (hasContactDetected()) {
                // Contact detected, stop approach
                cancelTrajectory(trajectory_handle);
                result.success = true;
                return result;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Timeout without contact - object might not be there
        cancelTrajectory(trajectory_handle);
        result.success = false;
        result.error_message = "Approach completed without contact detection";
        return result;
    }

    ExecutionResult closeGripper(double force, const ObjectAttributes& attributes)
    {
        ExecutionResult result;

        // Adjust force based on object properties
        double adjusted_force = adjustGraspForce(force, attributes);

        // Create gripper command
        control_msgs::action::GripperCommand::Goal gripper_goal;
        gripper_goal.command.position = calculateGripperPosition(attributes.size.width);
        gripper_goal.command.max_effort = adjusted_force;

        // Send gripper command
        auto gripper_future = gripper_client_->async_send_goal(gripper_goal);
        auto gripper_handle = gripper_future.get();

        if (!gripper_handle) {
            result.success = false;
            result.error_message = "Gripper command was rejected";
            return result;
        }

        // Wait for gripper execution
        auto result_future = gripper_client_->async_get_result(gripper_handle);
        if (result_future.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
            result.success = false;
            result.error_message = "Gripper command timed out";
            return result;
        }

        auto gripper_result = result_future.get();
        result.success = gripper_result.result->reached_goal;
        result.error_message = gripper_result.result->error_string;

        return result;
    }

    ExecutionResult verifyGraspSuccess(const ObjectDetection& target_object)
    {
        ExecutionResult result;

        // Check force/torque readings for grasp confirmation
        auto current_force = getCurrentForce();
        if (current_force < MIN_GRASP_FORCE) {
            result.success = false;
            result.error_message = "Insufficient grasp force detected";
            return result;
        }

        // Check tactile sensors for contact
        if (!hasTactileContact()) {
            result.success = false;
            result.error_message = "No tactile contact detected";
            return result;
        }

        // Optionally verify with vision (re-detect object)
        if (USE_VISION_VERIFICATION) {
            auto re_detection = reDetectObject(target_object);
            if (re_detection.confidence < MIN_REDETECTION_CONFIDENCE) {
                result.success = false;
                result.error_message = "Object no longer detected after grasp";
                return result;
            }
        }

        result.success = true;
        return result;
    }

    ExecutionResult attemptGraspRecovery(const GraspConfiguration& grasp_config,
                                       const ObjectDetection& target_object)
    {
        RCLCPP_WARN(rclcpp::get_logger("manipulation"), "Attempting grasp recovery");

        // Try alternative grasp approach
        auto alternative_grasp = generateAlternativeGrasp(grasp_config, target_object);

        if (alternative_grasp.valid) {
            return executeGrasp(alternative_grasp, target_object);
        }

        // Try different grasp force
        auto force_adjusted_grasp = adjustGraspConfiguration(grasp_config, "force");
        if (force_adjusted_grasp.valid) {
            return executeGrasp(force_adjusted_grasp, target_object);
        }

        // Try different grasp width
        auto width_adjusted_grasp = adjustGraspConfiguration(grasp_config, "width");
        if (width_adjusted_grasp.valid) {
            return executeGrasp(width_adjusted_grasp, target_object);
        }

        // All recovery attempts failed
        ExecutionResult result;
        result.success = false;
        result.error_message = "All grasp recovery attempts failed";
        return result;
    }

    double adjustGraspForce(double base_force, const ObjectAttributes& attributes)
    {
        // Adjust force based on object weight and material
        double weight_factor = std::min(2.0, attributes.weight / 0.5); // Scale with weight
        double material_factor = getMaterialFactor(attributes.material);

        return base_force * weight_factor * material_factor;
    }

    double getMaterialFactor(const std::string& material)
    {
        if (material == "plastic") return 0.8;  // Lower force for plastic
        if (material == "metal") return 1.2;    // Higher force for metal
        if (material == "glass") return 0.9;    // Careful with glass
        if (material == "wood") return 1.0;     // Standard for wood
        return 1.0; // Default
    }

    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_client_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr arm_client_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_sensor_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr tactile_subscriber_;

    // Force/torque sensor data
    geometry_msgs::msg::WrenchStamped current_wrench_;
    sensor_msgs::msg::JointState tactile_data_;

    static constexpr double PRE_GRASP_DISTANCE = 0.05; // 5cm above grasp pose
    static constexpr double MIN_GRASP_FORCE = 2.0; // 2N minimum grasp force
    static constexpr bool USE_VISION_VERIFICATION = true;
    static constexpr double MIN_REDETECTION_CONFIDENCE = 0.7;

    void ftCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        current_wrench_ = *msg;
    }

    void tactileCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        tactile_data_ = *msg;
    }

    bool hasContactDetected()
    {
        // Check if force exceeds threshold
        double force_magnitude = sqrt(pow(current_wrench_.wrench.force.x, 2) +
                                    pow(current_wrench_.wrench.force.y, 2) +
                                    pow(current_wrench_.wrench.force.z, 2));
        return force_magnitude > CONTACT_FORCE_THRESHOLD;
    }

    bool hasTactileContact()
    {
        // Check tactile sensor readings for contact
        for (const auto& effort : tactile_data_.effort) {
            if (abs(effort) > TACTILE_THRESHOLD) {
                return true;
            }
        }
        return false;
    }

    static constexpr double CONTACT_FORCE_THRESHOLD = 1.0; // 1N threshold
    static constexpr double TACTILE_THRESHOLD = 0.1; // Tactile threshold
};
```

## Object Manipulation Strategies

### Advanced Manipulation Techniques

The system implements various manipulation strategies:

```cpp
// Advanced manipulation strategies
class ManipulationStrategies
{
public:
    enum class ManipulationStrategy {
        PRECISE_GRASP,
        POWER_GRASP,
        PINCER_GRASP,
        SUCTION_GRASP,
        ADAPTIVE_GRASP
    };

    ManipulationResult executeManipulation(const ManipulationTask& task,
                                         const ObjectDetection& object,
                                         const RobotState& robot_state)
    {
        ManipulationResult result;

        // Select appropriate strategy based on object properties
        auto strategy = selectStrategy(task, object);

        switch (strategy) {
            case ManipulationStrategy::PRECISE_GRASP:
                result = executePreciseGrasp(task, object, robot_state);
                break;
            case ManipulationStrategy::POWER_GRASP:
                result = executePowerGrasp(task, object, robot_state);
                break;
            case ManipulationStrategy::PINCER_GRASP:
                result = executePincerGrasp(task, object, robot_state);
                break;
            case ManipulationStrategy::SUCTION_GRASP:
                result = executeSuctionGrasp(task, object, robot_state);
                break;
            case ManipulationStrategy::ADAPTIVE_GRASP:
                result = executeAdaptiveGrasp(task, object, robot_state);
                break;
        }

        return result;
    }

private:
    ManipulationStrategy selectStrategy(const ManipulationTask& task,
                                      const ObjectDetection& object)
    {
        // Select strategy based on object properties and task requirements
        if (object.attributes.material == "fragile" || object.attributes.weight < 0.1) {
            return ManipulationStrategy::PRECISE_GRASP;
        } else if (object.attributes.weight > 0.5 || object.attributes.material == "heavy") {
            return ManipulationStrategy::POWER_GRASP;
        } else if (object.class_name == "small_object" || object.attributes.size.width < 0.02) {
            return ManipulationStrategy::PINCER_GRASP;
        } else if (object.class_name == "flat_object" || object.attributes.shape == "flat") {
            return ManipulationStrategy::SUCTION_GRASP;
        } else {
            return ManipulationStrategy::ADAPTIVE_GRASP;
        }
    }

    ManipulationResult executePreciseGrasp(const ManipulationTask& task,
                                         const ObjectDetection& object,
                                         const RobotState& robot_state)
    {
        ManipulationResult result;

        // Use precision grasp with low force
        GraspConfiguration grasp;
        grasp.type = GraspType::PRECISION_GRASP;
        grasp.grasp_force = 2.0; // Low force for precision
        grasp.grasp_width = object.attributes.size.width * 1.1; // Slightly wider than object

        // Calculate precise grasp pose
        grasp.pose = calculatePreciseGraspPose(object);

        // Execute the grasp
        auto grasp_result = grasp_execution_system_->executeGrasp(grasp, object);

        if (grasp_result.success) {
            // Execute manipulation task
            result = executeTaskWithGrasp(task, grasp, object);
        } else {
            result.success = false;
            result.error_message = "Precision grasp failed: " + grasp_result.error_message;
        }

        return result;
    }

    ManipulationResult executePowerGrasp(const ManipulationTask& task,
                                       const ObjectDetection& object,
                                       const RobotState& robot_state)
    {
        ManipulationResult result;

        // Use power grasp with high force for heavy objects
        GraspConfiguration grasp;
        grasp.type = GraspType::POWER_GRASP;
        grasp.grasp_force = std::min(20.0, object.attributes.weight * 10.0); // Scale with weight
        grasp.grasp_width = object.attributes.size.width * 0.8; // Firm grip

        // Calculate power grasp pose
        grasp.pose = calculatePowerGraspPose(object);

        // Execute the grasp
        auto grasp_result = grasp_execution_system_->executeGrasp(grasp, object);

        if (grasp_result.success) {
            // Execute manipulation task
            result = executeTaskWithGrasp(task, grasp, object);
        } else {
            result.success = false;
            result.error_message = "Power grasp failed: " + grasp_result.error_message;
        }

        return result;
    }

    ManipulationResult executeAdaptiveGrasp(const ManipulationTask& task,
                                          const ObjectDetection& object,
                                          const RobotState& robot_state)
    {
        ManipulationResult result;

        // Generate multiple grasp candidates and select the best one
        auto grasp_planner = std::make_unique<GraspPlanner>();
        auto candidates = grasp_planner->planGrasps(object, robot_state);

        if (candidates.empty()) {
            result.success = false;
            result.error_message = "No valid grasps found for adaptive grasp";
            return result;
        }

        // Select best grasp based on current conditions
        auto best_grasp = grasp_planner->selectBestGrasp(candidates, object, robot_state);

        // Execute the selected grasp
        auto grasp_result = grasp_execution_system_->executeGrasp(best_grasp, object);

        if (grasp_result.success) {
            // Execute manipulation task
            result = executeTaskWithGrasp(task, best_grasp, object);
        } else {
            result.success = false;
            result.error_message = "Adaptive grasp failed: " + grasp_result.error_message;
        }

        return result;
    }

    ManipulationResult executeTaskWithGrasp(const ManipulationTask& task,
                                          const GraspConfiguration& grasp,
                                          const ObjectDetection& object)
    {
        ManipulationResult result;

        switch (task.type) {
            case TaskType::LIFT:
                result = executeLiftTask(task, grasp, object);
                break;
            case TaskType::PLACE:
                result = executePlaceTask(task, grasp, object);
                break;
            case TaskType::TRANSPORT:
                result = executeTransportTask(task, grasp, object);
                break;
            case TaskType::POUR:
                result = executePourTask(task, grasp, object);
                break;
            default:
                result.success = false;
                result.error_message = "Unknown task type";
                break;
        }

        return result;
    }

    std::unique_ptr<GraspExecutionSystem> grasp_execution_system_;
};
```

## Learning Outcomes

After studying this section, you should be able to:
- Understand the multi-modal perception architecture for object identification
- Design and implement deep learning-based object detection and recognition systems
- Create affordance recognition systems that understand object interaction possibilities
- Plan grasps considering stability, accessibility, safety, and efficiency
- Execute manipulation tasks with precise control and force feedback
- Implement advanced manipulation strategies for different object types
- Handle grasp failures and implement recovery mechanisms

## Key Insights

### Multi-Modal Perception
Effective object identification requires integration of multiple sensing modalities for robust performance.

### Affordance Understanding
Recognizing object affordances is crucial for determining appropriate manipulation strategies.

### Grasp Quality Assessment
Successful manipulation depends on careful evaluation of grasp stability and accessibility.

### Force Control
Precise force control is essential for handling objects of different materials and fragility.

### Recovery Strategies
Robust manipulation systems must include comprehensive failure detection and recovery mechanisms.

## Summary

Object identification and manipulation in autonomous humanoid systems require sophisticated integration of perception, planning, and control technologies. The system must accurately detect and recognize objects, understand their affordances, plan stable and accessible grasps, and execute manipulation tasks with precise control. Success depends on careful coordination between visual perception, 3D pose estimation, grasp planning, and force-controlled execution. When properly implemented, these systems enable robots to interact with the physical world in meaningful and useful ways, forming a crucial component of autonomous humanoid capabilities.