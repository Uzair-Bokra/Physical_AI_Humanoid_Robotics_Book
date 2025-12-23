# Navigation and Obstacle Handling in Autonomous Humanoid Systems

Navigation and obstacle handling represent critical capabilities for autonomous humanoid robots operating in dynamic human environments. This chapter explores the sophisticated systems required to enable safe, efficient, and reliable navigation while effectively handling static and dynamic obstacles in real-world scenarios.

## Navigation Architecture Overview

### Multi-Layer Navigation System

The navigation system employs a hierarchical architecture that operates at multiple levels of abstraction:

```
┌─────────────────────────────────────────────────────────────────┐
│                    TASK-LEVEL NAVIGATION                        │
│  High-level goal planning • Route selection • Mission planning  │
├─────────────────────────────────────────────────────────────────┤
│                   GLOBAL NAVIGATION                            │
│  Path planning • Map-based routing • Obstacle avoidance        │
├─────────────────────────────────────────────────────────────────┤
│                   LOCAL NAVIGATION                             │
│  Real-time obstacle avoidance • Dynamic path adjustment        │
├─────────────────────────────────────────────────────────────────┤
│                   MOTION CONTROL                               │
│  Low-level trajectory execution • Motor control                │
└─────────────────────────────────────────────────────────────────┘
```

### Navigation System Components

The navigation system integrates multiple components working in coordination:

#### Global Planner
- **Function**: Generates optimal paths based on static map data
- **Inputs**: Global map, start position, goal position
- **Outputs**: Global path (waypoint sequence)
- **Algorithms**: A*, Dijkstra, D* Lite, RRT variants

#### Local Planner
- **Function**: Executes path following while avoiding dynamic obstacles
- **Inputs**: Global path, local costmap, robot state
- **Outputs**: Velocity commands
- **Algorithms**: DWA, Trajectory Rollout, MPC

#### Costmap Management
- **Function**: Maintains representation of obstacles and free space
- **Inputs**: Sensor data, static map
- **Outputs**: Costmap for planning
- **Features**: Static obstacles, dynamic obstacles, inflation

#### Controller Interface
- **Function**: Translates navigation goals into robot motion
- **Inputs**: Velocity commands
- **Outputs**: Motor commands
- **Features**: Velocity smoothing, motion constraints

## Navigation Planning and Path Generation

### Global Path Planning

The global planner generates optimal paths using map-based algorithms:

```cpp
// Global path planning implementation
class GlobalPlanner
{
public:
    Path planPath(const Pose& start, const Pose& goal, const Costmap& costmap)
    {
        // Validate start and goal positions
        if (!isValidPosition(start, costmap) || !isValidPosition(goal, costmap)) {
            throw std::runtime_error("Invalid start or goal position");
        }

        // Convert poses to grid coordinates
        auto start_cell = poseToGrid(start, costmap);
        auto goal_cell = poseToGrid(goal, costmap);

        // Plan path using A* algorithm
        Path path = aStarPlan(start_cell, goal_cell, costmap);

        // Smooth the path for better execution
        Path smoothed_path = smoothPath(path, costmap);

        return smoothed_path;
    }

private:
    Path aStarPlan(const Cell& start, const Cell& goal, const Costmap& costmap)
    {
        // Priority queue for A* algorithm
        std::priority_queue<PriorityNode, std::vector<PriorityNode>, CompareNodes> open_set;

        // Track visited nodes and costs
        std::map<Cell, double> g_score;
        std::map<Cell, Cell> came_from;

        // Initialize start node
        open_set.push({start, 0.0});
        g_score[start] = 0.0;

        while (!open_set.empty()) {
            auto current = open_set.top().cell;
            open_set.pop();

            // Check if we reached the goal
            if (current == goal) {
                return reconstructPath(came_from, current);
            }

            // Explore neighbors
            for (const auto& neighbor : getNeighbors(current, costmap)) {
                double tentative_g_score = g_score[current] + distance(current, neighbor);

                // Check if this path is better
                if (tentative_g_score < g_score[neighbor]) {
                    came_from[neighbor] = current;
                    g_score[neighbor] = tentative_g_score;
                    double f_score = tentative_g_score + heuristic(neighbor, goal);
                    open_set.push({neighbor, f_score});
                }
            }
        }

        throw std::runtime_error("No path found");
    }

    Path smoothPath(const Path& original_path, const Costmap& costmap)
    {
        // Implement path smoothing using gradient descent or other techniques
        Path smoothed_path = original_path;

        // Iteratively improve the path by reducing length while maintaining safety
        for (int iteration = 0; iteration < SMOOTHING_ITERATIONS; ++iteration) {
            for (size_t i = 1; i < smoothed_path.size() - 1; ++i) {
                // Try to shortcut between non-adjacent points
                if (canShortcut(smoothed_path[i-1], smoothed_path[i+1], costmap)) {
                    smoothed_path.erase(smoothed_path.begin() + i);
                    i--; // Adjust index after removal
                }
            }
        }

        return smoothed_path;
    }

    struct Cell {
        int x, y;
        bool operator==(const Cell& other) const {
            return x == other.x && y == other.y;
        }
        bool operator<(const Cell& other) const {
            return x < other.x || (x == other.x && y < other.y);
        }
    };

    struct PriorityNode {
        Cell cell;
        double priority;
    };

    struct CompareNodes {
        bool operator()(const PriorityNode& a, const PriorityNode& b) const {
            return a.priority > b.priority;
        }
    };

    static constexpr int SMOOTHING_ITERATIONS = 10;
};
```

### Local Path Planning

The local planner handles real-time obstacle avoidance:

```cpp
// Local path planning and obstacle avoidance
class LocalPlanner
{
public:
    VelocityCommand computeVelocity(const Path& global_path,
                                  const Pose& current_pose,
                                  const Twist& current_velocity,
                                  const Costmap& local_costmap,
                                  const NavigationParameters& params)
    {
        // Get relevant portion of global path
        auto relevant_path = getRelevantPath(global_path, current_pose);

        // Generate candidate trajectories
        auto trajectories = generateTrajectories(current_pose, current_velocity, params);

        // Evaluate trajectories based on multiple criteria
        auto best_trajectory = evaluateTrajectories(trajectories,
                                                  relevant_path,
                                                  local_costmap,
                                                  current_pose,
                                                  params);

        // Extract velocity command from best trajectory
        return extractVelocity(best_trajectory);
    }

private:
    std::vector<Trajectory> generateTrajectories(const Pose& current_pose,
                                               const Twist& current_velocity,
                                               const NavigationParameters& params)
    {
        std::vector<Trajectory> trajectories;

        // Generate trajectories with different velocity combinations
        for (double vx = params.min_vel_x; vx <= params.max_vel_x; vx += params.vel_x_samples) {
            for (double vy = params.min_vel_y; vy <= params.max_vel_y; vy += params.vel_y_samples) {
                for (double wz = params.min_vel_theta; wz <= params.max_vel_theta; wz += params.vel_theta_samples) {
                    // Check if velocity is feasible
                    if (isVelocityFeasible(vx, vy, wz, current_velocity, params)) {
                        auto trajectory = generateTrajectory(current_pose,
                                                          {vx, vy, wz},
                                                          params.sim_time,
                                                          params.sim_granularity);
                        trajectories.push_back(trajectory);
                    }
                }
            }
        }

        return trajectories;
    }

    Trajectory evaluateTrajectories(const std::vector<Trajectory>& trajectories,
                                  const Path& global_path,
                                  const Costmap& local_costmap,
                                  const Pose& current_pose,
                                  const NavigationParameters& params)
    {
        Trajectory best_trajectory;
        double best_score = std::numeric_limits<double>::lowest();

        for (const auto& trajectory : trajectories) {
            // Calculate scores for different criteria
            double path_distance_score = calculatePathDistanceScore(trajectory, global_path);
            double goal_distance_score = calculateGoalDistanceScore(trajectory, global_path);
            double obstacle_score = calculateObstacleScore(trajectory, local_costmap);
            double velocity_score = calculateVelocityScore(trajectory, params);

            // Combine scores with weights
            double total_score = params.path_distance_weight * path_distance_score +
                               params.goal_distance_weight * goal_distance_score +
                               params.obstacle_weight * obstacle_score +
                               params.velocity_weight * velocity_score;

            if (total_score > best_score) {
                best_score = total_score;
                best_trajectory = trajectory;
            }
        }

        return best_trajectory;
    }

    bool isVelocityFeasible(double vx, double vy, double wz,
                           const Twist& current_velocity,
                           const NavigationParameters& params)
    {
        // Check acceleration limits
        double dvx = vx - current_velocity.linear.x;
        double dvy = vy - current_velocity.linear.y;
        double dwz = wz - current_velocity.angular.z;

        if (std::abs(dvx) > params.max_acc_x * params.sim_granularity ||
            std::abs(dvy) > params.max_acc_y * params.sim_granularity ||
            std::abs(dwz) > params.max_acc_theta * params.sim_granularity) {
            return false;
        }

        // Check velocity limits
        if (std::abs(vx) > params.max_vel_x || std::abs(vy) > params.max_vel_y || std::abs(wz) > params.max_vel_theta) {
            return false;
        }

        return true;
    }

    Trajectory generateTrajectory(const Pose& start_pose,
                                const Twist& velocity,
                                double sim_time,
                                double time_granularity)
    {
        Trajectory trajectory;
        Pose current_pose = start_pose;
        double time = 0.0;

        while (time <= sim_time) {
            // Integrate velocity to get new pose
            Pose new_pose;
            new_pose.x = current_pose.x + velocity.linear.x * time_granularity * cos(current_pose.theta) -
                        velocity.linear.y * time_granularity * sin(current_pose.theta);
            new_pose.y = current_pose.y + velocity.linear.x * time_granularity * sin(current_pose.theta) +
                        velocity.linear.y * time_granularity * cos(current_pose.theta);
            new_pose.theta = current_pose.theta + velocity.angular.z * time_granularity;

            trajectory.points.push_back(new_pose);
            current_pose = new_pose;
            time += time_granularity;
        }

        return trajectory;
    }
};
```

## Costmap Management and Obstacle Representation

### Static and Dynamic Costmaps

The system maintains multiple costmaps for different purposes:

```cpp
// Costmap management system
class CostmapManager
{
public:
    CostmapManager()
    {
        // Initialize static costmap from map server
        static_costmap_ = initializeStaticCostmap();

        // Initialize dynamic costmap for obstacle updates
        dynamic_costmap_ = initializeDynamicCostmap();

        // Initialize obstacle layers
        obstacle_layer_ = std::make_unique<ObstacleLayer>();
        inflation_layer_ = std::make_unique<InflationLayer>();
    }

    void updateCostmap(const std::vector<Obstacle>& sensor_obstacles,
                      const std::vector<Obstacle>& dynamic_obstacles)
    {
        // Clear dynamic obstacles from previous iteration
        dynamic_costmap_.clearObstacles();

        // Add sensor-detected obstacles
        for (const auto& obstacle : sensor_obstacles) {
            dynamic_costmap_.addObstacle(obstacle);
        }

        // Add known dynamic obstacles
        for (const auto& obstacle : dynamic_obstacles) {
            dynamic_costmap_.addObstacle(obstacle);
        }

        // Apply inflation to create safety margins
        applyInflation(dynamic_costmap_);

        // Combine static and dynamic costmaps
        combined_costmap_ = combineCostmaps(static_costmap_, dynamic_costmap_);
    }

    const Costmap& getCombinedCostmap() const
    {
        return combined_costmap_;
    }

private:
    Costmap initializeStaticCostmap()
    {
        // Load static map from map server
        auto static_map = loadStaticMap();

        // Convert to costmap format
        Costmap costmap;
        costmap.width = static_map.info.width;
        costmap.height = static_map.info.height;
        costmap.resolution = static_map.info.resolution;
        costmap.origin = static_map.info.origin;

        // Convert occupancy grid to cost values
        for (size_t i = 0; i < static_map.data.size(); ++i) {
            if (static_map.data[i] == 100) { // Occupied
                costmap.cells[i] = 254; // High cost
            } else if (static_map.data[i] == 0) { // Free
                costmap.cells[i] = 0; // No cost
            } else { // Unknown
                costmap.cells[i] = 128; // Medium cost
            }
        }

        return costmap;
    }

    Costmap initializeDynamicCostmap()
    {
        // Create dynamic costmap with same dimensions as static
        Costmap costmap = static_costmap_;
        costmap.cells.assign(costmap.width * costmap.height, 0);
        return costmap;
    }

    void applyInflation(Costmap& costmap)
    {
        // Apply inflation to obstacles to create safety margins
        for (int x = 0; x < costmap.width; ++x) {
            for (int y = 0; y < costmap.height; ++y) {
                if (costmap.getCell(x, y) >= 250) { // Obstacle
                    inflateCell(costmap, x, y, INFLATION_RADIUS);
                }
            }
        }
    }

    void inflateCell(Costmap& costmap, int center_x, int center_y, double radius)
    {
        int radius_cells = static_cast<int>(radius / costmap.resolution);

        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
                int x = center_x + dx;
                int y = center_y + dy;

                if (x >= 0 && x < costmap.width && y >= 0 && y < costmap.height) {
                    double distance = sqrt(dx*dx + dy*dy) * costmap.resolution;

                    if (distance <= radius) {
                        double cost = calculateInflationCost(distance, radius);
                        costmap.setCell(x, y, std::max(costmap.getCell(x, y), cost));
                    }
                }
            }
        }
    }

    Costmap combineCostmaps(const Costmap& static_map, const Costmap& dynamic_map)
    {
        Costmap combined = static_map;

        for (size_t i = 0; i < combined.cells.size(); ++i) {
            // Take maximum of static and dynamic costs
            combined.cells[i] = std::max(static_map.cells[i], dynamic_map.cells[i]);
        }

        return combined;
    }

    Costmap static_costmap_;
    Costmap dynamic_costmap_;
    Costmap combined_costmap_;
    std::unique_ptr<ObstacleLayer> obstacle_layer_;
    std::unique_ptr<InflationLayer> inflation_layer_;

    static constexpr double INFLATION_RADIUS = 0.55; // 55cm inflation radius
};
```

### Obstacle Detection and Classification

The system classifies and handles different types of obstacles:

```cpp
// Obstacle detection and classification
class ObstacleClassifier
{
public:
    ObstacleClassification classifyObstacle(const Obstacle& obstacle)
    {
        ObstacleClassification classification;

        // Classify based on size
        double area = obstacle.width * obstacle.height;
        if (area < 0.01) { // Small obstacle
            classification.type = ObstacleType::SMALL_OBJECT;
            classification.threat_level = ThreatLevel::LOW;
        } else if (area < 1.0) { // Medium obstacle
            classification.type = ObstacleType::MEDIUM_OBJECT;
            classification.threat_level = ThreatLevel::MEDIUM;
        } else { // Large obstacle
            classification.type = ObstacleType::LARGE_OBJECT;
            classification.threat_level = ThreatLevel::HIGH;
        }

        // Classify based on movement
        if (obstacle.velocity_magnitude > 0.1) { // Moving obstacle
            classification.type = ObstacleType::MOVING_OBJECT;
            classification.threat_level = ThreatLevel::HIGH;
        }

        // Classify based on height
        if (obstacle.height < 0.1) { // Low obstacle
            classification.traversable = true;
        } else { // High obstacle
            classification.traversable = false;
        }

        return classification;
    }

    NavigationStrategy determineStrategy(const ObstacleClassification& classification)
    {
        switch (classification.type) {
            case ObstacleType::STATIC:
                return NavigationStrategy::AVOID_STATIC;
            case ObstacleType::DYNAMIC_SLOW:
                return NavigationStrategy::WAIT_AND_PROCEED;
            case ObstacleType::DYNAMIC_FAST:
                return NavigationStrategy::PREDICTIVE_AVOIDANCE;
            case ObstacleType::HUMAN:
                return NavigationStrategy::SOCIAL_AVOIDANCE;
            case ObstacleType::SMALL_OBJECT:
                return NavigationStrategy::SMALL_OBJECT_AVOIDANCE;
            default:
                return NavigationStrategy::GENERAL_AVOIDANCE;
        }
    }

    bool isSafeToNavigate(const Obstacle& obstacle, const RobotState& robot_state)
    {
        // Calculate minimum safe distance based on robot size and speed
        double min_safe_distance = calculateMinSafeDistance(robot_state);

        // Check if obstacle is within safe distance
        double distance = calculateDistance(robot_state.pose, obstacle.pose);

        if (distance < min_safe_distance) {
            return false;
        }

        // Check if obstacle trajectory intersects with robot trajectory
        if (obstacle.velocity_magnitude > 0.01) {
            auto predicted_path = predictObstaclePath(obstacle, PREDICTION_TIME);
            auto robot_path = predictRobotPath(robot_state, PREDICTION_TIME);

            if (pathsIntersect(predicted_path, robot_path)) {
                return false;
            }
        }

        return true;
    }

private:
    double calculateMinSafeDistance(const RobotState& robot_state)
    {
        // Calculate safe distance based on robot speed and stopping distance
        double stopping_distance = (robot_state.velocity * robot_state.velocity) / (2 * MAX_DECELERATION);
        double safety_margin = SAFETY_DISTANCE_BASE + (robot_state.velocity * SAFETY_DISTANCE_FACTOR);

        return stopping_distance + safety_margin;
    }

    static constexpr double MAX_DECELERATION = 0.5; // m/s^2
    static constexpr double SAFETY_DISTANCE_BASE = 0.3; // m
    static constexpr double SAFETY_DISTANCE_FACTOR = 0.5; // m*s/m
    static constexpr double PREDICTION_TIME = 3.0; // seconds
};
```

## Dynamic Obstacle Handling

### Moving Obstacle Prediction and Avoidance

The system predicts and handles moving obstacles:

```cpp
// Dynamic obstacle prediction and avoidance
class DynamicObstacleHandler
{
public:
    DynamicObstacleHandler()
    {
        // Initialize tracking for moving obstacles
        obstacle_tracker_ = std::make_unique<ObstacleTracker>();
        prediction_engine_ = std::make_unique<PredictionEngine>();
        avoidance_planner_ = std::make_unique<AvoidancePlanner>();
    }

    VelocityCommand handleMovingObstacles(const std::vector<Obstacle>& moving_obstacles,
                                        const Pose& robot_pose,
                                        const Twist& robot_velocity,
                                        const Costmap& local_costmap,
                                        const NavigationParameters& params)
    {
        // Update obstacle tracking
        obstacle_tracker_->updateObstacles(moving_obstacles);

        // Predict obstacle trajectories
        auto predictions = prediction_engine_->predictTrajectories(moving_obstacles);

        // Generate avoidance maneuvers
        auto avoidance_command = avoidance_planner_->planAvoidance(
            predictions, robot_pose, robot_velocity, local_costmap, params);

        return avoidance_command;
    }

private:
    // Obstacle tracking system
    class ObstacleTracker
    {
    public:
        void updateObstacles(const std::vector<Obstacle>& new_obstacles)
        {
            // Associate new detections with existing tracks
            for (auto& new_obstacle : new_obstacles) {
                bool associated = false;

                for (auto& [track_id, track] : tracks_) {
                    if (calculateDistance(new_obstacle.pose, track.last_pose) < ASSOCIATION_THRESHOLD) {
                        // Update existing track
                        track.update(new_obstacle);
                        associated = true;
                        break;
                    }
                }

                if (!associated) {
                    // Create new track
                    int new_id = next_track_id_++;
                    tracks_[new_id] = ObstacleTrack(new_obstacle);
                }
            }

            // Remove old tracks that haven't been updated
            removeOldTracks();
        }

        std::vector<Obstacle> getTrackedObstacles() const
        {
            std::vector<Obstacle> tracked;
            for (const auto& [id, track] : tracks_) {
                tracked.push_back(track.getEstimatedObstacle());
            }
            return tracked;
        }

    private:
        struct ObstacleTrack
        {
            ObstacleTrack(const Obstacle& initial_obstacle) : last_obstacle(initial_obstacle)
            {
                last_update_time = std::chrono::steady_clock::now();
            }

            void update(const Obstacle& new_obstacle)
            {
                // Update position with weighted average
                last_obstacle.pose.x = 0.7 * last_obstacle.pose.x + 0.3 * new_obstacle.pose.x;
                last_obstacle.pose.y = 0.7 * last_obstacle.pose.y + 0.3 * new_obstacle.pose.y;

                // Update velocity estimate
                auto current_time = std::chrono::steady_clock::now();
                double dt = std::chrono::duration<double>(current_time - last_update_time).count();

                if (dt > 0) {
                    last_obstacle.velocity.x = (new_obstacle.pose.x - last_obstacle.pose.x) / dt;
                    last_obstacle.velocity.y = (new_obstacle.pose.y - last_obstacle.pose.y) / dt;
                }

                last_obstacle = new_obstacle;
                last_update_time = current_time;
            }

            Obstacle getEstimatedObstacle() const
            {
                return last_obstacle;
            }

            Obstacle last_obstacle;
            std::chrono::steady_clock::time_point last_update_time;
        };

        std::map<int, ObstacleTrack> tracks_;
        int next_track_id_ = 0;

        void removeOldTracks()
        {
            auto current_time = std::chrono::steady_clock::now();
            std::vector<int> to_remove;

            for (const auto& [id, track] : tracks_) {
                if (std::chrono::duration<double>(current_time - track.last_update_time).count() > MAX_TRACK_AGE) {
                    to_remove.push_back(id);
                }
            }

            for (int id : to_remove) {
                tracks_.erase(id);
            }
        }

        static constexpr double ASSOCIATION_THRESHOLD = 0.5; // meters
        static constexpr double MAX_TRACK_AGE = 5.0; // seconds
    };

    // Prediction engine for obstacle trajectories
    class PredictionEngine
    {
    public:
        std::vector<ObstacleTrajectory> predictTrajectories(const std::vector<Obstacle>& obstacles)
        {
            std::vector<ObstacleTrajectory> predictions;

            for (const auto& obstacle : obstacles) {
                auto trajectory = predictTrajectory(obstacle);
                predictions.push_back(trajectory);
            }

            return predictions;
        }

    private:
        ObstacleTrajectory predictTrajectory(const Obstacle& obstacle)
        {
            ObstacleTrajectory trajectory;
            trajectory.obstacle_id = obstacle.id;

            // Predict trajectory using constant velocity model
            Pose current_pose = obstacle.pose;
            Twist velocity = obstacle.velocity;

            for (double t = 0; t <= PREDICTION_HORIZON; t += PREDICTION_STEP) {
                Pose future_pose;
                future_pose.x = current_pose.x + velocity.x * t;
                future_pose.y = current_pose.y + velocity.y * t;
                future_pose.theta = current_pose.theta + velocity.theta * t;

                trajectory.points.push_back(future_pose);
            }

            return trajectory;
        }

        static constexpr double PREDICTION_HORIZON = 3.0; // seconds
        static constexpr double PREDICTION_STEP = 0.1; // seconds
    };

    // Avoidance planning system
    class AvoidancePlanner
    {
    public:
        VelocityCommand planAvoidance(const std::vector<ObstacleTrajectory>& predictions,
                                    const Pose& robot_pose,
                                    const Twist& robot_velocity,
                                    const Costmap& local_costmap,
                                    const NavigationParameters& params)
        {
            // Generate candidate avoidance trajectories
            auto candidates = generateAvoidanceTrajectories(robot_pose, robot_velocity, params);

            // Evaluate candidates for obstacle avoidance
            auto best_candidate = evaluateAvoidanceTrajectories(candidates, predictions, local_costmap);

            return extractVelocity(best_candidate);
        }

    private:
        std::vector<Trajectory> generateAvoidanceTrajectories(const Pose& robot_pose,
                                                           const Twist& current_velocity,
                                                           const NavigationParameters& params)
        {
            // Generate trajectories that actively avoid predicted obstacle paths
            std::vector<Trajectory> trajectories;

            // Try different velocity modifications to avoid obstacles
            for (double speed_factor = 0.5; speed_factor <= 1.5; speed_factor += 0.1) {
                for (double turn_factor = -0.5; turn_factor <= 0.5; turn_factor += 0.1) {
                    Twist modified_velocity;
                    modified_velocity.linear.x = current_velocity.linear.x * speed_factor;
                    modified_velocity.linear.y = current_velocity.linear.y * speed_factor;
                    modified_velocity.angular.z = current_velocity.angular.z + turn_factor;

                    if (isVelocityFeasible(modified_velocity, params)) {
                        auto trajectory = generateTrajectory(robot_pose, modified_velocity,
                                                          params.avoidance_time, 0.1);
                        trajectories.push_back(trajectory);
                    }
                }
            }

            return trajectories;
        }

        Trajectory evaluateAvoidanceTrajectories(const std::vector<Trajectory>& candidates,
                                              const std::vector<ObstacleTrajectory>& predictions,
                                              const Costmap& local_costmap)
        {
            Trajectory best_trajectory;
            double best_score = std::numeric_limits<double>::lowest();

            for (const auto& trajectory : candidates) {
                double collision_risk = calculateCollisionRisk(trajectory, predictions);
                double path_cost = calculatePathCost(trajectory, local_costmap);
                double energy_cost = calculateEnergyCost(trajectory);

                // Prefer trajectories that avoid obstacles while maintaining efficiency
                double score = -collision_risk * 10.0 - path_cost - energy_cost * 0.1;

                if (score > best_score) {
                    best_score = score;
                    best_trajectory = trajectory;
                }
            }

            return best_trajectory;
        }

        double calculateCollisionRisk(const Trajectory& robot_trajectory,
                                   const std::vector<ObstacleTrajectory>& obstacle_trajectories)
        {
            double total_risk = 0.0;

            for (const auto& obstacle_traj : obstacle_trajectories) {
                for (size_t i = 0; i < robot_trajectory.points.size(); ++i) {
                    if (i < obstacle_traj.points.size()) {
                        double distance = calculateDistance(robot_trajectory.points[i],
                                                         obstacle_traj.points[i]);
                        if (distance < SAFETY_RADIUS) {
                            // Higher risk for closer distances
                            total_risk += (SAFETY_RADIUS - distance) / SAFETY_RADIUS;
                        }
                    }
                }
            }

            return total_risk;
        }

        static constexpr double SAFETY_RADIUS = 0.6; // meters
    };

    std::unique_ptr<ObstacleTracker> obstacle_tracker_;
    std::unique_ptr<PredictionEngine> prediction_engine_;
    std::unique_ptr<AvoidancePlanner> avoidance_planner_;
};
```

## Social Navigation and Human-Aware Navigation

### Socially-Aware Navigation

The system incorporates social navigation principles for human environments:

```cpp
// Social navigation and human-aware navigation
class SocialNavigationSystem
{
public:
    SocialNavigationSystem()
    {
        // Initialize social behavior parameters
        initializeSocialParameters();
    }

    VelocityCommand computeSocialVelocity(const Path& global_path,
                                        const Pose& current_pose,
                                        const std::vector<Person>& detected_people,
                                        const NavigationParameters& params)
    {
        // Calculate base velocity following global path
        auto base_velocity = computePathFollowingVelocity(global_path, current_pose);

        // Apply social behavior modifications
        auto social_velocity = modifyForSocialBehavior(base_velocity,
                                                     current_pose,
                                                     detected_people,
                                                     params);

        return social_velocity;
    }

private:
    VelocityCommand modifyForSocialBehavior(const VelocityCommand& base_velocity,
                                          const Pose& robot_pose,
                                          const std::vector<Person>& people,
                                          const NavigationParameters& params)
    {
        VelocityCommand modified_velocity = base_velocity;

        for (const auto& person : people) {
            // Calculate distance and relative position to person
            double distance = calculateDistance(robot_pose, person.pose);
            double angle = calculateAngle(robot_pose, person.pose);

            // Apply different social behaviors based on distance
            if (distance < SOCIAL_DISTANCE_PERSONAL) {
                // Too close - slow down significantly
                modified_velocity.linear *= 0.3;
                modified_velocity.angular = std::copysign(0.5, angle); // Move away
            } else if (distance < SOCIAL_DISTANCE_SOCIAL) {
                // In social space - moderate adjustments
                double adjustment_factor = (distance - SOCIAL_DISTANCE_INTIMATE) /
                                         (SOCIAL_DISTANCE_SOCIAL - SOCIAL_DISTANCE_INTIMATE);
                modified_velocity.linear *= std::max(0.5, adjustment_factor);

                // Yield to people moving in same direction
                if (isSameDirection(person, robot_pose)) {
                    modified_velocity.linear *= 0.7; // Slow down to let person pass
                }
            } else if (distance < SOCIAL_DISTANCE_PUBLIC) {
                // In public space - minimal adjustments
                if (person.velocity.magnitude() > 0.1) {
                    // Person is moving - predict path and adjust
                    auto person_trajectory = predictPersonTrajectory(person);
                    if (pathsMayIntersect(robot_pose, modified_velocity, person_trajectory)) {
                        // Adjust to avoid potential collision
                        modified_velocity.angular += calculateAvoidanceAngle(person.pose, robot_pose);
                    }
                }
            }
        }

        // Ensure velocity constraints are still satisfied
        modified_velocity.linear = std::min(modified_velocity.linear, params.max_vel_x);
        modified_velocity.angular = std::min(modified_velocity.angular, params.max_vel_theta);

        return modified_velocity;
    }

    bool isSameDirection(const Person& person, const Pose& robot_pose) const
    {
        // Check if person and robot are moving in similar directions
        double person_direction = person.velocity.theta;
        double robot_direction = robot_pose.theta;

        double angle_diff = std::abs(person_direction - robot_direction);
        // Normalize to [-π, π]
        if (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        if (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        return std::abs(angle_diff) < M_PI_4; // Within 45 degrees
    }

    double calculateAvoidanceAngle(const Pose& person_pose, const Pose& robot_pose) const
    {
        // Calculate angle to move away from person
        double dx = person_pose.x - robot_pose.x;
        double dy = person_pose.y - robot_pose.y;
        double distance = sqrt(dx*dx + dy*dy);

        if (distance > 0) {
            // Move perpendicular to the line connecting robot and person
            double approach_angle = atan2(dy, dx);
            double avoidance_angle = approach_angle + M_PI_2; // 90 degrees offset

            // Scale by inverse distance (closer = larger adjustment)
            return avoidance_angle * (0.5 / std::max(distance, 0.5));
        }

        return 0.0;
    }

    void initializeSocialParameters()
    {
        // Set social distance parameters based on proxemics research
        SOCIAL_DISTANCE_INTIMATE = 0.45;   // 1.5 feet
        SOCIAL_DISTANCE_PERSONAL = 0.75;   // 2.5 feet
        SOCIAL_DISTANCE_SOCIAL = 1.2;      // 4 feet
        SOCIAL_DISTANCE_PUBLIC = 3.6;      // 12 feet
    }

    double SOCIAL_DISTANCE_INTIMATE;
    double SOCIAL_DISTANCE_PERSONAL;
    double SOCIAL_DISTANCE_SOCIAL;
    double SOCIAL_DISTANCE_PUBLIC;
};
```

## Recovery Behaviors and Failure Handling

### Navigation Recovery System

The system implements comprehensive recovery behaviors for navigation failures:

```cpp
// Navigation recovery behaviors
class NavigationRecoverySystem
{
public:
    NavigationRecoverySystem()
    {
        // Initialize recovery behaviors
        recovery_behaviors_.push_back(std::make_unique<ClearCostmapRecovery>());
        recovery_behaviors_.push_back(std::make_unique<BacktrackRecovery>());
        recovery_behaviors_.push_back(std::make_unique<WaitRecovery>());
        recovery_behaviors_.push_back(std::make_unique<RotateRecovery>());
        recovery_behaviors_.push_back(std::make_unique<ExploreRecovery>());
    }

    bool attemptRecovery(NavigationState& nav_state)
    {
        RCLCPP_WARN(rclcpp::get_logger("recovery"),
                   "Attempting navigation recovery, current state: %s",
                   nav_state.getStateString().c_str());

        // Try each recovery behavior in order of preference
        for (auto& behavior : recovery_behaviors_) {
            RCLCPP_INFO(rclcpp::get_logger("recovery"),
                       "Trying recovery behavior: %s",
                       behavior->getName().c_str());

            if (behavior->canRun()) {
                auto result = behavior->run(nav_state);

                if (result == RecoveryResult::SUCCESS) {
                    RCLCPP_INFO(rclcpp::get_logger("recovery"),
                               "Recovery successful with behavior: %s",
                               behavior->getName().c_str());
                    return true;
                } else if (result == RecoveryResult::FAILURE) {
                    RCLCPP_WARN(rclcpp::get_logger("recovery"),
                               "Recovery behavior failed: %s",
                               behavior->getName().c_str());
                    continue; // Try next behavior
                } else if (result == RecoveryResult::RETRY) {
                    // Behavior wants to be tried again
                    continue;
                }
            }
        }

        RCLCPP_ERROR(rclcpp::get_logger("recovery"),
                    "All recovery behaviors failed");
        return false;
    }

private:
    // Base recovery behavior interface
    class RecoveryBehavior
    {
    public:
        virtual RecoveryResult run(NavigationState& nav_state) = 0;
        virtual bool canRun() const = 0;
        virtual std::string getName() const = 0;
        virtual ~RecoveryBehavior() = default;
    };

    // Clear costmap recovery behavior
    class ClearCostmapRecovery : public RecoveryBehavior
    {
    public:
        RecoveryResult run(NavigationState& nav_state) override
        {
            RCLCPP_INFO(rclcpp::get_logger("recovery"), "Clearing costmaps");

            // Clear both global and local costmaps
            nav_state.clearGlobalCostmap();
            nav_state.clearLocalCostmap();

            // Wait for costmaps to update
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // Check if we can now plan a path
            if (nav_state.canPlanPath()) {
                RCLCPP_INFO(rclcpp::get_logger("recovery"), "Path planning now possible after costmap clearing");
                return RecoveryResult::SUCCESS;
            }

            return RecoveryResult::FAILURE;
        }

        bool canRun() const override { return true; }
        std::string getName() const override { return "ClearCostmapRecovery"; }
    };

    // Backtrack recovery behavior
    class BacktrackRecovery : public RecoveryBehavior
    {
    public:
        RecoveryResult run(NavigationState& nav_state) override
        {
            RCLCPP_INFO(rclcpp::get_logger("recovery"), "Attempting to backtrack");

            // Get recent path history
            auto recent_path = nav_state.getRecentPath();
            if (recent_path.empty()) {
                return RecoveryResult::FAILURE;
            }

            // Find a safe backtrack point (some distance back from current position)
            auto backtrack_pose = findSafeBacktrackPoint(recent_path, nav_state.getCurrentPose());

            if (backtrack_pose.valid) {
                // Plan path back to the safe point
                auto backtrack_path = nav_state.planPath(nav_state.getCurrentPose(), backtrack_pose);

                if (!backtrack_path.empty()) {
                    // Execute backtrack maneuver
                    bool success = executeBacktrack(backtrack_path);
                    return success ? RecoveryResult::SUCCESS : RecoveryResult::FAILURE;
                }
            }

            return RecoveryResult::FAILURE;
        }

        bool canRun() const override { return true; }
        std::string getName() const override { return "BacktrackRecovery"; }

    private:
        Pose findSafeBacktrackPoint(const Path& recent_path, const Pose& current_pose)
        {
            // Look for a point in the recent path that's at least 1 meter away
            for (int i = recent_path.size() - 1; i >= 0; --i) {
                double distance = calculateDistance(recent_path[i], current_pose);
                if (distance >= 1.0) { // At least 1 meter back
                    return recent_path[i];
                }
            }

            // If no point is far enough, return the last point
            if (!recent_path.empty()) {
                return recent_path.back();
            }

            return Pose{0, 0, 0, false}; // Invalid pose
        }

        bool executeBacktrack(const Path& path)
        {
            // Execute the backtrack path
            // Implementation would involve sending path to local planner
            return true; // Simplified for example
        }
    };

    // Wait recovery behavior
    class WaitRecovery : public RecoveryBehavior
    {
    public:
        RecoveryResult run(NavigationState& nav_state) override
        {
            RCLCPP_INFO(rclcpp::get_logger("recovery"), "Waiting for dynamic obstacles to clear");

            // Stop robot movement
            nav_state.stopRobot();

            // Wait for a period to allow dynamic obstacles to move
            auto start_time = std::chrono::steady_clock::now();
            auto timeout_time = start_time + std::chrono::seconds(10);

            while (std::chrono::steady_clock::now() < timeout_time) {
                // Check if obstacles have cleared
                if (nav_state.getDynamicObstacles().empty()) {
                    RCLCPP_INFO(rclcpp::get_logger("recovery"), "Dynamic obstacles have cleared");
                    return RecoveryResult::SUCCESS;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            RCLCPP_INFO(rclcpp::get_logger("recovery"), "Wait timeout, obstacles still present");
            return RecoveryResult::FAILURE;
        }

        bool canRun() const override { return true; }
        std::string getName() const override { return "WaitRecovery"; }
    };

    // Rotate recovery behavior
    class RotateRecovery : public RecoveryBehavior
    {
    public:
        RecoveryResult run(NavigationState& nav_state) override
        {
            RCLCPP_INFO(rclcpp::get_logger("recovery"), "Attempting rotation to clear local minima");

            // Rotate robot to try different local minima
            double current_yaw = nav_state.getCurrentPose().theta;
            double target_yaw = current_yaw + M_PI; // Rotate 180 degrees

            // Execute rotation
            bool success = nav_state.rotateToYaw(target_yaw, M_PI_4); // 45 deg/s

            if (success) {
                // After rotation, clear local costmap to remove artifacts
                nav_state.clearLocalCostmap();

                // Wait briefly for sensors to update
                std::this_thread::sleep_for(std::chrono::milliseconds(500));

                return RecoveryResult::SUCCESS;
            }

            return RecoveryResult::FAILURE;
        }

        bool canRun() const override { return true; }
        std::string getName() const override { return "RotateRecovery"; }
    };

    // Explore recovery behavior
    class ExploreRecovery : public RecoveryBehavior
    {
    public:
        RecoveryResult run(NavigationState& nav_state) override
        {
            RCLCPP_INFO(rclcpp::get_logger("recovery"), "Attempting exploration maneuver");

            // Find an exploratory direction that might lead to a clear path
            auto exploratory_direction = findExploratoryDirection(nav_state);

            if (exploratory_direction.valid) {
                // Plan and execute short exploratory movement
                auto exploration_path = planExploration(nav_state, exploratory_direction);

                if (!exploration_path.empty()) {
                    bool success = executeExploration(exploration_path);
                    return success ? RecoveryResult::SUCCESS : RecoveryResult::FAILURE;
                }
            }

            return RecoveryResult::FAILURE;
        }

        bool canRun() const override { return true; }
        std::string getName() const override { return "ExploreRecovery"; }

    private:
        Direction findExploratoryDirection(const NavigationState& nav_state)
        {
            // Look for directions with lower obstacle density
            auto local_costmap = nav_state.getLocalCostmap();
            auto robot_pose = nav_state.getCurrentPose();

            // Check 8 directions around the robot
            std::vector<Direction> directions = {
                {M_PI/4, true}, {M_PI/2, true}, {3*M_PI/4, true}, {M_PI, true},
                {-3*M_PI/4, true}, {-M_PI/2, true}, {-M_PI/4, true}, {0, true}
            };

            // Score each direction based on obstacle density
            for (auto& dir : directions) {
                double score = calculateDirectionScore(local_costmap, robot_pose, dir.angle);
                dir.score = score;
            }

            // Find direction with lowest obstacle density
            auto best_dir = std::min_element(directions.begin(), directions.end(),
                                           [](const Direction& a, const Direction& b) {
                                               return a.score < b.score;
                                           });

            return *best_dir;
        }

        double calculateDirectionScore(const Costmap& costmap, const Pose& robot_pose, double angle)
        {
            // Calculate average cost in the given direction
            double total_cost = 0.0;
            int count = 0;

            // Sample points along the direction
            for (double dist = 0.5; dist <= 2.0; dist += 0.5) {
                double x = robot_pose.x + dist * cos(angle);
                double y = robot_pose.y + dist * sin(angle);

                // Convert to grid coordinates
                int grid_x = static_cast<int>((x - costmap.origin.x) / costmap.resolution);
                int grid_y = static_cast<int>((y - costmap.origin.y) / costmap.resolution);

                if (grid_x >= 0 && grid_x < costmap.width &&
                    grid_y >= 0 && grid_y < costmap.height) {
                    total_cost += costmap.getCell(grid_x, grid_y);
                    count++;
                }
            }

            return count > 0 ? total_cost / count : 255.0; // High cost if out of bounds
        }

        struct Direction {
            double angle;
            bool valid;
            double score;
        };
    };

    enum class RecoveryResult {
        SUCCESS,
        FAILURE,
        RETRY
    };

    std::vector<std::unique_ptr<RecoveryBehavior>> recovery_behaviors_;
};
```

## Performance Optimization and Real-Time Considerations

### Real-Time Navigation Optimization

The system is optimized for real-time performance:

```cpp
// Real-time navigation optimization
class RealTimeNavigationOptimizer
{
public:
    RealTimeNavigationOptimizer()
    {
        // Initialize optimization parameters
        initializeOptimizationParameters();
    }

    void optimizeNavigationCycle(NavigationState& nav_state)
    {
        // Optimize different aspects of navigation in real-time
        optimizePathPlanning(nav_state);
        optimizeLocalPlanning(nav_state);
        optimizeCostmapUpdates(nav_state);
        optimizeSensorProcessing(nav_state);
    }

private:
    void optimizePathPlanning(NavigationState& nav_state)
    {
        // Use anytime algorithms that can return best-so-far solution
        if (nav_state.getRemainingTime() < 0.05) { // Less than 50ms remaining
            // Use faster, approximate path planning
            nav_state.useApproximatePlanner();
        } else {
            // Use full-featured path planning
            nav_state.useOptimalPlanner();
        }
    }

    void optimizeLocalPlanning(NavigationState& nav_state)
    {
        // Adjust local planning horizon based on robot speed
        double current_speed = nav_state.getCurrentVelocity().linear.x;
        double adaptive_horizon = std::max(MIN_LOCAL_HORIZON,
                                        current_speed * TIME_HORIZON_FACTOR);

        nav_state.setLocalPlanningHorizon(adaptive_horizon);
    }

    void optimizeCostmapUpdates(NavigationState& nav_state)
    {
        // Use multi-resolution costmaps for efficiency
        if (nav_state.getRobotVelocity() > HIGH_SPEED_THRESHOLD) {
            // Use lower resolution for faster updates at high speed
            nav_state.setCostmapResolution(LOW_RESOLUTION);
        } else {
            // Use higher resolution for precision at low speed
            nav_state.setCostmapResolution(HIGH_RESOLUTION);
        }

        // Update costmap at adaptive frequency
        double update_frequency = calculateAdaptiveFrequency(nav_state);
        nav_state.setCostmapUpdateFrequency(update_frequency);
    }

    void optimizeSensorProcessing(NavigationState& nav_state)
    {
        // Use sensor fusion for more reliable obstacle detection
        auto fused_obstacles = fuseSensorData(nav_state.getLidarData(),
                                           nav_state.getCameraData(),
                                           nav_state.getDepthData());

        nav_state.updateObstacles(fused_obstacles);
    }

    double calculateAdaptiveFrequency(const NavigationState& nav_state)
    {
        // Calculate update frequency based on environment complexity
        auto obstacles = nav_state.getDynamicObstacles();
        double complexity_factor = calculateEnvironmentComplexity(obstacles);

        // Higher frequency for more complex environments
        return BASE_UPDATE_FREQ * (1.0 + complexity_factor * COMPLEXITY_WEIGHT);
    }

    double calculateEnvironmentComplexity(const std::vector<Obstacle>& obstacles)
    {
        if (obstacles.empty()) {
            return 0.0;
        }

        // Calculate complexity based on obstacle density and movement
        double density = static_cast<double>(obstacles.size()) / NAVIGATION_AREA;
        double movement_factor = 0.0;

        for (const auto& obstacle : obstacles) {
            movement_factor += obstacle.velocity.magnitude();
        }
        movement_factor /= obstacles.size();

        return std::min(1.0, density * 10.0 + movement_factor);
    }

    void initializeOptimizationParameters()
    {
        MIN_LOCAL_HORIZON = 1.0;      // meters
        TIME_HORIZON_FACTOR = 1.5;    // seconds per m/s
        HIGH_SPEED_THRESHOLD = 0.5;   // m/s
        LOW_RESOLUTION = 0.2;         // meters per cell
        HIGH_RESOLUTION = 0.05;       // meters per cell
        BASE_UPDATE_FREQ = 10.0;      // Hz
        COMPLEXITY_WEIGHT = 0.5;
        NAVIGATION_AREA = 50.0;       // square meters
    }

    double MIN_LOCAL_HORIZON;
    double TIME_HORIZON_FACTOR;
    double HIGH_SPEED_THRESHOLD;
    double LOW_RESOLUTION;
    double HIGH_RESOLUTION;
    double BASE_UPDATE_FREQ;
    double COMPLEXITY_WEIGHT;
    double NAVIGATION_AREA;
};
```

## Learning Outcomes

After studying this section, you should be able to:
- Understand the multi-layer architecture of navigation systems in autonomous robots
- Design and implement global and local path planning algorithms
- Create costmap management systems for obstacle representation
- Implement dynamic obstacle prediction and avoidance strategies
- Apply social navigation principles for human-aware navigation
- Design recovery behaviors for navigation failure handling
- Optimize navigation systems for real-time performance requirements

## Key Insights

### Multi-Layer Coordination
Effective navigation requires tight coordination between global planning, local planning, and motion control layers.

### Dynamic Obstacle Intelligence
Handling moving obstacles requires prediction, tracking, and adaptive avoidance strategies.

### Social Navigation
Navigation in human environments must consider social conventions and human comfort.

### Recovery Robustness
Comprehensive recovery behaviors are essential for reliable navigation in complex environments.

### Real-Time Optimization
Navigation systems must be optimized for real-time performance while maintaining safety and accuracy.

## Summary

Navigation and obstacle handling in autonomous humanoid systems requires sophisticated integration of multiple technologies and approaches. The system must handle static and dynamic obstacles, incorporate social navigation principles, and provide robust recovery behaviors for various failure scenarios. Success depends on careful coordination between global planning, local planning, costmap management, and real-time optimization. When properly implemented, these systems enable safe and efficient navigation in complex, dynamic human environments while maintaining natural interaction patterns.