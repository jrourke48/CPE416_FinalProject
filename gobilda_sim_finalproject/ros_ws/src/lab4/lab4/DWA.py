#!/usr/bin/env python3
"""
Dynamic Window Approach (DWA) Local Planner Node

This module implements a local path planner using the Dynamic Window Approach algorithm.
DWA samples velocities within a dynamically constrained window based on robot kinematics,
simulates forward trajectories, and selects the optimal velocity command based on multiple
objectives: heading towards goal, avoiding obstacles, and maintaining speed.

The algorithm operates in the velocity space (v, w) and considers:
- Physical velocity and acceleration limits of the robot
- Predicted trajectories using a differential drive motion model
- Multi-objective cost function for trajectory evaluation
"""

import math
from typing import Optional, List, Tuple
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, Pose
from nav_msgs.msg import Odometry, OccupancyGrid


class DwaLocalPlannerNode(Node):
    """
    ROS2 Node implementing Dynamic Window Approach for local planning.
    
    This node subscribes to odometry and costmap data, then publishes velocity
    commands that navigate the robot towards a goal while avoiding obstacles.
    The DWA algorithm runs at a fixed frequency to continuously update commands.
    """
    def __init__(self):
        """
        Initialize the DWA local planner node with ROS interfaces and parameters.
        
        Sets up publishers for velocity commands, subscribers for odometry and costmap data,
        and configures all DWA algorithm parameters including robot constraints, simulation
        settings, and cost function weights.
        """
        super().__init__('dwa_local_planner')

        # --- ROS interfaces ---
        # Publisher for velocity commands sent to the robot's motion controller
        self.cmd_pub = self.create_publisher(TwistStamped, '/gobilda/cmd_vel', 10)
        
        # Subscriber for odometry data from KISS-ICP (provides robot pose and velocity estimates)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )
        
        # Subscriber for local costmap data (provides obstacle information in the local area)
        self.localcostmap_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.local_costmap_callback,
            10
        )
        
        # --- Robot state variables ---
        # Goal state: (x, y, yaw, linear_velocity, angular_velocity) in world frame
        # Currently hard-coded; in a full system this would come from a global planner
        self.goal_state = (4, 0, 0, 0, 0)  # Target: x=4m, y=0m, no specific orientation/velocity
        
        # Current robot state: (x, y, yaw, linear_velocity, angular_velocity)
        # Updated continuously from odometry messages
        self.current_state = (0, 0, 0, 0, 0)
        
        # --- Latest sensor data ---
        # Store most recent messages to use in planning cycle
        self.latest_local_costmap = None  # OccupancyGrid message
        self.latest_odometry = None       # Odometry message
        
        # --- DWA physical limitations (robot kinematic constraints) ---
        # These limits define the feasible velocity space for the robot
        # Maximum linear velocity (m/s) - defines how fast the robot can move forward/backward
        self.vel_lim_x = 0.5
        
        # Maximum angular velocity (rad/s) - defines how fast the robot can rotate
        self.vel_lim_theta = 1.5
        
        # Maximum linear acceleration (m/s²) - limits how quickly velocity can change
        self.acc_lim_x = 0.5
        
        # Maximum angular acceleration (rad/s²) - limits how quickly rotation rate can change
        self.acc_lim_theta = 1.0
        
        # --- DWA trajectory simulation parameters ---
        # Time horizon for forward simulation of trajectories (seconds)
        # Longer simulation time allows better prediction but requires more computation
        self.sim_time = 2.0
        
        # Time step for numerical integration during simulation (seconds)
        # Smaller dt gives more accurate trajectories but increases computation
        self.sim_dt = 0.1
        
        # Number of linear velocity samples to evaluate in the dynamic window
        # More samples provide finer resolution but increase computation time
        self.vx_samples = 10
        
        # Number of angular velocity samples to evaluate in the dynamic window
        # More samples allow better turning maneuvers but increase computation
        self.vtheta_samples = 20
        self.count = 0
        
        # --- DWA cost function weights ---
        # These weights balance different objectives in trajectory evaluation
        # Higher weight means that objective is more important in decision making
        
        # Weight for heading cost - rewards trajectories that end closer to the goal
        self.heading_weight = 8.0
        
        # Weight for obstacle cost - penalizes trajectories that get too close to obstacles
        # Higher value makes robot more conservative around obstacles
        self.obstacle_weight = 4.0
        
        # Weight for velocity cost - rewards maintaining higher forward velocity
        # Encourages efficient motion when safe to do so
        self.velocity_weight = 0.5

        # Control loop timer - runs planning and command publishing at 10 Hz
        # This frequency balances responsiveness with computational load
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('DWA local planner node started')

    # ----------------- Callbacks -----------------

    def odom_callback(self, msg: Odometry):
        """
        Process incoming odometry messages from KISS-ICP lidar-based localization.

        This callback extracts the robot's current pose (position and orientation) and
        velocity (linear and angular) from the odometry message. The state is stored
        for use in the next planning cycle. KISS-ICP provides high-quality pose estimates
        by matching lidar scans.
        
        @param msg Incoming nav_msgs/msg/Odometry message containing pose and velocity.
        """
        # Store the raw message for potential additional processing
        self.latest_odometry = msg
        
        # Extract 2D position in the world frame (z component ignored for 2D planning)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Convert orientation from quaternion representation to yaw angle (rotation about z-axis)
        # Quaternions are used in ROS for numerical stability, but DWA works with Euler angles
        q = msg.pose.pose.orientation
        yaw = self.yaw_from_quat(q.x, q.y, q.z, q.w)

        # Extract linear velocity in the robot's forward direction
        vx = msg.twist.twist.linear.x
        
        # Extract angular velocity (rate of rotation about z-axis)
        vyaw = msg.twist.twist.angular.z

        # Update the current robot state tuple used by the DWA planner
        # Format: (x_position, y_position, heading_angle, forward_velocity, rotation_rate)
        # if self.count >= 10:
        #     self.get_logger().info(f'Odometry received: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        #     self.count = 0
        # else:
        #     self.count += 1
        self.current_state = (x, y, yaw, vx, vyaw)

    def local_costmap_callback(self, msg: OccupancyGrid):
        """
        Process incoming local costmap messages representing the environment.

        The local costmap is an occupancy grid that represents the space around the robot.
        Each cell contains a value indicating the probability of that cell being occupied
        by an obstacle (0 = free, 100 = occupied, -1 = unknown). This data is used by DWA
        to evaluate trajectory safety and avoid collisions.

        @param msg Incoming nav_msgs/msg/OccupancyGrid message with obstacle information.
        """
        # Store the latest costmap for use in the next planning cycle
        # The costmap is converted to obstacle points during trajectory evaluation
        self.latest_local_costmap = msg

    # ----------------- Main control loop -----------------

    def control_loop(self):
        """
        Main control loop that executes the DWA planning algorithm at regular intervals.
        
        This function is called periodically by the ROS timer (10 Hz). It checks that
        necessary sensor data is available, computes the optimal velocity command using
        DWA, and publishes the command to the robot. This closed-loop control enables
        continuous replanning as the robot moves and the environment changes.
        """
        # Safety check: ensure we have received odometry data before planning
        # Without odometry, we don't know the robot's current state
        # if self.latest_odometry is None:
        #     return
        
        # Safety check: ensure we have received costmap data before planning
        # Without a costmap, we cannot evaluate trajectory safety
        if self.latest_local_costmap is None:
            return

        # Check if goal is reached - stop if within tolerance
        x, y, _, _, _ = self.current_state
        gx, gy, _, _, _ = self.goal_state
        dist_to_goal = math.hypot(gx - x, gy - y)
        
        if dist_to_goal < 0.2:  # 20cm tolerance
            # Publish stop command
            cmd = TwistStamped()
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().info('Goal reached! Stopping.')
            return

        # Run the DWA algorithm to compute the best velocity command
        # This considers the robot's current state, desired goal, and obstacles
        cmd = self.compute_next_velo(
            state=self.current_state,
            goal=self.goal_state,
            costmap=self.latest_local_costmap
        )
        if self.count >= 10:
            self.get_logger().info(f'velocity commands: dx={cmd.twist.linear.x:.2f}, dyaw={cmd.twist.angular.z:.2f}')
            self.count = 0
        else:
            self.count += 1

        # Publish the computed velocity command to the robot's controller
        # The command will be executed until the next control cycle
        self.cmd_pub.publish(cmd)

    # ----------------- Helpers -----------------

    @staticmethod
    def yaw_from_quat(x, y, z, w) -> float:
        """
        Convert a quaternion to yaw angle (rotation about z-axis).
        
        Quaternions (x, y, z, w) are a 4D representation of 3D rotations that avoid
        singularities and gimbal lock. For 2D navigation, we only need the yaw angle,
        which represents rotation in the horizontal plane.
        
        @param x, y, z, w Components of the quaternion
        @return Yaw angle in radians (-π to π)
        """
        # Standard formula for extracting yaw from quaternion using atan2
        # This formula comes from converting quaternion to Euler angles
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def compute_next_velo(self,
                    state: Tuple[float, float, float, float, float],
                    goal: Tuple[float, float, float, float, float],
                    costmap: OccupancyGrid) -> TwistStamped:
        """
        Execute the complete DWA algorithm to find the optimal velocity command.
        
        This is the main DWA planning function that implements the following steps:
        1. Compute the dynamic window of feasible velocities based on current state
        2. Sample velocity pairs (v, w) uniformly within the dynamic window
        3. For each sample, simulate a trajectory forward in time
        4. Evaluate each trajectory using a multi-objective cost function
        5. Select the velocity command that produces the best trajectory
        
        The "dynamic" in Dynamic Window refers to how the feasible velocity space
        changes based on the robot's current velocity and acceleration constraints.
        
        @param state Current robot state: (x, y, yaw, linear_vel, angular_vel)
        @param goal Target goal state: (x, y, yaw, linear_vel, angular_vel)
        @param costmap OccupancyGrid containing obstacle information
        @return TwistStamped message with optimal linear and angular velocity commands
        """
        # Step 1: Compute the dynamic window - the set of velocities reachable in one timestep
        # This considers both absolute velocity limits and what's achievable given current velocity
        v_min, v_max, w_min, w_max = self.compute_dynamic_window(state)

        # Step 2: Generate a uniform grid of velocity samples within the dynamic window
        # Each sample is a (linear_velocity, angular_velocity) pair to test
        samples = self.sample_velocities(v_min, v_max, w_min, w_max)

        # Step 3: Convert the costmap from grid format to a list of obstacle points
        # This makes collision checking more efficient during trajectory evaluation
        obstacles = self.costmap_to_points(costmap)

        # Initialize tracking variables for the best trajectory found
        best_score = -float('inf')  # Start with worst possible score
        best_cmd = TwistStamped()          # Default to zero velocity if no good option found

        # Step 4 & 5: Evaluate each velocity sample by simulating and scoring its trajectory
        for v, w in samples:
            # Simulate robot motion under constant velocity (v, w) for sim_time seconds
            # Returns a sequence of (x, y, yaw) poses along the predicted path
            traj = self.rollout_trajectory(state, v, w)

            # Evaluate trajectory quality using multiple criteria:
            # - How close does it get to the goal?
            # - Does it collide with obstacles?
            # - Does it maintain good forward velocity?
            score = self.evaluate_trajectory(traj, goal, obstacles, v, w)

            # Keep track of the best trajectory found so far
            # Higher scores are better (rewards good behavior, penalizes bad behavior)
            if score > best_score:
                best_score = score
                best_cmd.twist.linear.x = v   # Store the linear velocity command
                best_cmd.twist.angular.z = w  # Store the angular velocity command

        # Return the velocity command that produced the best-scoring trajectory
        return best_cmd

    # ------------- Core DWA Components -------------
    
    def compute_dynamic_window(self, state: Tuple[float, float, float, float, float]) -> Tuple[float, float, float, float]:
        """
        Calculate the dynamic window - the set of velocities achievable in the next timestep.
        
        The dynamic window is determined by two constraints:
        1. Robot's absolute velocity limits (maximum speed in each direction)
        2. Acceleration limits (how much velocity can change in one timestep)
        
        The intersection of these constraints forms a "window" in velocity space that
        dynamically changes based on the robot's current velocity. This ensures all
        commanded velocities are physically achievable by the robot.
        
        @param state Current robot state: (x, y, yaw, current_linear_vel, current_angular_vel)
        @return Tuple of (v_min, v_max, w_min, w_max) defining the feasible velocity ranges
        """
        # Extract current velocities from the state
        x, y, yaw, vx, vtheta = state
        
        # Absolute velocity limits (maximum capabilities of the robot)
        max_v = self.vel_lim_x        # Maximum forward/backward linear velocity
        min_v = -self.vel_lim_x       # (symmetric limits)
        max_w = self.vel_lim_theta    # Maximum rotation rate
        min_w = -self.vel_lim_theta   # (symmetric limits)

        # Acceleration constraints
        acc_v = self.acc_lim_x        # Maximum linear acceleration
        acc_w = self.acc_lim_theta    # Maximum angular acceleration
        dt = self.sim_dt              # Time step for discretization

        # Compute reachable velocity ranges considering acceleration limits
        # The robot can only change velocity by (acceleration × timestep) in one cycle
        # Take the intersection of acceleration-limited and absolute-limited ranges
        
        # Linear velocity window: current velocity ± (max_acceleration × timestep)
        # Clamped to absolute velocity limits
        v_min = max(min_v, vx - acc_v * dt)  # Can't decelerate faster than acc_v
        v_max = min(max_v, vx + acc_v * dt)  # Can't accelerate faster than acc_v

        # Angular velocity window: same logic for rotation
        w_min = max(min_w, vtheta - acc_w * dt)
        w_max = min(max_w, vtheta + acc_w * dt)

        return v_min, v_max, w_min, w_max

    def sample_velocities(self,
                          v_min: float, v_max: float,
                          w_min: float, w_max: float) -> List[Tuple[float, float]]:
        """
        Generate a uniform grid of velocity samples within the dynamic window.
        
        This function discretizes the continuous velocity space into a finite set of
        samples to evaluate. The number of samples represents a trade-off:
        - More samples: better resolution, more likely to find optimal trajectory, but slower
        - Fewer samples: faster computation, but might miss good trajectories
        
        The samples form a 2D grid in (v, w) space, where v is linear velocity and
        w is angular velocity. Each combination represents a potential robot motion.
        
        @param v_min Minimum linear velocity in the dynamic window
        @param v_max Maximum linear velocity in the dynamic window
        @param w_min Minimum angular velocity in the dynamic window
        @param w_max Maximum angular velocity in the dynamic window
        @return List of (linear_velocity, angular_velocity) tuples to evaluate
        """
        vx_samples = int(self.vx_samples)    # Number of linear velocity samples
        w_samples = int(self.vtheta_samples) # Number of angular velocity samples

        # Generate linear velocity samples uniformly spaced across [v_min, v_max]
        vs = []
        if vx_samples <= 1:
            # Edge case: if only one sample requested, use the midpoint
            vs = [(v_min + v_max) / 2.0]
        else:
            # Calculate spacing between samples to cover the full range
            dv = (v_max - v_min) / max(vx_samples - 1, 1)
            vs = [v_min + i * dv for i in range(vx_samples)]

        # Generate angular velocity samples uniformly spaced across [w_min, w_max]
        ws = []
        if w_samples <= 1:
            # Edge case: if only one sample requested, use the midpoint
            ws = [(w_min + w_max) / 2.0]
        else:
            # Calculate spacing between samples to cover the full range
            dw = (w_max - w_min) / max(w_samples - 1, 1)
            ws = [w_min + i * dw for i in range(w_samples)]

        # Create all combinations of linear and angular velocities
        # This forms a grid in velocity space: total_samples = vx_samples × w_samples
        samples = []
        for v in vs:
            for w in ws:
                samples.append((v, w))
        return samples

    def rollout_trajectory(self,
                           state: Tuple[float, float, float, float, float],
                           v: float, w: float) -> List[Tuple[float, float, float]]:
        """
        Forward simulate robot motion using a differential drive kinematic model.
        
        This function predicts where the robot will be if it executes constant velocities
        (v, w) for the specified simulation time. The trajectory is computed using numerical
        integration with the differential drive kinematic equations:
        
        dx/dt = v * cos(yaw)  - Linear motion in x direction
        dy/dt = v * sin(yaw)  - Linear motion in y direction
        dyaw/dt = w           - Angular motion (rotation)
        
        The simulation assumes:
        - Constant velocity throughout the trajectory (no acceleration)
        - Perfect motion execution (no slippage or noise)
        - Instantaneous velocity changes (not physically realistic but acceptable for planning)
        
        @param state Current robot state: (x, y, yaw, current_v, current_w)
        @param v Linear velocity to apply during simulation
        @param w Angular velocity to apply during simulation
        @return List of (x, y, yaw) poses along the predicted trajectory
        """
        sim_time = self.sim_time  # Total duration to simulate forward
        dt = self.sim_dt          # Time step for numerical integration

        # Extract starting pose from current state
        # Note: we ignore the current velocities (vx, vtheta) and use the sampled (v, w) instead
        x, y, yaw, vx, vtheta = state

        traj = []  # Store the sequence of poses along the trajectory
        t = 0.0    # Current simulation time
        
        # Integrate forward using discrete time steps
        while t < sim_time:
            # Update position using forward Euler integration
            # The robot moves in the direction it's currently facing (yaw angle)
            x += v * math.cos(yaw) * dt   # X component of velocity
            y += v * math.sin(yaw) * dt   # Y component of velocity
            
            # Update heading (yaw angle) based on angular velocity
            yaw += w * dt
            
            # Store this pose as part of the trajectory
            traj.append((x, y, yaw))
            
            # Advance simulation time
            t += dt

        return traj

    # ------------- Trajectory Evaluation (Cost/Objective Functions) -------------

    def evaluate_trajectory(self,
                            traj: List[Tuple[float, float, float]],
                            goal: Tuple[float, float, float, float, float],
                            obstacles: List[Tuple[float, float]],
                            v: float,
                            w: float) -> float:
        """
        Evaluate the quality of a trajectory using a multi-objective cost function.
        
        DWA uses a weighted sum of multiple objectives to score each trajectory:
        1. Heading: How close does the trajectory get to the goal?
        2. Obstacle: How safe is the trajectory (distance to obstacles)?
        3. Velocity: Does the trajectory maintain good forward speed?
        
        The weights (heading_weight, obstacle_weight, velocity_weight) determine the
        relative importance of each objective. Tuning these weights is crucial for
        achieving desired behavior (e.g., more aggressive vs. more conservative).
        
        Higher scores indicate better trajectories. The velocity with the highest-scoring
        trajectory will be selected as the command to execute.
        
        @param traj Simulated trajectory as list of (x, y, yaw) poses
        @param goal Target goal state
        @param obstacles List of obstacle positions
        @param v Linear velocity used to generate this trajectory
        @param w Angular velocity used to generate this trajectory
        @return Scalar score (higher is better, -inf for invalid trajectories)
        """
        # Sanity check: ensure trajectory is not empty
        if not traj:
            return -float('inf')

        # Retrieve the tuned weight parameters for each objective
        heading_w = self.heading_weight    # Importance of reaching the goal
        obstacle_w = self.obstacle_weight  # Importance of safety/clearance
        velocity_w = self.velocity_weight  # Importance of maintaining speed

        # Compute individual cost components
        # Each function returns a scalar value for its respective objective
        heading_cost = self.heading_cost(traj, goal)      # Goal-reaching metric
        obstacle_cost = self.obstacle_cost(traj, obstacles) # Collision avoidance metric
        velocity_cost = self.velocity_cost(v)             # Speed maintenance metric

        # Combine all objectives into a single score using weighted sum
        # NOTE: Convention is to maximize score in DWA:
        # - Positive costs are good (e.g., high velocity, close to goal)
        # - Negative costs are bad (e.g., close to obstacles, far from goal)
        # The trajectory with the highest total score wins
        score = (
            heading_w * heading_cost +
            obstacle_w * obstacle_cost +
            velocity_w * velocity_cost
        )
        return score

    def heading_cost(self,
                     traj: List[Tuple[float, float, float]],
                     goal: Tuple[float, float, float, float, float]) -> float:
        """
        Evaluate how well a trajectory progresses towards the goal.
        
        This objective rewards trajectories whose endpoint is closer to the goal position.
        It's based on the Euclidean distance from the trajectory's final pose to the goal.
        
        The cost is negated so that:
        - Trajectories ending closer to the goal have higher (less negative) scores
        - Trajectories ending farther from the goal have lower (more negative) scores
        
        This encourages the planner to select velocities that make progress toward the goal.
        
        Alternative implementations might consider:
        - Angle alignment with goal heading
        - Distance along the entire trajectory, not just endpoint
        - Predicted time to reach goal
        
        @param traj Trajectory as list of (x, y, yaw) poses
        @param goal Target goal state: (goal_x, goal_y, goal_yaw, _, _)
        @return Negative distance to goal (higher is better, 0 is perfect)
        """
        # Extract goal position (ignore velocity components)
        gx, gy, _, _, _ = goal
        
        # Get the final position from the trajectory
        # We evaluate where the robot will be at the end of the trajectory
        x_end, y_end, _ = traj[-1]

        # Calculate Euclidean distance from trajectory endpoint to goal
        dist = math.hypot(gx - x_end, gy - y_end)
        
        # Return negative distance so closer is better (maximization framework)
        # Example: dist=2.0 → cost=-2.0, dist=1.0 → cost=-1.0 (better)
        return -dist

    def obstacle_cost(self,
                      traj: List[Tuple[float, float, float]],
                      obstacles: List[Tuple[float, float]]) -> float:
        """
        Evaluate trajectory safety based on proximity to obstacles.
        
        This objective penalizes trajectories that pass too close to obstacles.
        It computes the minimum distance from any point along the trajectory to
        any obstacle. Trajectories are scored as follows:
        
        - Collision trajectory (min_dist < threshold): Completely invalid (-inf score)
        - Safe trajectory: Scored by clearance distance (more clearance = higher score)
        
        This encourages the robot to maintain safe distances from obstacles while
        still allowing it to navigate through narrow spaces when necessary.
        
        The collision threshold (currently 0.1m) should be set based on:
        - Robot physical dimensions (width, length)
        - Sensor uncertainty and localization error
        - Desired safety margin
        
        Potential improvements:
        - Use robot footprint instead of point-mass approximation
        - Weight obstacles by uncertainty or dynamic nature
        - Consider different penalties for different obstacle types
        - Use exponential penalty that increases rapidly near obstacles
        
        @param traj Trajectory as list of (x, y, yaw) poses
        @param obstacles List of obstacle positions as (x, y) tuples
        @return Minimum clearance distance, or -inf if collision detected
        """
        # If no obstacles present, trajectory is perfectly safe
        if not obstacles:
            return 0.0

        # Find the closest point of approach between trajectory and any obstacle
        # We check every pose in the trajectory against every obstacle (brute force)
        min_dist = float('inf')
        for x, y, _ in traj:
            for ox, oy in obstacles:
                # Calculate Euclidean distance from this trajectory point to this obstacle
                d = math.hypot(ox - x, oy - y)
                if d < min_dist:
                    min_dist = d

        # Check for collision: if trajectory passes too close to an obstacle,
        # mark it as completely invalid by returning negative infinity
        # This ensures collision trajectories are never selected, regardless of other costs
        collision_threshold = 0.30  # meters - tune based on robot size and safety requirements
        if min_dist < collision_threshold:
            return -float('inf')  # Invalid trajectory - would cause collision

        # Safe trajectory: return clearance distance as score
        # Higher clearance = higher score = safer trajectory
        return min_dist

    def velocity_cost(self, v: float) -> float:
        """
        Encourage the robot to maintain forward velocity for efficient motion.
        
        This objective rewards higher forward velocities, promoting efficient progress
        when it's safe to do so. The simple linear relationship means:
        - Faster forward motion = higher score
        - Zero or backward motion = lower score
        
        This helps prevent the robot from unnecessarily slowing down or stopping
        when a clear path exists. However, this objective is balanced against
        heading and obstacle costs, so the robot will slow down when necessary
        for safety or to make sharp turns.
        
        Alternative implementations might:
        - Penalize backward motion more strongly
        - Use a nonlinear function to limit preference for very high speeds
        - Consider energy efficiency
        - Penalize high angular velocities (smoother paths)
        
        @param v Linear velocity command (positive = forward, negative = backward)
        @return Velocity score (higher forward velocity = higher score)
        """
        # Simple linear reward: directly proportional to forward velocity
        # This encourages fast motion when safe
        return v
    
    # ------------- Occupancy Grid Processing -------------

    def costmap_to_points(self, costmap: OccupancyGrid) -> List[Tuple[float, float]]:
        """
        Extract obstacle locations from an occupancy grid costmap.
        
        The OccupancyGrid represents the environment as a 2D grid where each cell
        contains an occupancy probability:
        - 0: Definitely free space
        - 100: Definitely occupied (obstacle)
        - -1: Unknown
        - Values in between: Varying confidence of occupancy
        
        This function converts the grid representation into a list of (x, y) points
        for efficient distance calculations during trajectory evaluation. Only cells
        above the occupancy threshold are considered obstacles.
        
        The conversion process:
        1. Iterate through all cells in the flattened grid array
        2. For occupied cells, convert grid indices to world coordinates
        3. Account for grid origin offset and cell resolution (meters/cell)
        
        Note: This is a simple extraction method. More sophisticated approaches could:
        - Inflate obstacles by robot radius for better safety margins
        - Cluster nearby occupied cells into larger obstacles
        - Filter out isolated noisy measurements
        - Use spatial data structures (e.g., KD-tree) for faster collision checking
        
        @param costmap OccupancyGrid message containing the local environment map
        @return List of (x, y) coordinates of obstacle points in world frame
        """
        points: List[Tuple[float, float]] = []
        
        # Extract costmap metadata for coordinate transformations
        width = costmap.info.width           # Number of cells in x direction
        height = costmap.info.height         # Number of cells in y direction
        resolution = costmap.info.resolution # Meters per cell
        origin_x = costmap.info.origin.position.x  # World x-coordinate of grid origin
        origin_y = costmap.info.origin.position.y  # World y-coordinate of grid origin
        
        # Iterate through all cells in the occupancy grid
        # The data is stored as a 1D array in row-major order
        for i, value in enumerate(costmap.data):
            # Occupancy threshold: cells with value > 50 are considered obstacles
            # This threshold provides some robustness to uncertainty
            if value > 50:
                # Convert 1D array index to 2D grid coordinates
                grid_x = i % width   # Column index (x coordinate in grid frame)
                grid_y = i // width  # Row index (y coordinate in grid frame)
                
                # Transform from grid coordinates to world coordinates
                # Add 0.5 to get the center of the cell rather than corner
                x = origin_x + (grid_x + 0.5) * resolution
                y = origin_y + (grid_y + 0.5) * resolution
                
                # Add this obstacle point to the list
                points.append((x, y))
        
        return points



def main(args=None):
    """
    Main entry point for the DWA local planner node.
    
    Initializes the ROS2 Python client library, creates the DWA planner node,
    and enters the ROS2 event loop. The node will continue running until
    interrupted (Ctrl+C), processing callbacks and publishing velocity commands
    at the configured rate.
    
    @param args Command-line arguments (optional, passed to rclpy.init)
    """
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the DWA local planner node
    node = DwaLocalPlannerNode()
    
    # Enter the ROS2 event loop - this will block until shutdown
    # Callbacks (timer, subscribers) will be processed during spin
    rclpy.spin(node)
    
    # Cleanup: destroy the node and shutdown ROS2
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    # Execute main function when script is run directly
    main()
