# MPC-Style-SCP-for-Quadrotor-Obstacle-Avoidance and Controller Design

**Model Predictive Control (MPC) Style Sequential Convex Programming (SCP) for Quadrotor Obstacle Avoidance**

This is an ongoing project, and more work will be added!

In this project, our aim is to create a robust Sequential Convex Programming (SCP) code for quadrotor obstacle avoidance. SCP works by creating convex subproblems from a non-convex problem—in our case, 3D quadrotor obstacle avoidance. This is achieved by linearizing the non-convexities and approximating them into convex subproblems, which are then solved iteratively within a trust region.

You can find the code for the Obstacle Avoidance SCP algorithm (A significant portion is currently censored as this is unpublished work. For access to the full code, feel free to email me).

[KutayDemiralay_Quadrotor_Obstacle_Avoidance_SimulinkData_Censored.ipynb](./KutayDemiralay_Quadrotor_Obstacle_Avoidance_SimulinkData_Censored.ipynb)


### Performance of SCP Code Without Wind

Below is how the SCP code performs without wind:

![SCP Obstacle Avoidance Without Wind](./images/SCPww.png)


**Figure 1:** As you can see, the node violation score is 0.0, meaning none of the nodes violate the obstacles in the route.

### Performance of SCP Code With Wind

However, when we add a constant wind force that the algorithm's dynamics are not aware of, the wind drags the drone off course, causing it to hit obstacles and drift away from our desired trajectory.

![SCP Quadrotor Obstacle Avoidance Under Presence of Wind](./images/SCPwow.png)

**Figure 2:** As you can see, the node violation score is around 41.72, meaning some of the nodes violate the obstacles in the route.

### Adding an MPC-Style Approach

To address challenges in trajectory optimization under disturbances such as wind, we have introduced an MPC-style approach to the Sequential Convex Programming (SCP) code. In this approach, the remaining trajectory is recalculated at each node, making the algorithm more responsive and adaptive to disturbances.



![Robust SCP-MPC in the presence of wind ](./images/SCPMPC.png)

**Figure 3:** The node violation score is now 0.0 despite the presence of strong wind. However, the fuel cost has increased to 351.65, which is nearly 10 times higher than in the case without wind. This indicates that while the trajectory is robust, it is not optimal in terms of fuel efficiency.


### Wind-Adaptive Residual Correction (WARC) technique:
To add lower-level robustness underneath the MPC of my quadcopter SCP algorithm, I use a novel technique to figure out the effect of wind from the previous node step.

#### Method:
1. **Wind Effect Estimation**:
   - After applying the control inputs calculated in the previous time step, I check where the drone ends up at the current node (using sensor data).
   - I compare this actual position (under wind) with where it would have been without wind. The deviation provides the impact of wind at that step.

2. **Wind Slope Calculation**:
   - By calculating the derivative (slope) of the displacement caused by the wind, I derive a correction factor to account for the wind disturbance.
   - This wind slope is added into the First-Order Hold (FOH) discretized matrices:
     \[
     x_{k+1} = A_k x_k + B_k u_k + B^+_k u_{k+1} + z_k
     \]
     where \( z_k \) (in the code, `z_bar`) represents the nonlinear residual, which accounts for the difference between the actual nonlinear system behavior and the linearized dynamics.

3. **Modification of `z_bar`**:
   - I modify `z_bar` to account for the wind's effect on the system. For the next time step, I use the wind effect from the previous step, assuming similar conditions in the upcoming step. 
   - This adjustment helps reduce drift caused by the wind without modifying the quadcopter's dynamics directly.

4. **MPC Integration**:
   - The wind information is incorporated into each MPC step, allowing the quadcopter to adjust its trajectory in real time.
   - This technique continuously corrects the trajectory using wind data while the main SCP algorithm operates as usual.

![Robust SCP-MPC, added with Wind-Adaptive Residual Correction  ](./images/SCPMPCwind.png)

**Figure 4:** The node violation score remains 0.0 despite the presence of strong wind. The fuel cost has now decreased to 343.01 from 351.65, achieved by adding the Wind-Adaptive Residual Correction (WARC) technique as a lower-level robustness layer beneath the existing robustifying MPC framework.


### Future Work: Developing a Robust Low-Level Controller

The current MPC-style approach alone appears insufficient for optimal trajectory planning under strong disturbances like wind. To improve this, future work will focus on developing a robust controller that works at a lower level than MPC. Options being considered include H-infinity synthesis or funnel synthesis, which would complement the MPC framework by ensuring robust trajectory optimization under dynamic conditions.

### Simulink Controller Design

An autonomous vehicle system has two key components: the first involves finding the optimal trajectory, and the second is designing a controller to ensure the vehicle follows that trajectory.

For this project, I designed a controller based on the principles outlined in "Quadcopter Modeling and Simulation Based on Parrot Minidrone." I simplified and adapted the model, tuned the PID controllers and other control parameters, and tailored the design to my specific quadcopter case. By integrating the commands from my SCP (Sequential Convex Programming) algorithm (same commands from Figure 1) , I developed a controller that effectively follows the optimized trajectory while accounting for the specific needs of my quadcopter.


The entire system was simulated using Simulink 3D Animation for visualization.

You can find the Simulink Model for the Quadcopter Obstalce Avoidance :

[QuadcopterObstacleAvoidanceSimulinkModel.slx](./QuadcopterObstacleAvoidanceSimulinkModel.slx)


![Simulink Main Block Diagram for Obstacle Avoidance Quadcopter Design](./images/SimulinkModel.png)

**Figure 3:** . Simulink Main Block Diagram for Obstacle Avoidance Quadcopter Design

![Quadcopter Trajectory Optimization](images/Trajoptgif.gif)

**Figure 4:** . Simulink Controller 3D Animation GIF (Same Scenario as Figure 1)
