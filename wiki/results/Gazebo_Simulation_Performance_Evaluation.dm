# Gazebo Simulation Performance Evaluation

Simulation performance was evaluated using the **Real-Time Factor (RTF)**:

[
RTF=\frac{\text{Simulated Time}}{\text{Wall Clock Time}}
]

An RTF close to **1.0** indicates real-time simulation.

## Results

| Configuration       | GUI     | RGB Camera  | LiDAR | Robot Motion  | Average RTF     |
| ------------------- | ------- | ----------- | ----- | ------------- | --------------- |
| Original simulation | **Yes** | **Enabled** | No    | None          | **≈ 0.13–0.20** |
| Camera disabled     | No      | Disabled    | No    | None          | **≈ 0.96–0.97** |
| Camera disabled     | No      | Disabled    | Yes   | None          | **≈ 0.96–0.97** |
| Camera disabled     | No      | Disabled    | Yes   | Random Walk   | **≈ 0.96**      |
| Camera disabled     | Yes     | Disabled    | Yes   | Manual motion | **≈ 0.62–0.65** |

## Discussion

The experiments show that the **RGB camera is the main computational bottleneck**. Disabling it increased the Real-Time Factor from approximately **0.13–0.20** to **0.96**, allowing the simulation to run almost in real time.

Adding the **2D LiDAR** had almost no measurable impact on the simulation speed. Likewise, running the Random Walk controller produced only a negligible reduction in the RTF.

Re-enabling the Gazebo GUI reduced the RTF to approximately **0.63**, showing that graphical rendering remains a significant source of computational load even after the camera is removed.

## Conclusion

For reinforcement learning experiments, the recommended configuration is **headless mode with the RGB camera disabled and the 2D LiDAR enabled**. This configuration achieves an RTF close to **1.0** while still providing reliable laser-based perception for navigation and obstacle avoidance.

