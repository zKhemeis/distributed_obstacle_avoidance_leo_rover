# Gazebo Simulation Performance Evaluation

The simulation performance was evaluated using the **Real-Time Factor (RTF)**:

$$
\mathrm{RTF}=\frac{\text{Simulated Time}}{\text{Wall Clock Time}}
$$

An RTF close to **1.0** indicates that the simulation is running at approximately real time.

---

## Results

| Configuration | GUI | RGB Camera | 2D LiDAR | Robot Motion | Average RTF |
|---------------|:---:|:----------:|:---------:|:------------:|:-----------:|
| Original simulation | Yes | Enabled | No | None | **≈ 0.13–0.20** |
| Camera disabled | No | Disabled | No | None | **≈ 0.96–0.97** |
| Camera disabled | No | Disabled | Yes | None | **≈ 0.96–0.97** |
| Camera disabled | No | Disabled | Yes | Random Walk | **≈ 0.96** |
| Camera disabled | Yes | Disabled | Yes | Manual motion | **≈ 0.62–0.65** |

---

## Discussion

The experiments show that the **RGB camera is the primary computational bottleneck**. Disabling the camera increased the Real-Time Factor from approximately **0.13–0.20** to **0.96–0.97**, allowing the simulation to run almost in real time.

Adding a **2D LiDAR** had **almost no measurable impact** on the simulation speed. Likewise, executing the Random Walk controller caused only a negligible decrease in the RTF, indicating that the computational cost of both the LiDAR sensor and the controller is relatively low.

When the Gazebo GUI was enabled again, the RTF dropped to approximately **0.62–0.65**. This demonstrates that graphical rendering remains a significant source of computational overhead, even after removing the RGB camera.

---

## Conclusion

For reinforcement learning experiments, the recommended configuration is:

- **Headless Gazebo**
- **RGB camera disabled**
- **2D LiDAR enabled**

This configuration consistently achieves an RTF close to **1.0**, while still providing reliable laser-based perception for navigation and obstacle avoidance.
