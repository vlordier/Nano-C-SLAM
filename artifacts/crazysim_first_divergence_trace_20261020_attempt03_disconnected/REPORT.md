# CrazySim disconnected control attempt 03: invalid capture

This attempt is retained for audit but excluded from evidence. Gazebo finished
before the sampler had discovered publishers and started receiving messages;
the sensor and actuator capture files were empty. The plugin's bounded trace
file alone is not a valid paired run. Attempt 04 corrected the startup order
by starting the sampler immediately before Gazebo.
