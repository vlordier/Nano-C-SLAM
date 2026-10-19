# Coordinator rejection after complete host mapping capture

The fresh domain-55 run completed its mission with 856 odometry callbacks and
produced a valid 60-sample host mapping capture with 195 map points, 3 stored
constraints, 60 finite poses, and optimizer completion. The server library
observer reported `observed` for the automatically registered Gazebo PID and
382 mapped libraries. The combined coordinator deliberately rejected the run
because its live status subscriber's events did not match a contiguous prefix
of the mission log. This exposed lossy intermediate DDS status delivery; the
follow-up accepts only an ordered, log-matching subset and still requires the
terminal status. This directory is partial diagnostic evidence, not a passing
combined manifest.
