# Ir. Paulo 2025
Code for Ir. Paulo 2025 Reefscape Off-Season Robot

<img width="2818" height="1878" alt="image" src="https://github.com/user-attachments/assets/2ff85f72-bebc-411c-8866-75b1175c294a" />


## Highlights

• Controls

Ir. Paulo uses a control board that merges driver and operator inputs into a single interface. The driver handles movement and scoring triggers while the operator controller provides overrides and fine control when needed.

• Vision

The Limelight camera provide AprilTag detection and pose estimation. The reef pose estimator filters desired tags based on target branches and fuses vision with odometry for alignment. Details are in our pose estimator.


## Packages

• [frc.robot.subsystems](https://github.com/1156UnderControl/Robot2025_Reefscape_Offseason/tree/main/src/main/java/frc/robot/subsystems)

Implements the climber, LED patterns, scoring mechanisms, and swerve drive modules.



• [frc.robot.pose_estimators](https://github.com/1156UnderControl/Robot2025_Reefscape_Offseason/tree/main/src/main/java/frc/robot/pose_estimators)

Provides pose estimation utilities that integrate vision and odometry.



• [frc.robot.pose_estimators](https://github.com/1156UnderControl/Robot2025_Reefscape_Offseason/tree/main/src/main/java/frc/robot/joysticks)

Houses driver and operator controller abstractions and the main control board mapping.



• [frc.robot.commands.Auto](https://github.com/1156UnderControl/Robot2025_Reefscape_Offseason/tree/main/src/main/java/frc/robot/commands/Auto)

Contains autonomous routines for scoring coral, removing algae, and positioning around the reef.



• [frc.robot.commands.Swerve](https://github.com/1156UnderControl/Robot2025_Reefscape_Offseason/tree/main/src/main/java/frc/robot/commands/Swerve)

Holds drive-related commands used for auto-aligning and autonomous scoring.
