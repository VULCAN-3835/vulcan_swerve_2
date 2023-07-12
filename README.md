# Swerve code project

# Overview of an FRC Swerve Code Project

An FRC swerve code project is a software development endeavor that aims to create a code for swerve drive robots.

Swerve drive is a type of drivetrain used in FIRST Robotics Competition (FRC) robots that provides excellent maneuverability and control. It enables a robot to move in any direction with precision and speed.

for now this project is not a part of the main Programming training and is only done by @Wyvern .

it will be a grate practice for Tomer to learn working with git and clean code (skills for a team-lead).

# Pyhesical Swerve Characteristics

- 4 swerve modules - SDS MK4 LL2
- 2 falcon500 motors in each module - one for driving and one for steering
- A Cancoder absolut value encoder capble of
- quare drive base with a size of 500x500 mm
- LimeLight camera - ask Amit for translation and rotation
(translation  = fancy way to say position)

# targets / features

A list of features I think needs the new swerve to have

## must have features

- [ ]  ‘’SwerveModule.java’’ have ‘’set(SwerveModuleState)’’
- [ ]  ‘’SwerveChassisSubsystem.java’’ have ‘’drive(ChassisSpeeds, fieldRelative)’’ and ‘’getPose()’’
- [ ]  A static utill class for the limelight
- [ ]  Can drive paths from pathplaner
- [ ]  GUI showing the robot pose on the field
