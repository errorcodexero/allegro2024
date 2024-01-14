package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.TestAutoMode;
import org.xero1425.base.subsystems.swerve.SDSSwerveDriveSubsystem;
import org.xero1425.base.subsystems.swerve.SwerveAlignDriveBaseAction;
import org.xero1425.base.subsystems.swerve.SwerveHolonomicPathFollower;
import org.xero1425.base.subsystems.swerve.SwervePowerAngleAction;
import org.xero1425.base.subsystems.swerve.SwerveSpeedAngleAction;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;

import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class AllegroTestAutoMode extends TestAutoMode {

    public AllegroTestAutoMode(AutoController ctrl) throws Exception {
        super(ctrl, "Allegro-Test-Mode");

        AllegroRobot2024 robotsubsystem = (AllegroRobot2024) ctrl.getRobot().getRobotSubsystem();

        SDSSwerveDriveSubsystem swerve = robotsubsystem.getSwerve();
        LimeLightSubsystem limelight = robotsubsystem.getLimelight();

        double angles[] = new double[4];
        double powers[] = new double[4];

        switch (getTestNumber()) {
            case 0:
                // Set the steering motor to the angle given, and the drive motor to the power
                // given. Run indefintely. Action will
                // stop the plot after the default plot interval (four seconds)
                addSubActionPair(swerve, new SwervePowerAngleAction(swerve, getDouble("angle"), getDouble("power")), true);
                break;

            case 1:
                // Set the steering motor to the angle given, and the drive motor to the power
                // given. Run until the duration has expired
                addSubActionPair(swerve, new SwervePowerAngleAction(swerve, getDouble("angle"), getDouble("power"),
                        getDouble("duration")), true);
                break;

            case 2:
                // Set the steering motor to the angle given, and the drive motor to the speed
                // given. Run indefintely. Action will
                // stop the plot after the default plot interval (four seconds). Since speed is
                // given, the PID controller will try to
                // maintain the target speed
                addSubActionPair(swerve, new SwerveSpeedAngleAction(swerve, getDouble("angle"), getDouble("speed")),
                        true);
                break;

            case 3:
                // Set the steering motor to the angle given, and the drive motor to the speed
                // given. Run until the duration has expired.
                // Since speed is given, the PID controller will try to maintain the target
                // speed
                addSubActionPair(swerve, new SwerveSpeedAngleAction(swerve, getDouble("angle"), getDouble("speed"),
                        getDouble("duration")), true);
                break;

            case 4:
                // Run the path follower against the path given
                addSubActionPair(swerve, new SwerveHolonomicPathFollower(swerve, getString("name"), true, 3.0), true);
                break;

            case 5:
                // Set the steering motor to the angle given, and the drive motor to the power
                // given. Run until the duration has expired
                angles[0] = getDouble("angle");
                angles[1] = getDouble("angle");
                angles[2] = getDouble("angle");
                angles[3] = getDouble("angle");
                powers[0] = getDouble("power");
                powers[1] = getDouble("power");
                powers[2] = getDouble("power");
                powers[3] = getDouble("power");
                addSubActionPair(swerve, new SwervePowerAngleAction(swerve, angles, powers, getDouble("duration")),
                        true);
                break;

            case 6:
                addSubActionPair(swerve, new SwerveAlignDriveBaseAction(swerve, limelight, 3.0), true);
                break;
        }
    }

}
