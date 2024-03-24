package org.xero1425.base.controllers;

import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.RobotSubsystem;
import org.xero1425.base.subsystems.swerve.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.SwerveHolonomicPathFollowerAction;
import org.xero1425.base.subsystems.swerve.SwervePowerAngleAction;
import org.xero1425.base.subsystems.swerve.SwerveSpeedAngleAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class 
SwerveTestAutoMode extends TestAutoMode {

    public SwerveTestAutoMode(AutoController ctrl, String name)
            throws BadParameterTypeException, MissingParameterException {
        super(ctrl, name);
    }

    protected boolean createTest() throws Exception {
        double angles[] = new double[4];
        double powers[] = new double[4];

        RobotSubsystem robotsubsystem = getAutoController().getRobot().getRobotSubsystem();
        DriveBaseSubsystem db = robotsubsystem.getDB();

        if (db != null && !(db instanceof SwerveBaseSubsystem)) {
            throw new Exception(
                    "SwerveTestAutoMode is used as the base class of the test automode but the drive base is not swerve");
        }

        int tno = getTestNumber();
        if (tno >= 10)
            return false;

        SwerveBaseSubsystem swerve = (SwerveBaseSubsystem) db;

        if (swerve != null) {
            switch (getTestNumber()) {
                case 0:
                    if (hasParameter("duration")) {
                        addSubActionPair(swerve, new SwervePowerAngleAction(swerve, getDouble("angle"),
                                getDouble("power"), getDouble("duration")), true);
                    } else {
                        addSubActionPair(swerve,
                                new SwervePowerAngleAction(swerve, getDouble("angle"), getDouble("power")), true);
                    }
                    break;

                case 1:
                    if (hasParameter("duration")) {
                        addSubActionPair(swerve, new SwerveSpeedAngleAction(swerve, getDouble("angle"),
                                getDouble("speed"), getDouble("duration")), true);
                    } else {
                        addSubActionPair(swerve, new SwerveSpeedAngleAction(swerve, getDouble("angle"),
                                getDouble("speed"), getDouble("duration")), true);
                    }
                    break;

                case 2:
                    angles[0] = getDouble("angle-fl");
                    angles[1] = getDouble("angle-fr");
                    angles[2] = getDouble("angle-bl");
                    angles[3] = getDouble("angle-br");
                    powers[0] = getDouble("power-fl");
                    powers[1] = getDouble("power-fr");
                    powers[2] = getDouble("power-bl");
                    powers[3] = getDouble("power-br");
                    addSubActionPair(swerve, new SwervePowerAngleAction(swerve, angles, powers, getDouble("duration")),
                            true);
                    break;

                case 3:
                    angles[0] = getDouble("angle-fl");
                    angles[1] = getDouble("angle-fr");
                    angles[2] = getDouble("angle-bl");
                    angles[3] = getDouble("angle-br");
                    powers[0] = getDouble("speed-fl");
                    powers[1] = getDouble("speed-fr");
                    powers[2] = getDouble("speed-bl");
                    powers[3] = getDouble("speed-br");
                    addSubActionPair(swerve, new SwerveSpeedAngleAction(swerve, angles, powers, getDouble("duration")),
                            true);
                    break;

                case 4:
                    // Run the path follower against the path given
                    addSubActionPair(swerve, new SwerveHolonomicPathFollowerAction(swerve, getString("name"), true, 3.0, false, 0.0), true);
                    break;
            }
        }
        return true;
    }
}
