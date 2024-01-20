package frc.robot.subsystems.oi;

import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.oi.Gamepad;
import org.xero1425.base.subsystems.oi.OISubsystem;
import org.xero1425.base.subsystems.oi.SwerveDriveGamepad;

public class Allegro2024OISubsystem extends OISubsystem {

    private final static SwerveDriveGamepad.SwerveButton[] resetButtons = { SwerveDriveGamepad.SwerveButton.Y, SwerveDriveGamepad.SwerveButton.B} ;
    private final static SwerveDriveGamepad.SwerveButton[] xActionButtons = { SwerveDriveGamepad.SwerveButton.LBack } ;

    public Allegro2024OISubsystem(Subsystem parent, DriveBaseSubsystem db) {
        super (parent,"allegro2024oi",GamePadType.Swerve, db, true, true);
    }

    @Override
    protected void gamePadCreated(Gamepad gp) {
        SwerveDriveGamepad swgp = (SwerveDriveGamepad)gp ;
        if (swgp != null) {
            swgp.setSwerveResetButtons(resetButtons);
            swgp.setSwerveXButtons(xActionButtons);
        }
    }
}
