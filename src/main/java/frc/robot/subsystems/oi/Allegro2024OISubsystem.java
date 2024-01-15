package frc.robot.subsystems.oi;

import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.oi.OISubsystem;

public class Allegro2024OISubsystem extends OISubsystem {

    public Allegro2024OISubsystem(Subsystem parent, DriveBaseSubsystem db) {
        super (parent,"allegro2024oi",GamePadType.Swerve, db, true, true);
    }
}