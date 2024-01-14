package org.xero1425.base.subsystems.swerve.common;

public class SwerveModuleConfig {
    public final double wheel_diameter ;
    public final double drive_reduction ;
    public final boolean drive_inverted ;
    public final double steer_reduction ;
    public final boolean steer_inverted ;

    public SwerveModuleConfig(double diam, double driveReduction, boolean driveInverted, double steerReduction, boolean steerInverted) {
        this.wheel_diameter = diam ;
        this.drive_reduction = driveReduction ;
        this.drive_inverted = driveInverted ;
        this.steer_reduction = steerReduction ;
        this.steer_inverted = steerInverted ;
    }

    public static final SwerveModuleConfig MK4I_L1 = new SwerveModuleConfig(
            0.10033,
            (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0),
            true,
            (14.0 / 50.0) * (10.0 / 60.0),
            false
    );

    public static final SwerveModuleConfig MK4I_L2 = new SwerveModuleConfig(
            0.10033,
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            true,
            (14.0 / 50.0) * (10.0 / 60.0),
            false
    );
    
    public static final SwerveModuleConfig MK4I_L3 = new SwerveModuleConfig(
            0.10033,
            (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0),
            true,
            (14.0 / 50.0) * (10.0 / 60.0),
            false
    );
}
