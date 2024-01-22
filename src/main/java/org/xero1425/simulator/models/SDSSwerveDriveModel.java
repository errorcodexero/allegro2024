package org.xero1425.simulator.models;

import org.xero1425.base.gyro.Pigeon2Gyro;
import org.xero1425.base.gyro.XeroGyro;
import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.RobotSubsystem;
import org.xero1425.base.subsystems.swerve.SDSSwerveDriveSubsystem;
import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.RobotController;

public class SDSSwerveDriveModel extends SimulationModel {
    private SDSSwerveModuleModel fl_ ;
    private SDSSwerveModuleModel fr_ ;
    private SDSSwerveModuleModel bl_ ;
    private SDSSwerveModuleModel br_ ;
    private Pigeon2 imu_ ;

    public SDSSwerveDriveModel(final SimulationEngine engine, final String model, final String inst) {
        super(engine, model, inst) ;

        fl_ = null ;
        fr_ = null; 
        bl_ = null ;
        br_ = null ;
    }
    
    @Override
    public boolean create(SimulationEngine engine) throws Exception {
        DriveBaseSubsystem db = engine.getRobot().getRobotSubsystem().getDB() ;
        if (!(db instanceof SDSSwerveDriveSubsystem)) {
            return false ;
        }
        SDSSwerveDriveSubsystem swdb = (SDSSwerveDriveSubsystem)db ;   
        XeroGyro gyro = swdb.getGyro() ;
        if (!(gyro instanceof Pigeon2Gyro)) {
            throw new Exception("SDSSwerveDriveModel required a Pigeon 2 gyro");
        }
        imu_ = ((Pigeon2Gyro)gyro).getPigeon2() ;
        
        return true ;
    }

    @Override
    public void run(final double dt) {
        imu_.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage()) ;
        imu_.setYaw(0.0) ;
    }    
    
    @Override
    public boolean processEvent(final String name, final SettingsValue value) {
        return false;
    }
}
