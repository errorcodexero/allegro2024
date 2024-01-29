package org.xero1425.simulator.models;

import org.xero1425.base.gyro.Pigeon2Gyro;
import org.xero1425.base.gyro.XeroGyro;
import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.SDSSwerveDriveSubsystem;
import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.RobotController;

public class SDSSwerveDriveModel extends SimulationModel {
    SDSSwerveDriveSubsystem db_ ;
    final SimulationEngine engine_ ;
    private SDSSwerveModuleModel fl_ ;
    private SDSSwerveModuleModel fr_ ;
    private SDSSwerveModuleModel bl_ ;
    private SDSSwerveModuleModel br_ ;
    private Pigeon2 imu_ ;

    public SDSSwerveDriveModel(final SimulationEngine engine, final String model, final String inst) {
        super(engine, model, inst) ;

        engine_ = engine ;
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
        db_ = (SDSSwerveDriveSubsystem)db ;   
        XeroGyro gyro = db_.getGyro() ;
        if (!(gyro instanceof Pigeon2Gyro)) {
            throw new Exception("SDSSwerveDriveModel required a Pigeon 2 gyro");
        }
        imu_ = ((Pigeon2Gyro)gyro).getPigeon2() ;

        setCreated();
        
        return true ;
    }

    @Override
    public void run(final double dt) {
        if (fl_ == null || fr_ == null || bl_ == null || br_ == null) {
            getModuleModels() ;
        }

        imu_.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage()) ;
        imu_.setYaw(0.0) ;
    }    
    
    @Override
    public boolean processEvent(final String name, final SettingsValue value) {
        return false;
    }

    private void getModuleModels() {
        fl_ = (SDSSwerveModuleModel)engine_.getModelByNameInst("sds-swerve-module", "fl") ;
        addDependentModel(fl_);

        fr_ = (SDSSwerveModuleModel)engine_.getModelByNameInst("sds-swerve-module", "fr") ;
        addDependentModel(fr_);

        bl_ = (SDSSwerveModuleModel)engine_.getModelByNameInst("sds-swerve-module", "bl") ;
        addDependentModel(bl_);

        br_ = (SDSSwerveModuleModel)engine_.getModelByNameInst("sds-swerve-module", "br") ;
        addDependentModel(br_);
    }
}
