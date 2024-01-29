package org.xero1425.simulator.models;

import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.SDSSwerveDriveSubsystem;
import org.xero1425.base.subsystems.swerve.SwerveModule;
import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.RobotController;

public class SDSSwerveModuleModel extends SimulationModel {
    private ISimMotorController steer_ ;
    private ISimMotorController drive_ ;
    private CANcoder encoder_ ;
    private double angle_ ;

    public SDSSwerveModuleModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst) ;
    }

    @Override
    public boolean create(SimulationEngine engine) throws Exception {
        DriveBaseSubsystem db = engine.getRobot().getRobotSubsystem().getDB() ;
        if (!(db instanceof SDSSwerveDriveSubsystem)) {
            return false ;
        }
        SDSSwerveDriveSubsystem swdb = (SDSSwerveDriveSubsystem)db ;        
        SwerveModule module = swdb.getModule(text2Module(getInstanceName())) ;

        steer_ = createSimulatedMotor(engine, "steer");
        drive_ = createSimulatedMotor(engine, "drive") ;
        encoder_ = module.getCANCoder() ;

        encoder_.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
        encoder_.getSimState().setRawPosition(0.0) ;            

        setCreated();

        return true ;
    }

    @Override
    public void run(double dt) {

        encoder_.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
        encoder_.getSimState().setRawPosition(angle_) ;

        steer_.run(dt) ;
        drive_.run(dt) ;
    }

    @Override
    public boolean processEvent(String name, SettingsValue value) {
        return false;
    }

    private int text2Module(String text) throws Exception {
        int ret = -1 ;

        if (text.equals("fl")) {
            ret = SDSSwerveDriveSubsystem.FL ;
        }
        else if (text.equals("fr")) {
            ret = SDSSwerveDriveSubsystem.FR ;
        }
        else if (text.equals("bl")) {
            ret = SDSSwerveDriveSubsystem.BL ;
        }
        else if (text.equals("br")) {
            ret = SDSSwerveDriveSubsystem.BR ;
        }
        else {
            throw new Exception("SwerveModule: invalid module instance, must be 'fl', 'fr', 'bl', or 'br'") ;
        }

        return ret ;
    }
}
