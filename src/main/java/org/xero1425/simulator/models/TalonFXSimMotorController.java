package org.xero1425.simulator.models;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import org.xero1425.base.motors.IMotorController;
import org.xero1425.base.motors.TalonFXMotorController;
import org.xero1425.simulator.engine.SimulationEngine;

public class TalonFXSimMotorController extends SimMotorController {
    private final static double kTicksPerRev = 2048 ;

    private TalonFXMotorController motor_ ;
    private DCMotor dcmotor_ ;
    private DCMotorSim sim_ ;
    private double gearing_ ;
    private String bus_ ;
    private int canid_ ;
    private double voltage_ ;
    private double velocity_ ;
    private double position_ ;

    public TalonFXSimMotorController(SimulationEngine engine, String bus, int canid, int count, double gearing, double moment) throws Exception {
        super(engine, bus, canid);

        bus_ = bus ;
        canid_ = canid ;        

        IMotorController ctrl = engine.getRobot().getMotorFactory().getMotorController(bus, canid) ;
        if (!(ctrl.getNativeController() instanceof TalonFX)) {
            throw new Exception("motor on bus '" + bus_ + "', can id " + canid_ + " is not a TalonFX V6 motor");
        }

        dcmotor_ = DCMotor.getFalcon500(count) ;
        sim_ = new DCMotorSim(dcmotor_, gearing, moment) ; 
        gearing_ = gearing ;     
        
        motor_ = (TalonFXMotorController)ctrl ;            
        getState().Orientation = ChassisReference.Clockwise_Positive ;
    }

    @Override
    public void run(double dt) {
        TalonFXSimState state = getState() ;

        state.setSupplyVoltage(RobotController.getBatteryVoltage());

        voltage_ = state.getMotorVoltage() ;
        sim_.setInputVoltage(voltage_);
        sim_.update(dt) ;

        position_ = sim_.getAngularPositionRotations() * gearing_ ;
        velocity_ = sim_.getAngularVelocityRPM() / 60.0 * gearing_ ;

        state.setRawRotorPosition(position_) ;
        state.setRotorVelocity(velocity_) ;
        
        addPlotData(RobotController.getBatteryVoltage(), state.getMotorVoltage(), position_, velocity_) ;
    }

    @Override
    public double ticksPerRev() {
        return kTicksPerRev ;
    }

    @Override
    public double voltage() {
        return voltage_ ;
    }

    @Override
    public double position() {
        return position_ ;
    }

    @Override
    public double velocity() {
        return velocity_ ;
    }

    private TalonFXSimState getState() {
        return ((TalonFX)motor_.getNativeController()).getSimState() ;
    }
}
