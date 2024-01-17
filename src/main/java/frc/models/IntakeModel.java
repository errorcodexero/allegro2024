package frc.models;

import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;
import org.xero1425.simulator.models.ISimMotorController;

public class IntakeModel extends SimulationModel {
    ISimMotorController up_down_ ;
    ISimMotorController rotate_ ;
    
    public IntakeModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst) ;
    }

    @Override
    public boolean create(SimulationEngine engine) throws Exception {
        double ticksPerVoltPerSecond ;
        
        ticksPerVoltPerSecond = getDoubleProperty("updown:ticks_per_second_per_volt") ;
        up_down_ = createSimulatedMotor(engine, "updown", ticksPerVoltPerSecond);

        ticksPerVoltPerSecond = getDoubleProperty("rotate:ticks_per_second_per_volt") ;
        rotate_ = createSimulatedMotor(engine, "rotate", ticksPerVoltPerSecond);

        return false;
    }

    @Override
    public boolean processEvent(String name, SettingsValue value) {
        return false ;
    }

    @Override
    public void run(double dt) {
        up_down_.run(dt) ;
        rotate_.run(dt) ;
    }    
}
