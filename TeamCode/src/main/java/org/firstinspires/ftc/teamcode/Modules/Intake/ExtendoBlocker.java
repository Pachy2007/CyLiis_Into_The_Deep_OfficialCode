package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IServoModule;
import org.firstinspires.ftc.teamcode.Wrappers.BetterServo;

@Config
public class ExtendoBlocker extends IServoModule {

    public static boolean reversed=false;

    public static double close=0.67 , open=0.54;
    public static double time=0.007;

    public ExtendoBlocker(){
        moduleName="ExtendoBlocker";
        setServos(
                new BetterServo("Latch" , Hardware.sch5  , BetterServo.RunMode.Time , open , reversed , time)
        );

        setStates();
        atStart();
        state=states.get("goOpen");
    }

    @Override
    public void setStates() {
        states.addState("open" , open);
        states.addState("goOpen" , states.get("open") , open);

        states.addState("close" , close);
        states.addState("goClose" , states.get("close") , close);
    }

    @Override
    public void updateStatesPosition() {
        states.get("open").updatePositions(open);
        states.get("close").updatePositions(close);

        states.get("goOpen").updatePositions(open);
        states.get("goClose").updatePositions(close);
    }

    @Override
    public void atStart() {

    }
}
