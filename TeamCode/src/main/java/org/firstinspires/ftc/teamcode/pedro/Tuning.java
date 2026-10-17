package org.firstinspires.ftc.teamcode.pedro;

import org.firstinspires.ftc.teamcode.pedro.procedures.TwoWheelTuner;
import org.firstinspires.ftc.teamcode.pedro.tuning.Procedure;
import org.firstinspires.ftc.teamcode.pedro.tuning.Tuner;

public class Tuning {

    @Tuner
    public static Procedure twoWheelTuner() {
        return new TwoWheelTuner();
    }
}
