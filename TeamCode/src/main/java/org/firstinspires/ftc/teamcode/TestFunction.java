package org.firstinspires.ftc.teamcode;

import java.util.function.Function;

public class TestFunction implements Function <Float, Float> {
    public Float apply(Float X){
        return X * X;
    }
}
