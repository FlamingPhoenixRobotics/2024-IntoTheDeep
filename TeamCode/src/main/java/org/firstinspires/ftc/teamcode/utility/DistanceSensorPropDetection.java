package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.jetbrains.annotations.NotNull;

public class DistanceSensorPropDetection {
    static double NO_DETECTION_THRESHOLD = 30; //In Inches
    int unusedSensor = 0; //The sensor which is blocked by the central truss and is unused
    DistanceSensor dl, dm, dr;
    public DistanceSensorPropDetection(@NotNull DistanceSensor dl_, @NotNull DistanceSensor dm_, @NotNull DistanceSensor dr_, int unusedSensor_){
        dl = dl_;
        dm = dm_;
        dr = dr_;
        unusedSensor = unusedSensor_;
    }

    //Returns either 0, 1, or 2. 0 = Left, 1 = Middle, 2 = Right
    public int getPropPos(){
        double currLeastDistance = 10000;
        int currLeastDistancePos = unusedSensor;

        if(dl.getDistance(DistanceUnit.INCH) < Math.min(NO_DETECTION_THRESHOLD, currLeastDistance) && unusedSensor != 0){
            currLeastDistance = dl.getDistance(DistanceUnit.INCH);
            currLeastDistancePos = 0;
        }
        if(dm.getDistance(DistanceUnit.INCH) < Math.min(NO_DETECTION_THRESHOLD, currLeastDistance) && unusedSensor != 1){
            currLeastDistance = dm.getDistance(DistanceUnit.INCH);
            currLeastDistancePos = 1;
        }
        if(dr.getDistance(DistanceUnit.INCH) < Math.min(NO_DETECTION_THRESHOLD, currLeastDistance) && unusedSensor != 2){
            currLeastDistance = dr.getDistance(DistanceUnit.INCH);
            currLeastDistancePos = 2;
        }

        return currLeastDistancePos;
    }
}
