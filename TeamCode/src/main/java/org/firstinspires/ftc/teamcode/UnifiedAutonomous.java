/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;
*/
/**
 * Created by FTCThales on 12/4/2017.
 */
/*
@Autonomous(name="UnifiedAutonomous", group="Linear Opmode")

public class UnifiedAutonomous extends LinearOpMode { //Unified autonomous class portable to both Red and Blue
    @Disabled
    //Define var for main robot hardware
    HardwareDrive robot = new HardwareDrive();
    //Basic elapsed runtime
    private ElapsedTime runtime = new ElapsedTime();
    //Runner for persistently recording the last position
    private RelicRecoveryVuMark lastPerceivedPositon;
    private int lastRedVal; //Runner for last perceived value of red; averages out with each iteration over the color sensor.
    public void killJewel(java.lang.String color) { //Pushes jewel off platform; takes team color as arg
        sensorColor = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "sensor_color_distance"); //Object for color sensor
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance"); //Object for distance sensor
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId); //
        parameters.vuforiaLicenseKey = "AfdO5vj/////AAAAGSpM5tOflkvMvW4RzPkR14sF7ZtBXS06d04V0BL1s3kqEDkbvcN9uoHhoUg+hPC5pKqRAuhHfpPvv6sNrQgXO6gJaL3kzjIOlcOhx35mONJDaQ4lu3cYAxeNISUTaUkmlTajAcqhGeCLj+m+0lNjg2lF3UmfzocsFnwl8Oi6117s9MDLo3/HFTmYw/QLVnSsvdUW6GRg7jnDG1sJJmTXtOkgmbHAGrvqUSevnxjnEw9w2ME69SsbZof7/J3Xyl38xE1ekM8qn3/nC4CsQF5xJFJkbnI4h9aATJx5szNP1Zu1CSON4+WSzynZrd7H4zcVA3rQZvqEuMsQ5OlKsOlsIWdLctOLXSHTcXh7+1iXU+DS";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; //Sets camera to rear for HiDef, might be more convenient to use front in round
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters); //Basic call to Vuforia factory func
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark"); //Basic reference to Vuforia trackable
        VuforiaTrackable relicTemplate = relicTrackables.get(0); //Trackable reference for pictogram; passed to Vuforia for object construction
        relicTemplate.setName("relicVuMarkTemplate"); //Useful for debugging, but not necessary.
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        relicTrackables.activate();
        if (color == "RED") {
            if (sensorColor.red() > sensorColor.blue() && sensorColor.green() < sensorColor.red()) {
                //Move robot in direction of red jewel
            }
            else {
                //Move robot away from "not red" jewel
            }
        }
        else {
            if (sensorColor.red() > sensorColor.blue() && sensorColor.red() > sensorColor.green()) {
                //Move robot away from red jewel
            }
            else {
                //Move robot towards "not red" jewel
            }
        }
    }
    private com.qualcomm.robotcore.hardware.ColorSensor sensorColor; //Color sensor object
    private DistanceSensor sensorDistance; //Distance sensor object
    private VuforiaLocalizer vuforia; //Vuforia import
    int x;
    //Below is to be moved into new definitions area; won't work until then
    while (x < 5) {
        if (vuMark.name() == lastPerceivedPositon) {

        }
    }
}
*/