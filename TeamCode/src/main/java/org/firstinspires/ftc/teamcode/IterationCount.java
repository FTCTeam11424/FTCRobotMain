package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by dcrenshaw on 1/27/18.
 */

@TeleOp(name="IterationCount", group="Pushbot")
public class IterationCount extends OpMode {
    HardwareDrive robot = new HardwareDrive();
    public int cIteratedFor = 0;
    @Override
    public void init() {
        robot.motor1 = hardwareMap.dcMotor.get("motor1");
        robot.motor2 = hardwareMap.dcMotor.get("motor2");
        robot.motor3 = hardwareMap.dcMotor.get("motor3");
        robot.motor4 = hardwareMap.dcMotor.get("motor4");
        robot.clawMotor = hardwareMap.dcMotor.get("clawMotor");
        robot.servo1 = hardwareMap.servo.get("servo1");
        robot.servo2 = hardwareMap.servo.get("servo2");
        robot.jewelServo = hardwareMap.servo.get("jewelServo");
    }
    public void loop() {
        if (gamepad2.left_stick_y == -1) {
            if (cIteratedFor <= 100) {
                robot.clawMotor.setPower(1);
                cIteratedFor++;
            }
        }
    }
}
