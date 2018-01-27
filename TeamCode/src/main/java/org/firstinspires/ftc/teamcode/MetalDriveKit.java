package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by dcrenshaw on 1/27/18.
 *
 * The Metal Drive Kit is a framework for connecting directly to the robot hardware for driving.
 * Metal is intended to be a more feature-rich replacement for existing drive proxy functions.
 */

public class MetalDriveKit {
    //Define servo and motor variables and set them to null
    public DcMotor motor1 = null;
    public DcMotor motor2 = null;
    public DcMotor motor3 = null;
    public DcMotor motor4 = null;
    public DcMotor clawMotor = null;
    public Servo servo1 = null;
    public Servo servo2 = null;
    public Servo jewelServo = null;
    //Reference to mapped servo/motor controller
    HardwareMap hwMap = null;

    private ElapsedTime period = new ElapsedTime();

    public void init (HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Define and connect variables to their matching motors on the robot
        motor1 = hwMap.dcMotor.get("motor1");
        motor2 = hwMap.dcMotor.get("motor2");
        motor3 = hwMap.dcMotor.get("motor3");
        motor4 = hwMap.dcMotor.get("motor4");

        //Sets the motors to appropriate direction, FORWARD=Clockwise, REVERSE=CounterClockwise
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.FORWARD);
        //Set all motors to zero power = no movement
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
        // Set all motors to run without encoders.
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Define and initialize ALL installed servos. Here is an example snippet:
     /* leftClaw = hwMap.servo.get("left_hand");
        rightClaw = hwMap.servo.get("right_hand");
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);
    */
    }
}
