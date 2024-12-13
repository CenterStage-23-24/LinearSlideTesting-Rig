package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.w3c.dom.Node;

import java.util.PriorityQueue;
import java.util.Queue;

@Config
@TeleOp(name  = "Intake Op-Mode")
public class IntakeTesting extends OpMode{
    private DcMotorEx slideMotor;
    private ColorSensor colorSensor;
    private ColorRangeSensor rangeSensor;
    private Servo servo;
    double servoOpenPos =0.0, servoClosedPos = .3;
    double targetARGB = 150994944;
    double toleranceARGB = 10000000;
    int argb;
    double distanceThreshold = 30;
    double distance;

    double power = -1;



    public void init(){
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        colorSensor = hardwareMap.get(ColorSensor.class, "IntakeCS");
        rangeSensor = hardwareMap.get(ColorRangeSensor.class, "IntakeCS");
        servo = hardwareMap.get(Servo.class, "IntakeServo");

        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void loop() {

        distance = rangeSensor.getDistance(DistanceUnit.MM);

        if (distance < distanceThreshold) {
            servo.setPosition(servoOpenPos);
            power = -0.35;
        } else {
            power = -1;
            servo.setPosition(servoClosedPos);
        }



        slideMotor.setPower(power);

        telemetry.addData("ARGB", argb);
        telemetry.addData("Color Distance", Math.abs(argb - targetARGB));
        telemetry.addData("Range", distance);
        telemetry.update();
    }


}

class buffer {
    int size;
    int[] buffer;
    int counter = 0;

    public buffer(int bufferSize) {
        buffer = new int[bufferSize];
        size = bufferSize;
    }

    public void add(int input) {
        buffer[counter] = input;
        counter++;
        counter %= 5;
    }

    public double value(){
        int sum = 0;
        for (int i = 0; i > size; i++) {
            sum += buffer[i];
        }
        int mean = sum / size;

        return mean;
    }

}
