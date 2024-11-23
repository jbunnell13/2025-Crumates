package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SpecimanArm {

    private String SPECIMAN_MOTOR_NAME = "specimanMotor";

    private DcMotor specimanMotor = null;
    private SpecimenClaw claw = null;

    public SpecimanArm(HardwareMap hardwareMap) {
        specimanMotor = hardwareMap.dcMotor.get(SPECIMAN_MOTOR_NAME);
        specimanMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        claw = new SpecimenClaw(hardwareMap);
    }

    public void update(Gamepad gamepad) {
        /*
        double power = 0;
        if(gamepad.dpad_up){
            ++power;
        } else if (gamepad.dpad_down){
            --power;
        }
        */
        update(gamepad.left_stick_y * 0.5);
        claw.update(gamepad);
        if( gamepad.left_bumper) { claw.grabSpecimen(); }
        else if( gamepad.right_bumper){ claw.releaseSpecimen(); }
    }

    public void update(double power) {
        specimanMotor.setPower(power);
    }

    //public void setPosition(double pos) {claw.setPosition(pos);}

    public void grabSpeciman(){ claw.grabSpecimen(); }
    public void releaseSpeciman(){ claw.releaseSpecimen(); }
    public void setClawJoint(double d) {
        claw.setClawJoint(d);
    }
}
