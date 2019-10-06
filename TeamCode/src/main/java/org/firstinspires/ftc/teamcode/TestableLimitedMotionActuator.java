package org.firstinspires.ftc.teamcode;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

public class TestableLimitedMotionActuator extends LimitedDcMotorDrivenActuator {

    public int maxLimit;
    public int minLimit;
    public double speed;
    TestableLimitedMotionActuator(final String MOTOR_NAME){
        super(MOTOR_NAME, 0, 0, DcMotorSimple.Direction.FORWARD,
                false, false, true,null,
                null, null, true,
                false, true, 0.1);
    }

    public void teleOpSwitchDirection(boolean switchDirectionButton)
    {
        if(switchDirectionButton)
        {
            switch (motor.getDirection()){
                case FORWARD:
                    motor.setDirection(DcMotorSimple.Direction.REVERSE);
                    break;

                case REVERSE:
                    motor.setDirection(DcMotorSimple.Direction.FORWARD);
                    break;

                default:
            }
        }
    }

    public void teleOpIncrementUpperLimit(boolean setUpperLimitIncrementer){
        if(setUpperLimitIncrementer)
            maxLimit++;
    }

    public void teleOpDecrementUpperLimit(boolean setUpperLimitDecremetner){
        if(setUpperLimitDecremetner && maxLimit >= minLimit){
            maxLimit--;
        }
    }

    public void teleOpIncrementLowerLimit(boolean setLowerLimitIncrementer){
        if(setLowerLimitIncrementer && minLimit <= maxLimit){
            minLimit++;
        }
    }

    public void teleOpDecrementLowerLimiter(boolean setLowerLimitDecrementer){
        if(setLowerLimitDecrementer)
            minLimit--;
    }

    public void teleOpSetSpeed(Gamepad gamepad){
        final double increment = 0.001;
        if (speed <= 1.0 && gamepad.dpad_right) //minium speed is 0
            speed += increment;

        else if(speed >= 0.0  && gamepad.dpad_left)
            speed -= increment;
    }

    @Override
    public void teleOpMoveWithButtons(boolean moveToMaxPosButton, boolean moveToMinPosButton, double speed) {
        if (moveToMaxPosButton || moveToMinPosButton)
        {
            if((motor.getCurrentPosition() <= maxLimit) && moveToMaxPosButton){
                speed = (Math.abs(speed));
            }

            else if ((motor.getCurrentPosition() >= minLimit) && moveToMinPosButton){
                speed = (-(Math.abs(speed)));
            }

            else speed = 0;
        }
        else speed = 0;

        motor.setPower(speed);
    }

}
