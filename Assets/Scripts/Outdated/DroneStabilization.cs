using UnityEngine;

public class DroneStabilization : MonoBehaviour
{
     public float roll, pitch, yaw;
    private float targetRoll, targetPitch, targetYaw;
    private float motorSpeedFront, motorSpeedBack, motorSpeedLeft, motorSpeedRight;

    // PID controllers for each axis
    private PID rollPID = new PID(1.0f, 0.1f, 0.01f);
    private PID pitchPID = new PID(1.0f, 0.1f, 0.01f);
    private PID yawPID = new PID(1.0f, 0.1f, 0.01f);

    public MotorController motorControllerFR, motorControllerFL, motorControllerBR, motorControllerBL;



    void Update()
    {
        // Read the gyro/accelerometer data (e.g., from MPU6050)
        roll = ReadRoll();
        pitch = ReadPitch();
        yaw = ReadYaw();

        // Target values for stabilization
        targetRoll = 0;
        targetPitch = 0;
        targetYaw = 0;

        // Calculate errors for each axis
        float rollError = targetRoll - roll;
        float pitchError = targetPitch - pitch;
        float yawError = targetYaw - yaw;

        // Apply PID controllers to compute adjustments
        float rollAdjustment = rollPID.Update(rollError);
        float pitchAdjustment = pitchPID.Update(pitchError);
        float yawAdjustment = yawPID.Update(yawError);

        // Calculate motor speeds based on PID adjustments
        motorSpeedFront = 1000 + rollAdjustment + pitchAdjustment - yawAdjustment;
        motorSpeedBack = 1000 - rollAdjustment + pitchAdjustment + yawAdjustment;
        motorSpeedLeft = 1000 + rollAdjustment - pitchAdjustment + yawAdjustment;
        motorSpeedRight = 1000 - rollAdjustment - pitchAdjustment - yawAdjustment;

        // Set the motor speeds through motor controller
        motorControllerFR.SetMotorSpeed(motorSpeedFront);
        motorControllerFL.SetMotorSpeed(motorSpeedBack);
        motorControllerBR.SetMotorSpeed(motorSpeedLeft);
        motorControllerBL.SetMotorSpeed(motorSpeedRight);
    }

    // Methods to read sensor data (example placeholder methods)
    private float ReadRoll() { return 0; }
    private float ReadPitch() { return 0; }
    private float ReadYaw() { return 0; }
}



// PID Controller class to manage the PID calculations
public class PID
{
    private float Kp, Ki, Kd;
    private float previousError, integral;

    public PID(float p, float i, float d)
    {
        Kp = p;
        Ki = i;
        Kd = d;
    }

    public float Update(float error)
    {
        integral += error * Time.deltaTime;
        float derivative = (error - previousError) / Time.deltaTime;
        previousError = error;

        return Kp * error + Ki * integral + Kd * derivative;
    }
}