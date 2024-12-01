using System;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.InputSystem;

public class DroneController : MonoBehaviour
{
    public float voltage = 11.7f; // V
    private BrushlessMotor[] _motors;
    private Frame _frame;
    
    public Vector3 targetAngles = Vector3.zero;

    public float throttleInput = 0f;
    public float pitchInput = 0f;
    public float yawInput = 0f;
    public float rollInput = 0f;
    public float inputStep = 1f; 
    public float inputSpeed = 1f; 
    
    private float throttleMinValue = 0f;
    private float throttleMaxValue = 1f;

    private float minValue = -1f;
    private float maxValue = 1f;
    
    public PIDController yawPID;
    public PIDController pitchPID;
    public PIDController rollPID;  

    private bool increaseThrottle = false;
    private bool decreaseThrottle = false;

    private bool increasePitch = false;
    private bool decreasePitch = false;

    private bool increaseYaw = false;
    private bool decreaseYaw = false;

    private bool increaseRoll = false;
    private bool decreaseRoll = false;

    private bool isControlling = false;

    private float inputDecaySpeed = 5f;

    public BrushlessMotor[] GetBrushlessMotors() {
        return _motors;
    }

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        _motors = GetComponentsInChildren<BrushlessMotor>();
        _frame = GetComponentInChildren<Frame>();
    }

    private void Update()
    {
        var device = InputSystem.GetDevice<InputDevice>();
        
        // if (device is Gamepad)
        // {
        //     throttleInput = Mathf.Clamp(throttleInput, throttleMinValue, throttleMaxValue);
        //     pitchInput = Mathf.Clamp(pitchInput, minValue, maxValue);
        //     yawInput = Mathf.Clamp(yawInput, minValue, maxValue);
        //     rollInput = Mathf.Clamp(rollInput, minValue, maxValue);

        // }
        // else if (device is Keyboard)
        // {
          
        //     if (increasePitch) pitchInput = Mathf.Clamp(pitchInput + inputStep * inputSpeed * Time.deltaTime, minValue, maxValue);
        //     if (decreasePitch) pitchInput = Mathf.Clamp(pitchInput - inputStep * inputSpeed * Time.deltaTime, minValue, maxValue);

        //     if (increaseYaw) yawInput = Mathf.Clamp(yawInput + inputStep * inputSpeed * Time.deltaTime, minValue, maxValue);
        //     if (decreaseYaw) yawInput = Mathf.Clamp(yawInput - inputStep * inputSpeed * Time.deltaTime, minValue, maxValue);

        //     if (increaseRoll) rollInput = Mathf.Clamp(rollInput + inputStep * inputSpeed * Time.deltaTime, minValue, maxValue);
        //     if (decreaseRoll) rollInput = Mathf.Clamp(rollInput - inputStep * inputSpeed * Time.deltaTime, minValue, maxValue);

        //     if (increaseThrottle) throttleInput = Mathf.Clamp(throttleInput + inputStep * inputSpeed * Time.deltaTime, throttleMinValue, throttleMaxValue);
        //     if (decreaseThrottle) throttleInput = Mathf.Clamp(throttleInput - inputStep * inputSpeed * Time.deltaTime, throttleMinValue, throttleMaxValue);
        // }

        
    }

    void FixedUpdate()
    {   
        Vector3 currentPosition =  _frame.transform.position;
        Vector3 currentAngles = _frame.transform.eulerAngles;
        Vector3 angularVelocity = _frame.GetComponent<Rigidbody>().angularVelocity;

        float currentYaw = currentAngles.y;
        float currentPitch = currentAngles.x;
        float currentRoll = currentAngles.z;

        float errorYaw = Mathf.DeltaAngle(currentYaw, targetAngles.y);
        float errorPitch = Mathf.DeltaAngle(currentPitch, targetAngles.x);
        float errorRoll = Mathf.DeltaAngle(currentRoll, targetAngles.z) * -1f;

        float correctionYaw = yawPID.Update(errorYaw, Time.deltaTime) / 180;
        float correctionPitch = pitchPID.Update(errorPitch, Time.deltaTime) / 180;
        float correctionRoll = rollPID.Update(errorRoll, Time.deltaTime) / 180;
        
        Debug.Log($"Error angles1: Yaw: {errorYaw}, Pitch: {errorPitch}, Roll: {errorRoll}");
        Debug.Log($"Correction angles: Yaw: {correctionYaw}, Pitch: {correctionPitch}, Roll: {correctionRoll}");

        
        pitchInput = correctionPitch;
        rollInput = correctionRoll;
        yawInput = correctionYaw;
        
        

        
        float[] motorForces = new float[_motors.Length];

        

        for (int i = 0; i < _motors.Length; i++){
            motorForces[i] = ApplyMotorForces(_motors[i], throttleInput, pitchInput, rollInput, yawInput);

        }

        ApplyTorque(motorForces[0], motorForces[1], motorForces[2], motorForces[3]);
        Debug.Log($"FR Force: {motorForces[0]}, FL Force: {motorForces[1]}, RR Force: {motorForces[2]}, RL Force: {motorForces[3]}");
    }

    public float ApplyMotorForces(BrushlessMotor motor, float throttle, float pitch, float roll, float yaw) 
    {
        float baseThrust = motor.MaxThrust * throttle;
        
        Rigidbody motorRigidbody = motor.GetComponent<Rigidbody>();
        
        float motorForce = 0;
        if (motor.gameObject.CompareTag("motorFR")) {
            motorForce = baseThrust - pitch - roll - yaw; //CCW
        }
        else if (motor.gameObject.CompareTag("motorFL")) {
            motorForce = baseThrust - pitch + roll + yaw; //CW
        }
        else if (motor.gameObject.CompareTag("motorRR")) {
            motorForce = baseThrust + pitch - roll + yaw; //CCW
        }
        else if (motor.gameObject.CompareTag("motorRL")) {
            motorForce = baseThrust + pitch + roll - yaw; //CW
        }

        motorRigidbody.AddForce(motor.transform.up * motorForce);
        Debug.DrawLine(motor.transform.position, motor.transform.position + motor.transform.up * motorForce * 0.1f, Color.red);
        
        return motorForce;
        
    }

    public void ApplyTorque(float frForce, float flForce, float rrForce, float rlForce) {
        float torque = (frForce + rlForce) - (flForce + rrForce);
        Rigidbody frameRigidbody = GetComponentInChildren<Frame>().GetComponent<Rigidbody>();
        frameRigidbody.AddTorque(transform.up * torque * -1f, ForceMode.VelocityChange);
    }



    public void OnThrottle(InputValue value)
    {
        var device = InputSystem.GetDevice<InputDevice>();
        float input = value.Get<float>();

        if (device is Gamepad)
        {
            throttleInput = value.Get<float>();
        }
        else if (device is Keyboard)
        {
            increaseThrottle = input > 0;
            decreaseThrottle = input < 0;

        }

        ApplyTorque(motorForces[0], motorForces[1], motorForces[2], motorForces[3]);
        
    }
    

    public float ApplyMotorForces(BrushlessMotor motor, float throttle, float pitch, float roll, float yaw) 
    {
        float baseThrust = motor.MaxThrust * throttle;
        
        Rigidbody motorRigidbody = motor.GetComponent<Rigidbody>();
        
        float motorForce = 0;
        if (motor.gameObject.CompareTag("motorFR")) {
            motorForce = baseThrust - pitch - roll - yaw; //CCW
        }
        else if (motor.gameObject.CompareTag("motorFL")) {
            motorForce = baseThrust - pitch + roll + yaw; //CW
        }
        else if (motor.gameObject.CompareTag("motorRR")) {
            motorForce = baseThrust + pitch - roll + yaw; //CCW
        }
        else if (motor.gameObject.CompareTag("motorRL")) {
            motorForce = baseThrust + pitch + roll - yaw; //CW
        }

        motorRigidbody.AddForce(motor.transform.up * motorForce, ForceMode.Force);
        Debug.DrawLine(motor.transform.position, motor.transform.position + motor.transform.up * motorForce * 0.1f, Color.yellow);
        
        return motorForce;
        
    }

    public void ApplyTorque(float frForce, float flForce, float rrForce, float rlForce) {
        float torque = (frForce + rlForce) - (flForce + rrForce);
        Rigidbody frameRigidbody = GetComponentInChildren<Frame>().GetComponent<Rigidbody>();
        frameRigidbody.AddTorque(transform.up * torque * -1f, ForceMode.VelocityChange);
    }



    public void OnThrottle(InputValue value)
    {
        var device = InputSystem.GetDevice<InputDevice>();
        float input = value.Get<float>();
        if (device is Gamepad)
        {
            throttleInput = value.Get<float>();
        }
        else if (device is Keyboard)
        {
            increaseThrottle = input > 0;
            decreaseThrottle = input < 0;
            if (input == 0)
            {
                increasePitch = false;
                decreasePitch = false;
            }
        }
    }

    public void OnPitch(InputValue value)
    {
        var device = InputSystem.GetDevice<InputDevice>();
        float input = value.Get<float>();
        
        targetAngles.x = 45f * input;

        
    }

    public void OnYaw(InputValue value)
    {
        var device = InputSystem.GetDevice<InputDevice>();
        float input = value.Get<float>();
        
        targetAngles.y = float.MaxValue* input;
    }

    public void OnRoll(InputValue value)
    {
        var device = InputSystem.GetDevice<InputDevice>();
        float input = value.Get<float>();
        
        targetAngles.z = -45f * input;
    }
    


    float NormalizeAngle(float angle)
    {
        angle = (angle + 360) % 360;
        if (angle > 180) angle -= 360;
        return angle;
    }
}

