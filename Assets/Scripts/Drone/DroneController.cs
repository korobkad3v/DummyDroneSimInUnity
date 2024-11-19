using System;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.InputSystem;

public class DroneController : MonoBehaviour
{
    public float voltage = 11.7f; // V
    private BrushlessMotor[] _motors;
    
    public GameObject globalTarget;

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

    public Vector3 positionError;
    public Vector3 localPositionError;
    

    private bool increaseThrottle = false;
    private bool decreaseThrottle = false;

    private bool increasePitch = false;
    private bool decreasePitch = false;

    private bool increaseYaw = false;
    private bool decreaseYaw = false;

    private bool increaseRoll = false;
    private bool decreaseRoll = false;

    public BrushlessMotor[] GetBrushlessMotors() {
        return _motors;
    }

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        _motors = GetComponentsInChildren<BrushlessMotor>();
    }

    private void Update()
    {
        var device = InputSystem.GetDevice<InputDevice>();
        
        if (device is Gamepad)
        {
            throttleInput = Mathf.Clamp(throttleInput, throttleMinValue, throttleMaxValue);
        }
        else if (device is Keyboard)
        {
          
            if (increasePitch) pitchInput = Mathf.Clamp(pitchInput + inputStep * inputSpeed * Time.deltaTime, minValue, maxValue);
            if (decreasePitch) pitchInput = Mathf.Clamp(pitchInput - inputStep * inputSpeed * Time.deltaTime, minValue, maxValue);

            if (increaseYaw) yawInput = Mathf.Clamp(yawInput + inputStep * inputSpeed * Time.deltaTime, minValue, maxValue);
            if (decreaseYaw) yawInput = Mathf.Clamp(yawInput - inputStep * inputSpeed * Time.deltaTime, minValue, maxValue);

            if (increaseRoll) rollInput = Mathf.Clamp(rollInput + inputStep * inputSpeed * Time.deltaTime, minValue, maxValue);
            if (decreaseRoll) rollInput = Mathf.Clamp(rollInput - inputStep * inputSpeed * Time.deltaTime, minValue, maxValue);

            if (increaseThrottle) throttleInput = Mathf.Clamp(throttleInput + inputStep * inputSpeed * Time.deltaTime, throttleMinValue, throttleMaxValue);
            if (decreaseThrottle) throttleInput = Mathf.Clamp(throttleInput - inputStep * inputSpeed * Time.deltaTime, throttleMinValue, throttleMaxValue);
        }

        Debug.Log($"Device: {device.name}, Throttle: {throttleInput}");
    }

    void FixedUpdate()
    {   
        throttleInput = Mathf.Clamp(throttleInput, 0f, 1f);

        Vector3 currentPosition = transform.position;
        Vector3 currentAngles = transform.rotation.eulerAngles;
        float currentYaw = currentAngles.y;
        float currentPitch = currentAngles.x;
        float currentRoll = currentAngles.z;

        float pitchError = globalTarget.transform.eulerAngles.x - currentAngles.x;
        float rollError = globalTarget.transform.eulerAngles.z - currentAngles.z;
        float yawError = globalTarget.transform.eulerAngles.y - currentAngles.y;

        positionError = globalTarget.transform.position - currentPosition;
        localPositionError = transform.InverseTransformDirection(positionError);
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
        }
    }

    public void OnPitch(InputValue value)
    {
        var device = InputSystem.GetDevice<InputDevice>();
        float input = value.Get<float>();

        if (device is Gamepad)
        {
            pitchInput = input;
        }
        else if (device is Keyboard)
        {
            increasePitch = input > 0;
            decreasePitch = input < 0;
        }
    }

    public void OnYaw(InputValue value)
    {
        var device = InputSystem.GetDevice<InputDevice>();
        float input = value.Get<float>();

        if (device is Gamepad)
        {
            yawInput = input;
        }
        else if (device is Keyboard)
        {
            increaseYaw = input > 0;
            decreaseYaw = input < 0;
        }
    }

    public void OnRoll(InputValue value)
    {
        var device = InputSystem.GetDevice<InputDevice>();
        float input = value.Get<float>();

        if (device is Gamepad)
        {
            rollInput = input;
        }
        else if (device is Keyboard)
        {
            increaseRoll = input > 0;
            decreaseRoll = input < 0;
        }
    }

}

