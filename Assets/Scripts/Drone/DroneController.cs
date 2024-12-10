using System;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.InputSystem;
using TMPro;

public class DroneController : MonoBehaviour
{
    public float voltage = 11.7f; // V
    private BrushlessMotor[] _motors;
    private Prop[] _props;
    private Frame _frame;
    
    public Vector3 targetAngles;

    public TMP_Text LastInputText;

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

    private float[] motorForces;
    private float yawInputValue = 0f;

    public BrushlessMotor[] GetBrushlessMotors() {
        return _motors;
    }

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        _motors = GetComponentsInChildren<BrushlessMotor>();
        _frame = GetComponentInChildren<Frame>();
        _props = GetComponentsInChildren<Prop>();
        targetAngles = _frame.transform.eulerAngles;
    }

    private void Update()
    {
        var device = InputSystem.GetDevice<InputDevice>();
        targetAngles.y += yawInputValue * Time.deltaTime * 30f;
    }

    void FixedUpdate()
    {   
        Vector3 currentAngles = _frame.transform.eulerAngles;

        Debug.Log($"Target angles: {targetAngles}");

        float currentYaw = currentAngles.y;
        float currentPitch = currentAngles.x;
        float currentRoll = currentAngles.z;
        Debug.Log($"currentYaw: {currentYaw}, currentPitch: {currentPitch}, currentRoll: {currentRoll}");

        float errorYaw = Mathf.DeltaAngle(currentYaw, targetAngles.y);
        float errorPitch = Mathf.DeltaAngle(currentPitch, targetAngles.x);
        float errorRoll = Mathf.DeltaAngle(currentRoll, targetAngles.z) * -1f;

        Debug.Log($"errorYaw: {errorYaw}, errorPitch: {errorPitch}, errorRoll: {errorRoll}");

        float correctionYaw = yawPID.Update(errorYaw, Time.deltaTime) / 180;
        float correctionPitch = pitchPID.Update(errorPitch, Time.deltaTime) / 180;
        float correctionRoll = rollPID.Update(errorRoll, Time.deltaTime) / 180;
        
        pitchInput = correctionPitch;
        rollInput = correctionRoll;
        yawInput = correctionYaw;
        
        motorForces = new float[_motors.Length];

        

        for (int i = 0; i < _motors.Length; i++){
            motorForces[i] = ApplyMotorForces(_motors[i], throttleInput, pitchInput, rollInput, yawInput);
        }

        for (int i = 0; i < _props.Length; i++){
            
            _props[i].RotateProp(i%2==0,motorForces[i]);
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

        Vector3 forceDirection = motor.transform.up;
        motorRigidbody.AddForce(forceDirection * motorForce);
    
        Debug.DrawLine(motor.transform.position, motor.transform.position + forceDirection * motorForce * 0.1f, Color.blue);
        
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

        if (device is Gamepad gamepad)
        {
            throttleInput = value.Get<float>();
           
        }
        else if (device is Keyboard)
        {
            increaseThrottle = input > 0;
            decreaseThrottle = input < 0;

        }
        LastInputText.text = $"Throttle: {input}, Device: {device.name}";
        ApplyTorque(motorForces[0], motorForces[1], motorForces[2], motorForces[3]);
        
            
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
        yawInputValue = value.Get<float>();
    }

    public void OnRoll(InputValue value)
    {
        var device = InputSystem.GetDevice<InputDevice>();
        float input = value.Get<float>();
        
        targetAngles.z = -45f * input;
    }
    
}

