using System;
using UnityEngine;

public class DroneController : MonoBehaviour
{
    public float voltage = 11.7f; // V
    private BrushlessMotor[] _motors;
    
    public GameObject globalTarget;

    public float throttleCoeff = 0f;
    // public float pitchCoeff = 0f;
    // public float rollCoeff = 0f;
    // public float yawCoeff = 0f;

    
    public PIDController yawPID;
    public PIDController pitchPID;
    public PIDController rollPID;

    public Vector3 positionError;
    public Vector3 localPositionError;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        _motors = GetComponentsInChildren<BrushlessMotor>();
    }

    void FixedUpdate()
    {   
        throttleCoeff = Mathf.Clamp(throttleCoeff, 0f, 1f);

        // pitchCoeff = Mathf.Clamp(pitchCoeff, -1f, 1f);
        // rollCoeff = Mathf.Clamp(rollCoeff, -1f, 1f);
        // yawCoeff = Mathf.Clamp(yawCoeff, -1f, 1f);

        Vector3 currentPosition = transform.position;
        Vector3 currentAngles = transform.rotation.eulerAngles;
        float currentYaw = currentAngles.y;
        float currentPitch = currentAngles.x;
        float currentRoll = currentAngles.z;

        Debug.Log($"Current Yaw: {currentYaw}, Current Pitch: {currentPitch}, Current Roll: {currentRoll}");

        float pitchError = globalTarget.transform.eulerAngles.x - currentAngles.x;
        float rollError = globalTarget.transform.eulerAngles.z - currentAngles.z;
        float yawError = globalTarget.transform.eulerAngles.y - currentAngles.y;

        Debug.Log($"Pitch Error: {pitchError}, Roll Error: {rollError}, Yaw Error: {yawError}");

        positionError = globalTarget.transform.position - currentPosition;
        localPositionError = transform.InverseTransformDirection(positionError);

        Debug.Log($"Position Error (Global): {positionError}");
        Debug.Log($"Position Error (Local): {localPositionError}");

        float pitchCorrection = pitchPID.Update(pitchError, Time.fixedDeltaTime);
        float rollCorrection = rollPID.Update(rollError, Time.fixedDeltaTime);
        float yawCorrection = yawPID.Update(yawError, Time.fixedDeltaTime);

        Debug.Log($"Pitch Correction: {pitchCorrection}, Roll Correction: {rollCorrection}, Yaw Correction: {yawCorrection}");

        // Vector3 combinedError = new Vector3(localPositionError.x, yawError, localPositionError.z);

        // Debug.DrawLine(transform.position, transform.position + combinedError, Color.blue);
        foreach (var motor in _motors)
        {
            ApplyMotorForces(motor, throttleCoeff, pitchCorrection, rollCorrection, yawCorrection);
        }
        
    }

    public BrushlessMotor[] GetBrushlessMotors() {
        return _motors;
    }

    public void ApplyMotorForces(BrushlessMotor motor, float throttle, float pitchAdjustment, float rollAdjustment, float yawAdjustment) 
    {
        float baseThrust = motor.MaxThrust * throttle;
        Rigidbody motorRigidbody = motor.GetComponent<Rigidbody>();

        if (motor.gameObject.CompareTag("motorFR")) {
            motorRigidbody.AddForce(motor.transform.up *  (baseThrust + pitchAdjustment + rollAdjustment + yawAdjustment));
        }
        else if (motor.gameObject.CompareTag("motorFL")) {
            motorRigidbody.AddForce(motor.transform.up *  (baseThrust + pitchAdjustment - rollAdjustment - yawAdjustment));
        }
        else if (motor.gameObject.CompareTag("motorRR")) {
            motorRigidbody.AddForce(motor.transform.up *  (baseThrust - pitchAdjustment + rollAdjustment - yawAdjustment));
        }
        else if (motor.gameObject.CompareTag("motorRL")) {
            motorRigidbody.AddForce(motor.transform.up *  (baseThrust - pitchAdjustment - rollAdjustment + yawAdjustment));
        }
    }
}

