using UnityEngine;

public class DroneController : MonoBehaviour
{
    public float voltage = 11.7f; // V
    private BrushlessMotor[] _motors;
    private Rigidbody _rigidbody;

    public Transform target; 
    public float targetRadius = 1f;

    [SerializeField]
    private float currentYaw = 0f;

    [SerializeField]
    private float currentPitch = 0f;

    [SerializeField]
    private float currentRoll = 0f;

    [SerializeField]
    private float yawError;

    [SerializeField]
    private float pitchError;

    [SerializeField]
    private float rollError;


    public float throttle = 0f;
    public float yaw = 0f;
    public float pitch = 0f;
    public float roll = 0f; 

    public PIDController yawPID;
    public PIDController pitchPID;
    public PIDController rollPID;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        _motors = GetComponentsInChildren<BrushlessMotor>();
        _rigidbody = GetComponent<Rigidbody>();

        yawPID = new PIDController(1f, 0.1f, 0f);
        pitchPID = new PIDController(1f, 0.1f, 0f);
        rollPID = new PIDController(1f, 0.1f, 0f);

    }

    // Update is called once per frame
    void FixedUpdate()
    {
        Vector3 targetDirection = target.position - transform.position;
        float distanceToTarget = targetDirection.magnitude;

        //if (distanceToTarget  targetRadius)
        {
            float targetYaw = Mathf.Atan2(targetDirection.x, targetDirection.z) * Mathf.Rad2Deg;
            float targetPitch = Mathf.Asin(targetDirection.y / distanceToTarget) * Mathf.Rad2Deg;
            float targetRoll = 0f;

            yawError = targetYaw - transform.eulerAngles.y;
            pitchError = targetPitch - transform.eulerAngles.x;
            rollError = targetRoll - transform.eulerAngles.z; 

            float yawCorrection = yawPID.Update(yawError, Time.fixedDeltaTime);
            float pitchCorrection = pitchPID.Update(pitchError, Time.fixedDeltaTime);
            float rollCorrection = rollPID.Update(rollError, Time.fixedDeltaTime);

            // Применение тяги и момента для всех моторов
            foreach (BrushlessMotor motor in _motors)
            {
                // Применение тяги по направлению вверх
                Rigidbody motorRigidbody = motor.GetComponent<Rigidbody>();
                Vector3 thrustDirection = motor.transform.up * throttle * motor.thrust;

                

                // Применение моментa для коррекции углов по каждой оси
                Vector3 torque = Vector3.zero;
                torque += motor.transform.up * yawCorrection;
                torque += motor.transform.right * pitchCorrection;
                torque += motor.transform.forward * rollCorrection;

                // Применение момента на rigidbody
                motorRigidbody.AddForce(thrustDirection);
                _rigidbody.AddTorque(torque);

            }
        }
    }
}

public class PIDController
{
    private float _kp, _ki, _kd;
    private float _previousError;
    private float _integral;

    public PIDController(float kp, float ki, float kd)
    {
        _kp = kp;
        _ki = ki;
        _kd = kd;
    }

    public float Update(float error, float deltaTime)
    {
        // Пропорциональная часть
        float pTerm = _kp * error;

        // Интегральная часть
        _integral += error * deltaTime;
        float iTerm = _ki * _integral;

        // Дифференциальная часть
        float dTerm = _kd * (error - _previousError) / deltaTime;

        _previousError = error;

        return pTerm + iTerm + dTerm;
    }
}