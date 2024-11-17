using UnityEngine;

public class BrushlessMotor : MonoBehaviour
{
    public float KV = 2300f; // RPM/Volt
    private float voltage; // V
    public float motorEfficiency = 0.85f; // 0-1
    public float propellerDiameter = 0.127f; // meters ( 0.127f = 5inch)

    private float rpm;
    public float MaxThrust;

    public float airDensity = 1.225f; // kg/m^3
    public float propellerArea; // m^2
    public float C_L = 0.1f; // Lift coefficient approx.


    private Rigidbody _rigidbody;


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Awake()
    {
        _rigidbody = GetComponent<Rigidbody>();

        voltage = GetComponentInParent<DroneController>().voltage;
        rpm =  KV * voltage;
        // basicly area of a circle
        propellerArea = Mathf.PI * Mathf.Pow(propellerDiameter / 2, 2);
        
        // linear velocity
        float velocity = rpm * propellerDiameter * Mathf.PI / 60; // м/с

        MaxThrust = C_L * airDensity * propellerArea * Mathf.Pow(velocity, 2) * motorEfficiency;

    }
}
