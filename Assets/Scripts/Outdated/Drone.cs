using UnityEngine;

public class Drone : MonoBehaviour
{
    public GameObject motorPrefab;  // Drag motor prefab in Unity inspector
    public float motorSpeed = 1.0f;
    private GameObject[] motors = new GameObject[4];  // 4 motors
    private Rigidbody rb;
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        CreateMotors();
    }

    void CreateMotors()
    {
        // Example of placing motors at the corners of a drone
        motors[0] = Instantiate(motorPrefab, transform.position + new Vector3(0.2f, 0.2f, 0.2f), Quaternion.identity); // Front-right
        motors[1] = Instantiate(motorPrefab, transform.position + new Vector3(-0.2f, 0.2f, 0.2f), Quaternion.identity); // Front-left
        motors[2] = Instantiate(motorPrefab, transform.position + new Vector3(0.2f, 0.2f, -0.2f), Quaternion.identity); // Back-right
        motors[3] = Instantiate(motorPrefab, transform.position + new Vector3(-0.2f, 0.2f, -0.2f), Quaternion.identity); // Back-left

        // Optionally attach Rigidbody to motors (if you want them to interact physically as well)
        foreach (var motor in motors)
        {
            Vector3 originalScale = motor.transform.localScale;
            motor.transform.SetParent(transform);
            motor.transform.localScale = originalScale;
            motor.AddComponent<Rigidbody>();
        }
    }

    void FixedUpdate()
    {
        // Apply forces for thrust
        ApplyThrust();
    }

    void ApplyThrust()
    {
        // Motor forces based on speed; adjust for your drone's design
        float thrust = motorSpeed * 1f; // Scale thrust for motor speed

        // Example thrust vectors for each motor
        rb.AddForceAtPosition(Vector3.up * thrust, motors[0].transform.position); // Front right motor
        rb.AddForceAtPosition(Vector3.up * thrust, motors[1].transform.position); // Front left motor
        rb.AddForceAtPosition(Vector3.up * thrust, motors[2].transform.position); // Rear right motor
        rb.AddForceAtPosition(Vector3.up * thrust, motors[3].transform.position); // Rear left motor
    }
}
