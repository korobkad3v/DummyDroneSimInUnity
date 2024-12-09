using UnityEngine;

public class FPVDroneCamera : MonoBehaviour
{
    public Transform drone; // Reference to the drone's transform
    public Vector3 cameraOffset = new Vector3(0, 0.2f, 0.5f); // Offset for camera placement relative to the drone
    public float tiltFactor = 10f; // How much the camera tilts during motion
    public float smoothSpeed = 5f; // Smooth interpolation speed

    private Quaternion initialRotation;

    void Start()
    {
        // Store the camera's initial local rotation relative to the drone
        initialRotation = transform.localRotation;
    }

    void LateUpdate()
    {
        if (drone == null)
        {
            Debug.LogWarning("Drone reference is missing.");
            return;
        }

        // Follow the drone's position with the offset
        transform.position = drone.TransformPoint(cameraOffset);

        // Calculate tilt based on the drone's movement (e.g., forward and sideways tilt)
        Rigidbody droneRigidbody = drone.GetComponent<Rigidbody>();
        if (droneRigidbody != null)
        {
            Vector3 localVelocity = drone.InverseTransformDirection(droneRigidbody.linearVelocity);
            float tiltX = Mathf.Clamp(localVelocity.z * tiltFactor, -20, 20); // Forward/backward tilt
            float tiltZ = Mathf.Clamp(localVelocity.x * tiltFactor, -20, 20); // Sideways tilt

            Quaternion tiltRotation = Quaternion.Euler(tiltX, 0, tiltZ);
            transform.rotation = Quaternion.Lerp(
                transform.rotation,
                drone.rotation * initialRotation * tiltRotation,
                smoothSpeed * Time.deltaTime
            );
        }
        else
        {
            // Fallback to simple alignment if no Rigidbody
            transform.rotation = Quaternion.Lerp(
                transform.rotation,
                drone.rotation * initialRotation,
                smoothSpeed * Time.deltaTime
            );
        }
    }
}
