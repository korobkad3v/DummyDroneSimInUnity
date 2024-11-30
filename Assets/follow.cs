using UnityEngine;

public class follow : MonoBehaviour
{
    public Transform target;
    public float smoothSpeed = 0.125f;
    public Vector3 offset;

    private Vector3 velocity = Vector3.zero;

    // LateUpdate is called once per frame
    void LateUpdate()
    {
        // Create a position the camera should be in based on the offset
        Vector3 targetCamPos = target.position + offset;

        // Smoothly move the camera towards that position
        transform.position = Vector3.SmoothDamp(transform.position, targetCamPos, ref velocity, smoothSpeed);
    }
}

