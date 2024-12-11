using UnityEngine;

public class Frame : MonoBehaviour
{
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
    }

    public void OnTriggerStay(Collider other) {
        if (other.gameObject.TryGetComponent(out SignalZone signalZone)) {
            Debug.Log("Signal detected");
            GetComponentInParent<DroneController>().isSignalDetected = true;
        }
        Debug.Log("Trigger active with: " + other.name);
        
    }

    public void OnTriggerExit(Collider other) {

        if (other.gameObject.TryGetComponent(out SignalZone signalZone)) {
            Debug.Log("Signal lost");
            GetComponentInParent<DroneController>().isSignalDetected = false;
        }
        Debug.Log("Trigger leave : " + other.name);
    }

    // Update is called once per frame
    void Update()
    {
        Debug.DrawLine(transform.position, transform.position + Vector3.up * 3f, Color.red);

        

    }
}
