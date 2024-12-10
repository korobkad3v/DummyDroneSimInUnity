using UnityEngine;

public class AddTorque : MonoBehaviour
{   
    [SerializeField]
    private float torque = 10f;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        GetComponent<Rigidbody>().maxAngularVelocity = 100f;
    }

    // Update is called once per frame
    void Update()
    {
        GetComponent<Rigidbody>().AddTorque(transform.up * torque, ForceMode.VelocityChange);
    }
}

