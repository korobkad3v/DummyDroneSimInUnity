using UnityEngine;

public class Frame : MonoBehaviour
{
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        Quaternion InitialRotation = transform.rotation;
        transform.rotation = InitialRotation;
    }

    // Update is called once per frame
    void Update()
    {
      Debug.DrawLine(transform.position, transform.position + Vector3.up * 3f, Color.red);

        
        
    }
}
