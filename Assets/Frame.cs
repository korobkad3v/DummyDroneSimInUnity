using UnityEngine;

public class Frame : MonoBehaviour
{
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
      Debug.DrawLine(transform.position, transform.position + Vector3.up * 3f, Color.red);

        
        
    }
}
