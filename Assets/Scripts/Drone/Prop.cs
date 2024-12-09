using UnityEngine;
using UnityEngine.Rendering.Universal;

public class Prop : MonoBehaviour
{   
    public float rotateModifier = 10f;
    public float rotateSpeedMin = 10;


    // Update is called once per frame
    public void RotateProp(bool isClockwise, float rotateSpeed)
    {
        if (isClockwise)
        {
            transform.Rotate(Vector3.up, (rotateSpeed + rotateSpeedMin) * rotateModifier * Time.deltaTime);
        }
        else
        {
            transform.Rotate(Vector3.down, (rotateSpeed + rotateSpeedMin) * rotateModifier * Time.deltaTime);
        }
    }
}
