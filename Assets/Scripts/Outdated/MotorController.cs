using UnityEngine;

public class MotorController : MonoBehaviour
{
    private const float motorSpeed = 1000;
    private const float MAX_SPEED = 2000;  // Скорость вращения мотора
    public GameObject propeller;      // Пропеллер

    private Rigidbody rb;

    void Start()
    {
        rb.useGravity = false; // Отключаем гравитацию для мотора
    }

    void Update()
    {
        // Вращаем пропеллер
        propeller.transform.Rotate(Vector3.down * motorSpeed * Time.deltaTime);
    }

    public void SetMotorSpeed(float speed)
    {
        // Logic to apply PWM signals to the motors (this can vary based on the hardware used)
        Debug.Log($"Setting motor speeds:{speed}");
        
        // For example, in Unity you can control the motor's speed via physics or other simulation methods.
        // motorFront.SetSpeed(front);
        // motorBack.SetSpeed(back);
        // motorLeft.SetSpeed(left);
        // motorRight.SetSpeed(right);
    }
}