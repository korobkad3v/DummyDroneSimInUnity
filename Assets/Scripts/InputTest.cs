
using UnityEngine;
using UnityEngine.InputSystem;

public class DroneInput : MonoBehaviour
{
    private float throttleInput = 0f;
    private float pitchInput = 0f;
    private float yawInput = 0f;
    private float rollInput = 0f;

    public float inputStep = 0.1f; // Шаг изменения для клавиатуры
    public float inputSpeed = 1f; // Скорость изменения
    private float minValue = -1f;
    private float maxValue = 1f;

    // Для throttle
    private float throttleMinValue = 0f;
    private float throttleMaxValue = 1f;

    // Флаги для удержания клавиш
    private bool increasePitch = false;
    private bool decreasePitch = false;
    private bool increaseYaw = false;
    private bool decreaseYaw = false;
    private bool increaseRoll = false;
    private bool decreaseRoll = false;
    private bool increaseThrottle = false;
    private bool decreaseThrottle = false;

    private void Update()
    {
        // Проверка типа устройства
        var device = InputSystem.GetDevice<InputDevice>();
        
        // Обновляем значения для каждой оси с учетом устройства ввода
        if (device is Gamepad)
        {
            // Для геймпада используем стандартное аналоговое значение
            throttleInput = Mathf.Clamp(throttleInput, throttleMinValue, throttleMaxValue);
        }
        else if (device is Keyboard)
        {
            // Для клавиатуры используем шаговое изменение
            if (increasePitch) pitchInput = Mathf.Clamp(pitchInput + inputStep * inputSpeed * Time.deltaTime, minValue, maxValue);
            if (decreasePitch) pitchInput = Mathf.Clamp(pitchInput - inputStep * inputSpeed * Time.deltaTime, minValue, maxValue);

            if (increaseYaw) yawInput = Mathf.Clamp(yawInput + inputStep * inputSpeed * Time.deltaTime, minValue, maxValue);
            if (decreaseYaw) yawInput = Mathf.Clamp(yawInput - inputStep * inputSpeed * Time.deltaTime, minValue, maxValue);

            if (increaseRoll) rollInput = Mathf.Clamp(rollInput + inputStep * inputSpeed * Time.deltaTime, minValue, maxValue);
            if (decreaseRoll) rollInput = Mathf.Clamp(rollInput - inputStep * inputSpeed * Time.deltaTime, minValue, maxValue);

            if (increaseThrottle) throttleInput = Mathf.Clamp(throttleInput + inputStep * inputSpeed * Time.deltaTime, throttleMinValue, throttleMaxValue);
            if (decreaseThrottle) throttleInput = Mathf.Clamp(throttleInput - inputStep * inputSpeed * Time.deltaTime, throttleMinValue, throttleMaxValue);
        }

        Debug.Log($"Device: {device.name}, Throttle: {throttleInput}, Pitch: {pitchInput}, Yaw: {yawInput}, Roll: {rollInput}");
    }

    public void OnThrottle(InputValue value)
    {
        var device = InputSystem.GetDevice<InputDevice>();
        float input = value.Get<float>();

        if (device is Gamepad)
        {
            // Для геймпада просто получаем значение от 0 до 1
            throttleInput = value.Get<float>();
        }
        else if (device is Keyboard)
        {
            // Для клавиатуры применяем шаговое изменение
            increaseThrottle = input > 0;
            decreaseThrottle = input < 0;
        }
    }

    public void OnPitch(InputValue value)
    {
        var device = InputSystem.GetDevice<InputDevice>();
        float input = value.Get<float>();

        if (device is Gamepad)
        {
            pitchInput = input;
        }
        else if (device is Keyboard)
        {
            increasePitch = input > 0;
            decreasePitch = input < 0;
        }
    }

    public void OnYaw(InputValue value)
    {
        var device = InputSystem.GetDevice<InputDevice>();
        float input = value.Get<float>();

        if (device is Gamepad)
        {
            yawInput = input;
        }
        else if (device is Keyboard)
        {
            increaseYaw = input > 0;
            decreaseYaw = input < 0;
        }
    }

    public void OnRoll(InputValue value)
    {
        var device = InputSystem.GetDevice<InputDevice>();
        float input = value.Get<float>();

        if (device is Gamepad)
        {
            rollInput = input;
        }
        else if (device is Keyboard)
        {
            increaseRoll = input > 0;
            decreaseRoll = input < 0;
        }
    }
}