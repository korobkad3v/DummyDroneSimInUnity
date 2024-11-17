using UnityEngine;

[System.Serializable]
public class PIDController
{   
    
    [SerializeField]
    private float _kp, _ki, _kd;
    private float _previousError;
    private float _integral;

    public PIDController(float kp, float ki, float kd)
    {
        _kp = kp;
        _ki = ki;
        _kd = kd;
    }

    public void SetParameters(float kp, float ki, float kd){
        _kp = kp;
        _ki = ki;
        _kd = kd;
    }

    public float[] getValues(){
        return new float[]{_kp, _ki, _kd};
    }

    public float Update(float error, float deltaTime)
    {
        // Пропорциональная часть
        float pTerm = _kp * error;

        // Интегральная часть
        _integral += error * deltaTime;
        float iTerm = _ki * _integral;

        // Дифференциальная часть
        float dTerm = _kd * (error - _previousError) / deltaTime;

        _previousError = error;

        return pTerm + iTerm + dTerm;
    }
}