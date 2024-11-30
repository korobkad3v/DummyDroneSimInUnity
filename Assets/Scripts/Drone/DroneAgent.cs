using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class DroneAgent : Agent
{   
    public float rewards = 0f;
    private float timeInTarget = 0f;
    private float timeLimit = 10f;
    public DroneController droneController;

    public override void OnEpisodeBegin()
    {
        timeInTarget = 0f;
        


        transform.position = new Vector3 (Random.Range(-10, 10), 1, Random.Range(-10, 10));
        transform.rotation = Quaternion.identity;
        

        Rigidbody[] _rbs = GetComponentsInChildren<Rigidbody>();

        foreach (Rigidbody rb in _rbs)
        {
            rb.linearVelocity  = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            rb.transform.localRotation  = Quaternion.identity;
            rb.transform.localPosition = Vector3.zero;
            rb.Sleep();
        }

        droneController.globalTarget.transform.position = new Vector3(
            Random.Range(-5f, 5f),
            Random.Range(1f, 5f),
            Random.Range(-5f, 5f)
        );
    }

    public override void CollectObservations(VectorSensor sensor)    
    {
        sensor.AddObservation(transform.position);
        sensor.AddObservation(transform.rotation.eulerAngles);

        sensor.AddObservation(droneController);
        sensor.AddObservation(droneController.globalTarget);
        sensor.AddObservation(droneController.positionError);
        sensor.AddObservation(droneController.localPositionError);
        sensor.AddObservation(droneController.pitchPID.getValues());
        sensor.AddObservation(droneController.rollPID.getValues());
        sensor.AddObservation(droneController.yawPID.getValues());

        sensor.AddObservation(droneController.globalTarget.transform.position);

    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // actions
        droneController.yawPID.SetParameters(actions.ContinuousActions[0], actions.ContinuousActions[1], actions.ContinuousActions[2]);
        droneController.pitchPID.SetParameters(actions.ContinuousActions[3], actions.ContinuousActions[4], actions.ContinuousActions[5]);
        droneController.rollPID.SetParameters(actions.ContinuousActions[6], actions.ContinuousActions[7], actions.ContinuousActions[8]);

        float throttle = Mathf.Clamp(actions.ContinuousActions[9], 0f, 1f);
        
        float pitchCorrection = droneController.pitchPID.Update(droneController.positionError.x, Time.fixedDeltaTime);
        float rollCorrection = droneController.rollPID.Update(droneController.positionError.z, Time.fixedDeltaTime);
        float yawCorrection = droneController.yawPID.Update(droneController.positionError.y, Time.fixedDeltaTime);
        
        foreach (var motor in droneController.GetBrushlessMotors())
        {   
            //droneController.ApplyMotorForces(motor, throttle, pitchCorrection, rollCorrection, yawCorrection);
        }
        float distanceToTarget = Vector3.Distance(transform.position, droneController.globalTarget.transform.position);

        rewards += -distanceToTarget * 0.1f;
        AddReward(rewards);
    }

    private void OnTriggerStay(Collider other) {
        if (other.gameObject.CompareTag("target")) {
            timeInTarget += Time.deltaTime;
            rewards += timeInTarget / timeLimit;
            SetReward(rewards);
            if (timeInTarget >= timeLimit) {
                     rewards = 1f;
                SetReward(rewards);
                EndEpisode();
            }
        }
    }

    private void OnTriggerExit(Collider other) {
        if (other.gameObject.CompareTag("target")) {
            timeInTarget = 0f;
        }
    }
}
