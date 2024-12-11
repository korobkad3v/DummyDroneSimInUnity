using UnityEngine;

public class GameManager : MonoBehaviour
{
    
    public DroneController player;
    public Transform spawnPoint;

    void Awake()
    {
        player.transform.position = spawnPoint.position;
    }
}
