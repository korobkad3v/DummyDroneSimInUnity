using UnityEngine;
public class WindAreaVisualizer : MonoBehaviour
{
    public WindZone windZone;
    public GameObject visualArea;

    void Start()
    {
        // Set the visual area size based on the wind zone radius
        if (windZone != null && visualArea != null)
        {
            float radius = windZone.radius; // The wind zone radius
            visualArea.transform.localScale = new Vector3(radius, radius, radius);
            visualArea.SetActive(true);  // Ensure it's visible
        }
    }

    void Update()
    {
        // Optionally, update the position of the visual area to follow the wind zone
        if (windZone != null)
        {
            visualArea.transform.position = windZone.transform.position;
        }
    }

    void OnDrawGizmos()
    {
        // Set Gizmo color (e.g., light blue for wind)
        Gizmos.color = Color.cyan;

        // Draw a sphere in the scene to represent the wind zone
        Gizmos.DrawWireSphere(transform.position, windZone.radius);
    }

}

