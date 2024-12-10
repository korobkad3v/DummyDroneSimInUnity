using UnityEngine;

public class DrawLineInGame : MonoBehaviour
{
    public Transform startPoint; // First point of the line
    public Transform endPoint; // Second point of the line
    private LineRenderer lineRenderer;

    void Start()
    {
        // Add and configure the LineRenderer
        lineRenderer = GetComponent<LineRenderer>();
    }

    void Update()
    {
        if (startPoint != null && endPoint != null)
        {
            // Set positions of the LineRenderer
            lineRenderer.SetPosition(0, startPoint.position);
            lineRenderer.SetPosition(1, endPoint.position);
        }
    }
}
