using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LineDrawer : MonoBehaviour
{
    private LineRenderer lineRenderer;
    private float lineLength = 0f;

    void Start()
    {
        // Get Line Renderer component
        lineRenderer = GetComponent<LineRenderer>();

        // Set initial position
        Vector3 startPosition = new Vector3(0f, 0f, 0f);
        lineRenderer.SetPosition(0, startPosition);
        lineRenderer.SetPosition(1, startPosition);
    }

    void Update()
    {
        // Get input for line expansion (e.g., input axis)
        float inputAxis = Input.GetAxis("Vertical"); // Change to your desired input

        // Update line length based on input
        lineLength += inputAxis * Time.deltaTime;

        // Set the end position of the line
        Vector3 endPosition = new Vector3(0f, 0f, lineLength);
        lineRenderer.SetPosition(1, endPosition);
    }
}
