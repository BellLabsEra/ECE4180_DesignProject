using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LineMovement : MonoBehaviour
{
    public GameObject dashPrefab;
    public int numberOfDashes = 10;
    public float dashSpacing = 1f;

    private Vector3 startPosition;
    private Vector3 endPosition;

    void Start()
    {
        // Initial positions
        startPosition = Vector3.zero;
        endPosition = new Vector3(0f, 0f, dashSpacing);

        // Draw the initial line
        DrawDashes();
    }

    void Update()
    {
        // Update end position based on movement, input, or other criteria
        endPosition.z += Time.deltaTime;

        // Draw the line with dashes
        DrawDashes();
    }

    void DrawDashes()
    {
        // Clear existing dashes
        foreach (Transform child in transform)
        {
            Destroy(child.gameObject);
        }

        // Draw new dashes
        for (int i = 0; i < numberOfDashes; i++)
        {
            Vector3 dashPosition = Vector3.Lerp(startPosition, endPosition, (float)i / numberOfDashes);
            GameObject dash = Instantiate(dashPrefab, dashPosition, Quaternion.identity, transform);
        }

        // Update start position for the next iteration
        startPosition = endPosition;
    }
}
