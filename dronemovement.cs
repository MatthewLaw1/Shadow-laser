using UnityEngine;
using TMPro;

public class DroneCameraMovement : MonoBehaviour
{
    [Header("Movement Settings")]
    public float baseSpeed = 0.02f;         // Horizontal speed in Unity units per second
    public float verticalSpeed = 0.0066f;   // Vertical speed in Unity units per second
    public float boostMultiplier = 2f;       // Speed multiplier when boost is active

    [Header("Mouse Look Settings")]
    public float mouseSensitivity = 100f;    // Sensitivity for mouse look
    private float pitch = 0f;                 // Vertical camera rotation (up/down)

    [Header("GPS Overlay Settings")]
    public TMP_Text cameraGPSOverlayText;     // Assign the GPS overlay UI Text element here

    [Header("GPS Mapping Constants")]
    private const double LAT_REF = 37.778369;
    private const double LON_REF = -122.390004;
    private const double C = 3915.367;
    private const double F = 1287.471;
    private const double A = 0.13350;
    private const double B = -0.000166;
    private const double D = -0.000174;
    private const double E = 0.13450;
    private const double METERS_PER_DEG_LAT = 111000;
    private double METERS_PER_DEG_LON;

    // Reference to the drone GameObject
    public GameObject drone; // Assign your drone GameObject in the Unity Inspector

    void Start()
    {
        // Initialize METERS_PER_DEG_LON based on average latitude
        double AVG_LAT_RAD = 37.896032 * Mathf.Deg2Rad;
        METERS_PER_DEG_LON = 111000 * Mathf.Cos((float)AVG_LAT_RAD);

        if (cameraGPSOverlayText == null)
        {
            Debug.LogError("Camera GPS Overlay Text is not assigned in the Inspector.");
        }

        if (drone == null)
        {
            Debug.LogError("Drone GameObject is not assigned in the Inspector.");
        }
    }

    void Update()
    {
        HandleMovement();
        HandleMouseLook();
        UpdateGPSOverlay();
    }

    void HandleMovement()
    {
        // Speed multiplier for boost
        float speedMultiplier = Input.GetKey(KeyCode.LeftShift) ? boostMultiplier : 1f;

        // WASD movement
        float moveHorizontal = Input.GetAxis("Horizontal") * baseSpeed * speedMultiplier; // A/D keys
        float moveVertical = Input.GetAxis("Vertical") * baseSpeed * speedMultiplier;     // W/S keys

        // Q/E vertical movement
        float moveUp = 0f;
        if (Input.GetKey(KeyCode.E)) // E for up
        {
            moveUp = verticalSpeed * speedMultiplier;
        }
        else if (Input.GetKey(KeyCode.Q)) // Q for down
        {
            moveUp = -verticalSpeed * speedMultiplier;
        }

        // Combine movement directions
        Vector3 movement = new Vector3(moveHorizontal, moveUp, moveVertical);

        // Apply movement to the camera
        transform.Translate(movement, Space.Self);
    }

    void HandleMouseLook()
    {
        // Mouse input for yaw (left/right) and pitch (up/down)
        float yaw = Input.GetAxis("Mouse X") * mouseSensitivity * Time.deltaTime;
        pitch -= Input.GetAxis("Mouse Y") * mouseSensitivity * Time.deltaTime;
        pitch = Mathf.Clamp(pitch, -90f, 90f); // Limit vertical look angle to avoid flipping

        // Apply rotations
        transform.localRotation = Quaternion.Euler(pitch, transform.localRotation.eulerAngles.y + yaw, 0f);
    }

    void UpdateGPSOverlay()
    {
        if (cameraGPSOverlayText == null || drone == null)
        {
            return; // Do nothing if references are missing
        }

        // Get drone's current position in Unity
        Vector3 unityPosition = drone.transform.position;

        // Convert Unity position to GPS coordinates
        double[] gpsCoords = UnityToGPS(unityPosition.x, unityPosition.z);

        // Update the GPS overlay UI element
        cameraGPSOverlayText.text = $"Latitude: {gpsCoords[0]:F6}°\nLongitude: {gpsCoords[1]:F6}°";
    }

    private double[] UnityToGPS(double xUnity, double zUnity)
    {
        // Solve the linear equations to get deltaE and deltaN
        // X_unity = A * deltaE + B * deltaN + C
        // Z_unity = D * deltaE + E * deltaN + F

        // Build matrices for linear equations
        double[,] matrix = {
            { A, B },
            { D, E }
        };
        double[] constants = {
            xUnity - C,
            zUnity - F
        };

        // Solve for deltaE and deltaN
        double deltaE, deltaN;
        SolveLinearEquations(matrix, constants, out deltaE, out deltaN);

        // Convert deltaE and deltaN to latitude and longitude differences
        double deltaLat = deltaN / METERS_PER_DEG_LAT;
        double deltaLon = deltaE / METERS_PER_DEG_LON;

        // Compute GPS coordinates
        double latitude = LAT_REF + deltaLat;
        double longitude = LON_REF + deltaLon;

        return new double[] { latitude, longitude };
    }

    private void SolveLinearEquations(double[,] matrix, double[] constants, out double x, out double y)
    {
        // Use Cramer's Rule to solve 2x2 linear equations
        double a = matrix[0, 0];
        double b = matrix[0, 1];
        double c = matrix[1, 0];
        double d = matrix[1, 1];
        double e = constants[0];
        double f = constants[1];

        double denominator = a * d - b * c;
        if (denominator == 0)
        {
            x = 0;
            y = 0;
            Debug.LogError("Cannot solve linear equations: determinant is zero.");
            return;
        }

        x = (e * d - b * f) / denominator;
        y = (a * f - e * c) / denominator;
    }
}
