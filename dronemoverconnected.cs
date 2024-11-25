using UnityEngine;
using System.Text;
using RJCP.IO.Ports; // Import SerialPortStream namespace
using System.Threading;

public class DroneControllerSerial : MonoBehaviour
{
    public GameObject drone; // Assign your drone GameObject in Unity Inspector
    private Vector3 position; // Current position of the drone
    private Vector3 velocity; // Velocity calculated from acceleration
    private Vector3 acceleration; // Latest acceleration values
    private Vector3 gyro; // Latest gyroscope values

    private SerialPortStream serialPort;
    private Thread readThread;
    private bool isReading = true;

    public string portName = "/dev/tty.usbmodem203E345C57461"; // Your serial port
    public int baudRate = 115200; // Baud rate

    private float scaleFactor = 0.1f; // Scale for Unity world units

    void Start()
    {
        position = drone.transform.position;
        velocity = Vector3.zero;

        // Initialize SerialPortStream
        serialPort = new SerialPortStream(portName, baudRate);
        serialPort.DataBits = 8;
        serialPort.StopBits = StopBits.One;
        serialPort.Parity = Parity.None;
        serialPort.Open();

        if (serialPort.IsOpen)
        {
            Debug.Log($"Serial Port {portName} opened at {baudRate} baud.");
        }

        // Start reading data in a separate thread
        readThread = new Thread(ReadSerialData);
        readThread.IsBackground = true;
        readThread.Start();
    }

    void Update()
    {
        // Integrate acceleration to update velocity
        velocity += acceleration * Time.deltaTime;

        // Update position using velocity
        position += velocity * Time.deltaTime;

        // Apply the updated position to the drone
        drone.transform.position = position;

        // Debug the current state (optional)
        Debug.Log($"Position: {position}, Velocity: {velocity}, Acceleration: {acceleration}, Gyro: {gyro}");
    }

    private void ReadSerialData()
    {
        while (isReading)
        {
            try
            {
                if (serialPort.IsOpen && serialPort.BytesToRead > 0)
                {
                    string line = serialPort.ReadLine(); // Read a line from the serial port
                    ParseSerialData(line); // Parse the incoming data
                }
            }
            catch (System.Exception ex)
            {
                Debug.LogError($"Error reading serial data: {ex.Message}");
            }
        }
    }

    private void ParseSerialData(string line)
    {
        string[] fields = line.Split(',');
        if (fields.Length >= 6)
        {
            try
            {
                // Parse accelerometer data
                acceleration = new Vector3(
                    float.Parse(fields[0]), // acc_x
                    float.Parse(fields[1]), // acc_y
                    float.Parse(fields[2])  // acc_z
                ) * scaleFactor;

                // Parse gyroscope data
                gyro = new Vector3(
                    float.Parse(fields[3]), // gyro_x
                    float.Parse(fields[4]), // gyro_y
                    float.Parse(fields[5])  // gyro_z
                );
            }
            catch (System.Exception ex)
            {
                Debug.LogError($"Error parsing serial data: {ex.Message}");
            }
        }
    }

    private void OnDestroy()
    {
        isReading = false;

        if (serialPort != null && serialPort.IsOpen)
        {
            serialPort.Close();
        }

        if (readThread != null)
        {
            readThread.Abort();
        }
    }
}
