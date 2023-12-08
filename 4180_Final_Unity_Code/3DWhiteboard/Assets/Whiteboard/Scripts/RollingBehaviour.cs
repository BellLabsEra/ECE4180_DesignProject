using System;
using System.Collections;
using System.Collections.Generic;
using System.IO.Ports;
using UnityEngine;


public class RollingBehaviour : MonoBehaviour
{
    private static Quaternion rb_quat;

    internal class SerialCommunication : MonoBehaviour
    {


        private Quaternion new_quat = new Quaternion(0, 0, 0, 0);
        private string receivedString;
        private float[] recv_rotation = new float[4];

        public SerialPort _serialPort = new SerialPort("COM3", 115200);


        // Start is called before the first frame update
        void Start()
        {
            _serialPort.Open();
            //InvokeRepeating("Serial_Data_Reading", 0f, 0.00f);
            InvokeRepeating("Serial_Data_Reading", 0f, 0.01f);
        }

        void OnApplicationQuit()
        {
            _serialPort.Close();
        }

        // Update is called once per frame
        void Update()
        {
            //receivedString = Serial_Data_Reading();
            Serial_Data_Reading();
            // Ensures that the IMU is rotating based on local axis and not Earth's
            //new_quat = Quaternion.Inverse(new_quat);
            //Debug.Log(new_quat.ToString());
            //rb_quat = Quaternion.Slerp(rb_quat, new_quat, 0.20f);
            rb_quat = Quaternion.Slerp(rb_quat, Quaternion.Inverse(new_quat), 0.20f);

            //Debug.Log(rb_quat.ToString());
            //Debug.Log(rb_quat.ToString());
            //Debug.Log(rb_quat.ToString());

        }

        // Return the values collected by the IMU sent by the mbed
        // The data will be returned as 
        void Serial_Data_Reading()
        {
            try
            {
                receivedString = _serialPort.ReadLine();

                //Debug.Log(receivedString);
                string[] breakdown = receivedString.Split(',');
                for (int i = 0; i < breakdown.Length; i++)
                {
                    recv_rotation[i] = float.Parse(breakdown[i]);
                }
                new_quat.Set(recv_rotation[0], recv_rotation[1], recv_rotation[2], recv_rotation[3]);
                //Debug.Log(new_quat.ToString());
            }
            catch (Exception e)
            {
                //Debug.Log(e.Message);
            }
        }
    }

    public float speed = 2.0f;

    //private Vector3 heading;
    private Vector3 targetForward;
    private Vector3 up;
    private Vector3 prev_up;

    //private Vector3 constant_vec = new Vector3(0, 1, 0);
    //private Vector3 s;
    //private Vector3 v;
    //private Vector3 vrot;

    //private double roll;
    //private double pitch;
    //private double yaw;


    Rigidbody actual;

    Vector3 move;
    


    // Start is called before the first frame update
    void Start()
    {
        //rb = transform.Find("RotationData").GetComponent<Transform>();
        gameObject.AddComponent<SerialCommunication>();
        actual = GetComponent<Rigidbody>();
      
        //rb.position = new Vector3(0, 0, 0);
    }

    // Update is called once per frame
    void Update()
    {
        prev_up = up;

        //Debug.Log(rb_quat.ToString());
        // x treated as roll
        //roll = -Math.Atan2(2 * (rb_quat[3] * rb_quat[0] + rb_quat[1] * rb_quat[2]), 1 - 2 * (rb_quat[0] * rb_quat[0] + rb_quat[1] * rb_quat[1]));
        //pitch = Math.Asin(2 * (rb_quat[3] * rb_quat[1] - rb_quat[2] * rb_quat[0]));
        //yaw = (-Math.Atan2(2 * (rb_quat[3] * rb_quat[2] + rb_quat[0] * rb_quat[1]), 1 - 2 * (rb_quat[1] * rb_quat[1] + rb_quat[2] * rb_quat[2])) - Math.PI / 2);

        //heading = new Vector3((float)(Math.Cos(yaw) * Math.Cos(pitch)), (float)Math.Sin(pitch), (float)(Math.Sin(yaw) * Math.Cos(pitch)));

        //s = Vector3.Cross(heading, constant_vec);
        //v = Vector3.Cross(s, heading);
        //vrot = (((float)Math.Cos(roll) * v) + ((float)Math.Sin(roll) * Vector3.Cross(heading, v)));
        //Debug.DrawLine(new Vector3(0, 0, 0), vrot, Color.blue);
        //Debug.DrawLine(new Vector3(0, 0, 0), heading, Color.red);
        ////Debug.Log("Heading: " + heading.ToString());
        //Debug.Log("Up: " + vrot.ToString());
        
        targetForward = Vector3.Lerp(targetForward, rb_quat * Vector3.forward, 0.2f);
        up = Vector3.Lerp(up, rb_quat * Vector3.up, 0.2f);

        targetForward.Normalize();
        up.Normalize();

        Debug.Log(up.ToString());
        Debug.DrawLine(new Vector3(0,0,0), targetForward, Color.blue);
        Debug.DrawLine(new Vector3(0, 0, 0), up, Color.red);
        float distance_between = (prev_up - up).sqrMagnitude;
        //Debug.Log(distance_between.ToString());
        //rb.AddForce(0, vrot.y, vrot.z, ForceMode.Impulse);
        if ((up.y > 0.4 || up.z > 0.4 || up.y < -0.4 || up.z < -0.4) 
            && (distance_between < 5))
        {
            float scaley = 0.025f;
            float scalez = 0.025f;
            if (up.y > 0.4 || up.y < -0.4)
            {
                scaley = 0.05f;
            }
            if (up.z > 0.4 || up.z < -0.4)
            {
                scalez = 0.05f;
            }
            move = new Vector3(0, up.y * scaley, up.z * scalez);
            Debug.Log(move.ToString());
            actual.Move(actual.position + move, actual.rotation);
            //actual.angularVelocity = Vector2.zero;
        }
        else 
        {
            //actual.velocity = Vector2.zero;

        }
        actual.angularVelocity = Vector3.zero;

    }
}
