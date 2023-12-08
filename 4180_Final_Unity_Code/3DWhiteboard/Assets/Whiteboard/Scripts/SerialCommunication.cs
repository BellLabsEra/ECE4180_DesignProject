using System;
using System.Collections;
using System.Collections.Generic;
using System.IO.Ports;
using System.Net;
using Unity.VisualScripting;
using UnityEngine;

public class SerialConnection : MonoBehaviour
{
    #region parameters
    private SerialPort _serialPort = new SerialPort("COM3", 115200);
    public string receivedString;
    private float[] recv_rotation = new float[4];


    public Quaternion new_quat;
    public Rigidbody Cube;
    #endregion
    //[field: SerializeField] public Transform RotationData {  get; set; }

    public Quaternion rb_quat;

    // Start is called before the first frame update
    void Start()
    {
        _serialPort.Open();
        InvokeRepeating("Serial_Data_Reading", 0f, 0.01f);
        //InvokeRepeating("Serial_Data_Reading", 0f, 0.1f);
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
        //transform.rotation = Quaternion.Inverse(quat);
        new_quat = Quaternion.Inverse(new_quat);
        rb_quat = Quaternion.Slerp(rb_quat, new_quat, 0.05f);

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
            //recv_rotation = new float[breakdown.Length];
            for (int i = 0; i < breakdown.Length; i++)
            {
                recv_rotation[i] = float.Parse(breakdown[i]);
            }
            new_quat.Set(recv_rotation[0], recv_rotation[1], recv_rotation[2], recv_rotation[3]);
        }
        catch (Exception e)
        {
            Debug.Log(e.Message);
        }
    }


}
