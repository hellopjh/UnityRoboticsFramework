using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System.Text;
using System.Runtime.InteropServices;
using UnityEngine.Events;
using System;
using HapticGUI;
using System.Drawing.Text;
using UnityEngine.UIElements;
using System.Transactions;

#if UNITY_EDITOR
using UnityEditor;
#endif


public class HapticInterface : MonoBehaviour
{
    #region DLL_Imports
    [DllImport("HapticsDirect")] public static extern void getVersionString(StringBuilder dest, int len);  //!< Retreives the OpenHaptics version string.

    // Setup Functions
    [DllImport("HapticsDirect")] public static extern int initDevice(string deviceName);  //!< Connects to and Initializes a haptic device.
    [DllImport("HapticsDirect")] public static extern void getDeviceSN(string configName, StringBuilder dest, int len);   //!< Retrieves device serial number
    [DllImport("HapticsDirect")] public static extern void getDeviceModel(string configName, StringBuilder dest, int len);	//!< Retrieves devices model name

    [DllImport("HapticsDirect")] public static extern void getDeviceMaxValues(string configName, ref double max_stiffness, ref double max_damping, ref double max_force);
    [DllImport("HapticsDirect")] public static extern void startSchedulers(); //!< Starts the Open Haptic schedulers and assigns the required internal callbacks

    // Device Information
    [DllImport("HapticsDirect")] public static extern void getWorkspaceArea(string configName, double[] usable6, double[] max6); //!< Retrieves the bounds created by the physical limitations of the device. 

    // Updates
    [DllImport("HapticsDirect")] public static extern void getPosition(string configName, double[] position3); //!< Get the current position in mm of the device facing the device base. Left is + x, up is +y, toward user is +z. (Unity CSys)
    [DllImport("HapticsDirect")] public static extern void getVelocity(string configName, double[] velocity3); //!< Get the current velocity in mm/s of the device. Note: This value is smoothed to reduce high frequency jitter. (Unity CSys)
    [DllImport("HapticsDirect")] public static extern void getTransform(string configName, double[] matrix16); //!< Get the column-major transform of the device endeffector. (Unity CSys)
    [DllImport("HapticsDirect")] public static extern void getButtons(string configName, int[] buttons4, int[] last_buttons4, ref int inkwell); //!< Get the button, last button states and get whether the inkwell switch, if one exists is active.
    [DllImport("HapticsDirect")] public static extern void getCurrentForce(string configName, double[] currentforce3);  //!< Get the current force in N of the device. (Unity CSys)

    [DllImport("HapticsDirect")] public static extern void getJointAngles(string configName, double[] jointAngles, double[] gimbalAngles); //!< Get the joint angles in rad of the device. These are joint angles used for computing the kinematics of the armature relative to the base frame of the device. For Touch devices: Turret Left +, Thigh Up +, Shin Up + Get the angles in rad of the device gimbal.For Touch devices: From Neutral position Right is +, Up is -, CW is +

    [DllImport("HapticsDirect")] public static extern void getCurrentFrictionForce(string configName, double[] frictionForce);

    [DllImport("HapticsDirect")] public static extern void getGlobalForces(string configName, double[] vibrationForce, double[] constantForce, double[] springForce);

    [DllImport("HapticsDirect")] public static extern void getLocalForces(string configName, double[] stiffnessForce, double[] viscosityForce, double[] dynamicFrictionForce, double[] staticFrictionForce, double[] constantForce, double[] springForce);

    // Force output
    [DllImport("HapticsDirect")] public static extern void setForce(string configName, double[] lateral3, double[] torque3); //!< Adds an additional force to the haptic device. Can be eseed for scripted forces, but in most cases using an Effect is preferable. 

    [DllImport("HapticsDirect")] public static extern void setAnchorPosition(string configName, double[] position3); //!< Set the anchor position of the virtual stylus (Unity CSys)

    [DllImport("HapticsDirect")]
    public static extern void addContactPointInfo(string configName, double[] Location, double[] Normal, float MatStiffness, float MatDamping, double[] MatForce,
    float MatViscosity, float MatFrictionStatic, float MatFrictionDynamic, double[] MatConstForceDir, float MatConstForceMag, double[] MatSpringDir, float MatSpringMag, float MatPopThroughRel, float MatPopThroughAbs,
    double MatMass, double RigBSpeed, double[] RigBVelocity, double[] RigBAngularVelocity, double RigBMass, double[] ColImpulse, double PhxDeltaTime, double ImpulseDepth); //!< Add a collision contact point info to the contact points list
    [DllImport("HapticsDirect")] public static extern void updateContactPointInfo(string configName); //!< Update the contact point info list
    [DllImport("HapticsDirect")] public static extern void resetContactPointInfo(string configName); //!< Reset the contact point info list

    [DllImport("HapticsDirect")] public static extern void setVibrationValues(string configName, double[] direction3, double magnitude, double frequency, double time); //!< Set the parameters of the vibration
    [DllImport("HapticsDirect")] public static extern void setSpringValues(string configName, double[] anchor, double magnitude); //!< Set the parameters of the Spring FX
    [DllImport("HapticsDirect")] public static extern void setConstantForceValues(string configName, double[] direction, double magnitude); //!<Set the parameters of the Constant Force FX

    [DllImport("HapticsDirect")] public static extern void setGravityForce(string configName, double[] gForce3);


    //Cleanup functions
    //! Disconnects from all devices.

    [DllImport("HapticsDirect")] public static extern void disconnectAllDevices();

    //Error Handling Functions
    [DllImport("HapticsDirect")] public static extern int getHDError(StringBuilder Info, int len);
    #endregion


    private int DeviceHHD = -1;

    public class HapticDeviceInfo
    {
        public string name;
        public int button_status;
        public int[] buttons_curr;
        public int[] buttons_prev;
        public int inkwell;
        public double[] position;
        public double[] velocity;

        public HapticDeviceInfo(string _name)
        {
            name = _name;
            button_status = 0;
            buttons_curr = new int[4];
            buttons_prev = new int[4];
            inkwell = 0;
            position = new double[4];
            velocity = new double[4];
        }
    }
    HapticDeviceInfo left_haptic;
    HapticDeviceInfo right_haptic;

    void OnEnable()
    {
        left_haptic = new HapticDeviceInfo("Left Device");
        right_haptic = new HapticDeviceInfo("Right Device");
        InitializeHapticDevice(left_haptic.name);
        InitializeHapticDevice(right_haptic.name);

    }
    void Start()
    {
        startSchedulers();
    }
        
    void Update()
    {
        UpdateDeviceInformation(left_haptic);
        GetDeviceButtonStatus(left_haptic);

        UpdateDeviceInformation(right_haptic);
        GetDeviceButtonStatus(right_haptic);

        //if (left_haptic.button_status == 2 && right_haptic.button_status == 2)
        //{
        //    Debug.Log(string.Format("<color=green><b>[L]</b> {0:N3}, {1:N3}, {2:N3}. <b>[R]</b> {3:N3}, {4:N3}, {5:N3}.</color>"
        //        , left_haptic.position[0], left_haptic.position[1], left_haptic.position[2]
        //        , right_haptic.position[0], right_haptic.position[1], right_haptic.position[2]));
        //}
        //else if (left_haptic.button_status == 2 && right_haptic.button_status != 2)
        //{
        //    Debug.Log(string.Format("<color=green><b>[L]</b> {0:N3}, {1:N3}, {2:N3}.</color> [R] {3:N3}, {4:N3}, {5:N3}."
        //        , left_haptic.position[0], left_haptic.position[1], left_haptic.position[2]
        //        , right_haptic.position[0], right_haptic.position[1], right_haptic.position[2]));
        //}
        //else if (left_haptic.button_status != 2 && right_haptic.button_status == 2)
        //{
        //    Debug.Log(string.Format("[L] {0:N3}, {1:N3}, {2:N3}. <color=green><b>[R]</b> {3:N3}, {4:N3}, {5:N3}.</color>"
        //        , left_haptic.position[0], left_haptic.position[1], left_haptic.position[2]
        //        , right_haptic.position[0], right_haptic.position[1], right_haptic.position[2]));
        //}
        //else
        //{
        //    Debug.Log(string.Format("[L] {0:N3}, {1:N3}, {2:N3}. [R] {3:N3}, {4:N3}, {5:N3}."
        //        , left_haptic.position[0], left_haptic.position[1], left_haptic.position[2]
        //        , right_haptic.position[0], right_haptic.position[1], right_haptic.position[2]));
        //}
    }

    void OnDestroy()
    {
        DisconnectHapticDevice();
    }

    #region HapticDevice
    public bool InitializeHapticDevice(string _dev_name)
    {
        DeviceHHD = initDevice(_dev_name);
        if (DeviceHHD < 0)
        {
            // error
            Debug.LogError(string.Format("[Haptic::Init] Error occurred."));
            return false;
        }
        else
        {
            StringBuilder _sb = new StringBuilder(256);
            getDeviceSN(_dev_name, _sb, _sb.Capacity);
            string _serial_number = _sb.ToString();
            _sb.Clear();
            getDeviceModel(_dev_name, _sb, _sb.Capacity);
            string _model_type = _sb.ToString();
            Debug.Log(string.Format("<b>[InitHaptic::{0}]</b> Hatic device is connected. (Model: {1}, SN: {2}"
                , _dev_name, _serial_number, _model_type));
            return true;
        }
    }

    public void DisconnectHapticDevice()
    {
        disconnectAllDevices();
    }


    public void UpdateDeviceInformation(HapticDeviceInfo _handler)
    {
        double[] _pos = new double[3];
        double[] _vel = new double[3];
        double[] _trans_mat = new double[16];

        getPosition(_handler.name, _pos);
        getVelocity(_handler.name, _vel);
        getTransform(_handler.name, _trans_mat);

        for(int i=0; i<3; i++)
        {
            _handler.position[i] = _pos[i] * 0.001;  // convert from mm to m
            _handler.velocity[i] = _vel[i] * 0.001;
        }
    }

    public void GetDeviceButtonStatus(HapticDeviceInfo _handler)
    {
        int[] _tmp_buttons = new int[4];
        for (int i = 0; i < 4; i++) _handler.buttons_prev[i] = _handler.buttons_curr[i];
        getButtons(_handler.name, _handler.buttons_curr, _tmp_buttons, ref _handler.inkwell);
        if (_handler.buttons_prev[0] == 0 && _handler.buttons_curr[0] == 1)
        {
            _handler.button_status = 1;
            //Debug.Log(string.Format("<b>[Haptic::Event]</b> button is pressed ({0})."
            //    , _handler.name));
        }
        if(_handler.buttons_prev[0] == 1 && _handler.buttons_curr[0] == 1)
        {
            _handler.button_status = 2;
        }
        if(_handler.buttons_prev[0] == 1 && _handler.buttons_curr[0] == 0)
        {
            _handler.button_status = 3;
        }
    }
    #endregion
}
