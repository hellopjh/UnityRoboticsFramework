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
using System.Runtime.Remoting.Messaging;
using Unity.VisualScripting;



#if UNITY_EDITOR
using UnityEditor;
#endif

namespace BiPanda
{
    public class HapticInterface : MonoBehaviour
    {
        #region DLL_Imports
        [DllImport("HapticsDirect")] public static extern void getVersionString(StringBuilder dest, int len);  //!< Retreives the OpenHaptics version string.

        // Setup Functions
        [DllImport("HapticsDirect")] public static extern int initDevice(string deviceName);  //!< Connects to and Initializes a haptic device.
        [DllImport("HapticsDirect")] public static extern void getDeviceSN(string configName, StringBuilder dest, int len);   //!< Retrieves device serial number
        [DllImport("HapticsDirect")] public static extern void getDeviceModel(string configName, StringBuilder dest, int len);  //!< Retrieves devices model name

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
            public double[] rotation;
            public double[] velocity;

            public double[] workspace_usable;
            public double[] workspace_max;
            public double max_stiffness;
            public double max_damping;
            public double max_force;

            public HapticDeviceInfo(string _name)
            {
                name = _name;
                button_status = 0;
                buttons_curr = new int[4];
                buttons_prev = new int[4];
                inkwell = 0;
                position = new double[3];
                rotation = new double[3];
                velocity = new double[6];

                workspace_usable = new double[6];
                workspace_max = new double[6];
            }
        }
        HapticDeviceInfo left_haptic;
        HapticDeviceInfo right_haptic;

        void OnEnable()
        {
            left_haptic = new HapticDeviceInfo("Left Device");
            right_haptic = new HapticDeviceInfo("Right Device");
            InitializeHapticDevice(left_haptic);
            InitializeHapticDevice(right_haptic);

        }
        void Start()
        {
            startSchedulers();
        }

        void Update()
        {
            //UpdateDeviceInformation(left_haptic);
            //UpdateDeviceInformation(right_haptic);

            //if (left_haptic.button_status == 2 && right_haptic.button_status == 2)
            //{
            //    Debug.Log(string.Format("<color=green><b>[L]</b> {0:N3}, {1:N3}, {2:N3}, {3:N3}, {4:N3}, {5:N3}. <b>[R]</b> {6:N3}, {7:N3}, {8:N3}, {9:N3}, {10:N3}, {11:N3}.</color>"
            //        , left_haptic.position[0], left_haptic.position[1], left_haptic.position[2]
            //        , left_haptic.rotation[0], left_haptic.rotation[1], left_haptic.rotation[2]
            //        , right_haptic.position[0], right_haptic.position[1], right_haptic.position[2]
            //        , right_haptic.rotation[0], right_haptic.rotation[1], right_haptic.rotation[2]));
            //}
            //else if (left_haptic.button_status == 2 && right_haptic.button_status != 2)
            //{
            //    Debug.Log(string.Format("<color=green><b>[L]</b> {0:N3}, {1:N3}, {2:N3}, {3:N3}, {4:N3}, {5:N3}.</color> [R] {6:N3}, {7:N3}, {8:N3}, {9:N3}, {10:N3}, {11:N3}."
            //        , left_haptic.position[0], left_haptic.position[1], left_haptic.position[2]
            //        , left_haptic.rotation[0], left_haptic.rotation[1], left_haptic.rotation[2]
            //        , right_haptic.position[0], right_haptic.position[1], right_haptic.position[2]
            //        , right_haptic.rotation[0], right_haptic.rotation[1], right_haptic.rotation[2]));
            //}
            //else if (left_haptic.button_status != 2 && right_haptic.button_status == 2)
            //{
            //    Debug.Log(string.Format("[L] {0:N3}, {1:N3}, {2:N3}, {3:N3}, {4:N3}, {5:N3}. <color=green><b>[R]</b> {6:N3}, {7:N3}, {8:N3}, {9:N3}, {10:N3}, {11:N3}.</color>"
            //        , left_haptic.position[0], left_haptic.position[1], left_haptic.position[2]
            //        , left_haptic.rotation[0], left_haptic.rotation[1], left_haptic.rotation[2]
            //        , right_haptic.position[0], right_haptic.position[1], right_haptic.position[2]
            //        , right_haptic.rotation[0], right_haptic.rotation[1], right_haptic.rotation[2]));
            //}
            //else
            //{
            //    Debug.Log(string.Format("[L] {0:N3}, {1:N3}, {2:N3}, {3:N3}, {4:N3}, {5:N3}. [R] {6:N3}, {7:N3}, {8:N3}, {9:N3}, {10:N3}, {11:N3}."
            //        , left_haptic.position[0], left_haptic.position[1], left_haptic.position[2]
            //        , left_haptic.rotation[0], left_haptic.rotation[1], left_haptic.rotation[2]
            //        , right_haptic.position[0], right_haptic.position[1], right_haptic.position[2]
            //        , right_haptic.rotation[0], right_haptic.rotation[1], right_haptic.rotation[2]));
            //}
        }

        void OnDestroy()
        {
            DisconnectHapticDevice();
        }

        #region HapticDevice
        public bool InitializeHapticDevice(HapticDeviceInfo _handler)
        {
            DeviceHHD = initDevice(_handler.name);
            if (DeviceHHD < 0)
            {
                // error
                Debug.LogError(string.Format("[Haptic::Init] Error occurred."));
                return false;
            }
            else
            {
                StringBuilder _sb = new StringBuilder(256);
                getDeviceSN(_handler.name, _sb, _sb.Capacity);
                string _serial_number = _sb.ToString();
                _sb.Clear();
                getDeviceModel(_handler.name, _sb, _sb.Capacity);
                string _model_type = _sb.ToString();
                getDeviceMaxValues(_handler.name, ref _handler.max_stiffness, ref _handler.max_damping, ref _handler.max_force);
                getWorkspaceArea(_handler.name, _handler.workspace_usable, _handler.workspace_max);

                Debug.Log(string.Format("<b>[InitHaptic::{0}]</b> Hatic device is connected. (Model: {1}, SN: {2}, Fmax: {3:N3}"
                    , _handler.name, _serial_number, _model_type, _handler.max_force));
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
            double[] _rot = new double[3];
            double[] _q = new double[3];
            double[] _gimbal = new double[3];
            double[] _vel = new double[3];
            double[] _trans_mat = new double[16];

            getPosition(_handler.name, _pos);
            getJointAngles(_handler.name, _q, _gimbal);
            GetHapticRotation(_handler, ref _rot);
            getVelocity(_handler.name, _vel);
            getTransform(_handler.name, _trans_mat);
            GetDeviceButtonStatus(_handler);

            for (int i = 0; i < 3; i++)
            {
                _handler.position[i] = _pos[i] * 0.001;  // convert from mm to m
                _handler.rotation[i] = _rot[i];
                _handler.velocity[i] = _vel[i] * 0.001;
            }
        }

        public void GetDeviceButtonStatus(HapticDeviceInfo _handler)
        {            
            int[] _tmp_buttons = new int[4];
            getButtons(_handler.name, _handler.buttons_curr, _tmp_buttons, ref _handler.inkwell);
            if (_handler.buttons_prev[0] == 0 && _handler.buttons_curr[0] == 1)
            {
                _handler.button_status = 1;
                //Debug.Log(string.Format("<b>[Haptic::Event]</b> button is pressed ({0})."
                //    , _handler.name));
            }
            else if (_handler.buttons_prev[0] == 1 && _handler.buttons_curr[0] == 1)
            {
                _handler.button_status = 2;
            }
            else if (_handler.buttons_prev[0] == 1 && _handler.buttons_curr[0] == 0)
            {
                _handler.button_status = 3;
            }
            else if (_handler.buttons_prev[0] == 0 && _handler.buttons_curr[0] == 0)
            {
                _handler.button_status = 0;
            }
            for (int i = 0; i < 4; i++) _handler.buttons_prev[i] = _handler.buttons_curr[i];
        }

        public void GetHapticStatus(ref int button_left, ref float[] pose_left, ref int button_right, ref float[] pose_right)
        {
            UpdateDeviceInformation(left_haptic);
            UpdateDeviceInformation(right_haptic);

            button_left = left_haptic.button_status;
            button_right = right_haptic.button_status;
            for(int i=0; i<3; i++)
            {
                pose_left[i] = (float)left_haptic.position[i];
                pose_left[i + 3] = (float)left_haptic.rotation[i];
                pose_right[i] = (float)right_haptic.position[i];
                pose_right[i + 3] = (float)right_haptic.rotation[i];
            }
        }

        public void SetHapticForce(float[] f_left, float[] f_right)
        {
            double tmp_max_force = 0.7;
            Vector3 f_left_vec3 = new Vector3(f_left[0], f_left[1], f_left[2]);
            double f_left_magnitude = (double)f_left_vec3.magnitude;
            double[] f_left_arr = new double[3];
            if (f_left_magnitude >= tmp_max_force)
            {
                for(int i=0; i<3; i++)
                {
                    f_left_arr[i] = (double)f_left_vec3[i] * (tmp_max_force / f_left_magnitude);
                }
                f_left_magnitude = tmp_max_force;
            }
            else
            {
                for (int i = 0; i < 3; i++)
                {
                    f_left_arr[i] = (double)f_left_vec3[i];
                }
            }
            if (left_haptic.button_status == 2)
            {
                setConstantForceValues(left_haptic.name, f_left_arr, f_left_magnitude);
            }
            else
            {
                setConstantForceValues(left_haptic.name, f_left_arr, 0);
            }


            Vector3 f_right_vec3 = new Vector3(f_right[0], f_right[1], f_right[2]);
            double f_right_magnitude = (double)f_right_vec3.magnitude;
            double[] f_right_arr = new double[3];
            if (f_right_magnitude >= tmp_max_force)
            {
                for (int i = 0; i < 3; i++)
                {
                    f_right_arr[i] = (double)f_right_vec3[i] * (tmp_max_force / f_right_magnitude);
                }
                f_right_magnitude = tmp_max_force;
            }
            else
            {
                for (int i = 0; i < 3; i++)
                {
                    f_right_arr[i] = (double)f_right_vec3[i];
                }
            }
            if (right_haptic.button_status == 2)
            {
                setConstantForceValues(right_haptic.name, f_right_arr, f_right_magnitude);
            }
            else
            {
                setConstantForceValues(right_haptic.name, f_right_arr, 0);
            }
        }

        public void GetHapticTransform(ref double[] res)
        {
            //double[] res_mat = new double[16];
            getTransform(left_haptic.name, res);
            //Debug.Log(string.Format("[Transform-Position] {0:N3}, {1:N3}, {2:N3}."
            //    , res[3], res[7], res[11]));
        }

        public void GetHapticRotation(HapticDeviceInfo _handler, ref double[] vec)
        {
            double[] res_mat = new double[16];
            double[] rot_mat = new double[9];

            getTransform(_handler.name, res_mat);
            rot_mat = RfromSE3(res_mat);
            vec = SO3ToEuler(rot_mat);
        }

        public double[] SO3ToVec(double[] input)
        {
            // rotation matrix to vector
            // 아래로 ordering
            // (00)0 (01)3 (02)6
            // (10)1 (11)4 (12)7
            // (20)2 (21)5 (22)8

            double[] res = new double[3];
            double th_x, th_y, th_z;

            //th_x = -Math.Asin(input[7]);                // return radian
            //th_y = Math.Atan2(input[6], input[8]);
            //th_z = Math.Atan2(input[1], input[4]);
            double angle = Math.Acos((input[0] + input[4] + input[8] - 1) / 2);
            th_x = (input[5] - input[7]) / Math.Sqrt(Math.Pow(input[5] - input[7], 2) + Math.Pow(input[6] - input[2], 2) + Math.Pow(input[1] - input[3], 2));
            th_y = (input[6] - input[2]) / Math.Sqrt(Math.Pow(input[5] - input[7], 2) + Math.Pow(input[6] - input[2], 2) + Math.Pow(input[1] - input[3], 2));
            th_z = (input[1] - input[3]) / Math.Sqrt(Math.Pow(input[5] - input[7], 2) + Math.Pow(input[6] - input[2], 2) + Math.Pow(input[1] - input[3], 2));
            res[0] = th_x * angle;
            res[1] = th_y * angle;
            res[2] = th_z * angle;
            return res;
        }

        public double[] RfromSE3(Matrix4x4 mat)
        {
            double[] res = new double[9];
            res[0] = mat[0];
            res[1] = mat[1];
            res[2] = mat[2];
            res[3] = mat[4];
            res[4] = mat[5];
            res[5] = mat[6];
            res[6] = mat[8];
            res[7] = mat[9];
            res[8] = mat[10];
            return res;
        }

        public double[] RfromSE3(double[] mat)
        {
            double[] res = new double[9];
            res[0] = mat[0];
            res[1] = mat[1];
            res[2] = mat[2];
            res[3] = mat[4];
            res[4] = mat[5];
            res[5] = mat[6];
            res[6] = mat[8];
            res[7] = mat[9];
            res[8] = mat[10];
            return res;
        }

        public double[] VecToSO3(double[] vec)
        {
            Vector3 aa = new Vector3((float)vec[0], (float)vec[1], (float)vec[2]);
            float aa_angle = aa.magnitude;
            Vector3 aa_axis = aa.normalized;
            
            // 아래로 ordering
            // (00)0 (01)3 (02)6
            // (10)1 (11)4 (12)7
            // (20)2 (21)5 (22)8

            double[] mat = new double[9];
            //double t = (double)(1f - Mathf.Cos(aa_angle));
            //double c = (double)(Mathf.Cos(aa_angle));
            //double s = (double)(Mathf.Sin(aa_angle));
            //mat[0] = t * aa_axis.x * aa_axis.x + c;
            //mat[1] = t * aa_axis.x * aa_axis.y + aa_axis.z * s;
            //mat[2] = t * aa_axis.x * aa_axis.z - aa_axis.y * s;
            //mat[3] = t * aa_axis.x * aa_axis.y - aa_axis.z * s;
            //mat[4] = t * aa_axis.y * aa_axis.y + c;
            //mat[5] = t * aa_axis.y * aa_axis.z + aa_axis.x * s;
            //mat[6] = t * aa_axis.x * aa_axis.z + aa_axis.y * s;
            //mat[7] = t * aa_axis.y * aa_axis.z - aa_axis.x * s;
            //mat[8] = t * aa_axis.z * aa_axis.z + c;

            Vector3 sin_axis = Mathf.Sin(aa_angle) * aa_axis;
            Vector3 cos_axis = (1f - Mathf.Cos(aa_angle)) * aa_axis;
            float tmp = cos_axis.x * aa_axis.y;
            mat[3] = tmp - sin_axis.z;
            mat[1] = tmp + sin_axis.z;

            tmp = cos_axis.x * aa_axis.z;
            mat[6] = tmp + sin_axis.y;
            mat[2] = tmp - sin_axis.y;

            tmp = cos_axis.y * aa_axis.z;
            mat[7] = tmp - sin_axis.x;
            mat[5] = tmp + sin_axis.x;

            mat[0] = cos_axis.x * aa_axis.x + Mathf.Cos(aa_angle);
            mat[4] = cos_axis.y * aa_axis.y + Mathf.Cos(aa_angle);
            mat[8] = cos_axis.z * aa_axis.z + Mathf.Cos(aa_angle);

            Debug.Log(string.Format("angle: {0}, axis: {1}, {2}, {3}. cos(angle) = {4}", aa_angle, aa_axis.x, aa_axis.y, aa_axis.z, Mathf.Cos(aa_angle)));

            return mat;
        }

        public double[] EulerToSO3(double[] euler)
        {
            // bank, attitude, heading
            double sa = Math.Sin(euler[1]);
            double ca = Math.Cos(euler[1]);
            double sb = Math.Sin(euler[0]);
            double cb = Math.Cos(euler[0]);
            double sh = Math.Sin(euler[2]);
            double ch = Math.Cos(euler[2]);

            double[] mat = new double[9];
            mat[0] = ch * ca;
            mat[1] = sa;
            mat[2] = -sh * ca;
            mat[3] = -ch * sa * cb - sh * sb;
            mat[4] = ca * cb;
            mat[5] = sh * sa * cb - ch * sb;
            mat[6] = ch * sa * sb - sh * cb;
            mat[7] = -ca * sb;
            mat[8] = -sh * sa * sb - ch * cb;

            return mat;
        }

        public double[] SO3ToEuler(double[] mat)
        {
            double[] vec = new double[3];
            vec[0] = Math.Atan2(-mat[7], mat[4]);
            vec[1] = Math.Asin(mat[1]);
            vec[2] = Math.Atan2(-mat[2], mat[0]);
            return vec;
        }

        public void CompareRotationMatrix(ref double[] mat_a, ref double[] mat_b)
        {
            //double[] mat_a = new double[9];
            //double[] mat_b = new double[9];
            double[] vec = new double[3];

            double[] res_mat = new double[16];
            getTransform(left_haptic.name, res_mat);
            mat_a = RfromSE3(res_mat);
            //vec = SO3ToVec(mat_a);
            //mat_b = VecToSO3(vec);
            vec = SO3ToEuler(mat_a);
            mat_b = EulerToSO3(vec);

            //Vector3 aa = new Vector3((float)vec[0] * (180f/Mathf.PI), (float)vec[1] * (180f / Mathf.PI), (float)vec[2] * (180f / Mathf.PI));
            //Quaternion quat = Quaternion.AngleAxis(aa.magnitude, aa.normalized);
            //Matrix4x4 mat = Matrix4x4.Rotate(quat);
            //mat_b = RfromSE3(mat);
        }
        #endregion
    }
}
