using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using HTC.UnityPlugin.Vive;
using Valve.VR;
using JetBrains.Annotations;

namespace BiPanda
{
    public class ViveInterface : MonoBehaviour
    {
        private GameObject tracker_handler;
        private ViveDeviceInfo vive;

        public int trigger_debug;
        public int menu_debug;
        public int grip_debug;
        public bool trigger_bool_debug;
        public bool menu_bool_debug;
        public bool grip_bool_debug;
        public Vector3 pos_debug;
        public Quaternion rot_debug;

        enum InputInterfaceState : int
        {
            INTERFACE_RELEASED = 0,
            INTERFACE_DOWN = 1,
            INTERFACE_PRESSED = 2,
            INTERFACE_UP = 3
        }

        public class ViveDeviceInfo
        {
            public int trigger_status;
            public bool trigger_curr;
            public int menu_status;
            public bool menu_curr;
            public int grip_status;
            public bool grip_curr;
            public double[] position;   // m
            public double[] rotation;   // rad

            public ViveDeviceInfo()
            {
                trigger_status = 0;
                trigger_curr = false;
                menu_status = 0;
                menu_curr = false;
                grip_status = 0;
                grip_curr = false;
                position = new double[3];
                rotation = new double[3];
            }

            public void UpdateViveButton(bool _tr, bool _me)
            {
                // trigger
                trigger_curr = _tr;
                trigger_status = updateInterface(trigger_curr, trigger_status);

                // menu
                menu_curr = _me;
                menu_status = updateInterface(menu_curr, menu_status);

                // grip
                grip_curr = _me;
                grip_status = updateInterface(grip_curr, grip_status);
            }

            public void UpdateViveButton2(bool _tr, bool _me, bool _gr)
            {
                // trigger
                trigger_curr = _tr;
                trigger_status = updateInterface(trigger_curr, trigger_status);

                // menu
                menu_curr = _me;
                menu_status = updateInterface(menu_curr, menu_status);

                // grip
                grip_curr = _gr;
                grip_status = updateInterface(grip_curr, grip_status);
            }

            public void UpdateViveInfo(bool _trigger, bool _menu, Vector3 _pos, Quaternion _rot)
            {
                UpdateViveButton(_trigger, _menu);
                double[] _rot_vec = UnityQuaternionToDoubleAxisAngleArray(_rot);

                for (int i = 0; i < 3; i++)
                {
                    position[i] = (double)_pos[i];
                    rotation[i] = (double)_rot_vec[i];
                }
            }

            public void UpdateViveInfo2(bool _trigger, bool _menu, bool _grip, Vector3 _pos, Quaternion _rot)
            {
                UpdateViveButton2(_trigger, _menu, _grip);
                double[] _rot_vec = UnityQuaternionToDoubleAxisAngleArray(_rot);

                for (int i=0; i<3; i++)
                {
                    position[i] = (double)_pos[i];
                    rotation[i] = (double)_rot_vec[i];
                }
            }

            private int updateInterface(bool state_curr, int state_prev)
            {
                int state_res = (int)InputInterfaceState.INTERFACE_RELEASED;

                // state: triggered
                if (state_curr)
                {
                    if (state_prev == (int)InputInterfaceState.INTERFACE_RELEASED)
                    {
                        state_res = (int)InputInterfaceState.INTERFACE_DOWN;
                    }
                    else if (state_prev == (int)InputInterfaceState.INTERFACE_DOWN)
                    {
                        state_res = (int)InputInterfaceState.INTERFACE_PRESSED;
                    }
                    else if (state_prev == (int)InputInterfaceState.INTERFACE_PRESSED)
                    {
                        state_res = (int)InputInterfaceState.INTERFACE_PRESSED;
                    }
                    else if (state_prev == (int)InputInterfaceState.INTERFACE_UP)
                    {
                        state_res = (int)InputInterfaceState.INTERFACE_RELEASED;
                    }
                }
                // state: released
                else
                {
                    if (state_prev == (int)InputInterfaceState.INTERFACE_RELEASED)
                    {
                        state_res = (int)InputInterfaceState.INTERFACE_RELEASED;
                    }
                    else if (state_prev == (int)InputInterfaceState.INTERFACE_DOWN)
                    {
                        state_res = (int)InputInterfaceState.INTERFACE_PRESSED;
                    }
                    else if (state_prev == (int)InputInterfaceState.INTERFACE_PRESSED)
                    {
                        state_res = (int)InputInterfaceState.INTERFACE_UP;
                    }
                    else if (state_prev == (int)InputInterfaceState.INTERFACE_UP)
                    {
                        state_res = (int)InputInterfaceState.INTERFACE_RELEASED;
                    }
                }

                return state_res;
            }

            
        }

        void Start()
        {
            tracker_handler = GameObject.Find("Tracker");
            if(tracker_handler != null )
            {
                Debug.Log("[ViveInterface] Tracker is found.");
            }
            else
            {
                Debug.LogError("[ViveInterface] Failed to find tracker.");
            }
            vive = new ViveDeviceInfo();

            trigger_debug = 0;
            menu_debug = 0;
        }


        void Update()
        {
            pos_debug = tracker_handler.transform.position;
            rot_debug = tracker_handler.transform.rotation;

            trigger_bool_debug = ViveInput.GetPressEx(HandRole.RightHand, ControllerButton.Trigger);
            menu_bool_debug = ViveInput.GetPressEx(HandRole.RightHand, ControllerButton.Menu);
            grip_bool_debug = ViveInput.GetPressEx(HandRole.RightHand, ControllerButton.Grip);

            trigger_debug = vive.trigger_status;
            menu_debug = vive.menu_status;
            grip_debug = vive.grip_status;
        }




        public void GetViveStatus(ref int _trigger, ref int _menu, ref float[] _raw_pose)
        {
            vive.UpdateViveInfo(trigger_bool_debug, menu_bool_debug, pos_debug, rot_debug);
            
            _trigger = vive.trigger_status;
            _menu = vive.menu_status;
            for (int i = 0; i < 3; i++)
            {
                _raw_pose[i] = (float)vive.position[i];
                _raw_pose[i + 3] = (float)vive.rotation[i];
            }
        }

        public void GetViveStatus2(ref int _trigger, ref int _grip)
        {
            vive.UpdateViveInfo2(trigger_bool_debug, menu_bool_debug, grip_bool_debug, pos_debug, rot_debug);

            _trigger = vive.trigger_status;
            _grip = vive.grip_status;
        }

        #region transform
        public static double[] UnityQuaternionToDoubleAxisAngleArray(Quaternion _quat)
        {
            // unity to fr3 frame change is needed
            // todo
            //Quaternion _quat_fr3 = new Quaternion(-_quat.x, _quat.z, _quat.y, _quat.w);   // right
            Quaternion _quat_fr3 = new Quaternion(_quat.z, _quat.x, _quat.y, _quat.w);   // left

            //Quaternion _quat_fr3 = new Quaternion(_quat.x, -_quat.z, _quat.y, _quat.w);   // right

            Vector3 aa_axis;
            float aa_angle;
            _quat_fr3.ToAngleAxis(out aa_angle, out aa_axis);

            double[] rotvec = new double[3];
            for(int i=0; i<3; i++)
            {
                rotvec[i] = (double)(aa_axis[i] * aa_angle * Mathf.Deg2Rad);
            }
            return rotvec;
        }

        #endregion
    }
}
