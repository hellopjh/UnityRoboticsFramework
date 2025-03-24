using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System;
using System.Threading;
using System.Net.Http;
using System.ComponentModel.Design;
using Unity.Robotics.UrdfImporter;
using System.Security.Permissions;
using Valve.Newtonsoft.Json;
using Unity.VisualScripting;

namespace BiPanda
{
    public class RobotInterface2 : MonoBehaviour
    {
        #region Variables
        // client
        private BiPandaClient robot;
        [Header("Connection")]
        public string fci_ip;
        public bool connect;
        public bool disconnect;
        private bool is_connected;
        private bool is_client_thread_running;
        private Thread client_thread;
        private ThreadStart client_method;

        [Header("DCP Command")]
        public bool AlignTracker;
        private bool align_tracker_flag;
        public bool ModeConvert;
        public bool ToggleGripper;
        public bool ChangeNow;

        // time
        [Header("Controller Info")]
        public float loop_frequency;
        public float actual_loop_frequency;
        private DateTime loop_time_saved;
        private TimeSpan span;

        // teleoperation
        [Header("Command")]
        public uint command_id_1;
        public uint command_id_2;
        public uint command_id_1_wg;    // with gripper control (+100)
        public uint command_id_2_wg;
        public float[] command_task_posevec_1;
        public float[] command_task_posevec_2;
        private float[] command_float7_arr_1;
        private float[] command_float7_arr_2;
        [Header("Response")]
        public float[] response_joint_pos_1;
        public float[] response_external_wrench_1;
        public float[] response_joint_pos_2;
        public float[] response_external_wrench_2;

        [Header("Test")]
        private float[] test_command_1;
        private float[] test_command_2;
        public uint test_robot_index;
        public bool test_joint_motion_1;
        public bool test_joint_motion_2;
        public bool test_task_motion;
        private bool test_task_motion_flag;

        // robot visualizer
        private GameObject robot_left;
        private RobotVisualizer robot_left_joint_handler;
        private GameObject robot_right;
        private RobotVisualizer robot_right_joint_handler;

        [Header("Vive Interface")]
        public int trigger;
        public int menu;
        public float[] raw_pose;
        public float[] zero_pose;
        public float[] rel_pose;

        // aligner tracker
        private GameObject aligner;
        private float[] aligner_raw_pose;
        private Vector3 aligner_pos;
        private Quaternion aligner_rot;

        public int button_left;
        public float[] raw_pose_left;
        public float[] zero_pose_left;
        public float[] rel_pose_left;
        public float[] force_left;
        public int button_right;
        public float[] raw_pose_right;
        public float[] zero_pose_right;
        public float[] rel_pose_right;
        public float[] force_right;
        private ViveInterface vive_handler;
        private Matrix3x3 rotmat_zero_left, rotmat_raw_left, rotmat_rel_left;
        public double[] trans_mat_left;
        public double[] rmat_before;
        public double[] rmat_after;

        #endregion

        void Start()
        {
            fci_ip = "192.168.1.60";
            robot = new BiPandaClient(fci_ip);
            //connect = false;
            disconnect = false;
            is_connected = false;
            is_client_thread_running = false;

            loop_frequency = 1000f;

            AlignTracker = false;
            align_tracker_flag = false;

            command_id_1 = 0;
            command_id_2 = 0;
            command_task_posevec_1 = new float[6];
            command_task_posevec_2 = new float[6];
            command_float7_arr_1 = new float[7];
            command_float7_arr_2 = new float[7];
            response_joint_pos_1 = new float[7];
            response_external_wrench_1 = new float[6];
            response_joint_pos_2 = new float[7];
            response_external_wrench_2 = new float[6];

            test_command_1 = new float[7] { 0f, -Mathf.PI / 2f, 0f, -3f * Mathf.PI / 4f, 0f, Mathf.PI / 3f, Mathf.PI / 2f };
            test_command_2 = new float[7] { 0f, -Mathf.PI / 4f, 0f, -3f * Mathf.PI / 4f, 0f, Mathf.PI / 2f, Mathf.PI / 4f };
            test_robot_index = 1;
            test_joint_motion_1 = false;
            test_joint_motion_2 = false;
            //test_task_motion = false;

            if ((robot_left = GameObject.Find("fr3_left")) != null)
            {
                Debug.Log(string.Format("<b>fr3_left</b> was loaded successfully."));
                robot_left_joint_handler = robot_left.GetComponentInChildren<RobotVisualizer>();
            }
            else { Debug.LogError(string.Format("Failed to load <b>fr3_left</b>")); }
            if ((robot_right = GameObject.Find("fr3_right")) != null)
            {
                Debug.Log(string.Format("<b>fr3_right</b> was loaded successfully."));
                robot_right_joint_handler = robot_right.GetComponentInChildren<RobotVisualizer>();
            }
            else { Debug.LogError(string.Format("Failed to load <b>fr3_right</b>")); }

            raw_pose = new float[6];
            zero_pose = new float[6];
            rel_pose = new float[6];

            raw_pose_left = new float[6];
            zero_pose_left = new float[6];
            rel_pose_left = new float[6];
            force_left = new float[3];
            raw_pose_right = new float[6];
            zero_pose_right = new float[6];
            rel_pose_right = new float[6];
            force_right = new float[3];
            vive_handler = FindObjectOfType(typeof(ViveInterface)) as ViveInterface;
            trans_mat_left = new double[16];
            rmat_before = new double[9];
            rmat_after = new double[9];

            //aligner = GameObject.Find("Align Tracker");
            aligner = GameObject.Find("Tracker");
            if (aligner != null)
            {
                Debug.Log("Align Tracker is found.");
            }
            else
            {
                Debug.LogError("Failed to find Align Tracker");
            }
            aligner_raw_pose = new float[6];
            ModeConvert = false;
            ToggleGripper = false;
            ChangeNow = false;
        }


        void Update()
        {
            if (connect && !is_connected)
            {
                // start thread
                is_connected = true;
                OpenClient();
            }
            if (!connect && is_connected)
            {
                connect = true;
            }
            if (disconnect && is_connected)
            {
                // stop thread
                disconnect = false;
                is_connected = false;
                connect = false;
                CloseClient();
            }

            if(test_joint_motion_1)
            {
                test_joint_motion_1 = false;
                robot.connect();
                robot.move_joint_to(test_command_1, test_robot_index);
                robot.disconnect();
            }
            if (test_joint_motion_2)
            {
                test_joint_motion_2 = false;
                robot.connect();
                robot.move_joint_to(test_command_2, test_robot_index);
                robot.disconnect();
            }
            if (test_task_motion)
            {
                test_task_motion = false;
                test_task_motion_flag = true;
                float[] target_task = new float[6];
                //robot.connect();
                //robot.move_task_to(target_task, test_robot_index);
                //robot.disconnect();
            }
            if (AlignTracker)
            {
                AlignTracker = false;
                align_tracker_flag = true;
                Debug.Log("Align tracker frame.");
            }


            for(int i=0; i<7; i++)
            {
                robot_left_joint_handler.JointInput[i] = response_joint_pos_2[i] * Mathf.Rad2Deg;
                robot_right_joint_handler.JointInput[i] = response_joint_pos_1[i] * Mathf.Rad2Deg;
            }


            aligner_pos = aligner.transform.position;
            aligner_rot = aligner.transform.rotation;
            double[] aligner_rotvec = ViveInterface.UnityQuaternionToDoubleAxisAngleArray(aligner_rot);
            for(int i=0; i<3; i++)
            {
                aligner_raw_pose[i] = aligner_pos[i];
                aligner_raw_pose[i + 3] = (float)aligner_rotvec[i];
            }
            //Debug.Log(string.Format("aligner: {0:N3}, {1:N3}, {2:N3}, {3:N3}, {4:N3}, {5:N3}"
            //    , aligner_raw_pose[0], aligner_raw_pose[1], aligner_raw_pose[2]
            //    , aligner_raw_pose[3], aligner_raw_pose[4], aligner_raw_pose[5]));
        }

        void OnDestroy()
        {
            if(is_client_thread_running)
            {
                CloseClient();
            }
        }

        #region Client Thread
        public void OpenClient()
        {
            client_method = new ThreadStart(this.RunClient);
            client_thread = new Thread(client_method);
            client_thread.Start();
        }

        public void CloseClient()
        {
            is_client_thread_running = false;
            if(!client_thread.Join(1000))
            {
                Debug.LogError("Thread is still running.");
            }
        }

        public void RunClient()
        {
            loop_time_saved = DateTime.Now;

            vive_handler.GetViveStatus(ref trigger, ref menu, ref raw_pose);
            Debug.Log(string.Format("Start client thread."));
            is_client_thread_running = true;
            while(is_client_thread_running)
            {
                span = DateTime.Now - loop_time_saved;
                if(span.TotalSeconds > ((1f / loop_frequency) * 0.95f))
                { 
                    // 1000 Hz loop
                    vive_handler.GetViveStatus(ref trigger, ref menu, ref raw_pose);
                    ProcessViveCommand();

                    robot.connect();
                    if(align_tracker_flag)
                    {
                        align_tracker_flag = false;
                        command_id_1 = (uint)DCPCOMMAND.DCP_COMMAND_TRACKER_ALIGN;
                        if (ModeConvert)
                        {
                            command_id_1_wg = command_id_1 + 100;
                        }
                        else
                        {
                            command_id_1_wg = command_id_1;
                        }
                        if(ToggleGripper)
                        {
                            command_id_1_wg += 200;
                            ToggleGripper = false;
                        }
                    }
                    else if(test_task_motion_flag)
                    {
                        test_task_motion_flag = false;
                        command_id_1 = (uint)DCPCOMMAND.DCP_COMMAND_MOVE_TASK_TO;
                        if (ModeConvert)
                        {
                            command_id_1_wg = command_id_1 + 100;
                        }
                        else
                        {
                            command_id_1_wg = command_id_1;
                        }
                        if (ToggleGripper)
                        {
                            command_id_1_wg += 200;
                            ToggleGripper = false;
                        }
                    }
                    else
                    {
                        if (ModeConvert)
                        {
                            command_id_1_wg = command_id_1 + 100;
                        }
                        else
                        {
                            command_id_1_wg = command_id_1;
                        }
                        if (ToggleGripper)
                        {
                            command_id_1_wg += 200;
                            ToggleGripper = false;
                        }
                    }
                    if(ChangeNow)
                    {
                        command_id_1_wg += 1000;
                        ChangeNow = false;
                    }
                    robot.update_teleoperation_command((uint)command_id_1_wg, command_float7_arr_1, command_id_2_wg, command_float7_arr_2);
                    robot.disconnect();
                    robot.update_robot_status(ref response_joint_pos_1, ref response_external_wrench_1, ref response_joint_pos_2, ref response_external_wrench_2);
                    
                    actual_loop_frequency = 1f / (float)span.TotalSeconds;
                    loop_time_saved = DateTime.Now;
                }
            }
            Debug.Log(string.Format("Stop client thread."));
        }
        #endregion

        public void ProcessViveCommand()
        {
            /// LEFT
            if (button_left == 1)
            {
                // start teleoperation
                command_id_1 = (uint)StreamingCommand.STREAMING_COMMAND_START_TELEOPERATION;
                for(int i=0; i<3; i++)
                {
                    zero_pose_left[i] = raw_pose_left[i];
                    zero_pose_left[i + 3] = raw_pose_left[i + 3];
                    rel_pose_left[i] = 0;
                    rel_pose_left[i + 3] = raw_pose_left[i + 3];
                }
            }
            else if (button_left == 2)
            {
                // continue teleoperation
                command_id_1 = (uint)StreamingCommand.STREAMING_COMMAND_CONTINUE_TELEOPERATION;
                for (int i = 0; i < 3; i++)
                {
                    // position
                    rel_pose_left[i] = raw_pose_left[i] - zero_pose_left[i];

                    // rotation
                    rel_pose_left[i + 3] = raw_pose_left[i + 3];
                }
            }
            else if (button_left == 3)
            {
                // stop teleoperation
                command_id_1 = (uint)StreamingCommand.STREAMING_COMMAND_STOP_TELEOPERATION;
                for (int i = 0; i < 6; i++)
                {
                    zero_pose_left[i] = raw_pose_left[i];
                    rel_pose_left[i] = 0;
                }
            }
            else if (button_left == 0)
            {
                // null
                command_id_1 = (uint)StreamingCommand.STREAMING_COMMAND_REST;
                for (int i = 0; i < 6; i++)
                {
                    zero_pose_left[i] = raw_pose_left[i];
                    rel_pose_left[i] = 0;
                }
            }


            /// RIGHT
            if (trigger == 1)
            {
                // start teleoperation
                command_id_2 = (uint)StreamingCommand.STREAMING_COMMAND_START_TELEOPERATION;
                for (int i = 0; i < 3; i++)
                {
                    zero_pose[i] = raw_pose[i];
                    zero_pose[i + 3] = raw_pose[i + 3];
                    rel_pose[i] = 0;
                    rel_pose[i + 3] = raw_pose[i + 3];
                }
            }
            else if (trigger == 2)
            {
                // continue teleoperation
                command_id_2 = (uint)StreamingCommand.STREAMING_COMMAND_CONTINUE_TELEOPERATION;
                for (int i = 0; i < 3; i++)
                {
                    // position
                    rel_pose[i] = raw_pose[i] - zero_pose[i];

                    // rotation
                    rel_pose[i + 3] = raw_pose[i + 3];

                }
            }
            else if (trigger == 3)
            {
                // stop teleoperation
                command_id_2 = (uint)StreamingCommand.STREAMING_COMMAND_STOP_TELEOPERATION;
                for (int i = 0; i < 6; i++)
                {
                    zero_pose[i] = raw_pose[i];
                    rel_pose[i] = 0;
                }
            }
            else if (trigger == 0)
            {
                // null
                command_id_2 = (uint)StreamingCommand.STREAMING_COMMAND_REST;
                for (int i = 0; i < 6; i++)
                {
                    zero_pose[i] = raw_pose[i];
                    rel_pose[i] = 0;
                }
            }


            // gripper control by menu button
            if(menu == 1)
            {
                // gripper toggle
                command_id_2_wg = command_id_2 + 100;
            }
            else
            {
                command_id_2_wg = command_id_2;
            }
            
            


            // save command (left & right)
            for (int i = 0; i < 6; i++)
            {
                command_float7_arr_1[i] = aligner_raw_pose[i];      // align
                command_float7_arr_2[i] = rel_pose[i];              // teleoperation
            }
            command_float7_arr_1[6] = 0;
            command_float7_arr_2[6] = 0;
        }

    }
}
