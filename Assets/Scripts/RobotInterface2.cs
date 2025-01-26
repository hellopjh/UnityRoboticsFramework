using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System;
using System.Threading;
using System.Net.Http;
using System.ComponentModel.Design;
using Unity.Robotics.UrdfImporter;

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
        //public bool test_task_motion;

        // robot visualizer
        private GameObject robot_left;
        private RobotVisualizer robot_left_joint_handler;
        private GameObject robot_right;
        private RobotVisualizer robot_right_joint_handler;

        [Header("Haptic Interface")]
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
        private HapticInterface haptic;
        private Matrix3x3 rotmat_zero_left, rotmat_raw_left, rotmat_rel_left;
        public double[] trans_mat_left;
        public double[] rmat_before;
        public double[] rmat_after;

        #endregion

        void Start()
        {
            fci_ip = "192.168.1.60";
            robot = new BiPandaClient(fci_ip);
            connect = false;
            disconnect = false;
            is_connected = false;
            is_client_thread_running = false;

            loop_frequency = 1000f;

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

            raw_pose_left = new float[6];
            zero_pose_left = new float[6];
            rel_pose_left = new float[6];
            force_left = new float[3];
            raw_pose_right = new float[6];
            zero_pose_right = new float[6];
            rel_pose_right = new float[6];
            force_right = new float[3];
            haptic = FindObjectOfType(typeof(HapticInterface)) as HapticInterface;
            trans_mat_left = new double[16];
            rmat_before = new double[9];
            rmat_after = new double[9];

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
            //if(test_task_motion)
            //{
            //    test_task_motion = false;
            //    robot.connect();
            //    robot.move_task_to(test_command, test_robot_index);
            //    robot.disconnect();
            //}


            for(int i=0; i<7; i++)
            {
                robot_left_joint_handler.JointInput[i] = response_joint_pos_2[i] * Mathf.Rad2Deg;
                robot_right_joint_handler.JointInput[i] = response_joint_pos_1[i] * Mathf.Rad2Deg;
            }
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

            haptic.GetHapticStatus(ref button_left, ref raw_pose_left, ref button_right, ref raw_pose_right);
            Debug.Log(string.Format("Start client thread."));
            is_client_thread_running = true;
            while(is_client_thread_running)
            {
                span = DateTime.Now - loop_time_saved;
                if(span.TotalSeconds > ((1f / loop_frequency) * 0.95f))
                {
                    // 1000 Hz loop
                    haptic.GetHapticTransform(ref trans_mat_left);
                    //haptic.CompareRotationMatrix(ref rmat_before, ref rmat_after);

                    haptic.GetHapticStatus(ref button_left, ref raw_pose_left, ref button_right, ref raw_pose_right);
                    ProcessHapticCommand();

                    robot.connect();
                    robot.update_teleoperation_command(command_id_1, command_float7_arr_1, command_id_2, command_float7_arr_2);
                    robot.update_robot_status(ref response_joint_pos_1, ref response_external_wrench_1, ref response_joint_pos_2, ref response_external_wrench_2);
                    robot.disconnect();

                    //for(int i=0; i<3; i++)
                    //{
                    //    force_left[i] = response_external_wrench_1[i];
                    //    force_right[i] = response_external_wrench_2[i];
                    //}
                    //force_left[0] = response_external_wrench_1[];
                    force_left[1] = -response_external_wrench_1[2] / 30;
                    //force_left[2] = -response_external_wrench_1[0] / 30;
                    
                    force_right[1] = -response_external_wrench_2[2] / 30;
                    
                    haptic.SetHapticForce(force_left, force_right);

                    actual_loop_frequency = 1f / (float)span.TotalSeconds;
                    loop_time_saved = DateTime.Now;
                }
            }
            Debug.Log(string.Format("Stop client thread."));
        }
        #endregion

        public void ProcessHapticCommand()
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
            if (button_right == 1)
            {
                // start teleoperation
                command_id_2 = (uint)StreamingCommand.STREAMING_COMMAND_START_TELEOPERATION;
                for (int i = 0; i < 3; i++)
                {
                    zero_pose_right[i] = raw_pose_right[i];
                    zero_pose_right[i + 3] = raw_pose_right[i + 3];
                    rel_pose_right[i] = 0;
                    rel_pose_right[i + 3] = raw_pose_right[i + 3];
                }
            }
            else if (button_right == 2)
            {
                // continue teleoperation
                command_id_2 = (uint)StreamingCommand.STREAMING_COMMAND_CONTINUE_TELEOPERATION;
                for (int i = 0; i < 3; i++)
                {
                    // position
                    rel_pose_right[i] = raw_pose_right[i] - zero_pose_right[i];

                    // rotation
                    rel_pose_right[i + 3] = raw_pose_right[i + 3];

                }
            }
            else if (button_right == 3)
            {
                // stop teleoperation
                command_id_2 = (uint)StreamingCommand.STREAMING_COMMAND_STOP_TELEOPERATION;
                for (int i = 0; i < 6; i++)
                {
                    zero_pose_right[i] = raw_pose_right[i];
                    rel_pose_right[i] = 0;
                }
            }
            else if (button_right == 0)
            {
                // null
                command_id_2 = (uint)StreamingCommand.STREAMING_COMMAND_REST;
                for (int i = 0; i < 6; i++)
                {
                    zero_pose_right[i] = raw_pose_right[i];
                    rel_pose_right[i] = 0;
                }
            }

            // save command (left & right)
            for (int i = 0; i < 6; i++)
            {
                command_float7_arr_1[i] = rel_pose_left[i];
                command_float7_arr_2[i] = rel_pose_right[i];
            }
            command_float7_arr_1[6] = 0;
            command_float7_arr_2[6] = 0;
        }

    }
}
