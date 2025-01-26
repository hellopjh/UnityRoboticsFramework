using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.Net.Sockets;
using System.Threading;
using UnityEngine;
using System.Net.Http;
using Unity.VisualScripting;
using UnityEngine.Rendering;

namespace BiPanda
{
    enum MessageSpec : int
    {
        SIZE_USER_COMMAND       = 69,
        SIZE_ROBOT_RESPONSE     = 113,
    }
    enum StreamingCommand : int
    {
        STREAMING_COMMAND_REST                      = 0,
        STREAMING_COMMAND_START_TELEOPERATION       = 10,
        STREAMING_COMMAND_CONTINUE_TELEOPERATION    = 11,
        STREAMING_COMMAND_STOP_TELEOPERATION        = 12,
    }

    enum DCPCOMMAND : int
    {
        DCP_COMMAND_REST            = 0,
        DCP_COMMAND_MOVE_JOINT_TO   = 1,
        DCP_COMMAND_MOVE_JOINT_BY   = 2,
        DCP_COMMAND_MOVE_TASK_TO    = 3,
    }

    struct UserCommand
    {
        enum UserCommandDataSpec : int
        {
            /// sof                     (ubyte, 1 byte)
            /// invoke_id               (int32, 4 bytes)
            /// command_id_1            (int32, 4 bytes)
            /// del_task_pose_1         (float, 4*7 bytes)
            /// command_id_2            (int32, 4 bytes)
            /// del_task_pose_2         (float, 4*7 bytes)

            SIZE_SOF                    = 1,
            SIZE_INVOKE_ID              = 4,
            SIZE_COMMAND_ID             = 4,
            SIZE_DEL_TASK_POSE          = 28,
        }

        public byte sof;
        public uint invoke_id;
        public uint command_id_1;
        public float[] del_task_pose_1;
        public uint command_id_2;
        public float[] del_task_pose_2;

        public byte[] byte_arr;

        public void init()
        {
            sof = 0x34;
            invoke_id = 0;
            command_id_1 = 0;
            del_task_pose_1 = new float[7];
            command_id_2 = 0;
            del_task_pose_2 = new float[7];

            byte_arr = new byte[(int)MessageSpec.SIZE_USER_COMMAND];
        }

        public void prepare_message()
        {
            int buffer_idx = 0;
            byte[] tmp;

            byte_arr[buffer_idx] = sof;
            buffer_idx += (int)UserCommandDataSpec.SIZE_SOF;

            invoke_id += 1;
            if (invoke_id > 1024) invoke_id = 1;
            tmp = BitConverter.GetBytes(invoke_id);
            Buffer.BlockCopy(tmp, 0, byte_arr, buffer_idx, (int)UserCommandDataSpec.SIZE_INVOKE_ID);
            buffer_idx += (int)UserCommandDataSpec.SIZE_INVOKE_ID;

            tmp = BitConverter.GetBytes(command_id_1);
            Buffer.BlockCopy(tmp, 0, byte_arr, buffer_idx, (int)UserCommandDataSpec.SIZE_COMMAND_ID);
            buffer_idx += (int)UserCommandDataSpec.SIZE_COMMAND_ID;

            Buffer.BlockCopy(del_task_pose_1, 0, byte_arr, buffer_idx, (int)UserCommandDataSpec.SIZE_DEL_TASK_POSE);
            buffer_idx += (int)UserCommandDataSpec.SIZE_DEL_TASK_POSE;

            tmp = BitConverter.GetBytes(command_id_2);
            Buffer.BlockCopy(tmp, 0, byte_arr, buffer_idx, (int)UserCommandDataSpec.SIZE_COMMAND_ID);
            buffer_idx += (int)UserCommandDataSpec.SIZE_COMMAND_ID;

            Buffer.BlockCopy(del_task_pose_2, 0, byte_arr, buffer_idx, (int)UserCommandDataSpec.SIZE_DEL_TASK_POSE);
            buffer_idx += (int)UserCommandDataSpec.SIZE_DEL_TASK_POSE;
        }
    }

    struct RobotResponse
    {
        enum RobotResponseDataSpec : int
        {
            /// sof                     (ubyte, 1 byte)
            /// invoke_id               (int32, 4 bytes)
            /// response_id             (int32, 4 bytes)
            /// joint_pos_1             (float, 4*7=28 bytes)
            /// external_wrench_1       (float, 4*6=24 bytes)
            /// joint_pos_2             (float, 4*7=28 bytes)
            /// external_wrench_2       (float, 4*6=24 bytes)

            SIZE_SOF = 1,
            SIZE_INVOKE_ID              = 4,
            SIZE_RESPONSE_ID            = 4,
            SIZE_JOINT_POS              = 28,
            SIZE_EXTERNAL_WRENCH        = 24,
        }

        public byte sof;
        public uint invoke_id;
        public uint response_id;
        public float[] joint_pos_1;
        public float[] external_wrench_1;
        public float[] joint_pos_2;
        public float[] external_wrench_2;

        public byte[] byte_arr;

        public void init()
        {
            sof = 0x12;
            invoke_id = 0;
            response_id = 0;
            joint_pos_1 = new float[7];
            external_wrench_1 = new float[6];
            joint_pos_2 = new float[7];
            external_wrench_2 = new float[6];

            byte_arr = new byte[(int)MessageSpec.SIZE_ROBOT_RESPONSE];
        }

        public void parse_message()
        {
            int buffer_idx = 0;

            byte[] tmp_sof = new byte[(int)RobotResponseDataSpec.SIZE_SOF];
            Buffer.BlockCopy(byte_arr, buffer_idx, tmp_sof, 0, (int)RobotResponseDataSpec.SIZE_SOF);
            sof = tmp_sof[0];
            buffer_idx += (int)RobotResponseDataSpec.SIZE_SOF;

            byte[] tmp_invoke_id = new byte[(int)RobotResponseDataSpec.SIZE_INVOKE_ID];
            Buffer.BlockCopy(byte_arr, buffer_idx, tmp_invoke_id, 0, (int)RobotResponseDataSpec.SIZE_INVOKE_ID);
            invoke_id = BitConverter.ToUInt32(tmp_invoke_id, 0);
            buffer_idx += (int)RobotResponseDataSpec.SIZE_INVOKE_ID;

            byte[] tmp_response_id = new byte[(int)RobotResponseDataSpec.SIZE_RESPONSE_ID];
            Buffer.BlockCopy(byte_arr, buffer_idx, tmp_response_id, 0, (int)RobotResponseDataSpec.SIZE_RESPONSE_ID);
            response_id = BitConverter.ToUInt32(tmp_response_id, 0);
            buffer_idx += (int)RobotResponseDataSpec.SIZE_RESPONSE_ID;

            Buffer.BlockCopy(byte_arr, buffer_idx, joint_pos_1, 0, (int)RobotResponseDataSpec.SIZE_JOINT_POS);
            buffer_idx += (int)RobotResponseDataSpec.SIZE_JOINT_POS;

            Buffer.BlockCopy(byte_arr, buffer_idx, external_wrench_1, 0, (int)RobotResponseDataSpec.SIZE_EXTERNAL_WRENCH);
            buffer_idx += (int)RobotResponseDataSpec.SIZE_EXTERNAL_WRENCH;

            Buffer.BlockCopy(byte_arr, buffer_idx, joint_pos_2, 0, (int)RobotResponseDataSpec.SIZE_JOINT_POS);
            buffer_idx += (int)RobotResponseDataSpec.SIZE_JOINT_POS;

            Buffer.BlockCopy(byte_arr, buffer_idx, external_wrench_2, 0, (int)RobotResponseDataSpec.SIZE_EXTERNAL_WRENCH);
            buffer_idx += (int)RobotResponseDataSpec.SIZE_EXTERNAL_WRENCH;
        }
    }


    class BiPandaClient
    {
        private TcpClient _tcp_client;
        private NetworkStream _stream;

        private string _tcp_ip;
        private int _tcp_port;
        private int _tcp_timeout;

        private uint _response_invoke_id;
        private uint _response_response_id;
        private float[] _response_joint_pos_1;
        private float[] _response_external_wrench_1;
        private float[] _response_joint_pos_2;
        private float[] _response_external_wrench_2;

        public bool _is_connected;

        public BiPandaClient(string server_ip, int server_port=7078)
        {
            _tcp_ip = server_ip;
            _tcp_port = server_port;
            _tcp_timeout = 3000;        // ms

            _response_invoke_id = 0;
            _response_response_id = 0;
            _response_joint_pos_1 = new float[7];
            _response_external_wrench_1 = new float[6];
            _response_joint_pos_2 = new float[7];
            _response_external_wrench_2 = new float[6];

            _is_connected = false;
        }

        public bool connect(bool show_result = true)
        {
            _tcp_client = new TcpClient();

            try
            {
                _tcp_client.Connect(_tcp_ip, _tcp_port);
            }
            catch (Exception ex)
            {
                Debug.LogError("[CustomTCPClient:connect] failed to connect to server");
                return false;
            }

            try
            {
                _stream = _tcp_client.GetStream();
            }
            catch (Exception ex)
            {
                Debug.LogError("[CustomTCPClient:connect] failed to get NetworkStream");
                _tcp_client.Close();
                return false;
            }

            if (show_result)
            {
                //Debug.Log(string.Format("[CustomTCPClient:connect] successfully connected to {0}:{1}", _tcp_ip, _tcp_port));
            }

            _setTimeOut(_tcp_timeout);
            _is_connected = true;

            return true;
        }

        public bool disconnect()
        {
            _stream.Close();
            _tcp_client.Close();

            _is_connected = false;
            //Debug.Log("[CustomTCPClient:disconnect] disconnected.");

            return true;
        }

        private void _setTimeOut(int ms)
        {
            _tcp_client.ReceiveTimeout = ms;
        }

        private int _sendMessage(byte[] buf, int size)
        {
            try
            {
                _stream.Write(buf, 0, size);
            }
            catch (SocketException e)
            {
                Debug.LogError(e);
                return -1;
            }
            return 0;
        }

        private int _recvMessage(ref byte[] buf, int size)
        {
            int read_data_size = 0;
            try
            {
                read_data_size = _stream.Read(buf, 0, size);
            }
            catch (SocketException e)
            {
                Debug.LogError(e);
                return -1;
            }

            return read_data_size;
        }

        private int _handleMessage(UserCommand req, ref RobotResponse res)
        {
            if (!_tcp_client.Connected)
            {
                Debug.LogError("[CustomTCPClient:_handleMessage] TCP socket is not connected.");
                return -1;
            }

            byte[] writeBuff = new byte[1024];
            byte[] readBuff = new byte[1024];

            req.prepare_message();
            Buffer.BlockCopy(req.byte_arr, 0, writeBuff, 0, (int)MessageSpec.SIZE_USER_COMMAND);
            if (_sendMessage(writeBuff, (int)MessageSpec.SIZE_USER_COMMAND) != 0)
            {
                Debug.LogError("[CustomTCPClient:_handleMessage] Failed to send command: Disconnected");
                disconnect();
                return -1;
            }

            res.init();
            int numReadBytes;
            numReadBytes = _recvMessage(ref readBuff, (int)MessageSpec.SIZE_ROBOT_RESPONSE);
            if (numReadBytes != (int)MessageSpec.SIZE_ROBOT_RESPONSE)
            {
                Debug.LogError(String.Format("[CustomTCPClient:_handleMessage] Response message size is invalid {0}/{1}: Disconnected", numReadBytes, (int)MessageSpec.SIZE_ROBOT_RESPONSE));
                disconnect();
                return -1;
            }
            Buffer.BlockCopy(readBuff, 0, res.byte_arr, 0, (int)MessageSpec.SIZE_ROBOT_RESPONSE);
            res.parse_message();

            //Debug.Log("check::_handleMessage");

            return 0;
        }

        public int update_teleoperation_command(uint command_1, float[] target_command_arr_1, uint command_2, float[] target_command_arr_2)
        {
            UserCommand req = new UserCommand();
            req.init();
            req.command_id_1 = command_1;
            req.command_id_2 = command_2;
            for (int i = 0; i < 7; i++)
            {
                req.del_task_pose_1[i] = target_command_arr_1[i];
                req.del_task_pose_2[i] = target_command_arr_2[i];
            }

            RobotResponse res = new RobotResponse();

            if (_handleMessage(req, ref res) == 0)
            {
                _response_response_id = (uint)res.response_id;
                for (int i = 0; i < 6; i++)
                {
                    _response_joint_pos_1[i] = res.joint_pos_1[i];
                    _response_external_wrench_1[i] = res.external_wrench_1[i];
                    _response_joint_pos_2[i] = res.joint_pos_2[i];
                    _response_external_wrench_2[i] = res.external_wrench_2[i];
                }
                _response_joint_pos_1[6] = res.joint_pos_1[6];
                _response_joint_pos_2[6] = res.joint_pos_2[6];
                return 0;
            }
            else
            {
                _response_response_id = 99;
                for (int i = 0; i < 6; i++)
                {
                    _response_joint_pos_1[i] = 0f;
                    _response_external_wrench_1[i] = 0f;
                    _response_joint_pos_2[i] = 0f;
                    _response_external_wrench_2[i] = 0f;
                }
                _response_joint_pos_1[6] = 0f;
                _response_joint_pos_2[6] = 0f;
                return -1;
            }
        }

        public int update_robot_status(ref float[] joint_pos_1, ref float[] external_wrench_1, ref float[] joint_pos_2, ref float[] external_wrench_2)
        {
            for(int i=0; i<6; i++)
            {
                joint_pos_1[i] = _response_joint_pos_1[i];
                external_wrench_1[i] = _response_external_wrench_1[i];
                joint_pos_2[i] = _response_joint_pos_2[i];
                external_wrench_2[i] = _response_external_wrench_2[i];
            }
            joint_pos_1[6] = _response_joint_pos_1[6];
            joint_pos_2[6] = _response_joint_pos_2[6];

            return 0;
        }

        #region DCP Command
        public void move_joint_to(float[] q_des, uint arm_num)
        {
            float[] float_arr = new float[7];
            for(int i=0; i<7; i++) float_arr[i] = q_des[i];
            if(update_teleoperation_command((uint)DCPCOMMAND.DCP_COMMAND_MOVE_JOINT_TO, float_arr, (uint)DCPCOMMAND.DCP_COMMAND_MOVE_JOINT_TO, float_arr) != 0)
            {
                Debug.LogError("Communication failed.");
            }
            else
            {
                Debug.Log("Communication succedded.");
            }
        }

        public void move_task_to(float[] p_des, uint arm_num)
        {
            float[] float_arr = new float[7];
            for (int i = 0; i < 6; i++) float_arr[i] = p_des[i];
            if(update_teleoperation_command((uint)DCPCOMMAND.DCP_COMMAND_MOVE_TASK_TO, float_arr, (uint)DCPCOMMAND.DCP_COMMAND_MOVE_TASK_TO, float_arr) != 0)
            {
                Debug.LogError("Communication failed.");
            }
            else
            {
                Debug.Log("Communication succedded.");
            }
        }
        #endregion
    }
}
