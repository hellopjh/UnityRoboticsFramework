using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System;
using System.Threading;
using System.Net.Http;

namespace BiPanda
{
    public class RobotInterface : MonoBehaviour
    {
        #region Variables
        // client
        public bool connect;
        public bool disconnect;
        private bool is_connected;
        private bool is_client_thread_running;
        private Thread client_thread;
        private ThreadStart client_method;

        // time
        public float loop_frequency;
        public float actual_loop_frequency;
        private DateTime loop_time_saved;
        private TimeSpan span;

        #endregion

        void Start()
        {
            connect = false;
            disconnect = false;
            is_connected = false;
            is_client_thread_running = false;

            loop_frequency = 100f;
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

            Debug.Log(string.Format("Start client thread."));
            is_client_thread_running = true;
            while(is_client_thread_running)
            {
                span = DateTime.Now - loop_time_saved;
                if(span.TotalSeconds > ((1f / loop_frequency) * 0.95f))
                {
                    // 100 Hz loop


                    actual_loop_frequency = 1f / (float)span.TotalSeconds;
                    loop_time_saved = DateTime.Now;
                }
            }
            Debug.Log(string.Format("Stop client thread."));
        }
        #endregion
    }
}
