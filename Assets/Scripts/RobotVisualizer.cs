using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace BiPanda
{
    public class RobotVisualizer : MonoBehaviour
    {
        public float[] JointInput;
        private GameObject[] link_chain;

        // panda robot spec.
        float[] q_max = new float[7] { 166.003062f, 101.0010001f, 166.003062f, -3.99925f, 166.0031f, 215.0024f, 166.0031f };
        float[] q_min = new float[7] { -166.003062f, -101.0010001f, -166.003062f, -176.0012f, -166.0031f, -1.00268f, -166.0031f };
        float[] q_home = new float[7] { 0.0f, -45.0f, 0.0f, -135.0f, 0.0f, 90.0f, -45.0f };

        void Start()
        {
            JointInput = new float[7];
            //for (int idx = 0; idx < 7; idx++)
            //{
            //    JointInput[idx] = q_home[idx];
            //}
            link_chain = new GameObject[7];

            //Debug.Log("Parent: " + transform.gameObject);

            String child_name = "base/fr3_link0/fr3_link1";
            link_chain[0] = transform.gameObject.transform.Find(child_name).gameObject;
            child_name += "/fr3_link2";
            link_chain[1] = transform.gameObject.transform.Find(child_name).gameObject;
            child_name += "/fr3_link3";
            link_chain[2] = transform.gameObject.transform.Find(child_name).gameObject;
            child_name += "/fr3_link4";
            link_chain[3] = transform.gameObject.transform.Find(child_name).gameObject;
            child_name += "/fr3_link5";
            link_chain[4] = transform.gameObject.transform.Find(child_name).gameObject;
            child_name += "/fr3_link6";
            link_chain[5] = transform.gameObject.transform.Find(child_name).gameObject;
            child_name += "/fr3_link7";
            link_chain[6] = transform.gameObject.transform.Find(child_name).gameObject;
        }


        void Update()
        {
            for (int idx = 1; idx < 8; idx++)
            {
                if (JointInput[idx - 1] > q_max[idx - 1])
                {
                    JointInput[idx - 1] = q_max[idx - 1];
                }
                else if (JointInput[idx - 1] < q_min[idx - 1])
                {
                    JointInput[idx - 1] = q_min[idx - 1];
                }
            }

            link_chain[0].transform.localEulerAngles = new Vector3(0.0f, -JointInput[0], 0.0f);
            link_chain[1].transform.localEulerAngles = new Vector3(JointInput[1], 0.0f, 90.0f);
            link_chain[2].transform.localEulerAngles = new Vector3(-JointInput[2], 0.0f, -90.0f);
            link_chain[3].transform.localEulerAngles = new Vector3(-JointInput[3], 0.0f, -90.0f);
            link_chain[4].transform.localEulerAngles = new Vector3(JointInput[4], 0.0f, 90.0f);
            link_chain[5].transform.localEulerAngles = new Vector3(-JointInput[5], 0.0f, -90.0f);
            link_chain[6].transform.localEulerAngles = new Vector3(-JointInput[6], 0.0f, -90.0f);
        }
    }
}
