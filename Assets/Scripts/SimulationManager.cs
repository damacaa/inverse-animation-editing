using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Runtime.InteropServices;
using Random = UnityEngine.Random;



public class SimulationManager : MonoBehaviour
{
    public static SimulationManager instance;

    ICPPWrapper cpp;
    private void Awake()
    {
        if (instance)
        {
            Destroy(gameObject);
        }
        else
            instance = this;

        Debug.Log("Awake");
        cpp = new ICPPWrapper();
    }

    public int AddObject(Vector3 position)
    {
        return cpp.AddObject(position);
    }

    public IntPtr GetVertices(int id, out int count)
    {
        return cpp.GetVertices(id, out count);
    }

    private void OnDestroy()
    {
        cpp.Destroy();
    }

    // Update is called once per frame
    void Update()
    {
        cpp.Update();
        cpp.IncreaseCounter();

        //Debug.Log("Counter = " + cpp.GetCounter());
        //Debug.Log("Thread Counter = " + cpp.GetThreadCounter());
    }

}
