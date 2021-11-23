using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Runtime.InteropServices;
using Random = UnityEngine.Random;

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct Vector3f
{
    public Vector3f(float _x, float _y, float _z)
    {
        x = _x;
        y = _y;
        z = _z;
    }
    public Vector3f(UnityEngine.Vector3 v)
    {
        x = v.x;
        y = v.y;
        z = v.z;
    }

    public Vector3 UnityVector()
    {
        return new Vector3(x, y, z);
    }

    public float x, y, z;
}

public class ICPPWrapper
{

    public ICPPWrapper()
    {
        CPPWrapper.Destroy();
        CPPWrapper.Initialize();
    }

    public void Destroy()
    {
        Debug.Log("Destroy");
        CPPWrapper.Destroy();
    }

    public int AddObject(in Vector3 position)
    {
        return CPPWrapper.AddObject(new Vector3f(position));
    }


    public void Update()
    {
        CPPWrapper.Update(); /*Converts from Unity.Vector3 to our packed Vector3f struct*/
    }

    public IntPtr GetVertices(in int id, out int count)
    {
        return CPPWrapper.GetVertices(in id, out count);
    }


    #region examples
    public void IncreaseCounter()
    {
        CPPWrapper.IncreaseCounter();
    }

    public int GetCounter()
    {
        return CPPWrapper.GetCounter();
    }

    public int GetThreadCounter()
    {
        return CPPWrapper.GetThreadCounter();
    }
    #endregion

    #region C++ Functions
    /************************************************************
     *                      C++ Functions
     * **********************************************************/
    private const string moduleName = "UnityDLLExample";

    public class CPPWrapper
    {
        [DllImport(moduleName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Initialize();

        [DllImport(moduleName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Destroy();

        [DllImport(moduleName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void IncreaseCounter();

        [DllImport(moduleName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int GetCounter();

        [DllImport(moduleName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int GetThreadCounter();

        [DllImport(moduleName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Update();

        [DllImport(moduleName, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr GetVertices(in int id, out int count);

        [DllImport(moduleName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int AddObject(in Vector3f position);
    }
    #endregion
}
