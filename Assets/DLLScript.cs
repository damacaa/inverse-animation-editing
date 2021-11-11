using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Runtime.InteropServices;

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
        CPPWrapper.Initialize();
    }

    public void Destroy()
    {
        Debug.Log("Destroy");
        CPPWrapper.Destroy();
    }

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

    public void Update(in Vector3 _pos)
    {
        CPPWrapper.Update(new Vector3f(_pos)); /*Converts from Unity.Vector3 to our packed Vector3f struct*/
    }

    public IntPtr GetVertices(out int count)
    {
        return CPPWrapper.GetVertices(out count);
    }

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
        public static extern void Update(in Vector3f position);

        [DllImport(moduleName, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr GetVertices(out int count);
    }
    #endregion
}

public class DLLScript : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        Debug.Log("Start");
        cpp = new ICPPWrapper();
    }

    private void OnDestroy()
    {
        Debug.Log("OnDestroy");
        cpp.Destroy();
        nVertices = 0;
    }

    // Update is called once per frame
    void Update()
    {
        cpp.Update(transform.position); /*Pass current position of object attached to this script*/
        cpp.IncreaseCounter();

        Debug.Log("       Counter = " + cpp.GetCounter());
        Debug.Log("Thread Counter = " + cpp.GetThreadCounter());

        vertexArray = cpp.GetVertices(out int count);

        if (nVertices != count)
        {
            /*Extend array*/
            nVertices = count;
            vertices = new Vector3[nVertices];
        }

        Vector3 sum = new Vector3();

        unsafe
        {
            Vector3f* vectorPointer = (Vector3f*)vertexArray.ToPointer();

            for (int i = 0; i < count; i++)
            {
                vertices[i].x = vectorPointer[i].x;
                vertices[i].y = vectorPointer[i].y;
                vertices[i].z = vectorPointer[i].z;

                sum += vertices[i];
            }
        }

        Debug.Log("Sum = " + sum.x + ", " + sum.y + ", " + sum.z);
    }

    ICPPWrapper cpp;

    IntPtr vertexArray;
    int nVertices = 0;
    Vector3[] vertices;

#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        if (!Application.isPlaying)
            return;

        Gizmos.color = Color.white;
        foreach (Vector3 v in vertices)
        {
            Gizmos.DrawSphere(v, 0.1f);
        }
    }
#endif
}
