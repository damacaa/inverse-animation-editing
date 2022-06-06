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

    //public static Vector3[]  

    public float x, y, z;
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct Int
{
    public Int(int _i)
    {
        i = _i;
    }

    public int UnityInt()
    {
        return i;
    }

    public int i;
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct Float
{
    public Float(float _f)
    {
        f = _f;
    }

    public float UnityInt()
    {
        return f;
    }

    public float f;
}

public class ICPPWrapper
{
    public ICPPWrapper(SimulationManager.Integration integrationMethod, float timeStep, float tolerance)
    {
        CPPWrapper.Destroy();
        CPPWrapper.Initialize((int)integrationMethod, timeStep, tolerance);
    }

    public ICPPWrapper(string info)
    {
        CPPWrapper.Destroy();
        CPPWrapper.InitializeFromJSON(info);
    }

    public void Destroy()
    {
        Debug.Log("Destroy");
        CPPWrapper.Destroy();
    }

    public int AddObject(Vector3[] vertices, int[] triangles, float stiffness, float mass)
    {
        Vector3f[] _vertices = new Vector3f[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
        {
            _vertices[i].x = vertices[i].x;
            _vertices[i].y = vertices[i].y;
            _vertices[i].z = vertices[i].z;
        }

        Int[] _triangles = new Int[triangles.Length];
        for (int i = 0; i < triangles.Length; i++)
        {
            _triangles[i].i = triangles[i];
        }

        return CPPWrapper.AddObject(_vertices, vertices.Length, _triangles, triangles.Length, stiffness, mass);
    }

    internal int AddObject(Vector3[] vertPos, float vertMass, int[] springs, float[] springStiffness, float damping)
    {
        Vector3f[] _vertPos = new Vector3f[vertPos.Length];
        for (int i = 0; i < vertPos.Length; i++)
        {
            _vertPos[i].x = vertPos[i].x;
            _vertPos[i].y = vertPos[i].y;
            _vertPos[i].z = vertPos[i].z;
        }


        Int[] _springs = new Int[springs.Length];
        for (int i = 0; i < springs.Length; i++)
        {
            _springs[i].i = springs[i];
        }

        int nSprings = springs.Length / 2;
        Float[] _springStiffness = new Float[nSprings];
        for (int i = 0; i < nSprings; i++)
        {
            _springStiffness[i].f = springStiffness[i];
        }



        return CPPWrapper.AddObject(_vertPos, vertMass, vertPos.Length, _springs, _springStiffness, nSprings, damping);
    }

    internal int AddObject(Vector3[] vertPos, float[] vertVolume, int[] springs, float[] springStiffness, float[] springVolume, float density, float damping)
    {
        Vector3f[] _vertPos = new Vector3f[vertPos.Length];
        for (int i = 0; i < vertPos.Length; i++)
        {
            _vertPos[i].x = vertPos[i].x;
            _vertPos[i].y = vertPos[i].y;
            _vertPos[i].z = vertPos[i].z;
        }

        Float[] _vertVolume = new Float[vertPos.Length];
        for (int i = 0; i < vertPos.Length; i++)
        {
            _vertVolume[i].f = vertVolume[i];
        }

        Int[] _springs = new Int[springs.Length];
        for (int i = 0; i < springs.Length; i++)
        {
            _springs[i].i = springs[i];
        }

        int nSprings = springs.Length / 2;
        Float[] _springStiffness = new Float[nSprings];
        for (int i = 0; i < nSprings; i++)
        {
            _springStiffness[i].f = springStiffness[i];
        }

        Float[] _springVolume = new Float[nSprings];
        for (int i = 0; i < nSprings; i++)
        {
            _springVolume[i].f = springVolume[i];
        }

        return CPPWrapper.AddObject(_vertPos, _vertVolume, vertPos.Length, _springs, _springStiffness, _springVolume, nSprings, density, damping);
    }

    internal float Estimate(float v, int iterations)
    {
        return CPPWrapper.Estimate(v, iterations);
    }

    internal void StartSimulation(bool multithreading)
    {
        Debug.Log("Start");
        CPPWrapper.StartSimulation(multithreading);
    }

    public void AddFixer(Vector3 position, Vector3 scale)
    {
        CPPWrapper.AddFixer(new Vector3f(position), new Vector3f(scale));
    }


    public void Update()
    {
        CPPWrapper.Update(); /*Converts from Unity.Vector3 to our packed Vector3f struct*/
    }

    public Vector3[] GetVertices(int id)
    {

        IntPtr vertexArray = CPPWrapper.GetVertices(id, out int count);

        if (count == 0)
        {
            Debug.Log($"Vertices of object {id} not ready");
        }

        Vector3[] vertices = new Vector3[count];

        unsafe
        {
            Vector3f* vectorPointer = (Vector3f*)vertexArray.ToPointer();

            for (int i = 0; i < count; i++)
            {
                vertices[i].x = vectorPointer[i].x;
                vertices[i].y = vectorPointer[i].y;
                vertices[i].z = vectorPointer[i].z;
            }
        }

        return vertices;
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
    private const string moduleName = "UnityDLL";

    public class CPPWrapper
    {
        [DllImport(moduleName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Initialize(int integrationMethod, float timeStep, float tolerance);

        [DllImport(moduleName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void InitializeFromJSON(string info);

        [DllImport(moduleName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void StartSimulation(bool multithreading);

        [DllImport(moduleName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float Estimate(float value, int iterations);

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
        public static extern IntPtr GetVertices(int id, out int count);

        [DllImport(moduleName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int AddObject(Vector3f[] vertices, int nVertices, Int[] triangles, int nTriangles, float stiffness, float mass);

        [DllImport(moduleName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int AddObject(Vector3f[] vertPos, float vertMass, int nVerts, Int[] springs, Float[] springStiffness, int nSprings, float damping);

        [DllImport(moduleName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int AddObject(Vector3f[] vertPos, Float[] vertVolume, int nVerts, Int[] springs, Float[] springStiffness, Float[] springVolume, int nSprings, float density, float damping);

        [DllImport(moduleName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void AddFixer(Vector3f position, Vector3f scale);

    }
    #endregion
}
