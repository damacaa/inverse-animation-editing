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

    public int AddObject(Vector3 position, Vector3[] vertices)
    {
        IntPtr vertexArray;
        
        unsafe
        {
            vertexArray = Marshal.AllocHGlobal(vertices.Length * sizeof(Vector3f));
            Vector3f* vectorPointer = (Vector3f*)vertexArray.ToPointer();

            for (int i = 0; i < vertices.Length; i++)
            {
                vectorPointer[i].x = vertices[i].x;
                vectorPointer[i].y = vertices[i].y;
                vectorPointer[i].z = vertices[i].z;
            }
        }

        unsafe
        {
            Vector3f* vectorPointer = (Vector3f*)vertexArray.ToPointer();
            for (int i = 0; i < vertices.Length; i++)
            {
                Debug.Log("//////////////");
                Debug.Log(vertices[i]);
                Debug.Log(vectorPointer[i].x + ", " +
                    vectorPointer[i].y + ", " +
                    vectorPointer[i].z);
            }
        }

        return cpp.AddObject(position, vertexArray, vertices.Length);
        //return 0;
    }

    public Vector3[] GetVertices(int id)
    {
        IntPtr vertexArray = cpp.GetVertices(id, out int count);

        if (count == 0)
        {
            unsafe
            {
                Vector3f* vectorPointer = (Vector3f*)vertexArray.ToPointer();

                Debug.Log("Id: " + vectorPointer[0].x);
                Debug.Log("Size: " + vectorPointer[0].y);
                //Debug.Log(vectorPointer[0].z);
            }
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

                Debug.Log(vertices[i]);
            }
        }

        return vertices;
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
