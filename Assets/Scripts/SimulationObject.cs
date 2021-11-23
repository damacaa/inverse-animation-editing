using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimulationObject : MonoBehaviour
{
    int id;

    IntPtr vertexArray;
    int nVertices = 0;
    Vector3[] vertices;
    MeshFilter meshFilter;
    Mesh mesh;

    private void Awake()
    {
        meshFilter = GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;
    }

    void Start()
    {
        vertices = new Vector3[nVertices];
        id = SimulationManager.instance.AddObject(transform.position);
        gameObject.name = "Id: " + id;
    }

    // Update is called once per frame
    void Update()
    {
        vertexArray = SimulationManager.instance.GetVertices(id, out int count);

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

        if (nVertices != count)
        {
            /*Extend array*/
            nVertices = count;
            vertices = new Vector3[nVertices];
        }



        unsafe
        {
            Vector3f* vectorPointer = (Vector3f*)vertexArray.ToPointer();

            for (int i = 0; i < count; i++)
            {
                vertices[i].x = vectorPointer[i].x;
                vertices[i].y = vectorPointer[i].y;
                vertices[i].z = vectorPointer[i].z;

                vertices[i] += transform.position;
            }
        }

        mesh.vertices = vertices;
        int side = (int)Mathf.Sqrt(vertices.Length);
        int[] triangles = new int[3 * (vertices.Length - side)];
        for (int i = 0; i < vertices.Length - side; i++)
        {
            triangles[i] = i;
            triangles[i + 1] = i + 1;
            triangles[i + 2] = i + side;
        }
        mesh.triangles = triangles;
    }

#if UNITY_EDITOR
    /*private void OnDrawGizmos()
    {
        if (!Application.isPlaying)
            return;

        Gizmos.color = Color.white;
        foreach (Vector3 v in vertices)
        {
            Gizmos.DrawSphere(v, .5f);
        }
    }*/
#endif
}
