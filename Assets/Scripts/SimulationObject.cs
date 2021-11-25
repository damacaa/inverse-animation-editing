using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

public class SimulationObject : MonoBehaviour
{
    int id;

    IntPtr vertexArray;
    int nVertices = 0;

    MeshFilter meshFilter;
    Mesh mesh;

    private void Awake()
    {
        meshFilter = GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;
    }

    void Start()
    {
        id = SimulationManager.instance.AddObject(transform.position, mesh.vertices, mesh.triangles);
        gameObject.name = "Id: " + id;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        mesh.vertices = SimulationManager.instance.GetVertices(id);
        mesh.RecalculateNormals();
        mesh.RecalculateTangents();
    }

#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        if (!Application.isPlaying)
            return;

        for (int i = 0; i < mesh.vertices.Length; i++)
        {
            Handles.Label(transform.TransformPoint(mesh.vertices[i]), i.ToString());
        }
    }
#endif
}
