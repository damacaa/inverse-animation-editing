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

    [SerializeField]
    float mass = 1.0f;
    [SerializeField]
    float stiffness = 100f;

    private void Awake()
    {
        meshFilter = GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;
    }

    void Start()
    {
        Vector3[] vertices = mesh.vertices;

        for (int i = 0; i < vertices.Length; i++)
        {
            vertices[i] = transform.TransformPoint(vertices[i]);
        }

        id = SimulationManager.instance.AddObject(transform.position, vertices, mesh.triangles, stiffness, mass);
        gameObject.name = "Id: " + id;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        Vector3[] vertices = SimulationManager.instance.GetVertices(id);

        for (int i = 0; i < vertices.Length; i++)
        {
            vertices[i] = transform.InverseTransformPoint(vertices[i]);
        }

        mesh.vertices = vertices;
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
