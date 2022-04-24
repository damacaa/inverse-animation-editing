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

    Mesh mesh;

    [SerializeField]
    public float mass = 1.0f;
    [SerializeField]
    public float stiffness = 100f;

    private void Awake()
    {
        SimulationManager.simulationObjects.Add(this);

        mesh = GetComponent<MeshFilter>().mesh;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        Vector3[] vertices = SimulationManager.instance.GetVertices(id);

        if (vertices.Length == 0)
            return;

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
