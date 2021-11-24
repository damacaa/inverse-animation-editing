using System;
using System.Collections;
using System.Collections.Generic;
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
        id = SimulationManager.instance.AddObject(transform.position, mesh.vertices);
        gameObject.name = "Id: " + id;
    }

    // Update is called once per frame
    void Update()
    {
        mesh.vertices = SimulationManager.instance.GetVertices(id);
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
