using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

public class SimulationObject : MonoBehaviour
{
    public int id;

    [SerializeField]
    Mesh mesh;
    [SerializeField]
    public SimulationObjectData data;


    [SerializeField]
    public float stiffness = 100f;
    [SerializeField]
    public float density = 1.0f;
    [SerializeField]
    public float damping = 1.0f;

    [SerializeField]
    Vector3[] debugVerts;
    [SerializeField]
    int[] debugSprings;

    private void Awake()
    {
        SimulationManager.simulationObjects.Add(this);
    }


    private void BuildData()
    {
        Debug.Log("Building " + name);

        data = new SimulationObjectData();
        mesh = GetComponent<MeshFilter>().sharedMesh;

        //Node positions
        Vector3[] vertices = new Vector3[mesh.vertices.Length];
        for (int i = 0; i < mesh.vertices.Length; i++)
        {
            vertices[i] = transform.TransformPoint(mesh.vertices[i]);
        }
        data.vertPos = vertices;
        debugVerts = vertices;

        //Node volumes
        data.vertVolume = new float[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
        {
            data.vertVolume[i] = 0;
        }

        //Creating edges
        EdgeEqualityComparer edgeEqualityComparer = new EdgeEqualityComparer();
        Dictionary<Edge, Edge> edgeDictionary = new Dictionary<Edge, Edge>(edgeEqualityComparer);
        for (int i = 0; i < mesh.triangles.Length; i += 3)
        {
            Edge[] edges = new Edge[3];
            edges[0] = new Edge(mesh.triangles[i], mesh.triangles[i + 1], mesh.triangles[i + 2], stiffness);
            edges[1] = new Edge(mesh.triangles[i], mesh.triangles[i + 2], mesh.triangles[i + 1], stiffness);
            edges[2] = new Edge(mesh.triangles[i + 1], mesh.triangles[i + 2], mesh.triangles[i], stiffness);

            Vector3 side1 = vertices[mesh.triangles[i + 1]] - vertices[mesh.triangles[i]];
            Vector3 side2 = vertices[mesh.triangles[i + 2]] - vertices[mesh.triangles[i]];

            //float area = 0.5f * GetNormal(nodes[triangles[i]].pos, nodes[triangles[i + 1]].pos, nodes[triangles[i + 2]].pos).magnitude;
            //float area = 1f;
            float area = 0.5f * Vector3.Cross(side1, side2).magnitude;

            for (int x = 0; x < 3; x++)
            {
                //Add the volume to each node
                data.vertVolume[mesh.triangles[i + x]] += area / 3;

                edges[x].volume += area;

                //edges[x].volume = 1;

                Edge otherEdge;
                if (edgeDictionary.TryGetValue(edges[x], out otherEdge))
                {
                    //La arista está en el diccionario
                    otherEdge.volume += area;


                    Edge newEdge = new Edge(edges[x].other, otherEdge.other, -1, stiffness / 2f);
                    newEdge.volume += 2f * area;
                    edgeDictionary.Add(newEdge, newEdge);
                }
                else
                {
                    //La arista no está en el diccionario
                    edgeDictionary.Add(edges[x], edges[x]);
                }

            }
        }

        //Creating a spring for each edge
        int nSprings = edgeDictionary.Count;
        data.springs = new int[nSprings * 2];
        data.springStiffness = new float[nSprings];
        data.springVolume = new float[nSprings];

        int id = 0;
        foreach (Edge e in edgeDictionary.Values)
        {
            data.springs[2 * id] = e.a;
            data.springs[(2 * id) + 1] = e.b;
            data.springStiffness[id] = e.stiffness;
            data.springVolume[id] = e.volume;

            id++;
        }

        data.density = density;
        data.damping = damping;

        debugSprings = data.springs;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        Vector3[] vertices = SimulationManager.instance.GetVertices(id);

        if (vertices.Length == 0)
            return;

        debugVerts = vertices;

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

        /*for (int i = 0; i < mesh.vertices.Length; i++)
        {
            Handles.Label(transform.TransformPoint(mesh.vertices[i]), i.ToString());
        }*/

        for (int i = 0; i < debugSprings.Length; i += 2)
        {
            Gizmos.DrawLine(debugVerts[debugSprings[i]], debugVerts[debugSprings[i + 1]]);
        }
    }

    private void OnValidate()
    {
        /*if (!isActiveAndEnabled)
            return;
        mesh = GetComponent<MeshFilter>().sharedMesh;
        BuildData();*/
    }
#endif
}

[System.Serializable]
public class SimulationObjectData
{
    public Vector3[] vertPos;
    public float[] vertVolume;
    public int[] springs;
    public float[] springStiffness;
    public float[] springVolume;
    public float density;
    public float damping;
}
