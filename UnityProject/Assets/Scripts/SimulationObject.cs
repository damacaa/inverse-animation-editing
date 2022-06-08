using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;

public class SimulationObject : MonoBehaviour
{
    public int id = -1;

    Mesh mesh;

    [Header("Parameters")]

    [SerializeField]
    public float stiffness = 100f;
    [SerializeField]
    public float density = 1.0f;
    [SerializeField]
    public float damping = 1.0f;
    [SerializeField]
    bool useBendingSprings = true;
    [SerializeField]
    string optimizationSettings = "nn";

    public SimulationObjectData Data
    {
        get { return Init(); }
        set
        {
            _data = value;
            mesh = GetComponent<MeshFilter>().mesh;
            GenerateDebugInfo();
        }
    }

    [Header("Stuff")]
    [SerializeField]
    SimulationObjectData _data;
    [SerializeField]
    Vector3[] debugVerts;
    [SerializeField]
    int[] debugSprings;

    Gradient gradient;
    GradientColorKey[] colorKey;
    GradientAlphaKey[] alphaKey;
    Color[] debugColors;
    float maxStiffness;

    void GenerateDebugInfo()
    {
        gradient = new Gradient();

        // Populate the color keys at the relative time 0 and 1 (0 and 100%)
        colorKey = new GradientColorKey[2];
        colorKey[0].color = Color.green;
        colorKey[0].time = 0.0f;
        colorKey[1].color = Color.red;
        colorKey[1].time = 1.0f;

        // Populate the alpha  keys at relative time 0 and 1  (0 and 100%)
        alphaKey = new GradientAlphaKey[2];
        alphaKey[0].alpha = 1.0f;
        alphaKey[0].time = 0.0f;
        alphaKey[1].alpha = 1.0f;
        alphaKey[1].time = 1.0f;

        gradient.SetKeys(colorKey, alphaKey);

        maxStiffness = _data.springStiffness.Max();
        debugColors = new Color[_data.springStiffness.Length];
        for (int i = 0; i < debugColors.Length; i++)
        {
            debugColors[i] = gradient.Evaluate(_data.springStiffness[i] / maxStiffness);
        }

        debugVerts = _data.vertPos;
        debugSprings = _data.springs;
    }


    public SimulationObjectData Init()
    {
        mesh = GetComponent<MeshFilter>().mesh;
        _data = BuildData(mesh);
        GenerateDebugInfo();

        return _data;
    }


    private SimulationObjectData BuildData(Mesh mesh)
    {
        if (TryGetComponent<PlaneGenerator>(out PlaneGenerator pg))
            pg.BuildMesh();

        Debug.Log("Building " + name);

        SimulationObjectData data = new SimulationObjectData();

        Fixer[] fixers = FindObjectsOfType<Fixer>();

        //Nodes
        Vector3[] vertices = new Vector3[mesh.vertices.Length];
        bool[] vertIsFixed = new bool[vertices.Length];
        float[] vertMass = new float[vertices.Length];
        float[] vertVol = new float[vertices.Length];

        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 worldPos = transform.TransformPoint(mesh.vertices[i]);
            vertices[i] = worldPos;

            vertIsFixed[i] = false;
            foreach (var f in fixers)
            {
                if (f.CheckVert(worldPos))
                {
                    vertIsFixed[i] = true;
                    break;
                }
            }

            vertMass[i] = 0;
            vertVol[i] = 0;
        }
        data.vertPos = vertices;
        data.vertIsFixed = vertIsFixed;


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

            float area = 0.5f * Vector3.Cross(side1, side2).magnitude;

            for (int x = 0; x < 3; x++)
            {
                vertVol[mesh.triangles[i + x]] += area / 3;
                edges[x].volume += area;


                Edge otherEdge;
                if (edgeDictionary.TryGetValue(edges[x], out otherEdge))
                {
                    if (useBendingSprings)
                    {
                        //TO DO: Need to calculate different area
                        otherEdge.volume += area;

                        Edge newEdge = new Edge(edges[x].other, otherEdge.other, -1, stiffness / 4f);
                        newEdge.volume += 2f * area;
                        edgeDictionary.Add(newEdge, newEdge);
                    }
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

        int id = 0;
        foreach (Edge e in edgeDictionary.Values)
        {
            data.springs[2 * id] = e.a;
            data.springs[(2 * id) + 1] = e.b;
            float length = (vertices[e.a] - vertices[e.b]).magnitude;
            data.springStiffness[id] = e.stiffness * e.volume / (length * length);
            //stiffness * volume / (length0 * length0);

            id++;
        }

        //Node mass
        for (int i = 0; i < vertices.Length; i++)
        {
            vertMass[i] = vertVol[i] * density;
        }
        data.vertMass = vertMass;

        data.damping = damping;
        data.optimizationSettings = optimizationSettings;

        return data;
    }
    private SimulationObjectData BuildDataO(Mesh mesh)
    {
        Debug.Log("Building " + name);

        SimulationObjectData data = new SimulationObjectData();

        Fixer[] fixers = FindObjectsOfType<Fixer>();

        //Node positions
        Vector3[] vertices = new Vector3[mesh.vertices.Length];
        bool[] vertIsFixed = new bool[vertices.Length];
        float[] vertMass = new float[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 worldPos = transform.TransformPoint(mesh.vertices[i]);
            vertices[i] = worldPos;

            vertIsFixed[i] = false;
            foreach (var f in fixers)
            {
                if (f.CheckVert(worldPos))
                {
                    vertIsFixed[i] = true;
                    break;
                }
            }

            vertMass[i] = density / vertices.Length;
        }
        data.vertPos = vertices;
        data.vertIsFixed = vertIsFixed;
        data.vertMass = vertMass;

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

            float area = 0.5f * Vector3.Cross(side1, side2).magnitude;

            for (int x = 0; x < 3; x++)
            {
                Edge otherEdge;
                if (edgeDictionary.TryGetValue(edges[x], out otherEdge))
                {
                    if (useBendingSprings)
                    {
                        Edge newEdge = new Edge(edges[x].other, otherEdge.other, -1, stiffness / 4f);
                        edgeDictionary.Add(newEdge, newEdge);
                    }
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

        int id = 0;
        foreach (Edge e in edgeDictionary.Values)
        {
            data.springs[2 * id] = e.a;
            data.springs[(2 * id) + 1] = e.b;
            data.springStiffness[id] = e.stiffness;

            id++;
        }

        data.damping = damping;
        data.optimizationSettings = optimizationSettings;

        return data;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        Vector3[] vertices = SimulationManager.instance.GetVertices(id);

        if (vertices.Length == 0)
            return;


        debugVerts = (Vector3[])vertices.Clone();

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

        for (int i = 0; i < debugSprings.Length / 2; i++)
        {
            Gizmos.color = debugColors[i];
            Gizmos.DrawLine(debugVerts[debugSprings[2 * i]], debugVerts[debugSprings[(2 * i) + 1]]);
            //Gizmos.DrawLine(transform.TransformPoint( mesh.vertices[debugSprings[i]]), transform.TransformPoint(mesh.vertices[debugSprings[i + 1]]));
        }
    }

    private void OnValidate()
    {
        /*if (!isActiveAndEnabled)
            return;
        mesh = GetComponent<MeshFilter>().sharedMesh;
        BuildData();*/
    }

    public SimulationObjectData GetDataInEditor()
    {
        SimulationObjectData data = BuildData(GetComponent<PlaneGenerator>().Mesh);
        return data;
    }

#endif
}

[System.Serializable]
public class SimulationObjectData
{
    public Vector3[] vertPos;
    //public float[] vertVolume;
    public bool[] vertIsFixed;
    public float[] vertMass;
    public int[] springs;
    public float[] springStiffness;
    //public float[] springVolume;
    //public float density;
    public float damping;
    public string optimizationSettings;
}

[System.Serializable]
public class ParametrizedSimulationObjectData
{
    public Vector3[] vertPos;
    public float[] vertVolume;
    public int[] springs;
    public float[] springStiffness;
    public float[] springVolume;
    public float density;
    public float damping;
}