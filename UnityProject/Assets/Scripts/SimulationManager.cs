using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Runtime.InteropServices;
using Random = UnityEngine.Random;



public class SimulationManager : MonoBehaviour
{
    public static SimulationManager instance;
    public static SimulationManager Instance
    {
        get
        {
            if (!instance)
            {
                instance = FindObjectOfType<SimulationManager>();
                if (!instance)
                    instance = new GameObject().AddComponent<SimulationManager>();
            }

            return instance;
        }
    }

    public static List<SimulationObject> simulationObjects = new List<SimulationObject>();


    public enum Mode
    {
        None,
        SimulateSingleThread,
        SimulateMultiThread,
        Estimate
    }

    [SerializeField]
    Mode mode = Mode.SimulateMultiThread;


    [SerializeField]
    Integration integrationMethod = Integration.Implicit;
    [SerializeField]
    float timeStep = 0.01f;
    float lastTime = 0;
    [SerializeField]
    float parameter = 0.5f;
    [SerializeField]
    int iterations = 1;

    ICPPWrapper cpp;
    private void Awake()
    {
        cpp = new ICPPWrapper(integrationMethod, timeStep);
        instance = this;
    }

    private void Start()
    {
        foreach (SimulationObject so in simulationObjects)
        {
            Mesh mesh = so.GetComponent<MeshFilter>().mesh;

            Vector3[] vertices = mesh.vertices;

            for (int i = 0; i < vertices.Length; i++)
            {
                vertices[i] = so.transform.TransformPoint(vertices[i]);
            }

            int id = SimulationManager.Instance.AddObject(so.transform.position, vertices, mesh.triangles, so.stiffness, so.mass);
            so.name = "Id: " + id;
        }

        switch (mode)
        {
            case Mode.None:
                break;
            case Mode.SimulateSingleThread:
                cpp.StartSimulation(false);
                break;
            case Mode.SimulateMultiThread:
                cpp.StartSimulation(true);
                break;
            case Mode.Estimate:
                print(cpp.Estimate(parameter, iterations));
                break;
            default:
                break;
        }

        /*if (simulate)
            cpp.StartSimulation();
        else
            print(cpp.Estimate(0.5f));*/
    }

    public int AddObject(Vector3 position, Vector3[] vertices, int[] triangles, float stiffness, float mass)
    {
        Vector3f[] __vertices = new Vector3f[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
        {
            __vertices[i].x = vertices[i].x;
            __vertices[i].y = vertices[i].y;
            __vertices[i].z = vertices[i].z;
        }

        Int[] _triangles = new Int[triangles.Length];
        for (int i = 0; i < triangles.Length; i++)
        {
            _triangles[i].i = triangles[i];
        }

        return cpp.AddObject(position, __vertices, _triangles, stiffness, mass);
    }

    public void AddFixer(Vector3 position, Vector3 scale)
    {
        cpp.AddFixer(position, scale);
    }

    public Vector3[] GetVertices(int id)
    {
        IntPtr vertexArray = cpp.GetVertices(id, out int count);

        if (count == 0)
        {
            Debug.Log("Vertices not ready");
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

        //Debug.Log(vertices[1]);

        return vertices;
    }

    private void OnDestroy()
    {
        cpp.Destroy();
    }

    // Update is called once per frame
    void FixedUpdate()
    {

        //cpp.Update();
        //cpp.IncreaseCounter();

        //Debug.Log("Counter = " + cpp.GetCounter());
        //Debug.Log("Thread Counter = " + cpp.GetThreadCounter());
    }

    private void Update()
    {
        if (mode == Mode.SimulateSingleThread && Time.time > lastTime + timeStep)
        {
            cpp.Update();
            lastTime = Time.time;
        }
    }
}

public enum Integration
{
    Explicit = 0,
    Symplectic = 1,
    Implicit = 2,
};
