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
        /*foreach (SimulationObject so in simulationObjects)
        {
            Mesh mesh = so.GetComponent<MeshFilter>().mesh;

            Vector3[] vertices = mesh.vertices;

            for (int i = 0; i < vertices.Length; i++)
            {
                vertices[i] = so.transform.TransformPoint(vertices[i]);
            }

            int id = SimulationManager.Instance.AddObject(vertices, mesh.triangles, so.stiffness, so.mass);
            so.id = id;
            so.name = "Id: " + id;
        }*/

        foreach (SimulationObject so in simulationObjects)
        {
            Vector3[] vertPos = so.data.vertPos;
            float[] vertVolume = so.data.vertVolume;
            int[] springs = so.data.springs;
            float[] springStiffness = so.data.springStiffness;
            float[] springVolume = so.data.springVolume;
            float density = so.data.density;
            float damping = so.data.damping;

            int id = SimulationManager.Instance.AddObject(vertPos, vertVolume, springs, springStiffness, springVolume, density, damping);
            so.id = id;
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

    /*public int AddObject(Vector3[] vertices, int[] triangles, float stiffness, float mass)
    {
                return cpp.AddObject(vertices, triangles, stiffness, mass);
    }*/

    public int AddObject(Vector3[] vertPos, float[] vertVolume, int[] springs, float[] springStiffness, float[] springVolume,
    float density, float damping)
    {
        return cpp.AddObject(vertPos, vertVolume, springs, springStiffness, springVolume, density, damping);
    }

    public void AddFixer(Vector3 position, Vector3 scale)
    {
        cpp.AddFixer(position, scale);
    }

    public Vector3[] GetVertices(int id)
    {
        Vector3[] vertices = cpp.GetVertices(id);


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

        if (mode == Mode.SimulateSingleThread)
            cpp.Update();

        //cpp.Update();
        //cpp.IncreaseCounter();

        //Debug.Log("Counter = " + cpp.GetCounter());
        //Debug.Log("Thread Counter = " + cpp.GetThreadCounter());
    }

    /*private void Update()
    {
        if (mode == Mode.SimulateSingleThread && Time.time > lastTime + timeStep)
        {
            cpp.Update();
            lastTime = Time.time;
        }
    }*/
}

public enum Integration
{
    Explicit = 0,
    Symplectic = 1,
    Implicit = 2,
};
