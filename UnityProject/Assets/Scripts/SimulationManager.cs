using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Runtime.InteropServices;
using Random = UnityEngine.Random;
using System.IO;

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



    public enum Mode
    {
        None,
        SimulateSingleThread,
        SimulateMultiThread
    }


    [Header("Simulation parameters")]
    [SerializeField]
    Mode mode = Mode.SimulateMultiThread;
    [SerializeField]
    Integration integrationMethod = Integration.Implicit;
    [SerializeField]
    float timeStep = 0.01f;
    [SerializeField]
    float tolerance = 1e-2f;


    [Header("Optimization parameters")]
    [SerializeField]
    int iterations = 100;
    [SerializeField]
    int forwardSubSteps = 3;
    [SerializeField]
    bool useFile = false;
    [SerializeField]
    bool printTimes = false;
    [SerializeField]
    TextAsset sceneInfo;

    [Header("Scene")]
    [SerializeField]
    public string title;

    public List<SimulationObject> simulationObjects = new List<SimulationObject>();
    public List<MSSCollider> colliders = new List<MSSCollider>();
    [SerializeField]
    Wind wind;

    ICPPWrapper cpp;

    public bool Waiting { get; private set; }
    private void Awake()
    {
        instance = this;
        //cpp = new ICPPWrapper(integrationMethod, timeStep, tolerance);
    }

    IEnumerator Start()
    {
        //Collect information from scene
        SimulationObjectData[] objects = new SimulationObjectData[simulationObjects.Count];
        for (int i = 0; i < objects.Length; i++)
        {
            objects[i] = simulationObjects[i].Data;
            simulationObjects[i].id = i;
        }

        ColliderData[] colliders_ = new ColliderData[colliders.Count];
        for (int i = 0; i < colliders_.Length; i++)
        {
            if (!colliders[i].gameObject.activeInHierarchy)
                continue;

            colliders_[i] = colliders[i].Data;
            colliders[i].id = i + objects.Length;
        }


        if (sceneInfo != null && useFile)
        {
            //Read scene file and convert to object
            SimulationInfo si = JsonUtility.FromJson<SimulationInfo>(sceneInfo.text);

            //Replace node mass and spring stiffness
            for (int i = 0; i < si.objects.Length; i++)
            {
                objects[i].vertMass = si.objects[i].vertMass;
                objects[i].springStiffness = si.objects[i].springStiffness;
            }
        }

        string json = SceneToJson(objects, colliders_);
        cpp = new ICPPWrapper(json);

        Waiting = true;
        while (!Input.GetKey(KeyCode.Space))
        {
            yield return null;
        }
        Waiting = false;

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
            default:
                break;
        }

        yield return null;
    }

    private string SceneToJson(SimulationObjectData[] objects, ColliderData[] colliders_)
    {
        SimulationInfo info = new SimulationInfo();
        info.delta = timeStep;
        info.integrationMethod = (int)integrationMethod;
        info.tolerance = tolerance;
        info.optimizationIterations = iterations;
        info.forwardSubSteps = forwardSubSteps;
        info.printTimes = printTimes;

        info.objects = objects;
        info.colliders = colliders_;

        info.title = title;

        if (wind)
            info.windVel = wind.windVelocity;
        else
            info.windVel = Vector3.zero;

        return JsonUtility.ToJson(info);
    }

    public static string SceneToJsonInEditor()
    {
        SimulationManager manager = FindObjectOfType<SimulationManager>();

        SimulationObjectData[] objects = new SimulationObjectData[manager.simulationObjects.Count];
        for (int i = 0; i < objects.Length; i++)
        {
            objects[i] = manager.simulationObjects[i].GetDataInEditor();
        }

        ColliderData[] colliders_ = new ColliderData[manager.colliders.Count];
        for (int i = 0; i < colliders_.Length; i++)
        {
            colliders_[i] = manager.colliders[i].Data;
        }

        string json = manager.SceneToJson(objects, colliders_);
        return json;
    }

    public static string GetName()
    {
        SimulationManager manager = FindObjectOfType<SimulationManager>();
        return manager.title + ".txt";
    }

    /*public int AddObject(Vector3[] vertices, int[] triangles, float stiffness, float mass)
    {
                return cpp.AddObject(vertices, triangles, stiffness, mass);
    }*/

    public int AddObject(Vector3[] vertPos, float vertMass, int[] springs, float[] springStiffness, float damping)
    {
        return cpp.AddObject(vertPos, vertMass, springs, springStiffness, damping);
    }

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
    public enum Integration
    {
        Explicit = 0,
        Symplectic = 1,
        Implicit = 2,
    };

    [System.Serializable]
    public class SimulationInfo
    {
        public int integrationMethod;
        public float delta;
        public float tolerance;
        public int optimizationIterations;
        public SimulationObjectData[] objects;
        public ColliderData[] colliders;
        public Vector3 windVel;
        public bool printTimes;
        public int forwardSubSteps;
        public string title;
    }
}

