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

    [SerializeField]
    Mode mode = Mode.SimulateMultiThread;

    [Header("Simulation parameters")]
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
    bool useFile = false;
    [SerializeField]
    TextAsset sceneInfo;

    [Header("Scene")]
    public List<SimulationObject> simulationObjects = new List<SimulationObject>();
    public List<MSSCollider> colliders = new List<MSSCollider>();

    ICPPWrapper cpp;
    private void Awake()
    {
        instance = this;
        //cpp = new ICPPWrapper(integrationMethod, timeStep, tolerance);
    }

    private void Start()
    {
        if (sceneInfo == null || !useFile)
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
                colliders_[i] = colliders[i].Data;
                colliders[i].id = i;
            }

            string json = SceneToJson(objects, colliders_);
            //File.WriteAllText(Application.dataPath+"\\scene.txt", json);
            cpp = new ICPPWrapper(json);
        }
        else
        {
            //Read scene file
            SimulationInfo si = JsonUtility.FromJson<SimulationInfo>(sceneInfo.text);

            for (int i = 0; i < simulationObjects.Count; i++)
            {
                simulationObjects[i].Data = si.objects[i];
                simulationObjects[i].id = i;
            }

            cpp = new ICPPWrapper(sceneInfo.text);
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
            default:
                break;
        }
    }

    private string SceneToJson(SimulationObjectData[] objects, ColliderData[] colliders_)
    {
        SimulationInfo info = new SimulationInfo();
        info.delta = timeStep;
        info.integrationMethod = (int)integrationMethod;
        info.tolerance = tolerance;
        info.optimizationIterations = iterations;

        info.objects = objects;
        info.colliders = colliders_;

        return JsonUtility.ToJson(info);
    }

    public string SceneToJson(SimulationObjectData[] objects)
    {
        SimulationInfo info = new SimulationInfo();
        info.delta = timeStep;
        info.integrationMethod = (int)integrationMethod;
        info.tolerance = tolerance;
        info.optimizationIterations = iterations;

        info.objects = objects;

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
            manager.colliders[i].id = i;
        }

        string json = manager.SceneToJson(objects, colliders_);
        return json;
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
    }
}

