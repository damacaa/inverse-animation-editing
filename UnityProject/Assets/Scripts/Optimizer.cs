using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;

public class Optimizer
{
    [MenuItem("Optimizer/Generate scene file")]
    public static void GenerateSceneFile()
    {
        string path = (Application.dataPath).ToString() + "/scene.txt";
        string path_f = (Application.dataPath).ToString() + "/Files/" + SimulationManager.GetName();

        Debug.Log(path + " has been updated");
        File.WriteAllText(path, SimulationManager.SceneToJsonInEditor());
        File.WriteAllText(path_f, SimulationManager.SceneToJsonInEditor());
    }

    [MenuItem("Optimizer/Optimize")]
    public static void Optimize()
    {
        string strCmdText;
        string root = Directory.GetParent(Directory.GetParent(Application.dataPath).ToString()).ToString();
        // C:/Users/danie/AppData/Local/Programs/Python/Python310/python.exe
        strCmdText = $"/c cd ../Python & python {root}/Python/OptimizerForUnity.py & pause";// Path to Python script
        System.Diagnostics.Process.Start("CMD.exe", strCmdText); //Start cmd process
    }


    [MenuItem("Optimizer/Generate and optimize")]
    public static void GenerateAndOptimize()
    {
        GenerateSceneFile();
        Optimize();
    }

    [MenuItem("Optimizer/Update package")]
    public static void UpdatePackage()
    {
        string strCmdText;
        strCmdText = "/k d: & cd \"D:\\Projects\\MassSpringSimulator\\Python\" & python -m pip install \"d:/ Projects / MassSpringSimulator / UnityDLL\"";   //This command to open a new notepad
        System.Diagnostics.Process.Start("CMD.exe", strCmdText); //Start cmd process
    }
}
