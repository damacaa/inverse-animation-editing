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
        string path = (Application.dataPath).ToString() +"/scene.txt";
        string path_f = (Application.dataPath).ToString() +"/Files/"+ SimulationManager.GetName();

        Debug.Log(path + " has been updated");
        File.WriteAllText(path, SimulationManager.SceneToJsonInEditor());
        File.WriteAllText(path_f, SimulationManager.SceneToJsonInEditor());
    }

    [MenuItem("Optimizer/Optimize")]
    public static void Optimize()
    {
        string strCmdText;
        strCmdText = "/c python d:/Projects/MassSpringSimulator/Python/OptimizerForUnity.py & pause";   //This command to open a new notepad
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
