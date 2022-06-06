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

        Debug.Log(path + " has been updated");
        File.WriteAllText(path, SimulationManager.SceneToJsonInEditor());
    }

    [MenuItem("Optimizer/Optimize")]
    public static void Optimize()
    {
        string strCmdText;
        strCmdText = "/C python d:/Projects/MassSpringSimulator/Python/hello.py";   //This command to open a new notepad
        System.Diagnostics.Process.Start("CMD.exe", strCmdText); //Start cmd process
    }
}
