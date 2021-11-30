using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Fixer : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        SimulationManager.instance.AddFixer(transform.position, transform.localScale);
    }

    // Update is called once per frame
    void Update()
    {

    }
}
