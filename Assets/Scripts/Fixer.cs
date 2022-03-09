using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Fixer : MonoBehaviour
{
    [SerializeField]
    bool hideMesh = false;
    // Start is called before the first frame update
    void Start()
    {
        SimulationManager.instance.AddFixer(transform.position, transform.localScale);
        if (hideMesh)
            GetComponent<MeshRenderer>().enabled = false;
    }

    // Update is called once per frame
    void Update()
    {

    }
}
