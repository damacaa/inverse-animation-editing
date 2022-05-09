using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Fixer : MonoBehaviour
{
    [SerializeField]
    bool hideMesh = false;
    Collider collider;
    
    // Start is called before the first frame update
    void Start()
    {
        //SimulationManager.instance.AddFixer(transform.position, transform.localScale);
        if (hideMesh)
            GetComponent<MeshRenderer>().enabled = false;
        collider = GetComponent<Collider>();
    }

    internal bool CheckVert(Vector3 vector3)
    {
        if(!collider)
            collider = GetComponent<Collider>();

        return collider.bounds.Contains(vector3);
    }
}
