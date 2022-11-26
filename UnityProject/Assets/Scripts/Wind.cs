using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Wind : MonoBehaviour
{
    [SerializeField]
    float speed = 1.0f;
    public Vector3 windVelocity
    {
        get { return transform.forward * speed; }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
