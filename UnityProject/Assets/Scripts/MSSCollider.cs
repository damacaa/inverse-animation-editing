using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MSSCollider : MonoBehaviour
{
    public ColliderData Data
    {
        get
        {
            ColliderData cd = new ColliderData();
            cd.type = 1;//0 plane //1 sphere
            cd.pos = transform.position;
            cd.rot = transform.rotation.eulerAngles;
            cd.scale = transform.localScale + new Vector3(thickness, thickness, thickness);
            return cd;
        }
    }
    public int id;
    public float thickness = 0;

}

[System.Serializable]
public class ColliderData
{
    public int type;
    public Vector3 pos;
    public Vector3 rot;
    public Vector3 scale;
}