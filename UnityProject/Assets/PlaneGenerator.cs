using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlaneGenerator : MonoBehaviour
{
    [SerializeField]
    protected int divisions = 10;

    // Start is called before the first frame update
    void Start()
    {

    }

    public void BuildMesh()
    {
        //Mesh info
        int verticesPerFace = divisions + 2;

        int verticesPerWidth, verticesPerHeight;
        if (transform.localScale.x > transform.localScale.y)
        {
            verticesPerWidth = verticesPerFace;
            verticesPerHeight = Mathf.Max(3, Mathf.RoundToInt(verticesPerFace * (transform.localScale.y / transform.localScale.x)));
        }
        else
        {
            verticesPerWidth = Mathf.Max(3, Mathf.RoundToInt(verticesPerFace * (transform.localScale.x / transform.localScale.y)));
            verticesPerHeight = verticesPerFace;
        }

        print($"Builidng {verticesPerWidth} x {verticesPerHeight} plane");

        float localWidth = 1;
        float localHeight = 1;

        Vector3[] vertices = new Vector3[verticesPerWidth * verticesPerHeight];
        Vector2[] uv = new Vector2[verticesPerWidth * verticesPerHeight];

        //Vertices
        for (int i = 0; i < verticesPerWidth; i++)
        {
            for (int j = 0; j < verticesPerHeight; j++)
            {
                float x = -(localWidth / 2f) + i * (localWidth / (verticesPerWidth - 1));
                float z = -(localHeight / 2f) + j * (localHeight / (verticesPerHeight - 1));

                //Nodes
                Vector3 pos = new Vector3(x, 0, z);//new Vector3(x, 0, y);//Horizontal plane
                vertices[(verticesPerWidth * j) + i] = pos;

                //UVs
                float u = (x + (localWidth / 2f)) / localWidth;
                float v = (z + (localHeight / 2f)) / localHeight;
                uv[(verticesPerWidth * j) + i] = new Vector2(u, v);
            }
        }


        //Triangles
        List<int> t = new List<int>();
        for (int i = 0; i < verticesPerWidth - 1; i++)
        {
            for (int j = 0; j < verticesPerHeight - 1; j++)
            {
                //Clockwise
                int a = (verticesPerWidth * j) + i;
                int b = (verticesPerWidth * (j + 1)) + i;
                int c = (verticesPerWidth * (j + 1)) + i + 1;
                int d = (verticesPerWidth * j) + i + 1;

                t.Add(a);
                t.Add(b);
                t.Add(c);

                t.Add(a);
                t.Add(c);
                t.Add(d);
            }
        }

        Mesh mesh = new Mesh();
        mesh.name = "Plane " + verticesPerWidth + "x" + verticesPerHeight;

        mesh.vertices = vertices;
        mesh.uv = uv;
        mesh.triangles = t.ToArray();
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        mesh.RecalculateTangents();

        GetComponent<MeshFilter>().mesh = mesh;
    }
}
