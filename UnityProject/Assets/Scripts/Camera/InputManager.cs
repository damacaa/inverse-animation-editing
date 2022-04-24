using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class InputManager : MonoBehaviour
{
    public static InputManager instance;
    public bool choosingWhereToBuild = false; //A structure card has been selected
    bool zooming = false;//Is zooming
    bool isMobile = false;
    public bool forceMobile = false;
    public Vector3 offset;
    public Vector3 mousePosition;
    Vector3 lastMousePosition;
    public Color wrongColor;

    [SerializeField]
    private float mouseSensitivity = 3.0f;
    [SerializeField]
    private float scrollSensitivity = 15.0f;
    [SerializeField]
    private float pinchSensitivity = 15.0f;
    //Gameobject that will be placed where structure is about to be built
    public GameObject cursor;
    private GameObject cursorBase;



    private void Awake()
    {
        instance = this;
    }
    void Start()
    {
        isMobile = forceMobile;

        //If no cursor is assigned, a cube will be created and used
        if (cursor == null)
        {
            cursor = GameObject.CreatePrimitive(PrimitiveType.Cube);
            Destroy(cursor.GetComponent<Collider>());
            cursor.SetActive(false);
        }
        else
        {
            cursor = GameObject.Instantiate(cursor);
            cursor.SetActive(false);
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.C))
        {
            CameraBehaviour.instance.Reset();
        }

        CameraBehaviour.instance.Rotate(-Input.GetAxis("Horizontal") * Time.deltaTime, -Input.GetAxis("Vertical") * Time.deltaTime);

        lastMousePosition = mousePosition;
        if (isMobile && Input.touchCount > 0)
        {
            mousePosition = Input.touches[0].position;
            CheckPinch();
        }
        else
        {
            mousePosition = Input.mousePosition;
            //Zoom
            CameraBehaviour.instance.Zoom(Input.mouseScrollDelta.y); //Zoom with mouse wheel
        }

        //Click
        if (Input.GetMouseButtonDown(0))
        {
            MouseDown();
        }

        //Click release
        if (Input.GetMouseButtonUp(0) && !isMobile)
        {
            MouseUp();
        }

        //Drag
        if (Input.GetMouseButton(0))
        {
            MouseDrag();
        }
    }

    private void MouseDrag()
    {
        if (choosingWhereToBuild)
        {
            //Casts a ray to find out where does the player want to place the structure
            Ray ray;
            if (isMobile)
                ray = Camera.main.ScreenPointToRay(mousePosition + offset);
            else
                ray = Camera.main.ScreenPointToRay(mousePosition);
            RaycastHit hit = new RaycastHit();

            if (Physics.Raycast(ray, out hit, Mathf.Infinity))
            {
                if (true)
                {
                    cursor.GetComponent<MeshRenderer>().material.color = Color.white;


                    //Cursor activates and moves to position
                    cursor.SetActive(true);

                    cursor.transform.localScale = Vector3.one;
                    cursor.transform.position = hit.point;

                    cursor.transform.up = hit.normal;
                }
                else
                {
                    cursor.GetComponent<MeshRenderer>().material.color = wrongColor;
                }
            }
            else
            {
                //If mouse isn't over the world, cursor is hidden and card is shown again
                cursor.SetActive(false);
            }
        }
        else if (!zooming && Input.mousePosition.x <= Screen.width * 0.9f)
        {
            //If not zooming, camera will be moved
            if (Mathf.Abs(Input.GetAxis("Mouse X")) > 30 || Mathf.Abs(Input.GetAxis("Mouse Y")) > 30)
                return;

            CameraBehaviour.instance.Rotate(Input.GetAxis("Mouse X"), Input.GetAxis("Mouse Y"));
        }
    }

    private void MouseDown()
    {
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        RaycastHit hit = new RaycastHit();

        if (Physics.Raycast(ray, out hit, Mathf.Infinity))
        {
            switch (hit.collider.tag)
            {
                default:
                    break;
            }
        }
    }

    public void MouseUp()
    {
        Ray ray;
        if (isMobile)
            ray = Camera.main.ScreenPointToRay(mousePosition + offset);
        else
            ray = Camera.main.ScreenPointToRay(mousePosition);
        RaycastHit hit = new RaycastHit();

        if (Physics.Raycast(ray, out hit, Mathf.Infinity))
        {
            switch (hit.collider.tag)
            {
                default:
                    break;
            }
        }

        cursor.SetActive(false);
    }

    bool CheckPinch()
    {
        if (choosingWhereToBuild)
            return false;

        int activeTouches = Input.touchCount;

        if (activeTouches < 2)//If less than two touches, can't zoom
        {
            zooming = false;
            return false;
        }

        zooming = true;

        Vector2 touch0 = Input.GetTouch(0).position;
        Vector2 touch1 = Input.GetTouch(1).position;
        //Deltas are the position change since las frame
        Vector2 delta0 = Input.GetTouch(0).deltaPosition;
        Vector2 delta1 = Input.GetTouch(1).deltaPosition;

        if (Vector2.Dot(delta0, delta1) < 0)//If deltas form an angle greater than 90 degrees
        {
            float currentDist = Vector2.Distance(touch0, touch1);
            float previousDist = Vector2.Distance(touch0 - delta0, touch1 - delta1);
            float difference = previousDist - currentDist;

            if (difference < 50)
            {
                CameraBehaviour.instance.Zoom((difference) * Time.deltaTime * pinchSensitivity);
            }
        }

        return true;
    }
}
//https://answers.unity.com/questions/1698508/detect-mobile-client-in-webgl.html?childToView=1698985#answer-1698985
