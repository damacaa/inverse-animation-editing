using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ScreenshotManager : MonoBehaviour
{
    [SerializeField]
    int frameRate = 10;
    private Coroutine movieCoroutine;

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.S))
        {
            print("Screenshot");
            string file = System.DateTime.Now.ToString("yyyyMMddTHHmmss");
            TakeScreenshot(file, "C:/Screenshots/");
        }

        if (Input.GetKeyDown(KeyCode.M))
        {
            if (movieCoroutine == null)
            {
                print("Started recording");
                string file = System.DateTime.Now.ToString("yyyyMMddTHHmmss");
                movieCoroutine = StartCoroutine(RecordMultipleFrames(file, frameRate));
            }
            else
            {
                print("Stopped recording");
                StopCoroutine(movieCoroutine);
                movieCoroutine = null;
            }
        }
    }

    public void TakeScreenshot(string name, string path)
    {
        ScreenCapture.CaptureScreenshot(path + name + ".png");
    }

    IEnumerator RecordMultipleFrames(string name, int frameRate)
    {
        int count = 0;

        float nextTime = 0;
        float delay = 1f / frameRate;

        while (Application.isPlaying)
        {
            if (Time.time > nextTime)
            {
                nextTime = Time.time + delay;
                TakeScreenshot(name + "-" + count, "C:/Screenshots/");
                count++;
            }
            yield return null;
        }

        yield return null;
    }
}
