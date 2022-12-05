#region Libraries
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;
using System;
using System.Text;
using System.Threading;
//using System.Threading.Tasks;
using UnityEngine.Networking;
using System.Linq;
using System.Text.RegularExpressions;
using System.Reflection;
using System.Collections.Specialized;
using Verruckt.Operations;
using Verruckt.Editor;
using System.Drawing;
using System.IO.Ports;
//using System.Windows;
using Microsoft.Win32;
#endregion

#region Main Class
[ExecuteInEditMode]
[CustomEditor(typeof(ScriptableObject))]
[CanEditMultipleObjects]
public class laserPathSystem : EditorWindow
{
    #region Variables

    #region File Variables
    string path = "";
    string[] files, dirs;
    List<string> fs;
    int fSel = -1, dSel = -1;
    bool fileSelected = false;
    #endregion

    #region Processing Variables
    float xSMM = 51.168f, ySMM = 50.955f;
    int xSTot = 0, ySTot = 0;
    int xSP = 0, ySP = 0;
    int wPx = 0, hPx = 0;
    Texture2D tx;
    #endregion

    #region Transmitting Data Variables
    string cPort = "";
    int baudRate = 74880;
    SerialPort port;
    string inb = "";
    bool IO = false;
    int initSteps = 0;
    int sendSteps = 0;
    int sendLim = 0;
    bool sendData = false;
    bool sending = false;
    bool confirmed = false;
    bool deadPort = false;
    List<string> inbound, outbound;
    string ibDisplay, obDisplay;
    int ySent = 0;
    int offX = 0;
    int offY = 0;
    string lU = "";
    byte[] bs;
    bool st = false;
    int maxD = 0;
    int minD = 1000;
    #endregion

    List<string> l;
    Vector2 sP;
    bool drawDone = false;
    List<string> cmdCumulation;
    
    #region Path Cycle Send Variables
    bool sendPaths = false;
    bool portChosen = false;
    #endregion

    #region Visualization Variables
    bool visualization = false;
    bool visualizeAsImage = false;
    Texture2D cameraImage;
    int imageFormat = 0;
    bool formatChanged = false;
    long vizW, vizH;
    float xMax, yMax, xPart, yPart, wPart, hPart;
    List<List<Vector2>> drawPaths;
    List<Vector2> drawRoutine;
    List<Vector2> fillInitPoints;
    List<Vector2> fillDimensions;
    Texture2D pathViz;
    string route;
    string dataOut;
    UnityEngine.Color[] pxs;
    #endregion

    #region PostProcess Variables
    bool postProcess = false;
    bool cutProcess = false;
    List<string> process1;
    List<string> fillSeq = new List<string>();
    List<string> frontSeg = new List<string>();
    List<string> backSeg = new List<string>();
    List<string> viaSeq = new List<string>();
    List<string> kC = new List<string>();
    #endregion
    
    #region Pick and Place Control Variables
    bool pnpRun = false, pnpPort = false;
    bool xMove = false, xDir = false, xDone = false;
    bool yMove = false, yDir = false, yDone = false;
    bool zMove = false, zDir = false, zDone = false;
    bool axisSlow = false;
    bool vacSend = false, vacDone = false;
    bool tblSend = false, tblDone = false;
    int tblState = 0, tblIncr = 0;
    #endregion
    

    #region Wristband Data Analysis Variables
    bool viaBluetooth = false;
    string msgIn = "";
    #endregion

    #region Quaternion Visualizer
    bool visualizeQuat = false;
    GameObject vizObj;
    Transform vizTrans;
    bool doVisualize = false;
    List<int> tTracking;
    float tAVG = 0;
    System.Net.Sockets.NetworkStream btStream;
    Vector3 wRot, wAcc, wGyro, wMag, wTouch;
    Vector3 ypr;
    Quaternion wQuat;
    float compassHeading = 0;
    ulong delT;
    Vector3 accStationarySetPoint, locPos, worldPos, worldZero, locZero;
    Vector3 velocities, displacements;
    #endregion

    #region Device Locator
    bool locateDevice = false;
    string vdPD = "";
    #endregion

    #region Vector Tools Variables
    bool vectorTools = false;
    bool vectorAutoReduction = false;
    List<Vector2> vectorsAdded;
    List<Vector2> vectorsProduced;
    List<string> cmdFormatted;
    bool reduceVectors;
    Vector2 reductionValues;
    #endregion

    #region Stencil Creation Variables
    List<bounds> padBounds;
    bool createStencil = false;
    Vector2 machSperMM;
    Vector2 curLinePos;
    List<List<string>> subIteratorLines;
    #endregion

    #region Bitmap Converter
    string codeOut = "";
    Texture2D bmp;
    int bW, bH;
    bool bitmapSelect = false, bitmapConvert = false;
    #endregion

    #region Raster Creation Variables
    bool createRaster = false;
    Vector2 cPos;
    Canvas cnv;
    GameObject canvasObj;
    GameObject parentObj;
    GameObject quadObj;
    GameObject circleObj;
    bool chooseObjs = false;
    List<GameObject> objs;
    bool rDir = false;
    #endregion

    #region Controller Variables
    float stepLength = .01f;
    bool useController = false;
    bool endController = false;
    bool continuousJog = false;
    bool laserOn = false;
    bool spindleOn = false;
    float xAxis = 0;
    float yAxis = 0;
    float zAxis = 0;
    float lAxis = 0;
    float sAxis = 0;
    float stpAxis = 0;
    bool homeBtn = false;
    bool laserBtn = false;
    bool spindleBtn = false;
    #endregion

    //BluetoothAddress UTAddress = new BluetoothAddress(c44f335ba37b);
    #endregion

    #region Scene GUI 
    void SceneGUI(SceneView sceneView)
    {
        // This will have scene events including mouse down on scenes objects
        Event cur = Event.current;

        if (pnpRun && pnpPort)
        {
            switch(cur.type)
            {
                case EventType.KeyDown:
                    {
                        if (Event.current.keyCode == KeyCode.UpArrow)
                        {
                            xMove = true;
                            xDir = false;
                            axisSlow = false;
                        }
                        else if (Event.current.keyCode == KeyCode.DownArrow)
                        {
                            xMove = true;
                            xDir = true;
                            axisSlow = false;
                        }
                        if (Event.current.keyCode == KeyCode.W)
                        {
                            xMove = true;
                            xDir = false;
                            axisSlow = true;
                        }
                        else if (Event.current.keyCode == KeyCode.S)
                        {
                            xMove = true;
                            xDir = true;
                            axisSlow = true;
                        }
                        else if (Event.current.keyCode == KeyCode.LeftArrow)
                        {
                            yMove = true;
                            yDir = true;
                            axisSlow = false;
                        }
                        else if (Event.current.keyCode == KeyCode.RightArrow)
                        {
                            yMove = true;
                            yDir = false;
                            axisSlow = false;
                        }
                        else if (Event.current.keyCode == KeyCode.A)
                        {
                            yMove = true;
                            yDir = true;
                            axisSlow = true;
                        }
                        else if (Event.current.keyCode == KeyCode.D)
                        {
                            yMove = true;
                            yDir = false;
                            axisSlow = true;
                        }
                        else if (Event.current.keyCode == KeyCode.N)
                        {
                            zMove = true;
                            zDir = false;
                            axisSlow = false;
                        }
                        else if (Event.current.keyCode == KeyCode.M)
                        {
                            zMove = true;
                            zDir = true;
                            axisSlow = false;
                        }
                        else if (Event.current.keyCode == KeyCode.J)
                        {
                            zMove = true;
                            zDir = false;
                            axisSlow = true;
                        }
                        else if (Event.current.keyCode == KeyCode.K)
                        {
                            zMove = true;
                            zDir = true;
                            axisSlow = true;
                        }
                        else if (Event.current.keyCode == KeyCode.Period)
                        {
                            tblIncr = 1;
                        }
                        else if (Event.current.keyCode == KeyCode.Comma)
                        {
                            tblIncr = -1;
                        }
                    }
                    break;


                case EventType.KeyUp:
                    {
                        if (Event.current.keyCode == KeyCode.UpArrow || Event.current.keyCode == KeyCode.DownArrow || Event.current.keyCode == KeyCode.W || Event.current.keyCode == KeyCode.S)
                        {
                            xMove = false;
                        }
                        else if (Event.current.keyCode == KeyCode.LeftArrow || Event.current.keyCode == KeyCode.RightArrow || Event.current.keyCode == KeyCode.A || Event.current.keyCode == KeyCode.D)
                        {
                            yMove = false;
                        }
                        else if (Event.current.keyCode == KeyCode.N || Event.current.keyCode == KeyCode.M || Event.current.keyCode == KeyCode.J || Event.current.keyCode == KeyCode.K)
                        {
                            zMove = false;
                        }
                        else if (Event.current.keyCode == KeyCode.Space)
                        {
                            vacSend = !vacSend;
                        }
                        else if (Event.current.keyCode == KeyCode.L)
                        {
                            tblSend = !tblSend;
                            tblState = -1;
                            tblIncr = 0;
                        }
                        else if (Event.current.keyCode == KeyCode.Semicolon || Event.current.keyCode == KeyCode.RightAlt)
                        {
                            tblSend = !tblSend;
                            tblState = 0;
                            tblIncr = 0;
                        }
                        else if (Event.current.keyCode == KeyCode.Quote)
                        {
                            tblSend = !tblSend;
                            tblState = 1;
                            tblIncr = 0;
                        }
                        else if (Event.current.keyCode == KeyCode.Period)
                        {
                            tblIncr = 0;
                        }
                        else if (Event.current.keyCode == KeyCode.Comma)
                        {
                            tblIncr = 0;
                        }
                    }
                    break;
            }
        }
    }
    #endregion

    #region On Enable
    private void OnEnable()
    {
        stepLength = .01f;
         xAxis = 0;
         yAxis = 0;
         zAxis = 0;
         lAxis = 0;
         sAxis = 0;
         stpAxis = 0;
         homeBtn = false;
         laserBtn = false;
         spindleBtn = false;
        laserOn = false;
        spindleOn = false;
        useController = false;
        endController = false;
        continuousJog = false;
        visualizeAsImage = false;
        cameraImage = null;
        imageFormat = 0;
        formatChanged = false;
        createRaster = false;
        bitmapSelect = false;
        bitmapConvert = false;
        chooseObjs = false;
        bW = 0;
        bH = 0;
        bmp = null;
        codeOut = "";
        SceneView.onSceneGUIDelegate += SceneGUI;
        accStationarySetPoint = new Vector3(.002f, .0005f, .9997f);
        createStencil = false;
        vectorTools = false;
        vectorAutoReduction = false;
        vectorsAdded = new List<Vector2>();
        vectorsProduced = new List<Vector2>();
        reduceVectors = false;
        reductionValues = Vector2.zero;
        drawDone = false;
        cmdCumulation = new List<string>();
        visualizeQuat = false;
        doVisualize = false;
        locateDevice = false;
        vdPD = "";
        viaBluetooth = false;
        pnpRun = false;
        pnpPort = false;
        xMove = false;
        xDir = false;
        yMove = false;
        yDir = false;
        zMove = false;
        zDir = false;
        xDone = false;
        yDone = false;
        zDone = false;
        axisSlow = false;
        vacSend = false;
        vacDone = false;
        cutProcess = false;
        process1 = new List<string>();
        List<string> mods = new List<string>();
        pathViz = null;
        postProcess = false;
        tx = null;
        sendPaths = false;
        confirmed = false;
        fileSelected = false;
        portChosen = false;
        path = "C:/Users/robzo/Desktop/";
        files = Directory.GetFiles(path);
        dirs = Directory.GetDirectories(path);
        for (int n = 0; n <= dirs.Length - 1; n++)
        {
            dirs[n] = dirs[n].Substring(path.Length);
        }

        string[] fls = Directory.GetFiles(path);
        List<string> lfs = new List<string>();
        for (int n = 0; n < fls.Length; n++)
        {
            if (fls[n].Contains(".kicad_pcb"))
            {
                lfs.Add(fls[n]);
            }
        }
        files = lfs.ToArray();
        for (int n = 0; n <= files.Length - 1; n++)
        {
            files[n] = files[n].Substring(path.Length);
        }


        xSMM = 51.168f;
        ySMM = 50.955f;
        VrrktCoroutine.totalWipe();
        inb = "";
        IO = false;
        sendSteps = 0;
        sendLim = 0;
        sendData = false;
        sending = false;
        deadPort = false;
        sendData = false;
        sending = false;

        wPx = 0;
        hPx = 0;
        fSel = -1;
        dSel = -1;
        sP = Vector2.zero;
        tx = null;
        if (port != null)
        {
            if (port.IsOpen)
            {
                port.Close();
                port = null;
            }
        }
        offX = 0;
        offY = 0;
        l = new List<string>();
        Repaint();
    }
    #endregion

    #region INIT
    [MenuItem("Window/Custom/Laser Path System")]
    static void Init()
    {
        Debug.Log("Init");
        laserPathSystem vR = (laserPathSystem)GetWindow(typeof(laserPathSystem));
        vR.maxSize = new Vector2(Screen.width, Screen.height);
        EditorUtility.SetDirty(vR);
        vR.Show();
    }
    #endregion

    #region GUI
    private void OnGUI()
    {
        #region Safeguard Restart Button
        if (GUILayout.Button("PANIC", GUILayout.MinWidth(Screen.width - 2), GUILayout.Height(20)))
        {
            if (port.IsOpen)
            {
                port.Close();
                OnEnable();
            }
        }
        #endregion

        EditorGUILayout.BeginVertical();
        GUI.skin.box.stretchWidth = true;
        GUI.skin.button.stretchWidth = true;
        GUI.skin.textField.stretchWidth = true;

        #region Mode Selection
        if (!sendPaths && !postProcess && !cutProcess && !pnpRun && !viaBluetooth && !locateDevice && !visualizeQuat && !vectorTools && !createStencil && !bitmapSelect && !createRaster && !useController)
        {
            #region Send Paths
            if (GUILayout.Button("Send Path Routine"))
            {
                sendPaths = true;
                fileSelected = false;
                postProcess = false;
                confirmed = false;
                portChosen = false;
                cutProcess = false;
                fSel = -1;
                pnpRun = false;
            }
            #endregion
            
            #region PostProcess Kicad Board File
            if (GUILayout.Button("PostProcess Kicad File"))
            {
                postProcess = true;
                sendPaths = false;
                fileSelected = false;
                confirmed = false;
                portChosen = false;
                cutProcess = false;
                pnpRun = false;
            }
            #endregion
            
            #region Pick And Place
            if (GUILayout.Button("Pick And Place Control"))
            {
                sendPaths = false;
                postProcess = false;
                fileSelected = false;
                pnpRun = true;
                xMove = false;
                yMove = false;
                zMove = false;
                pnpPort = false;
            }
            #endregion
            
            #region Wristband Data Visualization
            if (GUILayout.Button("Wristband Bluetooth Data Visualizer"))
            {
                viaBluetooth = true;
                sendPaths = false;
                postProcess = false;
                fileSelected = false;
                pnpRun = false;
                xMove = false;
                yMove = false;
                zMove = false;
                pnpPort = false;
            }
            #endregion

            #region Locate Device
            if (GUILayout.Button("Locate Device"))
            {
                locateDevice = true;
            }
            #endregion

            #region Visualize Quaternion
            if (GUILayout.Button("Visualize Quaternion"))
            {
                visualizeQuat = true;
                doVisualize = false;
            }
            #endregion

            #region Vector Tools
            if (GUILayout.Button("Vector Tools"))
            {
                vectorTools = true;
                vectorAutoReduction = false;
            }
            #endregion

            #region Create Stencil
            if (GUILayout.Button("Create Stencil Routine"))
            {
                createStencil = true;
                fileSelected = false;
                confirmed = false;
            }
            #endregion

            #region Convert Bitmap
            if (GUILayout.Button("Bitmap Converter"))
            {
                bitmapSelect = true;
                bitmapConvert = false;
                bmp = null;
                bW = 0;
                bH = 0;
            }
            #endregion

            #region Random Chart Generation
            if (GUILayout.Button("Create Random Chart"))
            {
                VrrktCoroutine.start(createRandomChart());
            }
            #endregion

            #region Create Raster
            if (GUILayout.Button("Create Raster Routine"))
            {
                createRaster = true;
                sendPaths = false;
                fileSelected = false;
                postProcess = false;
                confirmed = false;
                portChosen = false;
                cutProcess = false;
                fSel = -1;
                pnpRun = false;
            }
            #endregion

            #region Use Controller
            if (GUILayout.Button("Use Controller For Machine"))
            {
                useController = true;
                sendPaths = false;
                portChosen = false;
            }
            #endregion
        }
        #endregion

        #region Send Routine
        if (sendPaths)
        {
            #region Select Routine
            if (!fileSelected)
            {
                sP = GUILayout.BeginScrollView(sP);

                #region Path
                GUILayout.BeginHorizontal();

                GUILayout.Label("Current Path:");
                GUILayout.Label(path);

                GUILayout.EndHorizontal();
                #endregion

                GUILayout.Space(20);

                #region Directories

                if (GUILayout.Button("Go Up"))
                {
                    path = path.Substring(0, path.LastIndexOf("/"));
                    path = path.Substring(0, path.LastIndexOf("/") + 1);

                    dirs = Directory.GetDirectories(path);
                    for (int n = 0; n <= dirs.Length - 1; n++)
                    {
                        dirs[n] = dirs[n].Substring(path.Length);
                    }

                    if (fSel != -1)
                    {
                        fSel = -1;
                    }

                    string[] fls = Directory.GetFiles(path);
                    List<string> lfs = new List<string>();
                    for(int n = 0; n < fls.Length; n++)
                    {
                        if (fls[n].Contains(".txt"))
                        {
                            lfs.Add(fls[n]);
                        }
                    }
                    files = lfs.ToArray();
                    for (int n = 0; n <= files.Length - 1; n++)
                    {
                        files[n] = files[n].Substring(path.Length);
                    }
                }
                dSel = GUILayout.SelectionGrid(dSel, dirs, 4, GUILayout.Width(Screen.width - 10));

                if (dSel != -1)
                {
                    path += dirs[dSel];
                    
                    if (path.LastIndexOf("/") < path.Length)
                    {
                        path += "/";
                    }
                    dirs = Directory.GetDirectories(path);
                    for (int n = 0; n <= dirs.Length - 1; n++)
                    {
                        dirs[n] = dirs[n].Substring(path.Length);
                    }

                    if (fSel != -1)
                    {
                        fSel = -1;
                    }

                    string[] fls = Directory.GetFiles(path);
                    List<string> lfs = new List<string>();
                    for (int n = 0; n < fls.Length; n++)
                    {
                        if (fls[n].Contains(".txt"))
                        {
                            lfs.Add(fls[n]);
                        }
                    }
                    files = lfs.ToArray();

                    for (int n = 0; n <= files.Length - 1; n++)
                    {
                        files[n] = files[n].Substring(path.Length);
                    }

                    dSel = -1;
                }
                #endregion

                GUILayout.Space(20);

                #region Files
                fSel = GUILayout.SelectionGrid(fSel, files, 2, GUILayout.Width(Screen.width - 10));
                #endregion

                GUILayout.Space(20);

                #region Controls

                #region Proceed
                if (GUILayout.Button("Proceed"))
                {
                    if (fSel != -1)
                    {
                        if (files[fSel].Contains(".vrrktPTH") || files[fSel].Contains(".txt"))
                        {
                            VrrktCoroutine.start(loadData());
                        }
                        else
                        {
                            Debug.Log("Incorrect file type chosen");
                            fSel = -1;
                        }
                    }
                }
                #endregion

                #region Go Back
                if (GUILayout.Button("Back"))
                {
                    OnEnable();
                }
                #endregion

                #endregion

                GUILayout.EndScrollView();
            }
            #endregion

            #region Confirm Selection
            if (fileSelected && !confirmed && !portChosen)
            {
                #region Routine Display
                GUILayout.BeginHorizontal();

                if (cmdCumulation != null)
                {
                    GUILayout.Box("Total Lines: " + cmdCumulation.Count);
                }

                GUILayout.EndHorizontal();
                #endregion
                
                #region Controls

                #region Confirm Button
                if (GUILayout.Button("Confirm"))
                {
                    //VrrktCoroutine.start(initialVisualization());
                    confirmed = true;
                }
                #endregion

                #region Back Button
                if (GUILayout.Button("Back"))
                {
                    OnEnable();
                }
                #endregion

                #endregion
            }
            #endregion

            #region Visualization
            if (visualization)
            {
                //GUILayout.Box(pathViz);
            }
            #endregion

            #region Select Port
            if (fileSelected && confirmed && !portChosen)
            {
                #region Port Input
                GUILayout.BeginHorizontal();

                GUILayout.Label("COM Port to connect to");
                cPort = GUILayout.TextField(cPort, GUILayout.Width(Screen.width - 10));

                GUILayout.EndHorizontal();
                #endregion

                #region Controls

                #region Proceed Button
                if (GUILayout.Button("Proceed"))
                {
                    if (cPort != "" && cPort.Contains("COM"))
                    {
                        port = new SerialPort(cPort, baudRate);

                        try
                        {
                            port.Open();
                        }
                        catch (Exception e)
                        {
                            Debug.Log("Error opening port");
                            Debug.Log(e.Message);
                            Debug.Log(e.StackTrace);
                            cPort = "";
                        }

                        if (cPort != "")
                        {
                            VrrktCoroutine.start(sendor());
                            confirmed = false;
                            sending = true;
                            portChosen = true;
                        }
                    }
                }
                #endregion

                #region Back Button
                if (GUILayout.Button("Back"))
                {
                    OnEnable();
                }
                #endregion

                #endregion
            }
            #endregion

            #region Sending Routine
            if (sending)
            {
                #region Inbound Display
                GUILayout.BeginHorizontal();

                GUILayout.Label("Inbound: ");
                
                GUILayout.Box(inb);

                GUILayout.EndHorizontal();
                #endregion

                #region Last Command
                GUILayout.BeginHorizontal();

                GUILayout.Label("Last Command Sent: ");

                if (sendSteps > 0)
                {
                    GUILayout.Box(cmdCumulation[sendSteps - 1]);
                }

                GUILayout.EndHorizontal();
                #endregion

                #region Progress Display

                #region Upper
                GUILayout.BeginHorizontal();

                GUILayout.Label("Total Lines: ");
                GUILayout.Box(cmdCumulation.Count + "");

                GUILayout.Space(20);

                GUILayout.Label("Lines Sent: ");
                GUILayout.Box((sendSteps + 1) + "");

                GUILayout.EndHorizontal();
                #endregion

                #region Percentage
                GUILayout.Label(((((float)(sendSteps + 1) / (float)cmdCumulation.Count) * 100) + "%"));
                #endregion

                #endregion

                #region Controls
                if (GUILayout.Button("Reset"))
                {
                    if (port != null)
                    {
                        if (port.IsOpen)
                        {
                            port.Close();
                        }
                    }

                    OnEnable();
                }
                #endregion
            }
            #endregion
        }
        #endregion
        
        #region PostProcess
        if (postProcess)
        {
            #region Select File
            if (!fileSelected)
            {
                sP = GUILayout.BeginScrollView(sP);

                #region Path
                GUILayout.BeginHorizontal();

                GUILayout.Label("Current Path:");
                GUILayout.Label(path);

                GUILayout.EndHorizontal();
                #endregion

                GUILayout.Space(20);

                #region Directories

                if (GUILayout.Button("Go Up"))
                {
                    path = path.Substring(0, path.LastIndexOf("/"));
                    path = path.Substring(0, path.LastIndexOf("/") + 1);

                    dirs = Directory.GetDirectories(path);
                    for (int n = 0; n <= dirs.Length - 1; n++)
                    {
                        dirs[n] = dirs[n].Substring(path.Length);
                    }

                    if (fSel != -1)
                    {
                        fSel = -1;
                    }

                    string[] fls = Directory.GetFiles(path);
                    List<string> lfs = new List<string>();
                    for (int n = 0; n < fls.Length; n++)
                    {
                        if (fls[n].Contains(".kicad_pcb"))
                        {
                            lfs.Add(fls[n]);
                        }
                    }
                    files = lfs.ToArray();
                    for (int n = 0; n <= files.Length - 1; n++)
                    {
                        files[n] = files[n].Substring(path.Length);
                    }
                }
                dSel = GUILayout.SelectionGrid(dSel, dirs, 4, GUILayout.Width(Screen.width - 10));

                if (dSel != -1)
                {
                    path += dirs[dSel];

                    if (path.LastIndexOf("/") < path.Length)
                    {
                        path += "/";
                    }
                    dirs = Directory.GetDirectories(path);
                    for (int n = 0; n <= dirs.Length - 1; n++)
                    {
                        dirs[n] = dirs[n].Substring(path.Length);
                    }

                    if (fSel != -1)
                    {
                        fSel = -1;
                    }

                    string[] fls = Directory.GetFiles(path);
                    List<string> lfs = new List<string>();
                    for (int n = 0; n < fls.Length; n++)
                    {
                        if (fls[n].Contains(".kicad_pcb"))
                        {
                            lfs.Add(fls[n]);
                        }
                    }
                    files = lfs.ToArray();
                    for (int n = 0; n <= files.Length - 1; n++)
                    {
                        files[n] = files[n].Substring(path.Length);
                    }

                    dSel = -1;
                }
                #endregion

                GUILayout.Space(20);

                #region Files
                fSel = GUILayout.SelectionGrid(fSel, files, 2, GUILayout.Width(Screen.width - 10));
                #endregion

                GUILayout.Space(20);

                #region Controls

                #region Proceed
                if (GUILayout.Button("Proceed"))
                {
                    if (fSel != -1)
                    {
                        if (files[fSel].Contains(".kicad_pcb"))
                        {
                            VrrktCoroutine.start(kicadProcessor());
                        }
                        else
                        {
                            Debug.Log("Incorrect file type chosen");
                            fSel = -1;
                        }
                    }
                }
                #endregion

                #region Go Back
                if (GUILayout.Button("Back"))
                {
                    OnEnable();
                }
                #endregion

                #endregion

                GUILayout.EndScrollView();
            }
            #endregion
        }
        #endregion
        
        #region Pick And Place Control
        if (pnpRun)
        {
            #region Port Selection
            if (!pnpPort)
            {
                #region Port Input
                GUILayout.BeginHorizontal();

                GUILayout.Label("COM Port to connect to");
                cPort = GUILayout.TextField(cPort, GUILayout.Width(Screen.width - 10));

                GUILayout.EndHorizontal();
                #endregion

                #region Controls

                #region Proceed Button
                if (GUILayout.Button("Proceed"))
                {
                    if (cPort != "" && cPort.Contains("COM"))
                    {
                        port = new SerialPort(cPort, baudRate);

                        try
                        {
                            port.Open();
                        }
                        catch (Exception e)
                        {
                            Debug.Log("Error opening port");
                            Debug.Log(e.Message);
                            Debug.Log(e.StackTrace);
                            cPort = "";
                        }

                        if (cPort != "")
                        {
                            pnpPort = true;
                            VrrktCoroutine.start(pickAndplace());
                        }
                    }
                }
                #endregion

                #region Back Button
                if (GUILayout.Button("Back"))
                {
                    OnEnable();
                }
                #endregion

                #endregion
            }
            #endregion

            #region Control
            else
            {
                #region X Region
                GUILayout.BeginHorizontal();

                GUILayout.Toggle(xMove, "X Moving");
                GUILayout.Toggle(xDir, "X Direction");

                GUILayout.EndHorizontal();
                #endregion

                #region Y Region
                GUILayout.BeginHorizontal();

                GUILayout.Toggle(yMove, "Y Moving");
                GUILayout.Toggle(yDir, "Y Direction");

                GUILayout.EndHorizontal();
                #endregion

                #region Z Region
                GUILayout.BeginHorizontal();

                GUILayout.Toggle(zMove, "Z Moving");
                GUILayout.Toggle(zDir, "Z Direction");

                GUILayout.EndHorizontal();
                #endregion

                #region Button
                if (GUILayout.Button("Back"))
                {
                    OnEnable();
                }
                #endregion
            }
            #endregion
        }
        #endregion
        
        #region Wristband Bluetooth Data Visualizer
        if (viaBluetooth)
        {
            #region Port Input
            if (!doVisualize)
            {
                cPort = GUILayout.TextField(cPort, GUILayout.MinWidth(Screen.width / 4));

                #region Controls
                if (GUILayout.Button("Visualize"))
                {
                    if (port != null && port.IsOpen)
                    {
                        port.Close();
                        port.Dispose();
                    }

                    if (cPort != "" && cPort.Contains("COM"))
                    {
                        port = new SerialPort(cPort);
                        port.Open();
                        VrrktCoroutine.start(wristbandVisualizer(false, false), true);
                        doVisualize = true;
                    }
                }

                if (GUILayout.Button("Visualize As Camera Feed"))
                {

                    if (port != null && port.IsOpen)
                    {
                        port.Close();
                        port.Dispose();
                    }

                    if (cPort != "" && cPort.Contains("COM"))
                    {
                        port = new SerialPort(cPort);
                        port.Open();
                        VrrktCoroutine.start(imageVisualizer(), true);
                        visualizeAsImage = true;
                        doVisualize = true;
                    }
                }

                if (GUILayout.Button("Visualize With Initial Command To Clear Calibration"))
                {
                    if (port != null && port.IsOpen)
                    {
                        port.Close();
                        port.Dispose();
                    }

                    if (cPort != "" && cPort.Contains("COM"))
                    {
                        port = new SerialPort(cPort);
                        port.Open();
                        VrrktCoroutine.start(wristbandVisualizer(true, true), true);
                        doVisualize = true;
                    }
                }

                if (GUILayout.Button("Visualize with Initial Perform Calibration"))
                {
                    if (port != null && port.IsOpen)
                    {
                        port.Close();
                        port.Dispose();
                    }

                    if (cPort != "" && cPort.Contains("COM"))
                    {
                        port = new SerialPort(cPort);
                        port.Open();
                        VrrktCoroutine.start(wristbandVisualizer(false, true), true);
                        doVisualize = true;
                    }
                }

                if (GUILayout.Button("Back"))
                {
                    OnEnable();
                }
                #endregion
            }
            #endregion

            #region Display Bluetooth Serial
            else
            {
                #region Image Visualization
                if (visualizeAsImage)
                {
                    if (cameraImage != null)
                    {
                        GUILayout.Box(cameraImage, GUILayout.Width(cameraImage.width), GUILayout.Height(cameraImage.height));
                    }
                }
                #endregion

                #region Image Formatting Controls
                if (visualizeAsImage)
                {
                    GUILayout.BeginHorizontal();

                    #region 0 - VGA
                    if (GUILayout.Button("VGA"))
                    {
                        imageFormat = 0;
                        formatChanged = true;
                    }
                    #endregion

                    #region 1 - SVGA
                    if (GUILayout.Button("SVGA"))
                    {
                        imageFormat = 1;
                        formatChanged = true;
                    }
                    #endregion

                    #region 2 - XGA
                    if (GUILayout.Button("XGA"))
                    {
                        imageFormat = 2;
                        formatChanged = true;
                    }
                    #endregion

                    #region 3 - SXGA
                    if (GUILayout.Button("SXGA"))
                    {
                        imageFormat = 3;
                        formatChanged = true;
                    }
                    #endregion

                    #region 4 - UXGA
                    if (GUILayout.Button("UXGA"))
                    {
                        imageFormat = 4;
                        formatChanged = true;
                    }
                    #endregion

                    GUILayout.EndHorizontal();
                }
                #endregion

                #region Text Input
                dataOut = GUILayout.TextField(dataOut, GUILayout.MinWidth(Screen.width * .5f));
                #endregion

                #region Controls
                if (GUILayout.Button("Send Command") || Input.GetKeyUp(KeyCode.Return))
                {
                    if (dataOut != "" && dataOut.IndexOf("/") != 0)
                    {
                        Debug.Log("Sent: " + dataOut);
                        visualizerSending(dataOut);
                        dataOut = "";
                    }
                }

                if (GUILayout.Button("Back"))
                {
                    OnEnable();
                }
                #endregion
            }
            #endregion
        }
        #endregion

        #region Device Locator
        if (locateDevice)
        {
            #region VIDPID Input
            vdPD = GUILayout.TextField(vdPD, GUILayout.MinWidth(Screen.width));

            #endregion

            #region Controls
            if (GUILayout.Button("Locate"))
            {
                deviceLocator(vdPD, 3);
            }

            if (GUILayout.Button("USB Digging"))
            {
                usbDeviceParser();
            }

            if (GUILayout.Button("Back"))
            {
                OnEnable();
            }
            #endregion
        }
        #endregion

        #region Visualize Quaternion
        if (visualizeQuat)
        {
            #region Assign Object
            if (!doVisualize)
            {
                vizObj = (GameObject)EditorGUILayout.ObjectField(vizObj, typeof(GameObject), GUILayout.MinWidth(Screen.width / 2));
                cPort = GUILayout.TextField(cPort, GUILayout.MinWidth(Screen.width / 4));

                #region Controls
                if (GUILayout.Button("Visualize"))
                {
                    vizObj.transform.rotation = Quaternion.identity;
                    if (port != null && port.IsOpen)
                    {
                        port.Close();
                        port.Dispose();
                    }

                    if (vizObj != null && cPort != "" &&  cPort.Contains("COM"))
                    {
                        vizTrans = vizObj.transform;
                        port = new SerialPort(cPort);
                        port.Open();
                        VrrktCoroutine.start(quatReceive(vizObj), true);
                        doVisualize = true;
                    }
                }

                if(GUILayout.Button("Back"))
                {
                    OnEnable();
                }
                #endregion
            }
            #endregion

            #region Visualize
            else
            {
                GUILayout.BeginVertical();
                //EditorGUILayout.Vector3Field("Relative Position", locPos);
                GUILayout.Label("Compass Heading");
                EditorGUILayout.FloatField(compassHeading);

                EditorGUILayout.Separator();

                EditorGUILayout.Vector3Field("Object Displacements", displacements);
                //EditorGUILayout.Vector3Field("Received Yaw/Pitch/Roll", ypr);

                EditorGUILayout.Separator();

                EditorGUILayout.Vector3Field("Object Velocities", velocities);
                // EditorGUILayout.Vector4Field("Object Quaternion", new Vector4(vizObj.transform.rotation.x, vizObj.transform.rotation.y, vizObj.transform.rotation.z, vizObj.transform.rotation.w));
                // EditorGUILayout.Vector4Field("Received Quaternion", new Vector4(wQuat.x, wQuat.y, wQuat.z, wQuat.w));

                EditorGUILayout.Separator();

                EditorGUILayout.Vector3Field("Acceleration", wAcc);
                
                EditorGUILayout.Vector3Field("Gyroscope", wGyro);
                
                EditorGUILayout.Vector3Field("Magnetometer", wMag);
                
                EditorGUILayout.Vector3Field("Touch", wTouch);

                EditorGUILayout.Separator();

                GUILayout.BeginHorizontal();

                GUILayout.Label("Average Response Time");

                EditorGUILayout.FloatField(tAVG);

                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();

                GUILayout.Label("Delta T Received");

                EditorGUILayout.FloatField(delT);

                GUILayout.EndHorizontal();


                GUILayout.EndVertical();

                if (GUILayout.Button("Back"))
                {
                    OnEnable();
                }
            }
            #endregion
        }
        #endregion

        #region Vector Tools
        if (vectorTools)
        {
            #region Mode Select
            if (!vectorAutoReduction)
            {
                if (GUILayout.Button("Auto Reduction Calculator"))
                {
                    vectorAutoReduction = true;
                    vectorsAdded = new List<Vector2>();
                    vectorsProduced = new List<Vector2>();
                    reductionValues = Vector2.zero;
                    reduceVectors = false;
                    cmdFormatted = new List<string>();
                    VrrktCoroutine.start(vectorAutoCalc());
                }
            }
            #endregion

            #region Auto Reduction
            if (vectorAutoReduction)
            {
                #region Reduction Value Entry Group
                GUILayout.BeginHorizontal();

                #region Reduction Value Entry
                reductionValues = EditorGUILayout.Vector2Field("Reduction Values", reductionValues, GUILayout.Width(Screen.width * .75f));
                #endregion

                #region Addition/Subtraction Selection
                reduceVectors = EditorGUILayout.Toggle("Subtract Reduction", reduceVectors, GUILayout.Width(Screen.width * .2f));
                #endregion
                

                GUILayout.EndHorizontal();
                #endregion

                #region Columnar Groups
                GUILayout.BeginHorizontal();

                #region Points Added Column
                GUILayout.BeginVertical(GUILayout.Width(Screen.width * .3f));

                GUILayout.Label("Vectors Added");

                if (vectorsAdded != null)
                {
                    for(int n = 0; n < vectorsAdded.Count; n++)
                    {
                        vectorsAdded[n] = EditorGUILayout.Vector2Field("", vectorsAdded[n], GUILayout.Width(Screen.width * .25f - 5));
                    }
                }

                if (GUILayout.Button("Add Vector"))
                {
                    if (vectorsAdded == null)
                    {
                        vectorsAdded = new List<Vector2>();
                    }
                    vectorsAdded.Add(new Vector2(1, 1));
                }
                GUILayout.EndVertical();
                #endregion

                #region Points Generated Column
                GUILayout.BeginVertical(GUILayout.Width(Screen.width * .3f));

                GUILayout.Label("Vectors Produced");

                if (vectorsProduced != null && vectorsProduced.Count > 0)
                {
                    for(int n = 0; n < vectorsProduced.Count; n++)
                    {
                        EditorGUILayout.Vector2Field("", vectorsProduced[n], GUILayout.Width(Screen.width * .25f - 5));
                    }
                }
                GUILayout.EndVertical();
                #endregion

                #region Command Formatted Column
                GUILayout.BeginVertical(GUILayout.Width(Screen.width * .3f));

                GUILayout.Label("Command Formatted");

                if (cmdFormatted != null && cmdFormatted.Count > 0)
                {
                    for(int n = 0; n < cmdFormatted.Count; n++)
                    {
                        GUILayout.TextField(cmdFormatted[n], GUILayout.Width(Screen.width * .25f - 5));
                    }
                }
                GUILayout.EndVertical();
                #endregion

                GUILayout.EndHorizontal();
                #endregion

                #region Controls

                #region Clear All
                if (GUILayout.Button("Clear All"))
                {
                    vectorsAdded = new List<Vector2>();
                    vectorsProduced = new List<Vector2>();
                }
                #endregion

                #region Back
                if (GUILayout.Button("Back"))
                {
                    OnEnable();
                }
                #endregion

                #endregion
            }
            #endregion
        }
        #endregion

        #region Create Stencil Routine
        if (createStencil)
        {
            #region Select File
            if (!fileSelected)
            {
                sP = GUILayout.BeginScrollView(sP);

                #region Path
                GUILayout.BeginHorizontal();

                GUILayout.Label("Current Path:");
                GUILayout.Label(path);

                GUILayout.EndHorizontal();
                #endregion

                GUILayout.Space(20);

                #region Directories

                if (GUILayout.Button("Go Up"))
                {
                    path = path.Substring(0, path.LastIndexOf("/"));
                    path = path.Substring(0, path.LastIndexOf("/") + 1);

                    dirs = Directory.GetDirectories(path);
                    for (int n = 0; n <= dirs.Length - 1; n++)
                    {
                        dirs[n] = dirs[n].Substring(path.Length);
                    }

                    if (fSel != -1)
                    {
                        fSel = -1;
                    }

                    files = Directory.GetFiles(path);
                }
                dSel = GUILayout.SelectionGrid(dSel, dirs, 4, GUILayout.Width(Screen.width - 10));

                if (dSel != -1)
                {
                    path += dirs[dSel];

                    if (path.LastIndexOf("/") < path.Length)
                    {
                        path += "/";
                    }
                    dirs = Directory.GetDirectories(path);
                    for (int n = 0; n <= dirs.Length - 1; n++)
                    {
                        dirs[n] = dirs[n].Substring(path.Length);
                    }

                    if (fSel != -1)
                    {
                        fSel = -1;
                    }

                    files = Directory.GetFiles(path);
                    for (int n = 0; n <= files.Length - 1; n++)
                    {
                        files[n] = files[n].Substring(path.Length);
                    }

                    dSel = -1;
                }
                #endregion

                GUILayout.Space(20);

                #region Files
                fSel = GUILayout.SelectionGrid(fSel, files, 2, GUILayout.Width(Screen.width - 10));
                #endregion

                GUILayout.Space(20);

                #region Controls

                #region Proceed
                if (GUILayout.Button("Proceed"))
                {
                    if (fSel != -1)
                    {
                        if (files[fSel].Contains(".txt"))
                        {
                            fileSelected = true;
                        }
                        else
                        {
                            Debug.Log("Incorrect file type chosen");
                            fSel = -1;
                        }
                    }
                }
                #endregion

                #region Go Back
                if (GUILayout.Button("Back"))
                {
                    OnEnable();
                }
                #endregion

                #endregion

                GUILayout.EndScrollView();
            }
            #endregion

            #region Input Machine Resolution
            else if (!confirmed)
            {
                machSperMM = EditorGUILayout.Vector2Field("Machine Steps / MM", machSperMM, GUILayout.Width(Screen.width * .9f));

                #region Controls
                if (GUILayout.Button("Begin Creation"))
                {
                    confirmed = true;
                    VrrktCoroutine.start(stencilCreator());
                    //start ENUM
                }

                if (GUILayout.Button("Back"))
                {
                    OnEnable();
                }
                #endregion
            }
            #endregion

            #region Creation Operator Running
            else
            {
                EditorGUILayout.Vector2Field("Current Line Indices", curLinePos, GUILayout.Width(Screen.width * .9f));

                EditorGUILayout.Separator();

                if (GUILayout.Button("Back"))
                {
                    OnEnable();
                }
            }
            #endregion
        }
        #endregion

        #region Bitmap Converter
        if (bitmapSelect)
        {
            #region Select File && Specify Dimensions
            if (!bitmapConvert)
            {
                #region Specify Dimensions
                GUILayout.BeginHorizontal();

                GUILayout.Label("Image Width");
                bW = EditorGUILayout.IntField(bW);

                GUILayout.Label("Image Height");
                bH = EditorGUILayout.IntField(bH);

                GUILayout.EndHorizontal();
                #endregion

                sP = GUILayout.BeginScrollView(sP);

                #region Path
                GUILayout.BeginHorizontal();

                GUILayout.Label("Current Path:");
                GUILayout.Label(path);

                GUILayout.EndHorizontal();
                #endregion

                GUILayout.Space(20);

                #region Directories

                if (GUILayout.Button("Go Up"))
                {
                    path = path.Substring(0, path.LastIndexOf("/"));
                    path = path.Substring(0, path.LastIndexOf("/") + 1);

                    dirs = Directory.GetDirectories(path);
                    for (int n = 0; n <= dirs.Length - 1; n++)
                    {
                        dirs[n] = dirs[n].Substring(path.Length);
                    }

                    if (fSel != -1)
                    {
                        fSel = -1;
                    }

                    files = Directory.GetFiles(path);
                }
                dSel = GUILayout.SelectionGrid(dSel, dirs, 4, GUILayout.Width(Screen.width - 10));

                if (dSel != -1)
                {
                    path += dirs[dSel];

                    if (path.LastIndexOf("/") < path.Length)
                    {
                        path += "/";
                    }
                    dirs = Directory.GetDirectories(path);
                    for (int n = 0; n <= dirs.Length - 1; n++)
                    {
                        dirs[n] = dirs[n].Substring(path.Length);
                    }

                    if (fSel != -1)
                    {
                        fSel = -1;
                    }

                    files = Directory.GetFiles(path);
                    for (int n = 0; n <= files.Length - 1; n++)
                    {
                        files[n] = files[n].Substring(path.Length);
                    }

                    dSel = -1;
                }
                #endregion

                GUILayout.Space(20);

                #region Files
                fSel = GUILayout.SelectionGrid(fSel, files, 2, GUILayout.Width(Screen.width - 10));
                #endregion

                GUILayout.Space(20);

                #region Controls

                #region Proceed
                if (GUILayout.Button("Begin Conversion"))
                {
                    if (fSel != -1)
                    {
                        if (bW > 0 && bH > 0 && bW % 8 == 0 && bH % 8 == 0)
                        {
                            Debug.Log("Begin Conversion");
                            bitmapConvert = true;
                            VrrktCoroutine.start(bitmapConverter());
                        }
                        else
                        {
                            Debug.Log("Incorrect file type chosen");
                            fSel = -1;
                        }
                    }
                }
                #endregion

                #region Go Back
                if (GUILayout.Button("Back"))
                {
                    OnEnable();
                }
                #endregion

                #endregion

                GUILayout.EndScrollView();
            }
            #endregion

            #region Receive Result
            else
            {
                #region Output
                EditorGUILayout.TextArea(codeOut, GUILayout.Width(Screen.width - 10), GUILayout.MinHeight(Screen.height * .25f));
                #endregion

                #region Controls
                if (GUILayout.Button("Finished"))
                {
                    OnEnable();
                }
                #endregion
            }
            #endregion
        }
        #endregion

        #region Create Raster
        if (createRaster)
        {
            #region Specify Objects
            if (!chooseObjs && !fileSelected && !confirmed)
            {
                #region Object Selection
                GUILayout.BeginVertical();

                canvasObj = (GameObject) EditorGUILayout.ObjectField("Canvas Object" ,canvasObj, typeof(GameObject), GUILayout.MinWidth(Screen.width * .75f));

                parentObj = (GameObject) EditorGUILayout.ObjectField("Parent Object", parentObj, typeof(GameObject), GUILayout.MinWidth(Screen.width * .75f));

                quadObj = (GameObject) EditorGUILayout.ObjectField("Quadrilateral Object", quadObj, typeof(GameObject), GUILayout.MinWidth(Screen.width * .75f));

                circleObj = (GameObject) EditorGUILayout.ObjectField("Circular Object", circleObj, typeof(GameObject), GUILayout.MinWidth(Screen.width * .75f));

                GUILayout.EndVertical();
                #endregion

                #region Controls
                if (GUILayout.Button("Proceed"))
                {
                    if (parentObj != null && canvasObj != null && circleObj != null && quadObj != null)
                    {
                        chooseObjs = true;
                    }
                    else
                    {
                        Debug.Log("Not all required objects are specified!!!");
                    }
                }

                if (GUILayout.Button("Back"))
                {
                    OnEnable();
                }
                #endregion
            }
            #endregion

            #region Select Routine
            else if (chooseObjs && !fileSelected && !confirmed)
            {
                sP = GUILayout.BeginScrollView(sP);

                #region Path
                GUILayout.BeginHorizontal();

                GUILayout.Label("Current Path:");
                GUILayout.Label(path);

                GUILayout.EndHorizontal();
                #endregion

                GUILayout.Space(20);

                #region Directories

                if (GUILayout.Button("Go Up"))
                {
                    path = path.Substring(0, path.LastIndexOf("/"));
                    path = path.Substring(0, path.LastIndexOf("/") + 1);

                    dirs = Directory.GetDirectories(path);
                    for (int n = 0; n <= dirs.Length - 1; n++)
                    {
                        dirs[n] = dirs[n].Substring(path.Length);
                    }

                    if (fSel != -1)
                    {
                        fSel = -1;
                    }

                    string[] fls = Directory.GetFiles(path);
                    List<string> lfs = new List<string>();
                    for (int n = 0; n < fls.Length; n++)
                    {
                        if (fls[n].Contains(".txt"))
                        {
                            lfs.Add(fls[n]);
                        }
                    }
                    files = lfs.ToArray();
                    for (int n = 0; n <= files.Length - 1; n++)
                    {
                        files[n] = files[n].Substring(path.Length);
                    }
                }
                dSel = GUILayout.SelectionGrid(dSel, dirs, 4, GUILayout.Width(Screen.width - 10));

                if (dSel != -1)
                {
                    path += dirs[dSel];

                    if (path.LastIndexOf("/") < path.Length)
                    {
                        path += "/";
                    }
                    dirs = Directory.GetDirectories(path);
                    for (int n = 0; n <= dirs.Length - 1; n++)
                    {
                        dirs[n] = dirs[n].Substring(path.Length);
                    }

                    if (fSel != -1)
                    {
                        fSel = -1;
                    }

                    string[] fls = Directory.GetFiles(path);
                    List<string> lfs = new List<string>();
                    for (int n = 0; n < fls.Length; n++)
                    {
                        if (fls[n].Contains(".txt"))
                        {
                            lfs.Add(fls[n]);
                        }
                    }
                    files = lfs.ToArray();

                    for (int n = 0; n <= files.Length - 1; n++)
                    {
                        files[n] = files[n].Substring(path.Length);
                    }

                    dSel = -1;
                }
                #endregion

                GUILayout.Space(20);

                #region Files
                fSel = GUILayout.SelectionGrid(fSel, files, 2, GUILayout.Width(Screen.width - 10));
                #endregion

                GUILayout.Space(20);

                #region Controls

                #region Proceed
                if (GUILayout.Button("Proceed"))
                {
                    if (fSel != -1)
                    {
                        if (files[fSel].Contains(".vrrktPTH") || files[fSel].Contains(".txt"))
                        {
                            VrrktCoroutine.start(loadData());
                        }
                        else
                        {
                            Debug.Log("Incorrect file type chosen");
                            fSel = -1;
                        }
                    }
                }
                #endregion

                #region Go Back
                if (GUILayout.Button("Back"))
                {
                    OnEnable();
                }
                #endregion

                #endregion

                GUILayout.EndScrollView();
            }
            #endregion

            #region Confirm Selection
            else if (chooseObjs &&  fileSelected && !confirmed)
            {
                #region Routine Display
                GUILayout.BeginHorizontal();

                if (cmdCumulation != null)
                {
                    GUILayout.Box("Total Lines: " + cmdCumulation.Count);
                }

                GUILayout.EndHorizontal();
                #endregion

                #region Controls

                #region Confirm Button
                if (GUILayout.Button("Confirm"))
                {
                    objs = new List<GameObject>();
                    cnv = canvasObj.GetComponent<Canvas>();
                    VrrktCoroutine.start(rasterController(), true);
                    confirmed = true;
                }
                #endregion

                #region Back Button
                if (GUILayout.Button("Back"))
                {
                    OnEnable();
                }
                #endregion

                #endregion
            }
            #endregion

            #region Visualize Progress
            else if (chooseObjs && fileSelected && confirmed)
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label("Current X Position");
                EditorGUILayout.FloatField(cPos.x, GUILayout.MinWidth(Screen.width * .5f));
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Current Y Position");
                EditorGUILayout.FloatField(cPos.y, GUILayout.MinWidth(Screen.width * .5f));
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Raster Direction");
                GUILayout.Toggle(rDir, "", GUILayout.MinWidth(Screen.width * .25f));
                GUILayout.EndHorizontal();
            }
            #endregion
        }
        #endregion

        #region Use Controller
        if (useController)
        {
            #region Define Port
            if (!portChosen)
            {
                #region Port Input
                GUILayout.BeginHorizontal();

                GUILayout.Label("COM Port to connect to");
                cPort = GUILayout.TextField(cPort, GUILayout.Width(Screen.width - 10));

                GUILayout.EndHorizontal();
                #endregion

                #region Controls

                #region Proceed Button
                if (GUILayout.Button("Proceed"))
                {
                    if (cPort != "" && cPort.Contains("COM"))
                    {
                        port = new SerialPort(cPort, baudRate);

                        try
                        {
                            //port.Open();
                        }
                        catch (Exception e)
                        {
                            Debug.Log("Error opening port");
                            Debug.Log(e.Message);
                            Debug.Log(e.StackTrace);
                            cPort = "";
                        }

                        if (cPort != "")
                        {
                            VrrktCoroutine.start(controllerControl());
                            portChosen = true;
                        }
                    }
                }
                #endregion

                #region Back Button
                if (GUILayout.Button("Back"))
                {
                    OnEnable();
                }
                #endregion

                #endregion
            }
            #endregion

            #region Controller Usage GUI Screen
            if (portChosen)
            {
                #region XYZ
                GUILayout.BeginHorizontal();

                #region X
                GUILayout.Box("X: " + xAxis);
                #endregion

                #region Y
                GUILayout.Box("Y: " + yAxis);
                #endregion

                #region Z
                    GUILayout.Box("Z: " + zAxis);
                #endregion

                GUILayout.EndHorizontal();
                #endregion

                #region Laser and Spindle
                GUILayout.BeginHorizontal();

                #region Laser
                    GUILayout.Toggle(laserOn, "Laser State");
                
                GUILayout.Toggle(laserBtn, "Laser Button");
                #endregion

                #region Spindle
                    GUILayout.Toggle(spindleOn, "Spindle State");
                
                GUILayout.Toggle(spindleBtn, "Spindle Button");
                #endregion

                GUILayout.EndHorizontal();
                #endregion
                
                #region Other Buttons
                GUILayout.BeginHorizontal();

                #region Step Length
                    GUILayout.Box("Step Length: " + stepLength);
                    GUILayout.Box("Step Length Axis: " + lAxis);
                #endregion

                #region Home & Jog
                GUILayout.Toggle(homeBtn, "Home Button");

                GUILayout.Toggle(continuousJog, "Continuous Jog Button");

                #endregion

                GUILayout.EndHorizontal();
                #endregion

                #region Controls
                if (GUILayout.Button("Return"))
                {
                    VrrktCoroutine.totalWipe();
                    OnEnable();
                }
                #endregion
            }
            #endregion
        }
        #endregion

        EditorGUILayout.EndVertical();

    }
    #endregion

    #region Vector Tools Functions

    #region Vector Auto Calculator
    IEnumerator vectorAutoCalc()
    {
        while (vectorAutoReduction)
        {
            if (vectorsAdded != null && vectorsAdded.Count > 0)
            {
                for (int n = 0; n < vectorsAdded.Count; n++)
                {
                    if (vectorsAdded[n] == Vector2.zero)
                    {
                        vectorsAdded.RemoveAt(n);
                    }
                }
            }

            if (vectorsAdded != null && vectorsAdded.Count > 0)
            {
                vectorsProduced = new List<Vector2>();
                for (int n = 0; n < vectorsAdded.Count; n++)
                {
                    if (reduceVectors)
                    {
                        vectorsProduced.Add(new Vector2(vectorsAdded[n].x - reductionValues.x, vectorsAdded[n].y - reductionValues.y));
                    }
                    else
                    {
                        vectorsProduced.Add(new Vector2(vectorsAdded[n].x + reductionValues.x, vectorsAdded[n].y + reductionValues.y));
                    }
                }

                cmdFormatted = new List<string>();
                for(int n = 0; n < vectorsProduced.Count; n++)
                {
                    cmdFormatted.Add("X" + vectorsProduced[n].x + "Y" + vectorsProduced[n].y);
                }
            }
            Repaint();
            yield return true;
        }

        OnEnable();
        yield return false;
    }
    #endregion

    #endregion

    #region Visualization Part 1 Operator
    IEnumerator initialVisualization()
    {
        xMax = 0;
        yMax = 0;
        drawPaths = new List<List<Vector2>>();
        drawRoutine = new List<Vector2>();
        fillInitPoints = new List<Vector2>();
        fillDimensions = new List<Vector2>();
        route = "";
        pathViz = null;
        yield return true;

        #region Convert Routine To Vector Lists
        for(int n = 2; n <= cmdCumulation.Count - 1; n++)
        {
            if (cmdCumulation[n][0] == 'm')
            {
                if (drawRoutine.Count > 0)
                {
                    drawPaths.Add(drawRoutine);
                    drawRoutine = new List<Vector2>();
                }

                if (cmdCumulation[n + 1].Contains('d'))
                {
                    route = cmdCumulation[n].Substring(cmdCumulation[n].IndexOf('m') + 1);
                    route = route.Substring(0, route.IndexOf(','));

                    if (route.IndexOf('$') >= 0)
                    {
                        route = route.Replace("$", "");
                    }

                    if (!float.TryParse(route, out xPart))
                    {
                        Debug.Log("Error parsing x value for move command on line " + n);
                        continue;
                    }
                    else
                    {
                        route = cmdCumulation[n].Substring(cmdCumulation[n].IndexOf(',') + 1);
                        route = route.Substring(0, route.IndexOf('$') - 1);

                        if (route.IndexOf('$') >= 0)
                        {
                            route = route.Replace("$", "");
                        }

                        if (!float.TryParse(route, out yPart))
                        {
                            Debug.Log("Error parsing y value for move command on line " + n);
                        }
                        else
                        {
                            drawRoutine = new List<Vector2>();
                            drawRoutine.Add(new Vector2(xPart, yPart));
                        }
                    }
                }
            }
            else if (cmdCumulation[n][0] == 'd')
            {
                route = cmdCumulation[n].Substring(cmdCumulation[n].IndexOf('d') + 1);
                route = route.Substring(0, route.IndexOf(','));

                if (route.IndexOf('$') >= 0)
                {
                    route = route.Replace("$", "");
                }


                if (!float.TryParse(route, out xPart))
                {
                    Debug.Log("Error parsing x value for draw command on line " + n + ". Substring is " + route);
                    continue;
                }
                else
                {
                    route = cmdCumulation[n].Substring(cmdCumulation[n].IndexOf(',') + 1);
                    route = route.Substring(0, route.IndexOf('$') - 1);

                    if (route.IndexOf('$') >= 0)
                    {
                        route = route.Replace("$", "");
                    }


                    if (!float.TryParse(route, out yPart))
                    {
                        Debug.Log("Error parsing y value for draw command on line " + n + ". Substring is " + route);
                    }
                    else
                    {
                        drawRoutine.Add(new Vector2(xPart, yPart));
                    }
                }
            }
            else if (cmdCumulation[n][0] == 'f')
            {
                route = cmdCumulation[n].Substring(cmdCumulation[n].IndexOf('f') + 1);
                route = route.Substring(0, route.IndexOf(','));

                if (route.IndexOf('$') >= 0)
                {
                    route = route.Replace("$", "");
                }


                if (!float.TryParse(route, out xPart))
                {
                    Debug.Log("Error parsing x value for fill command on line " + n + ". Substring is " + route);
                    continue;
                }
                else
                {
                    route = cmdCumulation[n].Substring(cmdCumulation[n].IndexOf(',') + 1);
                    route = route.Substring(0, route.IndexOf(',') - 1);

                    if (route.IndexOf('$') >= 0)
                    {
                        route = route.Replace("$", "");
                    }


                    if (!float.TryParse(route, out yPart))
                    {
                        Debug.Log("Error parsing y value for fill command on line " + n + ". Substring is " + route);
                    }
                    else
                    {
                        route = cmdCumulation[n].Substring(cmdCumulation[n].IndexOf(',') + 1);
                        route = route.Substring(route.IndexOf(',') + 1);
                        route = route.Substring(0, route.IndexOf(','));

                        if (route.IndexOf('$') >= 0)
                        {
                            route = route.Replace("$", "");
                        }


                        if (!float.TryParse(route, out wPart))
                        {
                            Debug.Log("Error parsing w value for fill command on line " + n + ". Substring is " + route);
                        }
                        else
                        {
                            route = cmdCumulation[n].Substring(cmdCumulation[n].IndexOf(',') + 1);
                            route = route.Substring(route.IndexOf(',') + 1);
                            route = route.Substring(route.IndexOf(',') + 1);

                            if (route.IndexOf('$') >= 0)
                            {
                                route = route.Replace("$", "");
                            }

                            if (!float.TryParse(route, out hPart))
                            {
                                Debug.Log("Error parsing h value for fill command on line " + n + ". Substring is " + route);
                            }
                            else
                            {
                                fillInitPoints.Add(new Vector2(xPart, yPart));
                                fillDimensions.Add(new Vector2(wPart, hPart));
                            }
                        }
                    }
                }
            }
            yield return true;
        }
        #endregion

        #region Find Largest Point Values
        for(int n = 0; n <= drawPaths.Count - 1; n++)
        {
            for(int w = 0; w <= drawPaths[n].Count - 1; w++)
            {
                if (drawPaths[n][w].x > xMax)
                {
                    xMax = drawPaths[n][w].x;
                }

                if (drawPaths[n][w].y > yMax)
                {
                    yMax = drawPaths[n][w].y;
                }
                yield return true;
            }
            yield return true;
        }

        for(int n = 0; n <= fillInitPoints.Count - 1; n++)
        {
            if (fillInitPoints[n].x + fillDimensions[n].x > xMax)
            {
                xMax = fillInitPoints[n].x + fillDimensions[n].x;
            }
            
            if (fillInitPoints[n].y + fillDimensions[n].y > yMax)
            {
                xMax = fillInitPoints[n].y + fillDimensions[n].y;
            }
            yield return true;
        }
        #endregion

        #region Calculate Visualization Dimensions
        vizW = (long)((xMax + 10) * xSMM);
        vizH = (long)((yMax + 10) * ySMM);
        yield return true;
        #endregion

        #region Initialize Image
        pathViz = new Texture2D((int)vizW, (int)vizH);
        pxs = pathViz.GetPixels();
        yield return true;
        #endregion

        #region Draw Initial Points
        for(int n = 0; n <= drawPaths.Count - 1; n++)
        {
            for(int xP = (int)(drawPaths[n][0].x * xSMM) - 5; xP <= (int)(drawPaths[n][0].x * xSMM) + 5; xP++)
            {
                for(int yP = (int)(drawPaths[n][0].y * ySMM) - 5; yP <= (int)(drawPaths[n][0].y * ySMM) + 5; yP++)
                {
                    pathViz.SetPixel(xP, yP, UnityEngine.Color.green);
                }
                yield return true;
            }
            yield return true;
        }

        for(int n = 0; n <= fillInitPoints.Count - 1; n++)
        {
            for (int xP = (int)(fillInitPoints[n].x * xSMM) - 5; xP <= (int)(fillInitPoints[n].x * xSMM) + 5; xP++)
            {
                for (int yP = (int)(fillInitPoints[n].y * ySMM) - 5; yP <= (int)(fillInitPoints[n].y * ySMM) + 5; yP++)
                {
                    pathViz.SetPixel(xP, yP, UnityEngine.Color.green);
                }
                yield return true;
            }
            yield return true;
        }
        #endregion

        #region Draw Fill Paths
        for(int n = 0; n <= fillInitPoints.Count - 1; n++)
        {
            for(int wP = (int)(fillInitPoints[n].x * xSMM); wP <= (int)(fillInitPoints[n].x * xSMM) + (fillDimensions[n].x * xSMM); wP++)
            {
                for (int hP = (int)(fillInitPoints[n].y * ySMM); hP <= (int)(fillInitPoints[n].y * ySMM) + (fillDimensions[n].y * ySMM); hP++)
                {
                    pathViz.SetPixel(wP, hP, UnityEngine.Color.blue);
                }
                yield return true;
            }
            yield return true;
        }
        #endregion

        #region Draw Draw Paths

        #endregion

        #region Update Image
        pathViz.Apply();
        yield return true;
        #endregion

        #region Finish Operation
        visualization = true;
        confirmed = true;
        yield return true;
        #endregion

        yield return false;
    }
    #endregion
    
    #region Load File Operator
    IEnumerator loadData()
    {
            FileStream f = File.Open(path + files[fSel], FileMode.Open);
            StreamReader r = new StreamReader(f);
            cmdCumulation = new List<string>();

            while(r.Peek() >= 0)
            {
                cmdCumulation.Add(r.ReadLine());
            }
            f.Close();
        fileSelected = true;
        yield return false;
    }
    #endregion
    
    #region Sending Operator
    IEnumerator sendor()
    {
        sendLim = cmdCumulation.Count - 1;
        sendSteps = 0;
        IO = false;
        yield return true;

        #region Send Steps
        while (sendSteps <= sendLim)
        {
            #region Send
            if (!IO)
            {
                port.Write(cmdCumulation[sendSteps]);
                IO = true;
                Repaint();
                yield return true;
            }
            #endregion

            #region Receive
            else
            {
                if (port.BytesToRead > 0)
                {
                    bs = new byte[1];
                    bs[0] = (byte)port.ReadByte();
                    inb += Encoding.Default.GetString(bs);
                }

                if (inb.Contains("$") && inb.Contains("@") && inb.Contains("K"))
                {
                    //Debug.Log("Sent: " + cmdCumulation[sendSteps] + ", Response is " + inb);
                    //port.DiscardInBuffer();
                    IO = false;
                    sendSteps++;
                    inb = "";
                    //Debug.Log("Received Step " + sendSteps);
                }
                else if (inb != "")
                {
                    //Debug.Log(inb);
                }
                yield return true;
            }
            #endregion
        }
        #endregion

        if (port != null)
        {
            if (port.IsOpen)
            {
                port.Close();
                port = null;
            }
        }
        Debug.Log("Sent All Data");
        OnEnable();

        yield return false;
    }
    #endregion

    #region New Processor Operator
    IEnumerator kicadProcessor()
    {
        string part, sub;

        List<string> frontCmds = new List<string>();
        List<string> backCmds = new List<string>();
        string lne = "";
        decimal reduct = 0;
        decimal minVal = 0;


        board board = new board();
        board rotatedBoard = new board();
        boardLayer front = new boardLayer();
        boardLayer back = new boardLayer();
        module module = new module();
        module mdl = new module();
        pad pad = new pad();
        trace trace = new trace();
        via via = new via();
        edgeCuts edgeCut = new edgeCuts();

        #region Load
        FileStream fL = File.Open(path + files[fSel], FileMode.Open);
        StreamReader r = new StreamReader(fL);

        while (r.Peek() >= 0)
        {
            lne = r.ReadLine();
            process1.Add(lne);
            yield return true;
        }
        fL.Close();
        #endregion

        #region Intermission Debugging
        //Debug.Log("Debugging after load");
        //for (int n = 0; n <= process1.Count - 1; n++)
        //{
        //    Debug.Log("[" + n + "]: " + process1[n]);
        //}
        #endregion

        #region Initial Trim
        for (int n = 0; n <= process1.Count - 1; n++)
        {
            #region Identify Module Beginning
            if (process1[n].Contains("(footprint "))
            {
                module = new module();
                mdl = null;
                for(int q = 0; q < 5; q++)
                {
                    if (process1[n + q].Contains("(at "))
                    {
                        module.moduleLine = process1[n + q];
                        break;
                    }
                }
                bool thruHole = false;

                #region Pull Pad For Module
                for (int q = n + 1; q < process1.Count; q++)
                {
                    if (process1[q].Contains("(pad "))
                    {
                        pad = new pad();
                        pad.padLine = process1[q];
                        module.pads.Add(pad);
                        
                        if (process1[q].Contains("thru_hole"))
                        {
                            pad.drill = true;
                            if (mdl != null)
                            {
                                pad p = new pad();
                                p.padLine = process1[q];
                                mdl.pads.Add(p);
                            }
                            else
                            {
                                mdl = new module();
                                for (int e = 0; e < 5; e++)
                                {
                                    if (process1[n + e].Contains("(at "))
                                    {
                                        mdl.moduleLine = process1[n + e];
                                        break;
                                    }
                                }
                                pad p = new pad();
                                p.padLine = process1[q];
                                mdl.pads.Add(p);
                            }
                        }
                    }
                    else if (process1[q].Contains("(footprint ") || process1[q].Contains("(model ") || process1[q].Contains("(offset (xyz") || process1[q].Contains("(scale (xyz") || process1[q].Contains("(rotate (xyz") || process1[q].Contains("(at (xyz ") || process1[q].Contains("(module ") || process1[q].Contains("(segment ") || process1[q].Contains("(via "))
                    {
                        break;
                    }
                }
                #endregion
                
                #region Place Within Layer
                if (process1[n].Contains("*.Cu") || process1[n].Contains("*.Cu") || mdl != null)
                {
                    front.modules.Add(module);
                    back.modules.Add(mdl);
                }
                else if (process1[n].Contains("F.Cu"))
                {
                    if (front == null)
                    {
                        Debug.Log("Front layer is null");
                    }
                    else if (front.traces == null)
                    {
                        Debug.Log("Front layer Traces is null");
                    }
                    else if (trace == null)
                    {
                        Debug.Log("trace object is null");
                    }
                    front.modules.Add(module);
                }
                else if (process1[n].Contains("B.Cu"))
                {
                    if (back == null)
                    {
                        Debug.Log("back layer is null");
                    }
                    else if (back.traces == null)
                    {
                        Debug.Log("back layer Traces is null");
                    }
                    else if (trace == null)
                    {
                        Debug.Log("trace object is null");
                    }
                    back.modules.Add(module);
                }
                #endregion

                n++;
            }
            #endregion

            #region Identify Segment Beginning
            else if (process1[n].Contains("(segment "))
            {
                trace = new trace();
                trace.traceLine = process1[n];
                if ((process1[n].Contains("(layer ") && process1[n].Contains("F.Cu")) || (process1[n].Contains("(layers ") && process1[n].Contains("F.Cu")))
                {
                    if (front == null)
                    {
                        Debug.Log("Front layer is null");
                    }
                    else if (front.traces == null)
                    {
                        Debug.Log("Front layer Traces is null");
                    }
                    else if (trace == null)
                    {
                        Debug.Log("trace object is null");
                    }
                    front.traces.Add(trace);
                }
                else if ((process1[n].Contains("(layer ") && process1[n].Contains("B.Cu")) || (process1[n].Contains("(layers ") && process1[n].Contains("B.Cu")))
                {
                    if (back == null)
                    {
                        Debug.Log("back layer is null");
                    }
                    else if (back.traces == null)
                    {
                        Debug.Log("back layer Traces is null");
                    }
                    else if (trace == null)
                    {
                        Debug.Log("trace object is null");
                    }
                    back.traces.Add(trace);
                }
                else if (process1[n].Contains("*.Cu)") || process1[n].Contains("*.Cu)"))
                {
                    front.traces.Add(trace);
                    back.traces.Add(trace);
                }
            }
            #endregion

            #region Identify Via Beginning
            else if (process1[n].Contains("(via "))
            {
                via = new via();
                via.viaLine = process1[n];
                front.vias.Add(via);
                //Debug.Log("Added Via to front");
            }
            #endregion

            #region Identify Border Cut Beginning
            else if (process1[n].Contains("Edge.Cuts"))
            {
                edgeCut.cutLines = process1[n];
            }
            #endregion
        }
        #endregion

        #region Process Edgecuts
        edgeCut.process();

        if (edgeCut.minValues.x < edgeCut.minValues.y)
        {
            minVal = edgeCut.minValues.x;
        }
        else
        {
            minVal = edgeCut.minValues.y;
        }
        front.borders = edgeCut;
        back.borders = edgeCut;
        reduct = minVal;
        #endregion
        
        #region Layer Component Processing

        front.process();

        back.process();

        #endregion

        #region Layer Component Debugging
        //Debug.Log("Front Counts. Modules:" + front.modules.Count + ", Traces:" + front.traces.Count + ", Vias:" + front.vias.Count);
        //Debug.Log("Back Counts. Modules:" + back.modules.Count + ", Traces:" + back.traces.Count + ", Vias:" + back.vias.Count);
        #endregion

        #region Construct Boards
        if (back.vias == null || back.vias.Count == 0)
        {
            back.vias = front.vias;
        }
        else if (front.vias == null || front.vias.Count == 0)
        {
            front.vias = back.vias;
        }
        board.front = front;

        board.back = back;
        #endregion

        #region Perform Reductions
        //board.reduction();
        #endregion
        
        #region Layer Component Debugging
        //Debug.Log("Base Front Counts. Modules:" + front.modules.Count + ", Traces:" + front.traces.Count + ", Vias:" + front.vias.Count);
        //Debug.Log("Base Back Counts. Modules:" + back.modules.Count + ", Traces:" + back.traces.Count + ", Vias:" + back.vias.Count);

        //Debug.Log("Rotated Front Counts. Modules:" + front.modules.Count + ", Traces:" + front.traces.Count + ", Vias:" + front.vias.Count);
        //Debug.Log("Rotated Back Counts. Modules:" + back.modules.Count + ", Traces:" + back.traces.Count + ", Vias:" + back.vias.Count);
        #endregion
        
        #region Optimize & Save Boards

        StreamWriter w;
        int u = 0;
        string pathBase = path + files[fSel];
        pathBase = pathBase.Substring(0, pathBase.IndexOf('.'));

        #region Regular.Front
        board.front.constructCommand();
        //Debug.Log("Board Front Command Count:" + board.front.layerCommandRoutine.Count);

        drawDone = false;
        cmdCumulation = new List<string>();
        //Debug.Log("Optimizing Front");
        VrrktCoroutine.start(boardDrawOptimizer(board.front));

        while (!drawDone)
        {
            yield return true;
        }
        board.front.layerCommandRoutine = cmdCumulation;

        board.front.layerCommandRoutine = rearrangeRoutine(board.front.layerCommandRoutine);
        board.front.layerCommandRoutine.Add("END/");

        if (board.front.layerCommandRoutine.Count > 7)
        {
            board.front.routineReduction();
            part = pathBase + "FRONT";
            string fDots = part + "-DOTS";
            part = part + ".txt";
            fDots += ".txt";
            fL = File.Create(part);
            w = new StreamWriter(fL);
            u = 0;
            while (u <= board.front.layerCommandRoutine.Count - 1)
            {
                //w.Write(cmdCumulation[u]);
                if (board.front.layerCommandRoutine[u] != "\n")
                {
                    if (!board.front.layerCommandRoutine[u].Contains("DRILLS/") && !board.front.layerCommandRoutine[u].Contains("WX"))
                    {
                        w.WriteLine(board.front.layerCommandRoutine[u]);
                        w.Flush();
                    }
                }
                u++;
            }
            yield return true;
            //Debug.Log("Lines written: " + u);
            fL.Close();

            fL = File.Create(fDots);
            w = new StreamWriter(fL);
            u = 0;

            while (u <= board.front.layerCommandRoutine.Count - 1)
            {
                //w.Write(cmdCumulation[u]);
                if (board.front.layerCommandRoutine[u] != "\n")
                {
                    if (board.front.layerCommandRoutine[u].Contains("DRILLS/") || board.front.layerCommandRoutine[u].Contains("WX") || u <= 3 || board.front.layerCommandRoutine[u] == "END/")
                    {
                        if (board.front.layerCommandRoutine[u].Contains("WX"))
                        {
                            board.front.layerCommandRoutine[u] = board.front.layerCommandRoutine[u].Replace("WX", "wX");
                        }
                        w.WriteLine(board.front.layerCommandRoutine[u]);
                        w.Flush();
                    }
                }
                u++;
            }
            yield return true;
            //Debug.Log("Lines written: " + u);
            fL.Close();

            fDots = fDots.Replace("-DOTS", "-DRILLS");

            fL = File.Create(fDots);
            w = new StreamWriter(fL);
            u = 0;

            while (u <= board.front.layerCommandRoutine.Count - 1)
            {
                //w.Write(cmdCumulation[u]);
                if (board.front.layerCommandRoutine[u] != "\n")
                {
                    if (board.front.layerCommandRoutine[u].Contains("DRILLS/") || board.front.layerCommandRoutine[u].Contains("wX") || u <= 3 || board.front.layerCommandRoutine[u] == "END/")
                    {
                        if (board.front.layerCommandRoutine[u].Contains("wX"))
                        {
                            board.front.layerCommandRoutine[u] = board.front.layerCommandRoutine[u].Replace("wX", "WX");
                        }
                        w.WriteLine(board.front.layerCommandRoutine[u]);
                        w.Flush();
                    }
                }
                u++;
            }
            yield return true;
            //Debug.Log("Lines written: " + u);
            fL.Close();
        }
        #endregion

        #region Regular.Back
        board.back.constructCommand();
        //Debug.Log("Board Back Command Count:" + board.back.layerCommandRoutine.Count);

        if (board.back.layerCommandRoutine.Count > 7)
        {
            board.back.routineReduction();
            drawDone = false;
            cmdCumulation = new List<string>();
            //Debug.Log("Optimizing Back");
            VrrktCoroutine.start(boardDrawOptimizer(board.back));

            while (!drawDone)
            {
                yield return true;
            }
            board.back.layerCommandRoutine = cmdCumulation;
            board.back.layerCommandRoutine = rearrangeRoutine(board.back.layerCommandRoutine);
            board.back.layerCommandRoutine.Insert(4, "MIR/");
            board.back.layerCommandRoutine.Add("END/");
        
                part = pathBase + "BACK";
                part = part + ".txt";
                fL = File.Create(part);
                w = new StreamWriter(fL);
                u = 0;
            while (u <= board.back.layerCommandRoutine.Count - 1)
            {
                //w.Write(cmdCumulation[u]);
                if (board.back.layerCommandRoutine[u] != "\n")
                {
                    if (!board.back.layerCommandRoutine[u].Contains("DRILLS/") && !board.back.layerCommandRoutine[u].Contains("WX"))
                    {
                        w.WriteLine(board.back.layerCommandRoutine[u]);
                        w.Flush();
                    }
                }
                u++;
            }
                yield return true;
                //Debug.Log("Lines written: " + u);
                fL.Close();
            }
        #endregion
        
        #endregion

        OnEnable();
        yield return false;
    }
    #endregion

    #region Kicad Processing Operator
    IEnumerator kicadProcessor1()
    {
        string part, sub;
        float maxX, maxY, minX, minY, nX, nY;

        #region Load & Initial Trim
        cmdCumulation = new List<string>();
        process1 = new List<string>();
        fillSeq = new List<string>();
        frontSeg = new List<string>();
        backSeg = new List<string>();
        viaSeq = new List<string>();
        List<string> kC = new List<string>();
        FileStream fL = File.Open(path + files[fSel], FileMode.Open);
        StreamReader r = new StreamReader(fL);
        string lne = "";

        while (r.Peek() >= 0)
        {
            lne = r.ReadLine();

            if (lne.Length > 0)
            {
                if (lne.Contains("(module "))
                {
                    lne = r.ReadLine();
                    process1.Add(lne);
                }
                else if (lne.Contains("(pad ") || lne.Contains("(segment ") || lne.Contains("(via "))
                {
                    process1.Add(lne);
                }
            }
            yield return true;
        }
        fL.Close();
        #endregion

        #region Intermission Debugging
        //Debug.Log("Debugging between load and trim");
        //for (int n = 0; n <= process1.Count - 1; n++)
        //{
            //Debug.Log("[" + n + "]: " + process1[n]);
        //}
        #endregion

        #region Further Trim
        cmdCumulation = new List<string>();
        int u = 0;
        while(u <= process1.Count - 1)
        {
            #region Module Trimming
            if (process1[u].Contains("(at") && !process1[u].Contains("(pad ") && !process1[u].Contains("(via "))
            {
                //Debug.Log("Module Initialization Found at line [" + u + "]");
                kC = new List<string>();
                kC.Add(process1[u]);
                for(int q = u + 1; q <= process1.Count - 1; q++)
                {
                    if (process1[q].Contains("(pad"))
                    {
                        //Debug.Log("Module Relevant Subline found at line [" + q + "]");
                        kC.Add(process1[q]);
                    }
                    else
                    {
                        //Debug.Log("Module break found at line [" + q + "]: " + process1[q]);
                        break;
                    }
                }
                moduleProcessor(kC, 1);
            }
            #endregion

            #region Segment Trimming
            else if (process1[u].Contains("(segment "))
            {
                segmentProcessor(process1[u], 1);
            }
            #endregion

            #region Via Trimming
            else if (process1[u].Contains("(via "))
            {
                viaProcessor(process1[u]);
            }
            #endregion

            #region Unused Line Debugging
            //else
            //{
            // Debug.Log("Unused line at [" + u + "]: " + process1[u]);
            // }
            #endregion

            u++;
            yield return true;
        }
        process1 = cmdCumulation;
        cmdCumulation = new List<string>();
        #endregion

        #region Intermission Debugging
        //Debug.Log("Debugging between trim and final conversion");
        //for(int n = 0; n <= process1.Count - 1; n++)
        //{
        //    Debug.Log("[" + n + "]: " + process1[n]);
        //}
        #endregion

        #region Final Conversion

        #region Fill Sequences
        u = 0;
        process1 = new List<string>();
        while(u <= fillSeq.Count - 1)
        {

            kC = new List<string>();
            kC.Add(fillSeq[u]);
            for (int q = u + 1; q <= fillSeq.Count - 1; q++)
            {
                if (fillSeq[q].Contains("PART "))
                {
                    kC.Add(fillSeq[q]);
                    u++;
                }
                else
                {
                    //Debug.Log("FILL break found. Line is: " + process1[q]);
                    break;
                }
            }
            moduleProcessor(kC, 2);

            u++;
            yield return true;
        }

        fillSeq = process1;
        #endregion

        #region Draw Segments

        #region Front Segments
        u = 0;
        process1 = new List<string>();
        while (u <= frontSeg.Count - 1)
        {
            segmentProcessor(frontSeg[u], 2);

            u++;
            yield return true;
        }
        frontSeg = process1;
        #endregion

        #region Back Segments
        u = 0;
        process1 = new List<string>();
        while (u <= backSeg.Count - 1)
        {
            segmentProcessor(backSeg[u], 2);

            u++;
            yield return true;
        }
        backSeg = process1;
        #endregion

        #endregion

        #endregion

        #region Obtain Reduction Values
        maxX = 0;
        minX = 0;
        maxY = 0;
        minY = 0;

        #region Fill Sequences
        for(int n = 0; n <= fillSeq.Count - 1; n++)
        {
            part = fillSeq[n];
            part = part.Substring(1);
            sub = part.Substring(0, part.IndexOf(','));
            if (!float.TryParse(sub, out nX))
            {
                Debug.Log("Failed to parse fill x. parsed is " + sub);
            }
            else
            {
                if (maxX == 0 || nX > maxX)
                {
                    maxX = nX;
                }
                else if (minX == 0 || nX < minX)
                {
                    minX = nX;
                }

                sub = part;
                sub = sub.Substring(sub.IndexOf(','));
                if (sub[0] == ',')
                {
                    sub = sub.Substring(1);
                }
                sub = sub.Substring(0, sub.IndexOf(','));
                if (!float.TryParse(sub, out nY))
                {
                    Debug.Log("Failed to parse fill y. parsed is " + sub);
                }
                else
                {
                    if (maxY == 0 || nY > maxY)
                    {
                        maxY = nY;
                    }
                    else if (minY == 0 || nY < minY)
                    {
                        minY = nY;
                    }
                }
            }
        }
        yield return true;
        #endregion

        #region Draw Segments

        #region Front Segments
        for (int n = 0; n <= frontSeg.Count - 1; n++)
        {
            part = frontSeg[n];
            part = part.Substring(1);
            sub = part.Substring(0, part.IndexOf(','));
            if (!float.TryParse(sub, out nX))
            {
                Debug.Log("Failed to parse front x. parsed is " + sub);
            }
            else
            {
                if (maxX == 0 || nX > maxX)
                {
                    maxX = nX;
                }
                else if (minX == 0 || nX < minX)
                {
                    minX = nX;
                }
                sub = part;
                sub = sub.Substring(sub.IndexOf(','));
                if (sub[0] == ',')
                {
                    sub = sub.Substring(1);
                }
                sub = sub.Substring(0, sub.IndexOf('$'));
                if (!float.TryParse(sub, out nY))
                {
                    Debug.Log("Failed to parse front y. parsed is " + sub);
                }
                else
                {
                    if (maxY == 0 || nY > maxY)
                    {
                        maxY = nY;
                    }
                    else if (minY == 0 || nY < minY)
                    {
                        minY = nY;
                    }
                }
            }
        }
        #endregion
        yield return true;

        #region Back Segments
        for (int n = 0; n <= backSeg.Count - 1; n++)
        {
            part = backSeg[n];
            part = part.Substring(1);
            sub = part.Substring(0, part.IndexOf(','));
            if (!float.TryParse(sub, out nX))
            {
                Debug.Log("Failed to parse back x. parsed is " + sub);
            }
            else
            {
                if (maxX == 0 || nX > maxX)
                {
                    maxX = nX;
                }
                else if (minX == 0 || nX < minX)
                {
                    minX = nX;
                }

                sub = part;
                sub = sub.Substring(sub.IndexOf(','));
                if (sub[0] == ',')
                {
                    sub = sub.Substring(1);
                }
                sub = sub.Substring(0, sub.IndexOf('$'));
                if (!float.TryParse(sub, out nY))
                {
                    Debug.Log("Failed to parse back y. parsed is " + sub);
                }
                else
                {
                    if (maxY == 0 || nY > maxY)
                    {
                        maxY = nY;
                    }
                    else if (minY == 0 || nY < minY)
                    {
                        minY = nY;
                    }
                }
            }
        }
        #endregion
        yield return true;

        #endregion

        #region Via Sequences
        for (int n = 0; n <= viaSeq.Count - 1; n++)
        {
            part = viaSeq[n];
            part = part.Substring(1);
            sub = part.Substring(0, part.IndexOf(','));
            if (!float.TryParse(sub, out nX))
            {
                Debug.Log("Failed to parse via x. parsed is " + sub);
            }
            else
            {
                if (maxX == 0 || nX > maxX)
                {
                    maxX = nX;
                }
                else if (minX == 0 || nX < minX)
                {
                    minX = nX;
                }

                sub = part.Substring(part.IndexOf(','));
                if (sub[0] == ',')
                {
                    sub = sub.Substring(1);
                }
                sub = sub.Substring(0, sub.IndexOf(','));
                if (!float.TryParse(sub, out nY))
                {
                    Debug.Log("Failed to parse via y. parsed is " + sub);
                }
                else
                {
                    if (maxY == 0 || nY > maxY)
                    {
                        maxY = nY;
                    }
                    else if (minY == 0 || nY < minY)
                    {
                        minY = nY;
                    }
                }
            }
        }
        #endregion
        yield return true;

        #endregion
        
        #region Perform Reductions
        float reduct = 0;
        bool mOd = false, error = false;
        string s = "", g = "";
        float i, o;
        
        if (minX < minY)
        {
            reduct = minX - 1;
        }
        else if (minX >= minY)
        {
            reduct = minY - 1;
        }

        maxX -= reduct;
        maxY -= reduct;

        #region Optimization Sequence

        #region Draw Segments

        #region Front Segments
        u = 0;
        while(u <= frontSeg.Count - 1)
        {
            if (frontSeg[u][0] == 'm')
            {
                mOd = false;
            }
            else
            {
                mOd = true;
            }
            s = frontSeg[u];
            s = s.Substring(1);
            s = s.Replace("$", "");
            g = s.Substring(0, s.IndexOf(','));

            if (g.Contains(','))
            {
                g = g.Replace(",", "");
            }

            if (!float.TryParse(g, out i))
            {
                Debug.Log("Failed to parse draw/move X value. Attempt is " + g + ", from line " + frontSeg[u]);
                error = true;
            }
            else
            {
                g = s.Substring(s.IndexOf(','));

                if (g.Contains(','))
                {
                    g = g.Replace(",", "");
                }

                if (!float.TryParse(g, out o))
                {
                    Debug.Log("Failed to parse draw/move Y value. Attempt is " + g + ", from line " + frontSeg[u]);
                    error = true;
                }
                else
                {
                    i -= reduct;
                    o -= reduct;

                    if (i > 0 && o > 0)
                    {
                        if (!mOd)
                        {
                            s = "m" + i + "," + o + "$";
                            frontSeg[u] = s;
                        }
                        else
                        {
                            s = "d" + i + "," + o + "$";
                            frontSeg[u] = s;
                        }
                    }
                    else
                    {
                        Debug.Log("Error in reducing draw/move x and y. i= " + i + ", o= " + o + ". Found on line: " + frontSeg[u]);
                        error = true;
                    }
                }
            }
            u++;
            yield return true;
        }
        #endregion

        #region Back Segments
        u = 0;
        while (u <= backSeg.Count - 1)
        {
            if (backSeg[u][0] == 'm')
            {
                mOd = false;
            }
            else
            {
                mOd = true;
            }
            s = backSeg[u];
            s = s.Substring(1);
            s = s.Replace("$", "");
            g = s.Substring(0, s.IndexOf(','));

            if (g.Contains(','))
            {
                g = g.Replace(",", "");
            }

            if (!float.TryParse(g, out i))
            {
                Debug.Log("Failed to parse draw/move X value. Attempt is " + g + ", from line " + backSeg[u]);
                error = true;
            }
            else
            {
                g = s.Substring(s.IndexOf(','));

                if (g.Contains(','))
                {
                    g = g.Replace(",", "");
                }

                if (!float.TryParse(g, out o))
                {
                    Debug.Log("Failed to parse draw/move Y value. Attempt is " + g + ", from line " + backSeg[u]);
                    error = true;
                }
                else
                {
                    i -= reduct;
                    o -= reduct;

                    if (i > 0 && o > 0)
                    {
                        if (!mOd)
                        {
                            s = "m" + i + "," + o + "$";
                            backSeg[u] = s;
                        }
                        else
                        {
                            s = "d" + i + "," + o + "$";
                            backSeg[u] = s;
                        }
                    }
                    else
                    {
                        Debug.Log("Error in reducing draw/move x and y. i= " + i + ", o= " + o + ". Found on line: " + backSeg[u]);
                        error = true;
                    }
                }
            }
            u++;
            yield return true;
        }
        #endregion

        #endregion

        #region Fill Sequences
        u = 0;
        while (u <= fillSeq.Count - 1)
        {
            o = 0;
            i = 0;
            s = fillSeq[u];
            s = s.Substring(1);

            g = s.Substring(0, s.IndexOf(','));

            if (g.Contains(','))
            {
                g = g.Replace(",", "");
            }

            if (!float.TryParse(g, out i))
            {
                Debug.Log("Failed to parse fill X value. Attempt is " + g + ", from line " + fillSeq[u]);
                error = true;
            }
            else
            {
                g = s.Substring(s.IndexOf(','));

                if (g[0] == ',')
                {
                    g = g.Substring(1);
                }

                g = g.Substring(0, g.IndexOf(','));

                if (g.Contains(','))
                {
                    g = g.Replace(",", "");
                }

                if (!float.TryParse(g, out o))
                {
                    Debug.Log("Failed to parse fill Y value. Attempt is " + g + ", from line " + fillSeq[u]);
                    error = true;
                }
                else
                {
                    i -= reduct;
                    o -= reduct;

                    if (i > 0 && o > 0)
                    {
                        g = fillSeq[u];
                        g = g.Substring(g.IndexOf(','));

                        if (g[0] == ',')
                        {
                            g = g.Substring(1);
                        }

                        g = g.Substring(g.IndexOf(','));

                        if (g[0] == ',')
                        {
                            g = g.Substring(1);
                        }

                        if (g[0] == ',' && g[1] == ',')
                        {
                            g = g.Substring(1);
                        }

                        s = "f" + i + "," + o + "," + g;
                        fillSeq[u] = s;
                    }
                    else
                    {
                        Debug.Log("Error in reducing fill x and y. i= " + i + ", o= " + o + ". Found on line: " + fillSeq[u]);
                        error = true;
                    }
                }
            }

            u++;
            yield return true;
        }
        #endregion

        #region Vias Sequences
        u = 0;
        while(u <= viaSeq.Count - 1)
        {
            s = viaSeq[u];
            s = s.Substring(1);
            part = s;
            s = s.Substring(0, s.IndexOf(','));

            if (!float.TryParse(s, out i))
            {
                Debug.Log("Failed to parse via X value. Attempt is " + s + ", from line " + viaSeq[u]);
                error = true;
            }
            else
            {
                s = part;
                s = s.Substring(s.IndexOf(','));
                if (s[0] == ',')
                {
                    s = s.Substring(1);
                }
                s = s.Substring(0, s.IndexOf(','));
                if (s.Contains(','))
                {
                    s = s.Replace(",", "");
                }

                if (!float.TryParse(s, out o))
                {
                    Debug.Log("Failed to parse via Y value. Attempt is " + s + ", from line " + viaSeq[u]);
                    error = true;
                }
                else
                {
                    i -= reduct;
                    o -= reduct;

                    if (i > 0 && o > 0)
                    {
                        s = part;
                        s = s.Substring(s.IndexOf(","));
                        
                        if (s[0] == ',')
                        {
                            s = s.Substring(1);
                        }

                        s = s.Substring(s.IndexOf(","));

                        if (s[0] == ',')
                        {
                            part = "V" + i + "," + o + s;
                        }
                        else
                        {
                            part = "V" + i + "," + o + "," + s;
                        }

                        viaSeq[u] = part;
                    }
                    else
                    {
                        Debug.Log("Error in reducing draw/move x and y. i= " + i + ", o= " + o + ". Found on line: " + frontSeg[u]);
                        error = true;
                    }
                }
            }

            u++;
            yield return true;
        }
        #endregion

        #endregion

        #endregion

        #region Convert Backside Values
        float mirV = 0, oPr = 0;

        #region Back Draws
        u = 0;
        while (u <= backSeg.Count - 1)
        {
            if (u == 0)
            {
                Debug.Log("Converting Backside Draws");
            }

            if (backSeg[u][0] == 'm')
            {
                mOd = false;
            }
            else
            {
                mOd = true;
            }
            s = backSeg[u];
            s = s.Substring(1);
            s = s.Replace("$", "");
            g = s.Substring(0, s.IndexOf(','));

            if (g.Contains(','))
            {
                g = g.Replace(",", "");
            }

            if (!float.TryParse(g, out i))
            {
                Debug.Log("Failed to parse draw/move X value. Attempt is " + g + ", from line " + backSeg[u]);
                error = true;
            }
            else
            {
                g = s.Substring(s.IndexOf(','));

                if (g.Contains(','))
                {
                    g = g.Replace(",", "");
                }

                if (!float.TryParse(g, out o))
                {
                    Debug.Log("Failed to parse draw/move Y value. Attempt is " + g + ", from line " + backSeg[u]);
                    error = true;
                }
                else
                {
                    mirV = o - 32.5f;
                    oPr = 32.5f - mirV;

                    if (i > 0 && oPr > 0)
                    {
                        if (!mOd)
                        {
                            s = "m" + i + "," + oPr + "$";
                            backSeg[u] = s;
                        }
                        else
                        {
                            s = "d" + i + "," + oPr + "$";
                            backSeg[u] = s;
                        }
                    }
                    else
                    {
                        Debug.Log("Error in reducing draw/move x and y. i= " + i + ", o= " + o + ". Found on line: " + backSeg[u]);
                        error = true;
                    }
                }
            }
            u++;
            yield return true;
        }
        #endregion

        #region Back Vias
        u = 0;
        List<string> backVias = new List<string>();
        while (u <= viaSeq.Count - 1)
        {
            if (u == 0)
            {
                Debug.Log("Converting Backside Vias");
            }

            s = viaSeq[u];
            part = s;
            s = s.Substring(1);
            s = s.Substring(0, s.IndexOf(','));

            if (!float.TryParse(s, out i))
            {
                Debug.Log("Failed to parse via X value. Attempt is " + s + ", from line " + viaSeq[u]);
                error = true;
            }
            else
            {
                s = part;
                s = s.Substring(s.IndexOf(','));
                if (s.Contains(','))
                {
                    s = s.Substring(1);
                }
                s = s.Substring(0, s.IndexOf(','));

                if (!float.TryParse(s, out o))
                {
                    Debug.Log("Failed to parse via Y value. Attempt is " + s + ", from line " + viaSeq[u]);
                    error = true;
                }
                else
                {
                    mirV = o - 32.5f;
                    oPr = 32.5f - mirV;

                    if (i > 0 && o > 0)
                    {
                        s = part;
                        s = s.Substring(s.IndexOf(","));

                        if (s[0] == ',')
                        {
                            s = s.Substring(1);
                        }

                        s = s.Substring(s.IndexOf(","));

                        if (s[0] == ',')
                        {
                            part = "V" + i + "," + oPr + s;
                        }
                        else
                        {
                            part = "V" + i + "," + oPr + "," + s;
                        }

                        backVias.Add(part);
                    }
                    else
                    {
                        Debug.Log("Error in reducing draw/move x and y. i= " + i + ", o= " + o + ". Found on line: " + backVias[u]);
                        error = true;
                    }
                }
            }

            u++;
            yield return true;
        }
        #endregion

        #endregion

        #region Save Routines

        #region Front Routine
        cmdCumulation = new List<string>();
        cmdCumulation = frontSeg;
        cmdCumulation.AddRange(viaSeq);
        cmdCumulation.AddRange(fillSeq);
        cmdCumulation.Insert(0, "Z$");
        cmdCumulation.Insert(0, "O$");
        part = path + files[fSel];
        part = part.Substring(0, part.IndexOf('.'));
        part = part + "FRONTPROCESSED";
        part = part + ".txt";
        fL = File.Create(part);
        StreamWriter w = new StreamWriter(fL);
        u = 0;
        while(u <= cmdCumulation.Count - 1)
        {
            //w.Write(cmdCumulation[u]);
            w.WriteLine(cmdCumulation[u]);
            w.Flush();
            u++;
        }
        yield return true;
        Debug.Log("Lines written: " + u);
        fL.Close();
        #endregion

        #region Back Routine
        cmdCumulation = new List<string>();
        cmdCumulation = backSeg;
        cmdCumulation.AddRange(backVias);
        cmdCumulation.Insert(0, "Z$");
        cmdCumulation.Insert(0, "O$");
        part = path + files[fSel];
        part = part.Substring(0, part.IndexOf('.'));
        part = part + "BACKPROCESSED";
        part = part + ".txt";
        fL = File.Create(part);
        w = new StreamWriter(fL);
        u = 0;
        while (u <= cmdCumulation.Count - 1)
        {
            //w.Write(cmdCumulation[u]);
            w.WriteLine(cmdCumulation[u]);
            w.Flush();
            u++;
        }
        yield return true;
        Debug.Log("Lines written: " + u);
        fL.Close();
        #endregion

        #endregion

        OnEnable();
        yield return false;
    }
    #endregion
    
    #region Module String Processor
    void moduleProcessor(List<string> s, int phs)
    {
        string p = "", g = "";
        MatchCollection m;
        int k = 0;
        float rootX = 0, rootY = 0, relX = 0, relY = 0, w = 0, h = 0, rot = 0, cmpRot = 0;

        #region Phase 1
        if (phs == 1)
        {
            p = s[0];
            p = p.Substring(p.IndexOf("(at "));
            p = p.Replace("(at ", "");
            p = p.Replace(")", "");
            p = p.Replace(' ', ',');
            p = "FILL" + p;

            if (fillSeq == null)
            {
                fillSeq = new List<string>();
            }
            fillSeq.Add(p);

            for(int n = 1; n <= s.Count - 1; n++)
            {
                p = s[n];
                p = p.Substring(p.IndexOf("(at "));
                p = p.Replace("(at ", "");
                if (!p.Contains("(drill"))
                {
                    p = p.Substring(0, p.IndexOf("(layers") - 1);
                }
                else
                {
                    p = p.Substring(0, p.IndexOf("(drill") - 1);
                }
                p = p.Substring(0, p.LastIndexOf(")"));
                p = p.Replace(") (size ", "|");
                p = p.Replace(")", "|");
                p = p.Replace(' ', ',');
                p = "PART " + p;
                fillSeq.Add(p);
            }
        }
        #endregion

        #region Phase 2
        else if (phs == 2)
        {
            //Debug.Log("Module processing phase 2");
            p = s[0];
            p = p.Substring(4);
            p = p.Substring(0, p.IndexOf(','));
            if (!float.TryParse(p, out rootX))
            {
                Debug.Log("Failed to parse root X. full line is: " + s[0] + ". parsed is: " + p);
            }
            else
            {
                p = s[0];
                p = p.Substring(p.IndexOf(',') + 1);

                #region Parent Rotation Handling
                if (p.Contains(','))
                {
                    p = p.Substring(p.IndexOf(',') + 1);

                    if (p.Contains(','))
                    {
                        p.Replace(",", "");
                    }

                    if (p.Contains('|'))
                    {
                        p = p.Substring(0, p.IndexOf('|'));
                    }

                    if (!float.TryParse(p, out cmpRot))
                    {
                        Debug.Log("Failed to parse parent rotation. full line is: " + s[0] + ". parsed is: " + p);
                    }
                }
                else
                {
                    cmpRot = 0;
                }
                #endregion

                p = s[0];
                p = p.Substring(p.IndexOf(',') + 1);

                if (p.Contains(','))
                {
                    p = p.Substring(0, p.IndexOf(','));
                }

                if (p.Contains(','))
                {
                    p.Replace(",", "");
                }

                if (p.Contains('|'))
                {
                    p = p.Substring(0, p.IndexOf('|'));
                }

                if (!float.TryParse(p, out rootY))
                {
                    Debug.Log("Failed to parse root Y. full line is: " + s[0] + ". parsed is: " + p);
                }
                else
                {
                    //Debug.Log("Root X: " + rootX + " | Root Y: " + rootY + " | list count is: " + s.Count);
                    

                    k = 1;
                    
                    while(k <= s.Count - 1)
                    {
                        p = s[k];
                        p = p.Substring(0, p.IndexOf("|"));
                        //Debug.Log("Relative Position Processing. Part is: " + p);


                        #region Relative X & Y
                        p = p.Replace("PART ", "");
                        g = p.Substring(0, p.IndexOf(","));

                        if (g.Contains(','))
                        {
                            g = g.Replace(",", "");
                        }

                        if (!float.TryParse(g, out relX))
                        {
                            Debug.Log("Failed to parse relative X. root line is: " + s[0] + ". Relative line is: " + s[k] + ". parsed is: " + g);
                        }
                        else
                        {
                            p = p.Substring(p.IndexOf(","));

                            if (p[0] == ',')
                            {
                                p = p.Substring(1);
                            }
                            g = p;

                            if (g.Contains(','))
                            {
                                g = g.Substring(0, g.IndexOf(','));
                            }
                            else
                            {
                                if (g.Contains("|"))
                                {
                                    g = g.Substring(0, g.IndexOf('|'));
                                }
                            }

                            if (g.Contains(','))
                            {
                                g = g.Replace(",", "");
                            }

                            if (!float.TryParse(g, out relY))
                            {
                                Debug.Log("Failed to parse relative Y. root line is: " + s[0] + ". Relative line is: " + s[k] + ". parsed is: " + g);
                            }
                        }
                        #endregion

                        #region Rotation Handling
                        if (p.Contains(','))
                        {
                            p = p.Substring(p.IndexOf(","));

                            if (p[0] == ',')
                            {
                                p = p.Substring(1);
                            }

                            if (!float.TryParse(p, out rot))
                            {
                                Debug.Log("Failed to parse rotation. root line is: " + s[0] + ". Relative line is: " + s[k] + ". parsed is: " + g);
                            }
                        }
                        else
                        {
                            //Debug.Log("Rotation is 0");
                            rot = 0;
                        }
                        #endregion

                        #region Width and Height
                        p = s[k];
                        p = p.Substring(p.IndexOf("|"));

                        if (p.Contains("|"))
                        {
                            p = p.Replace("|", "");
                        }

                        g = p.Substring(0, p.IndexOf(","));

                        if (g.Contains(','))
                        {
                            g = g.Replace(",", "");
                        }

                        if (!float.TryParse(g, out w))
                        {
                            Debug.Log("Failed to parse width. root line is: " + s[0] + ". Relative line is: " + s[k] + ". parsed is: " + g);
                        }
                        else
                        {
                            g = p.Substring(p.IndexOf(","));

                            if (g.Contains(','))
                            {
                                g = g.Replace(",", "");
                            }

                            if (!float.TryParse(g, out h))
                            {
                                Debug.Log("Failed to parse height. root line is: " + s[0] + ". Relative line is: " + s[k] + ". parsed is: " + g);
                            }
                        }
                        #endregion

                        g = "";

                        #region Rotation Handling
                        if (cmpRot == 0)
                        {
                            g = "f" + ((rootX + relX) - (w / 2)) + "," + ((rootY + relY) - (h / 2));
                        }
                        else if (cmpRot == 90)
                        {
                            g = "f" + ((rootX + relY) - (h / 2)) + "," + ((rootY - relX) - (w / 2));
                        }
                        else if (cmpRot == 180)
                        {
                            g = "f" + ((rootX - relX) - (w / 2)) + "," + ((rootY - relY) - (h / 2));
                        }
                        else if (cmpRot == 270)
                        {
                            g = "f" + ((rootX - relY) - (h / 2)) + "," + ((rootY + relX) - (w / 2));
                        }

                        if (rot == 0)
                        {
                            //Debug.Log("Rotation for module is 0");
                            g += "," + w + "," + h + "$";
                            process1.Add(g);
                        }
                        else if (rot == 90)
                        {
                            //Debug.Log("Rotation for module is 90");
                            g += "," + h + "," + w + "$";
                            process1.Add(g);
                        }
                        else if (rot == 180)
                        {
                            //Debug.Log("Rotation for module is 180");
                            g += "," + w + "," + h + "$";
                            process1.Add(g);
                        }
                        else if (rot == 270)
                        {
                            //Debug.Log("Rotation for module is 270");
                            g += "," + h + "," + w + "$";
                            process1.Add(g);
                        }
                        //else
                        //{
                            //Debug.Log("Rotation for module is " + rot);
                            //Debug.Log("rootX: " + rootX + ", rootY: " + rootY + ", relX: " + relX + ", relY: " + relY + ", w: " + w + ", h: " + h + ".");
                        //}
                        #endregion

                        k++;
                    }
                }
            }
        }
        #endregion
    }
    #endregion

    #region Draw String Processor
    void segmentProcessor(string s, int phs)
    {
        string p = "", g = "";

        #region Phase 1
        if (phs == 1)
        {
            p = s;
            p = p.Substring(p.IndexOf("(start "));
            p = p.Replace("(start ", "");
            p = p.Substring(0, p.IndexOf("(width") - 1);
            p = p.Replace(" (end ", "");
            p = p.Replace(' ', ',');
            p = p.Replace(")", "|");
            if (s.Contains("layer F.Cu"))
            {
                if (frontSeg == null)
                {
                    frontSeg = new List<string>();
                }
                frontSeg.Add("FDRAW" + p);
            }
            else if (s.Contains("layer B.Cu"))
            {
                if (backSeg == null)
                {
                    backSeg = new List<string>();
                }
                backSeg.Add("BDRAW" + p);
            }
        }
        #endregion

        #region Phase 2
        else if (phs == 2)
        {
            //Debug.Log("Segment Phase 2 Processing. Input string: " + s);
            #region Front Side Segments
            if (s.Contains("FDRAW"))
            {
                p = s.Replace("FDRAW", "");
                p = p.Substring(0, p.IndexOf('|'));

                if (p.Contains('|'))
                {
                    p = p.Replace("|", "");
                }

                g = "m" + p + "$";
                //Debug.Log("Initial Move post parsing: " + g);
                process1.Add(g);

                p = s.Replace("FDRAW", "");
                p = p.Substring(p.IndexOf('|'));

                if (p.Contains('|'))
                {
                    p = p.Replace("|", "");
                }

                g = "d" + p + "$";
                //Debug.Log("Destination post parsing: " + g);
                process1.Add(g);
            }
            #endregion

            #region Back Side Segments
            else if (s.Contains("BDRAW"))
            {
                p = s.Replace("BDRAW", "");
                p = p.Substring(0, p.IndexOf('|'));

                if (p.Contains('|'))
                {
                    p = p.Replace("|", "");
                }

                g = "m" + p + "$";
                //Debug.Log("Initial Move post parsing: " + g);
                process1.Add(g);

                p = s.Replace("BDRAW", "");
                p = p.Substring(p.IndexOf('|'));

                if (p.Contains('|'))
                {
                    p = p.Replace("|", "");
                }

                g = "d" + p + "$";
                //Debug.Log("Destination post parsing: " + g);
                process1.Add(g);
            }
            #endregion
        }
        #endregion
    }
    #endregion

    #region Via String Processor
    void viaProcessor(string s)
    {
        string p = "", g = "";

        #region Trim Off Ends
        s = s.Substring(0, s.IndexOf("(layers"));
        s = s.Substring(s.IndexOf("(at ") + 4);
        s = "V" + s + "$";
        #endregion

        #region Convert Midsections
        s = s.Replace(") (size ", ",");
        s = s.Replace(") (drill ", ",");
        s = s.Replace(") ", "");
        s = s.Replace(' ', ',');
        #endregion

        #region Add To List
        if (viaSeq == null)
        {
            viaSeq = new List<string>();
        }

        viaSeq.Add(s);
        #endregion
    }
    #endregion
    
    #region Draw Optimization Processor
    IEnumerator drawOptimizer()
    {
        #region Initializers
        List<List<string>> groups = new List<List<string>>();
        List<string> group = new List<string>();
        List<string> rebuild = new List<string>();
        string l1 = "", l2 = "";
        string m = "", d = "";
        int p = 0;
        bool part1 = false, part2 = false, algorithmComplete = false;
        bool duplicates = true, mirror = false;
        #endregion
        
        #region Set Non Draw Commands Aside
        if (rebuild == null)
        {
            rebuild = new List<string>();
        }

        for (int n = 0; n <= cmdCumulation.Count - 1; n++)
        {
            if (cmdCumulation[n][0] == 'f')
            {
                rebuild.Add(cmdCumulation[n]);
            }
        }

        for (int n = 0; n <= rebuild.Count - 1; n++)
        {
            cmdCumulation.Remove(rebuild[n]);
        }
        #endregion

        #region Algorithm
        while (!algorithmComplete)
        {
            #region Obtain Pair
            while (!part1 && !part2 && !algorithmComplete)
            {
                //Debug.Log("Getting Pair");
                group = new List<string>();
                for (int n = 0; n <= cmdCumulation.Count - 1; n++)
                {
                    if (cmdCumulation[n][0] == 'M')
                    {
                        group.Add(cmdCumulation[n]);

                        for (int q = n; q <= cmdCumulation.Count - 1; q++)
                        {
                            if (cmdCumulation[q][0] == 'D')
                            {
                                group.Add(cmdCumulation[q]);
                                break;
                            }
                        }

                        if (group.Count == 2)
                        {
                            cmdCumulation.RemoveRange(n, 2);
                            part1 = true;
                            duplicates = true;
                            mirror = false;
                            part2 = false;
                            m = group[0];
                            d = group[1];
                            break;
                        }
                    }
                    else if (n == cmdCumulation.Count - 1)
                    {
                        part1 = true;
                        part2 = true;
                        mirror = true;
                        duplicates = true;
                        algorithmComplete = true;
                        //Debug.Log("Algorithm Complete");
                    }
                }
                yield return true;
            }
            #endregion

            #region Part 1
            while (part1 && !part2 && !algorithmComplete)
            {

                #region Find Duplicates
                while (duplicates && !mirror)
                {
                    //Debug.Log("Part 1 duplicates");
                    if (cmdCumulation.Count == 0)
                    {
                        //Debug.Log("cmd count error");
                        part1 = true;
                        part2 = true;
                        mirror = true;
                        duplicates = true;
                        algorithmComplete = true;
                        //Debug.Log("Algorithm Complete");
                    }
                    for (int n = 0; n <= cmdCumulation.Count - 1; n++)
                    {
                        if (cmdCumulation[n] == m && n > 0)
                        {
                            l1 = cmdCumulation[n];
                            l2 = cmdCumulation[n + 1];
                            cmdCumulation.RemoveRange(n, 2);
                            l2 = l2.Replace('D', 'M');
                            group.Insert(0, l2);
                            group[1] = group[1].Replace('M', 'D');
                            m = group[0];
                            l1 = "";
                            l2 = "";
                            //Debug.Log("Duplicate found");
                            break;
                        }
                        else if (n == cmdCumulation.Count - 1)
                        {
                            //Debug.Log("Shift to mirror");
                            d = m.Replace('M', 'D');
                            duplicates = false;
                            mirror = true;
                            break;
                        }
                    }
                    yield return true;
                }
                #endregion

                #region Find Mirror
                while (mirror && !duplicates)
                {
                   // Debug.Log("Part 1 mirror");
                    for (int n = 0; n <= cmdCumulation.Count - 1; n++)
                    {
                        if (cmdCumulation[n] == d && n > 0)
                        {
                            l2 = cmdCumulation[n];
                            l1 = cmdCumulation[n - 1];
                            cmdCumulation.RemoveRange(n - 1, 2);
                            group.RemoveAt(0);
                            group.Insert(0, l2);
                            group.Insert(0, l1);
                            d = "";
                            m = group[0];
                            mirror = false;
                            duplicates = true;
                            l1 = "";
                            l2 = "";
                            break;
                        }
                        else if (n == cmdCumulation.Count - 1)
                        {
                            part1 = false;
                            part2 = true;
                            duplicates = true;
                            mirror = false;
                            d = group[group.Count - 1];
                        }
                    }
                    yield return true;
                }
                #endregion
            }
            #endregion

            #region Part 2
            while (!part1 && part2 && !algorithmComplete)
            {
                #region Find Duplicates
                while (duplicates && !mirror)
                {
                    //Debug.Log("Part 2 duplicates");
                    if (cmdCumulation.Count == 0)
                    {
                       // Debug.Log("cmd count error");
                        part1 = true;
                        part2 = true;
                        mirror = false;
                        duplicates = false;
                        algorithmComplete = true;
                        //Debug.Log("Algorithm Complete");
                    }
                    else
                    {
                        for (int n = 0; n <= cmdCumulation.Count - 1; n++)
                        {
                            if (cmdCumulation[n] == d && n > 0)
                            {
                                try
                                {
                                    l2 = cmdCumulation[n];
                                    l1 = cmdCumulation[n - 1];
                                    cmdCumulation.RemoveRange(n - 1, 2);
                                    l1 = l1.Replace('M', 'D');
                                    group.Add(l1);
                                    d = group[group.Count - 1];
                                    l1 = "";
                                    l2 = "";
                                }
                                catch (Exception e)
                                {
                                    //Debug.Log("Exception at part 2 duplicates. n = " + n);
                                    OnEnable();
                                }
                                break;
                            }
                            else if (n == cmdCumulation.Count - 1)
                            {
                                mirror = true;
                                duplicates = false;
                                l1 = "";
                                l2 = "";
                                m = group[group.Count - 1];
                                m = m.Replace('D', 'M');
                            }
                        }
                    }
                    yield return true;
                }
                #endregion

                #region Find Mirror
                while (mirror)
                {
                    //Debug.Log("Part 2 mirror");
                    if (cmdCumulation.Count == 0)
                    {
                        //Debug.Log("cmd count error");
                        part1 = true;
                        part2 = true;
                        mirror = false;
                        duplicates = false;
                        algorithmComplete = true;
                        //Debug.Log("Algorithm Complete");
                    }
                    else
                    {
                        for (int n = 0; n <= cmdCumulation.Count - 1; n++)
                        {
                            if (cmdCumulation[n] == m && n > 0)
                            {
                                l1 = cmdCumulation[n];
                                l2 = cmdCumulation[n + 1];
                                cmdCumulation.RemoveRange(n, 2);
                                group.Add(l2);
                                d = group[group.Count - 1];
                                l1 = "";
                                l2 = "";
                                mirror = false;
                                duplicates = true;
                                m = "";
                                break;
                            }
                            else if (n == cmdCumulation.Count - 1)
                            {
                                if (rebuild == null)
                                {
                                    rebuild = new List<string>();
                                }
                                rebuild.Add("rT/");
                                for (int q = 0; q <= group.Count - 1; q++)
                                {
                                    rebuild.Add(group[q]);
                                }
                                rebuild.Add("eT/");
                                l1 = "";
                                l2 = "";
                                m = "";
                                d = "";
                                group = new List<string>();
                                part1 = false;
                                part2 = false;
                                duplicates = false;
                                mirror = false;
                            }
                        }
                    }
                    yield return true;
                }
                #endregion
            }
            #endregion

            #region Debugging
            if (part1 && part2)
            {
                //Debug.Log("Both parts are true");
            }
            #endregion
        }
        #endregion

        #region Recombine Groups
        //Debug.Log("Recombining");
        for (int n = 0; n <= rebuild.Count - 1; n++)
        {
            cmdCumulation.Add(rebuild[n]);
            yield return true;
        }
        #endregion

        drawDone = true;
        yield return false;
    }
    #endregion

    #region Draw Optimization Processor
    IEnumerator boardDrawOptimizer(boardLayer b)
    {
        #region Initializers
        //List of all non trace commands
        List<string> nonTrace = new List<string>();

        //List of Lists of Initial Trace Command Groups
        List<List<string>> allGroups = new List<List<string>>();

        //List of trace commands for use as current group
        List<string> useGroup = new List<string>();

        //List of trace commands after algorithm
        List<string> finalTraces = new List<string>();

        //Initial List
        List<string> routine = b.layerCommandRoutine;

        //Rebuilt List
        List<string> rebuild = new List<string>();

        //Empty List object for use
        List<string> use = new List<string>();

        //Control Bools
        bool part1 = false, part2 = false, algorithmComplete = false;
        bool duplicates = true, mirror = false;

        
        List<List<string>> groups = new List<List<string>>();
        List<string> group = new List<string>();
        string l1 = "", l2 = "";
        string m = "", d = "";
        int p = 0;
        #endregion
        

        #region Set Non Draw Commands Aside
        if (rebuild == null)
        {
            rebuild = new List<string>();
        }

        for (int n = 0; n <= routine.Count - 1; n++)
        {
            //if line doesn't contain initial trace command, place into nontrace

            if (routine[n][0] != 'M')
            {
                nonTrace.Add(routine[n]);
            }
            else
            {
                //if line contains MX0Y0/, line must be initial offset command. Place into nontrace
                if (routine[n] == "MX0Y0/")
                {
                    nonTrace.Add(routine[n]);
                }
                else
                {
                    //Add initial move line and the subsequent draw line
                    useGroup = new List<string>();
                    useGroup.Add(routine[n]);
                    useGroup.Add(routine[n + 1]);
                    allGroups.Add(useGroup);
                    n += 1;
                }
            }
        }
        #endregion

        //m = "Pre Optimization Traces. Count:" + (allGroups.Count);
        //for (int n = 0; n < allGroups.Count; n++)
        //{
        //    m += "\n" + allGroups[n][0] + "\n" + allGroups[n][1];
        //}
        //Debug.Log(m);

        #region Handle No Trace Count
        if (allGroups.Count <= 0)
        {
            algorithmComplete = true;

            #region Recombine Groups
            rebuild = new List<string>();
            rebuild.AddRange(nonTrace);
            rebuild.AddRange(finalTraces);

            m = "Post Combination Optimization Routine.";
            for (int n = 0; n < rebuild.Count; n++)
            {
                m += "\n" + rebuild[n];
            }
            Debug.Log(m);

            cmdCumulation = rebuild;
            #endregion

            drawDone = true;
            yield return false;
        }
        #endregion

        #region Algorithm
        while (!algorithmComplete)
        {
            #region Grab a Group
            if (!part1 && !part2)
            {
                if (allGroups.Count > 1)
                {
                    useGroup = new List<string>();
                    for(int n = 0; n < allGroups[0].Count; n++)
                    {
                        useGroup.Add(allGroups[0][n]);
                    }
                    allGroups.Remove(allGroups[0]);
                    part1 = true;
                    duplicates = true;
                    mirror = false;
                    //Debug.Log("Grabbed new group. Remaining groups: " + allGroups.Count);
                }
                else
                {
                    use = new List<string>();
                    use.Add("rT/");
                    use.AddRange(allGroups[0]);
                    use.Add("eT/");
                    finalTraces.AddRange(use);
                    algorithmComplete = true;
                }
            }
            #endregion

            #region Part 1
            if (part1 && !part2 && !algorithmComplete)
            {
                //Debug.Log("Starting Part 1");
            }

            while (part1 && !part2 && !algorithmComplete)
            {
                #region Detect Duplicates
                if (duplicates && !mirror)
                {
                    for (int n = 0; n < allGroups.Count; n++)
                    {
                        if (useGroup[0] == allGroups[n][0])
                        {
                            use = new List<string>();
                            use.Add(allGroups[n][1]);
                            use.Add(allGroups[n][0]);
                            for (int q = 1; q < useGroup.Count; q++)
                            {
                                use.Add(useGroup[q]);
                            }

                            use[0] = use[0].Replace('D', 'M');
                            use[1] = use[1].Replace('M', 'D');
                            allGroups.RemoveAt(n);
                            useGroup = new List<string>();
                            useGroup.AddRange(use);

                            if (allGroups.Count <= 0)
                            {
                                use = new List<string>();
                                use.Add("rT/");
                                use.AddRange(useGroup);
                                use.Add("eT/");
                                finalTraces.AddRange(use);
                                algorithmComplete = true;
                            }
                            break;
                        }
                        else if (n >= allGroups.Count - 1)
                        {
                            mirror = true;
                            duplicates = false;
                        }
                    }
                    yield return true;
                }
                #endregion

                #region Detect Mirror
                if (!duplicates && mirror)
                {
                    for (int n = 0; n < allGroups.Count; n++)
                    {
                        if (useGroup[0].Substring(1) == allGroups[n][1].Substring(1))
                        {
                            use = new List<string>();
                            use.Add(allGroups[n][0]);
                            use.Add(allGroups[n][1]);
                            use.Add(useGroup[1]);
                            for (int q = 2; q < useGroup.Count; q++)
                            {
                                use.Add(useGroup[q]);
                            }

                            allGroups.RemoveAt(n);
                            useGroup = new List<string>();
                            useGroup.AddRange(use);
                            if (allGroups.Count <= 0)
                            {
                                use = new List<string>();
                                use.Add("rT/");
                                use.AddRange(useGroup);
                                use.Add("eT/");
                                finalTraces.AddRange(use);
                                algorithmComplete = true;
                            }
                            break;
                        }
                        else if (n >= allGroups.Count - 1)
                        {
                            mirror = false;
                            duplicates = false;
                        }
                    }
                    yield return true;
                }
                #endregion

                if (!duplicates && !mirror)
                {
                    part1 = false;
                    part2 = true;
                    duplicates = true;
                    mirror = false;
                }
                else if (duplicates && mirror)
                {
                    Debug.Log("Part1 Duplicates && Mirror");
                }
                yield return true;
            }
            #endregion

            #region Part 2
            if (!part1 && part2 && !algorithmComplete)
            {
                //Debug.Log("Starting Part 2");
            }

            while (!part1 && part2 && !algorithmComplete)
            {
                #region Detect Duplicates
                if (duplicates && !mirror)
                {
                    for (int n = 0; n < allGroups.Count; n++)
                    {
                        if (useGroup[useGroup.Count - 1].Substring(1) == allGroups[n][0].Substring(1))
                        {
                            use = new List<string>();
                            use.Add(useGroup[0]);
                            for (int q = 1; q < useGroup.Count; q++)
                            {
                                use.Add(useGroup[q]);
                            }
                            use.Add(allGroups[n][1]);

                            allGroups.RemoveAt(n);
                            useGroup = new List<string>();
                            useGroup.AddRange(use);
                            if (allGroups.Count <= 0)
                            {
                                use = new List<string>();
                                use.Add("rT/");
                                use.AddRange(useGroup);
                                use.Add("eT/");
                                finalTraces.AddRange(use);
                                algorithmComplete = true;
                            }
                            break;
                        }
                        else if (n >= allGroups.Count - 1)
                        {
                            mirror = true;
                            duplicates = false;
                        }
                    }
                    yield return true;
                }
                #endregion

                #region Detect Mirror
                if (!duplicates && mirror)
                {
                    for (int n = 0; n < allGroups.Count; n++)
                    {
                        if (useGroup[useGroup.Count - 1] == allGroups[n][1])
                        {
                            use = new List<string>();
                            use.Add(useGroup[0]);
                            for (int q = 1; q < useGroup.Count; q++)
                            {
                                use.Add(useGroup[q]);
                            }
                            use.Add(allGroups[n][0]);

                            use[use.Count - 1] = use[use.Count - 1].Replace('M', 'D');
                            allGroups.RemoveAt(n);
                            useGroup = new List<string>();
                            useGroup.AddRange(use);
                            if (allGroups.Count <= 0)
                            {
                                use = new List<string>();
                                use.Add("rT/");
                                use.AddRange(useGroup);
                                use.Add("eT/");
                                finalTraces.AddRange(use);
                                algorithmComplete = true;
                            }
                            break;
                        }
                        else if (n >= allGroups.Count - 1)
                        {
                            mirror = false;
                            duplicates = false;
                        }
                    }
                    yield return true;
                }
                #endregion

                if (!duplicates && !mirror)
                {
                    part1 = false;
                    part2 = false;
                    duplicates = true;
                    mirror = false;
                    use = new List<string>();
                    use.Add("rT/");
                    use.AddRange(useGroup);
                    use.Add("eT/");
                    finalTraces.AddRange(use);
                }
                else if (duplicates && mirror)
                {
                    Debug.Log("Part2 Duplicates && Mirror");
                }
                yield return true;
            }
            #endregion

            #region Debugging
            if (part1 && part2)
            {
                Debug.Log("Part1 && Part2");
            }
            #endregion

            yield return true;
        }
        #endregion

        #region End Of Operation Procedures
        if (!drawDone)
        {
            int tcnt = 0;
            List<int> rs = new List<int>();
            List<int> es = new List<int>();
            for (int n = 0; n < finalTraces.Count; n++)
            {
                if (finalTraces[n].Contains("rT/"))
                {
                    rs.Add(n);
                }
            }
            for (int n = 1; n < rs.Count; n++)
            {
                es.Add(rs[n] - 1);
            }
            es.Add(finalTraces.Count - 1);

            List<int> diffs = new List<int>();
            for (int n = 0; n < es.Count; n++)
            {
                diffs.Add((es[n] - rs[n]) + 1);
            }

            for (int n = 0; n < diffs.Count; n++)
            {
                tcnt += diffs[n] - 3;
            }
            m = "Final Trace Grouping. Count: " + finalTraces.Count;
            for (int n = 0; n < finalTraces.Count; n++)
            {
                m += "\n" + finalTraces[n];
            }
            //Debug.Log(m);
            //Debug.Log("Apparent Count: " + tcnt);
            //m = "Post Optimization Traces.";
            //  for (int n = 0; n < rebuild.Count; n++)
            // {
            //     m += "\n" + rebuild[n];
            //  }
            //  Debug.Log(m);

            #region Recombine Groups
            rebuild = new List<string>();
            rebuild.AddRange(nonTrace);
            rebuild.AddRange(finalTraces);

           // m = "Post Combination Optimization Routine.";
           // for (int n = 0; n < rebuild.Count; n++)
           // {
           //     m += "\n" + rebuild[n];
            //}
            //Debug.Log(m);

            cmdCumulation = rebuild;
            #endregion
        }
        #endregion

        drawDone = true;
        yield return false;
    }
    #endregion

    #region Board Draw Optimization Processor
    IEnumerator boardDrawOptimizer1(boardLayer b)
    {
        #region Initializers
        List<string> routine = b.layerCommandRoutine;
        List<List<string>> groups = new List<List<string>>();
        List<string> group = new List<string>();
        List<string> rebuild = new List<string>();
        string l1 = "", l2 = "";
        string m = "", d = "";
        int p = 0;
        bool part1 = false, part2 = false, algorithmComplete = false;
        bool duplicates = true, mirror = false;
        #endregion


        #region Set Non Draw Commands Aside
        if (rebuild == null)
        {
            rebuild = new List<string>();
        }

        for (int n = 0; n <= routine.Count - 1; n++)
        {
            if (routine[n][0] != 'M' && routine[n][0] != 'D')
            {
                rebuild.Add(routine[n]);
            }
            else if (routine[n] == "MX0Y0/")
            {
                rebuild.Add(routine[n]);
            }
        }

        for (int n = 0; n <= rebuild.Count - 1; n++)
        {
            routine.Remove(rebuild[n]);
        }
        #endregion

        m = "Pre Optimization Traces. Count:" + (routine.Count / 2);
        for (int n = 0; n < routine.Count; n++)
        {
            m += "\n" + routine[n];
        }
        Debug.Log(m);

        #region Algorithm
        while (!algorithmComplete)
        {
            #region Obtain Pair
            while (!part1 && !part2 && !algorithmComplete)
            {
                //Debug.Log("Getting Pair");
                group = new List<string>();
                for (int n = 0; n <= routine.Count - 1; n++)
                {
                    if (routine[n][0] == 'M')
                    {
                        group.Add(routine[n]);

                        for (int q = n; q <= routine.Count - 1; q++)
                        {
                            if (routine[q][0] == 'D')
                            {
                                group.Add(routine[q]);
                                break;
                            }
                        }

                        if (group.Count == 2)
                        {
                            routine.RemoveRange(n, 2);
                            part1 = true;
                            duplicates = true;
                            mirror = false;
                            part2 = false;
                            m = group[0];
                            d = group[1];
                            break;
                        }
                    }
                    else if (n == routine.Count - 1)
                    {
                        part1 = true;
                        part2 = true;
                        mirror = true;
                        duplicates = true;
                        algorithmComplete = true;
                        //Debug.Log("Algorithm Complete");
                    }
                }
                yield return true;
            }
            #endregion

            #region Part 1
            while (part1 && !part2 && !algorithmComplete)
            {
                #region Find Duplicates
                while (duplicates && !mirror)
                {
                    //Debug.Log("Part 1 duplicates");
                    if (routine.Count == 0)
                    {
                        //Debug.Log("cmd count error");
                        part1 = true;
                        part2 = true;
                        mirror = true;
                        duplicates = true;
                        algorithmComplete = true;
                        //Debug.Log("Algorithm Complete");
                    }
                    for (int n = 0; n <= routine.Count - 1; n++)
                    {
                        if (routine[n] == m && n > 0)
                        {
                            l1 = routine[n];
                            l2 = routine[n + 1];
                            routine.RemoveRange(n, 2);
                            l2 = l2.Replace('D', 'M');
                            group.Insert(0, l2);
                            group[1] = group[1].Replace('M', 'D');
                            m = group[0];
                            l1 = "";
                            l2 = "";
                            //Debug.Log("Duplicate found");
                            break;
                        }
                        else if (n == routine.Count - 1)
                        {
                            //Debug.Log("Shift to mirror");
                            d = m.Replace('M', 'D');
                            duplicates = false;
                            mirror = true;
                            break;
                        }
                    }
                    yield return true;
                }
                #endregion

                #region Find Mirror
                while (mirror && !duplicates)
                {
                    // Debug.Log("Part 1 mirror");
                    for (int n = 0; n <= routine.Count - 1; n++)
                    {
                        if (routine[n] == d && n > 0)
                        {
                            l2 = routine[n];
                            l1 = routine[n - 1];
                            routine.RemoveRange(n - 1, 2);
                            group.RemoveAt(0);
                            group.Insert(0, l2);
                            group.Insert(0, l1);
                            d = "";
                            m = group[0];
                            mirror = false;
                            duplicates = true;
                            l1 = "";
                            l2 = "";
                            break;
                        }
                        else if (n == routine.Count - 1)
                        {
                            part1 = false;
                            part2 = true;
                            duplicates = true;
                            mirror = false;
                            d = group[group.Count - 1];
                        }
                    }
                    yield return true;
                }
                #endregion
            }
            #endregion

            #region Part 2
            while (!part1 && part2 && !algorithmComplete)
            {
                #region Find Duplicates
                while (duplicates && !mirror)
                {
                    //Debug.Log("Part 2 duplicates");
                    if (routine.Count == 0)
                    {
                        // Debug.Log("cmd count error");
                        part1 = true;
                        part2 = true;
                        mirror = false;
                        duplicates = false;
                        algorithmComplete = true;
                        //Debug.Log("Algorithm Complete");
                    }
                    else
                    {
                        for (int n = 0; n <= routine.Count - 1; n++)
                        {
                            if (routine[n] == d && n > 0)
                            {
                                try
                                {
                                    l2 = routine[n];
                                    l1 = routine[n - 1];
                                    routine.RemoveRange(n - 1, 2);
                                    l1 = l1.Replace('M', 'D');
                                    group.Add(l1);
                                    d = group[group.Count - 1];
                                    l1 = "";
                                    l2 = "";
                                }
                                catch (Exception e)
                                {
                                    //Debug.Log("Exception at part 2 duplicates. n = " + n);
                                    OnEnable();
                                }
                                break;
                            }
                            else if (n == routine.Count - 1)
                            {
                                mirror = true;
                                duplicates = false;
                                l1 = "";
                                l2 = "";
                                m = group[group.Count - 1];
                                m = m.Replace('D', 'M');
                            }
                        }
                    }
                    yield return true;
                }
                #endregion

                #region Find Mirror
                while (mirror)
                {
                    //Debug.Log("Part 2 mirror");
                    if (routine.Count == 0)
                    {
                        //Debug.Log("cmd count error");
                        part1 = true;
                        part2 = true;
                        mirror = false;
                        duplicates = false;
                        algorithmComplete = true;
                        //Debug.Log("Algorithm Complete");
                    }
                    else
                    {
                        for (int n = 0; n <= routine.Count - 1; n++)
                        {
                            if (routine[n] == m && n > 0)
                            {
                                l1 = routine[n];
                                l2 = routine[n + 1];
                                routine.RemoveRange(n, 2);
                                group.Add(l2);
                                d = group[group.Count - 1];
                                l1 = "";
                                l2 = "";
                                mirror = false;
                                duplicates = true;
                                m = "";
                                break;
                            }
                            else if (n == routine.Count - 1)
                            {
                                if (rebuild == null)
                                {
                                    rebuild = new List<string>();
                                }
                                rebuild.Add("rT/");
                                for (int q = 0; q <= group.Count - 1; q++)
                                {
                                    rebuild.Add(group[q]);
                                }
                                rebuild.Add("eT/");
                                l1 = "";
                                l2 = "";
                                m = "";
                                d = "";
                                group = new List<string>();
                                part1 = false;
                                part2 = false;
                                duplicates = false;
                                mirror = false;
                            }
                        }
                    }
                    yield return true;
                }
                #endregion
            }
            #endregion

            #region Debugging
            if (part1 && part2)
            {
                //Debug.Log("Both parts are true");
            }
            #endregion
        }
        #endregion

        m = "Post Optimization Traces.";
        for (int n = 0; n < rebuild.Count; n++)
        {
            m += "\n" + rebuild[n];
        }
        Debug.Log(m);

        #region Recombine Groups
        //Debug.Log("Recombining");
        for (int n = 0; n <= rebuild.Count - 1; n++)
        {
            routine.Add(rebuild[n]);
            yield return true;
        }

        m = "Post Combination Optimization Routine.";
        for (int n = 0; n < routine.Count; n++)
        {
            m += "\n" + routine[n];
        }
        Debug.Log(m);

        cmdCumulation = routine;
        #endregion

        drawDone = true;
        yield return false;
    }
    #endregion

    #region BACKUP Draw Optimizer
    IEnumerator drawOptimizer1()
    {
        #region Initializers
        List<List<string>> groups = new List<List<string>>();
        List<string> group = new List<string>();
        List<string> rebuild = new List<string>();
        string l1 = "", l2 = "";
        string m = "", d = "";
        int p = 0;
        bool part1 = false, part2 = false, algorithmComplete = false;
        bool duplicates = true, mirror = false;
        #endregion

        #region Load Routine
        cmdCumulation = new List<string>();
        FileStream fL = File.Open(path + files[fSel], FileMode.Open);
        StreamReader r = new StreamReader(fL);

        while (r.Peek() >= 0)
        {
            cmdCumulation.Add(r.ReadLine());
            yield return true;
        }
        fL.Close();
        #endregion

        #region Set Non Draw Commands Aside
        if (rebuild == null)
        {
            rebuild = new List<string>();
        }

        for(int n = 0; n <= cmdCumulation.Count - 1; n++)
        {
            if (cmdCumulation[n][0] == 'f')
            {
                rebuild.Add(cmdCumulation[n]);
            }
        }

        for(int n = 0; n <= rebuild.Count - 1; n++)
        {
            cmdCumulation.Remove(rebuild[n]);
        }
        #endregion

        #region Algorithm
        while (!algorithmComplete)
        {
            #region Obtain Pair
            while (!part1 && !part2 && !algorithmComplete)
            {
                Debug.Log("Getting Pair");
                group = new List<string>();
                for(int n = 0; n <= cmdCumulation.Count - 1; n++)
                {
                    if (cmdCumulation[n][0] == 'M')
                    {
                        group.Add(cmdCumulation[n]);
                        
                        for(int q = n; q <= cmdCumulation.Count - 1; q++)
                        {
                            if (cmdCumulation[q][0] == 'D')
                            {
                                group.Add(cmdCumulation[q]);
                                break;
                            }
                        }

                        if (group.Count == 2)
                        {
                            cmdCumulation.RemoveRange(n, 2);
                            part1 = true;
                            duplicates = true;
                            mirror = false;
                            part2 = false;
                            m = group[0];
                            d = group[1];
                            break;
                        }
                    }
                    else if (n == cmdCumulation.Count - 1)
                    {
                        part1 = true;
                        part2 = true;
                        mirror = true;
                        duplicates = true;
                        algorithmComplete = true;
                        Debug.Log("Algorithm Complete");
                    }
                }
                yield return true;
            }
            #endregion

            #region Part 1
            while (part1 && !part2 && !algorithmComplete)
            {

                #region Find Duplicates
                while (duplicates && !mirror)
                {
                    Debug.Log("Part 1 duplicates");
                    if (cmdCumulation.Count == 0)
                    {
                        Debug.Log("cmd count error");
                        part1 = true;
                        part2 = true;
                        mirror = true;
                        duplicates = true;
                        algorithmComplete = true;
                        Debug.Log("Algorithm Complete");
                    }
                    for (int n = 0; n <= cmdCumulation.Count - 1; n++)
                    {
                        if (cmdCumulation[n] == m && n > 0)
                        {
                            l1 = cmdCumulation[n];
                            l2 = cmdCumulation[n + 1];
                            cmdCumulation.RemoveRange(n, 2);
                            l2 = l2.Replace('D', 'M');
                            group.Insert(0, l2);
                            group[1] = group[1].Replace('M', 'D');
                            m = group[0];
                            l1 = "";
                            l2 = "";
                            Debug.Log("Duplicate found");
                            break;
                        }
                        else if (n == cmdCumulation.Count - 1)
                        {
                            Debug.Log("Shift to mirror");
                            d = m.Replace('M', 'D');
                            duplicates = false;
                            mirror = true;
                            break;
                        }
                    }
                    yield return true;
                }
                #endregion

                #region Find Mirror
                while(mirror && !duplicates)
                {
                    Debug.Log("Part 1 mirror");
                    for (int n = 0; n <= cmdCumulation.Count - 1; n++)
                    {
                        if (cmdCumulation[n] == d && n > 0)
                        {
                            l2 = cmdCumulation[n];
                            l1 = cmdCumulation[n - 1];
                            cmdCumulation.RemoveRange(n - 1, 2);
                            group.RemoveAt(0);
                            group.Insert(0, l2);
                            group.Insert(0, l1);
                            d = "";
                            m = group[0];
                            mirror = false;
                            duplicates = true;
                            l1 = "";
                            l2 = "";
                            break;
                        }
                        else if (n == cmdCumulation.Count - 1)
                        {
                            part1 = false;
                            part2 = true;
                            duplicates = true;
                            mirror = false;
                            d = group[group.Count - 1];
                        }
                    }
                    yield return true;
                }
                #endregion
            }
            #endregion

            #region Part 2
            while (!part1 && part2 && !algorithmComplete)
            {
                #region Find Duplicates
                while (duplicates && !mirror)
                {
                    Debug.Log("Part 2 duplicates");
                    if (cmdCumulation.Count == 0)
                    {
                        Debug.Log("cmd count error");
                        part1 = true;
                        part2 = true;
                        mirror = false;
                        duplicates = false;
                        algorithmComplete = true;
                        Debug.Log("Algorithm Complete");
                    }
                    else
                    {
                        for (int n = 0; n <= cmdCumulation.Count - 1; n++)
                        {
                            if (cmdCumulation[n] == d && n > 0)
                            {
                                try
                                {
                                    l2 = cmdCumulation[n];
                                    l1 = cmdCumulation[n - 1];
                                    cmdCumulation.RemoveRange(n - 1, 2);
                                    l1 = l1.Replace('M', 'D');
                                    group.Add(l1);
                                    d = group[group.Count - 1];
                                    l1 = "";
                                    l2 = "";
                                }
                                catch (Exception e)
                                {
                                    Debug.Log("Exception at part 2 duplicates. n = " + n);
                                    OnEnable();
                                }
                                break;
                            }
                            else if (n == cmdCumulation.Count - 1)
                            {
                                mirror = true;
                                duplicates = false;
                                l1 = "";
                                l2 = "";
                                m = group[group.Count - 1];
                                m = m.Replace('D', 'M');
                            }
                        }
                    }
                    yield return true;
                }
                #endregion

                #region Find Mirror
                while (mirror)
                {
                    Debug.Log("Part 2 mirror");
                    if (cmdCumulation.Count == 0)
                    {
                        Debug.Log("cmd count error");
                        part1 = true;
                        part2 = true;
                        mirror = false;
                        duplicates = false;
                        algorithmComplete = true;
                        Debug.Log("Algorithm Complete");
                    }
                    else
                    {
                        for (int n = 0; n <= cmdCumulation.Count - 1; n++)
                        {
                            if (cmdCumulation[n] == m && n > 0)
                            {
                                l1 = cmdCumulation[n];
                                l2 = cmdCumulation[n + 1];
                                cmdCumulation.RemoveRange(n, 2);
                                group.Add(l2);
                                d = group[group.Count - 1];
                                l1 = "";
                                l2 = "";
                                mirror = false;
                                duplicates = true;
                                m = "";
                                break;
                            }
                            else if (n == cmdCumulation.Count - 1)
                            {
                                if (rebuild == null)
                                {
                                    rebuild = new List<string>();
                                }
                                rebuild.Add("rT/");
                                for (int q = 0; q <= group.Count - 1; q++)
                                {
                                    rebuild.Add(group[q]);
                                }
                                rebuild.Add("eT/");
                                l1 = "";
                                l2 = "";
                                m = "";
                                d = "";
                                group = new List<string>();
                                part1 = false;
                                part2 = false;
                                duplicates = false;
                                mirror = false;
                            }
                        }
                    }
                    yield return true;
                }
                #endregion
            }
            #endregion

            #region Debugging
            if (part1 && part2)
            {
                Debug.Log("Both parts are true");
            }
            #endregion
        }
        #endregion

        #region Recombine Groups
        Debug.Log("Recombining");
        for (int n = 0; n <= rebuild.Count - 1; n++)
        {
                cmdCumulation.Add(rebuild[n]);
            yield return true;
        }
        #endregion
        
        #region Save Draw Optimized Routine
        fL = File.Create(path + files[fSel]);
        StreamWriter w = new StreamWriter(fL);
        int u = 0;
        while (u <= cmdCumulation.Count - 1)
        {
            //w.Write(cmdCumulation[u]);
            w.WriteLine(cmdCumulation[u]);
            w.Flush();
            u++;
        }
        yield return true;
        Debug.Log("Lines written: " + u);
        fL.Close();
        #endregion

        OnEnable();
        yield return false;
    }
    #endregion

    #region Fill Minor Axis Reduction Processor
    IEnumerator fillMinorReduction(float rd)
    {
        #region Initializers
        string l = "";
        string part = "";
        float x, y, w, h;
        #endregion

        #region Load Routine
        cmdCumulation = new List<string>();
        FileStream fL = File.Open(path + files[fSel], FileMode.Open);
        StreamReader r = new StreamReader(fL);

        while (r.Peek() >= 0)
        {
            cmdCumulation.Add(r.ReadLine());
            yield return true;
        }
        fL.Close();
        #endregion

        #region Algorithm
        int u = 0;
        while (u <= cmdCumulation.Count - 1)
        {
            if (cmdCumulation[u][0] == 'f')
            {
                l = cmdCumulation[u];
            }
            else
            {
                l = "";
            }

            if (l != "")
            {
                l = l.Substring(1, l.Length - 1);
                part = l.Substring(0, l.IndexOf(','));

                if (!float.TryParse(part, out x))
                {
                    Debug.Log("Failed to parse X. Line is: " + l + ", part is: " + part);
                }

                l = l.Substring(l.IndexOf(','));
                part = l.Substring(0, l.IndexOf(','));

                if (!float.TryParse(part, out y))
                {
                    Debug.Log("Failed to parse Y. Line is: " + l + ", part is: " + part);
                }

                l = l.Substring(l.IndexOf(','));
                part = l.Substring(0, l.IndexOf(','));
                float.TryParse(part, out w);

                if (!float.TryParse(part, out w))
                {
                    Debug.Log("Failed to parse W. Line is: " + l + ", part is: " + part);
                }

                l = l.Substring(l.IndexOf(','));
                part = l.Substring(0, l.IndexOf('$'));

                if (!float.TryParse(part, out h))
                {
                    Debug.Log("Failed to parse H. Line is: " + l + ", part is: " + part);
                }

                if (w > h)
                {
                    if (h > .25f)
                    {
                        h -= rd;
                    }
                }
                else
                {
                    if (w > .25f)
                    {
                        w -= rd;
                    }
                }

                l = "f" + x + "," + y + "," + w + "," + h + "$";
                cmdCumulation[u] = l;

                l = "";
                part = "";
            }

            u++;
        }
        #endregion

        OnEnable();

        yield return false;
    }
    #endregion

    #region Pick And Place ENUM
    IEnumerator pickAndplace()
    {
        byte[] bs;
        float cTime = 0;
        float delay = .001f;
        while(pnpRun && pnpPort)
        {
            if (port != null && port.IsOpen)
            {
                #region X Moves
                if (xDone != xMove)
                {
                    if (xMove)
                    {
                        if (axisSlow)
                        {
                            if (xDir)
                            {
                                port.Write("X++/");
                            }
                            else
                            {
                                port.Write("X--/");
                            }
                        }
                        else
                        {
                            if (xDir)
                            {
                                port.Write("X+/");
                            }
                            else
                            {
                                port.Write("X-/");
                            }
                        }
                    }
                    else
                    {
                        port.Write("X!/");
                    }

                    xDone = xMove;
                }
                #endregion

                #region Y Moves
                if (yDone != yMove)
                {
                    if (yMove)
                    {
                        if (axisSlow)
                        {
                            if (yDir)
                            {
                                port.Write("Y++/");
                            }
                            else
                            {
                                port.Write("Y--/");
                            }
                        }
                        else
                        {
                            if (yDir)
                            {
                                port.Write("Y+/");
                            }
                            else
                            {
                                port.Write("Y-/");
                            }
                        }
                    }
                    else
                    {
                        port.Write("Y!/");
                    }

                    yDone = yMove;
                }
                #endregion

                #region Z Moves
                if (zDone != zMove)
                {
                    if (zMove)
                    {
                        if (axisSlow)
                        {
                            if (zDir)
                            {
                                port.Write("Z++/");
                            }
                            else
                            {
                                port.Write("Z--/");
                            }
                        }
                        else
                        {
                            if (zDir)
                            {
                                port.Write("Z+/");
                            }
                            else
                            {
                                port.Write("Z-/");
                            }
                        }
                    }
                    else
                    {
                        port.Write("Z!/");
                    }

                    zDone = zMove;
                }
                #endregion

                #region Vacuum
                if (vacDone != vacSend)
                {
                    //Send Vacuum trigger command
                    port.Write("VAC/");
                    vacDone = vacSend;
                }
                #endregion

                #region Orientation Table Control

                #region Root State Control
                if (tblIncr == 0)
                {
                    if (tblSend != tblDone)
                    {
                        if (tblState == -1)
                        {
                            port.Write("TABLE-1/");
                        }
                        else if (tblState == 0)
                        {
                            port.Write("TABLE0/");
                        }
                        else if (tblState == 1)
                        {
                            port.Write("TABLE1/");
                        }

                        tblDone = tblSend;
                    }
                }
                #endregion

                #region Incremental Control
                else
                {
                    if (Time.realtimeSinceStartup - cTime >= delay)
                    {
                        if (tblIncr > 0)
                        {
                            port.Write("TABLE+/");
                        }
                        else
                        {
                            port.Write("TABLE-/");
                        }
                        cTime = Time.realtimeSinceStartup;
                    }
                }
                #endregion

                #endregion

                #region Read Incoming Messages To Keep Buffer Clean
                if (port.BytesToRead > 0)
                {
                    bs = new byte[port.BytesToRead];

                    for(int n = 0; n < bs.Length; n++)
                    {
                        bs[n] = (byte)port.ReadByte();
                    }
                }
                #endregion
                
            }
            else
            {
                pnpRun = false;
                pnpPort = false;
            }

            yield return true;
        }

        if (port != null)
        {
            if (port.IsOpen)
            {
                port.Close();
                port = null;
            }
        }
        OnEnable();
        yield return false;
    }
    #endregion
    
    #region Wristband Wisualizer ENUM
    IEnumerator wristbandVisualizer(bool calClear, bool cal)
    {
        if (!!cal && !calClear)
        {
            port.Write("K/");
        }
        else
        {
            if (cal && !calClear)
            {
                port.Write("CAL/");
            }
            else if (cal && calClear)
            {
                port.Write("CCAL/");
            }
        }
        byte[] b;
        while(port != null && port.IsOpen)
        {
            if (port.BytesToRead > 0)
            {
                b = new byte[1];
                b[0] = (byte)port.ReadByte();

                if (Convert.ToChar(b[0]) == '/')
                {
                    Debug.Log(msgIn);
                    msgIn = "";
                }
                else
                {
                    msgIn += Convert.ToChar(b[0]);
                }
            }
            yield return true;
        }
        OnEnable();
        yield return false;
    }

    void visualizerSending(string s)
    {
        if (port != null && port.IsOpen)
        {
            port.Write(s);
        }
    }
    #endregion

    #region Image Visualizer ENUM
    IEnumerator imageVisualizer()
    {
        List<byte> dat = null;
        bool rec = false;
        byte[] buf;
        Debug.Log("Image Visualizer Started");

        if (port != null && port.IsOpen)
        {
            port.Write("K/");
            Debug.Log("Sent Initial ACK");
        }
        else
        {
            Debug.Log("Error Sending Initial ACK");
            yield return false;
        }
        

        while(port != null && port.IsOpen)
        {
            #region Receiving Section
            if (port.BytesToRead > 0)
            {
                if (dat.Count == 0 || dat == null)
                {
                    cameraImage = null;
                    dat = new List<byte>();
                }
                buf = new byte[port.BytesToRead];
                port.Read(buf, dat.Count, buf.Length);
                dat.AddRange(buf);

                cameraImage = new Texture2D(Screen.width, (int)(Screen.height * .5));
                cameraImage.LoadRawTextureData(dat.ToArray());
                rec = true;
                Debug.Log("Read " + buf.Length + " bytes."); 
            }
            #endregion

            #region Sending Section
            if (rec || formatChanged)
            {
                if (port != null && port.IsOpen)
                {
                    if (rec)
                    {
                        port.Write("K/");
                        rec = false;
                        dat = new List<byte>();
                    }

                    if (formatChanged)
                    {
                        port.Write(imageFormat + "/");
                        formatChanged = false;
                        Debug.Log("Sent Format Change");
                    }
                }
            }
            #endregion

            yield return true;
        }

        yield return false;
    }
    #endregion

    #region Wristband Data Variable Parser

    #region Parser (String, Bool)
    Quaternion variableParser(string s, bool b)
    {
        string prt = "";
        float p = 0;
        Quaternion qOut;
        
        #region Parse Magnetometer
        wMag = new Vector3();
        prt = s.Substring(s.IndexOf("M") + 1);
        prt = prt.Substring(0, prt.IndexOf("N"));
        if (!float.TryParse(prt, out p))
        {
            Console.WriteLine("Parsing failed mag1. " + prt);
        }
        else
        {
            wMag.x = p;

            prt = s.Substring(s.IndexOf("N") + 1);
            prt = prt.Substring(0, prt.IndexOf("O"));
            if (!float.TryParse(prt, out p))
            {
                Console.WriteLine("Parsing failed mag2. " + prt);
            }
            else
            {
                wMag.y = p;

                prt = s.Substring(s.IndexOf("O") + 1);
                prt = prt.Substring(0, prt.IndexOf("I"));
                if (!float.TryParse(prt, out p))
                {
                    Console.WriteLine("Parsing failed mag3. " + prt);
                }
                else
                {
                    wMag.z = p;
                }
            }
        }
        #endregion

        #region Parse Quaternion
        qOut = new Quaternion();
        prt = s.Substring(s.IndexOf("I") + 1);
        prt = prt.Substring(0, prt.IndexOf("J"));
        if (!float.TryParse(prt, out p))
        {
            Console.WriteLine("Parsing failed quat11. " + prt);
        }
        else
        {
            qOut.x = p;

            prt = s.Substring(s.IndexOf("J") + 1);
            prt = prt.Substring(0, prt.IndexOf("K"));
            if (!float.TryParse(prt, out p))
            {
                Console.WriteLine("Parsing failed quat12. " + prt);
            }
            else
            {
                qOut.y = p;

                prt = s.Substring(s.IndexOf("K") + 1);
                prt = prt.Substring(0, prt.IndexOf("W"));
                if (!float.TryParse(prt, out p))
                {
                    Console.WriteLine("Parsing failed quat21. " + prt);
                }
                else
                {
                    qOut.z = p;

                    prt = s.Substring(s.IndexOf("W") + 1);
                    prt = prt.Substring(0, prt.IndexOf("U"));
                    if (!float.TryParse(prt, out p))
                    {
                        Console.WriteLine("Parsing failed quat22. " + prt);
                    }
                    else
                    {
                        qOut.w = p;
                    }
                }
            }
        }
        #endregion

        #region Parse Touch Readings
        wTouch = new Vector3();
        prt = s.Substring(s.IndexOf("U") + 1);
        prt = prt.Substring(0, prt.IndexOf("S"));
        if (!float.TryParse(prt, out p))
        {
            Console.WriteLine("Parsing failed touch1. " + prt);
        }
        else
        {
            wTouch.x = p;

            prt = s.Substring(s.IndexOf("S") + 1);
            prt = prt.Substring(0, prt.IndexOf("T"));
            if (!float.TryParse(prt, out p))
            {
                Console.WriteLine("Parsing failed touch2. " + prt);
            }
            else
            {
                wTouch.y = p;

                prt = s.Substring(s.IndexOf("T") + 1);
                prt = prt.Substring(0, prt.IndexOf("/" +
                    ""));
                if (!float.TryParse(prt, out p))
                {
                    Console.WriteLine("Parsing failed touch3. " + prt);
                }
                else
                {
                    wTouch.z = p;
                }
            }
        }
        #endregion

        return qOut;
    }
    #endregion

    #region Parser (Byte[])
    void variableParser1(byte[] b)
    {
        int msk = 0;

        #region Get Compass Heading Value
        if (msk > 1000)
        {
            compassHeading = BitConverter.ToSingle(b, 16);
        }
        #endregion

        #region Get Yaw Pitch Roll Values
        if (msk > 1000)
        {
            ypr = new Vector3();
            ypr.x = BitConverter.ToSingle(b, 21);
            ypr.y = BitConverter.ToSingle(b, 26);
            ypr.z = BitConverter.ToSingle(b, 31);
        }
        #endregion

        #region Get Mag Values
            wMag = new Vector3();
            wMag.x = BitConverter.ToSingle(b, 25);
            wMag.y = BitConverter.ToSingle(b, 29);
            wMag.z = BitConverter.ToSingle(b, 33);
            //Debug.Log("wMag Parsed: " + wMag.x + "," + wMag.y + "," + wMag.z);
        #endregion

        #region Get Acc Values
        wAcc = new Vector3();
        wAcc.x = BitConverter.ToSingle(b, 1);
        wAcc.y = BitConverter.ToSingle(b, 5);
        wAcc.z = BitConverter.ToSingle(b, 9);
        //Debug.Log("Acc Parsed: " + wAcc.x + "," + wAcc.y + "," + wAcc.z);
        #endregion

        #region Get Gyro Values
            wGyro = new Vector3();
            wGyro.x = BitConverter.ToSingle(b, 13);
            wGyro.y = BitConverter.ToSingle(b, 17);
            wGyro.z = BitConverter.ToSingle(b, 21);
            //Debug.Log("Gyro Parsed: " + wGyro.x + "," + wGyro.y + "," + wGyro.z);
        #endregion

        #region Get Quat Values
        if (msk > 1000)
        {
            wQuat = new Quaternion();
            wQuat.x = BitConverter.ToSingle(b, 21);
            wQuat.y = BitConverter.ToSingle(b, 26);
            wQuat.z = BitConverter.ToSingle(b, 31);
            wQuat.w = BitConverter.ToSingle(b, 36);
        }
        // Debug.Log("Quat Parsed: " + wQuat.x + "," + wQuat.y + "," + wQuat.z + "," + wQuat.w);
        #endregion

        #region Get Touch Values
        wTouch = new Vector3();
        wTouch.x = BitConverter.ToInt16(b, 37);
        wTouch.y = BitConverter.ToInt16(b, 39);
        wTouch.z = BitConverter.ToInt16(b, 41);
        //Debug.Log("Touch Parsed: " + wTouch.x + "," + wTouch.y + "," + wTouch.z);
        #endregion

        #region Get Time Duration
        if (msk > 1000)
        {
            delT = BitConverter.ToUInt32(b, 42);
            //Debug.Log("Delta T Parsed: " + delT);
        }
        #endregion

        //wAcc = QuaternionExtensions.gravityCompensation(wQuat, pAcc);
        //wQuat = QuaternionExtensions.GetQuaternion(wAcc, wGyro, wMag, wQuat, delT);
        //compassHeading = QuaternionExtensions.calculateHeading(wMag);
        //ypr = QuaternionExtensions.calculateYPR(wQuat);

    }
    #endregion

    #region Parser (Byte[]) - Character Search
    void variableParser(byte[] b)
    {
        char h = 'a';
        ypr = new Vector3();
        wMag = new Vector3();
        wAcc = new Vector3();
        wGyro = new Vector3();
        wQuat = Quaternion.identity;
        wTouch = new Vector3();
        Vector3 pAcc = new Vector3();
        int msk = 0;
        for (int n = 0; n < b.Length; n++)
        {
            h = Convert.ToChar(b[n]);

            #region Compass Heading
            if (msk == 1000)
            {
                if (h == 'C')
                {
                    compassHeading = BitConverter.ToSingle(b, n + 1);
                    n += 3;
                }
            }
            #endregion

            #region YPR
                if (h == 'V')
                {
                    //YAW
                    ypr.y = BitConverter.ToSingle(b, n + 1);
                    n += 3;
                }
                else if (h == 'R')
                {
                    //Pitch
                    ypr.x = BitConverter.ToSingle(b, n + 1);
                    n += 3;
                }
                else if (h == 'L')
                {
                    //Roll
                    ypr.z = BitConverter.ToSingle(b, n + 1);
                    n += 3;
                }
            #endregion

            #region Acc
            if (msk > 0)
            {
                if (h == 'X')
                {
                    pAcc.x = BitConverter.ToSingle(b, n + 1);
                    n += 3;
                }
                else if (h == 'Y')
                {
                    pAcc.y = BitConverter.ToSingle(b, n + 1);
                    n += 3;
                }
                else if (h == 'Z')
                {
                    pAcc.z = BitConverter.ToSingle(b, n + 1);
                    n += 3;
                }
            }
            else
            {
                if (h == 'X')
                {
                    wAcc.x = BitConverter.ToSingle(b, n + 1);
                    n += 3;
                }
                else if (h == 'Y')
                {
                    wAcc.y = BitConverter.ToSingle(b, n + 1);
                    n += 3;
                }
                else if (h == 'Z')
                {
                    wAcc.z = BitConverter.ToSingle(b, n + 1);
                    n += 3;
                }
            }
            #endregion

            #region MAG
            if (msk == 1000)
            {
                if (h == 'D')
                {
                    wMag.x = BitConverter.ToSingle(b, n + 1);
                    n += 3;
                }
                else if (h == 'F')
                {
                    wMag.y = BitConverter.ToSingle(b, n + 1);
                    n += 3;
                }
                else if (h == 'H')
                {
                    wMag.z = BitConverter.ToSingle(b, n + 1);
                    n += 3;
                }
            }
            #endregion

            #region Gyro
            if (msk == 1000)
            {
                if (h == 'A')
                {
                    wGyro.x = BitConverter.ToSingle(b, n + 1);
                    n += 3;
                }
                else if (h == 'B')
                {
                    wGyro.y = BitConverter.ToSingle(b, n + 1);
                    n += 3;
                }
                else if (h == 'G')
                {
                    wGyro.z = BitConverter.ToSingle(b, n + 1);
                    n += 3;
                }
            }
            #endregion

            #region Quat
            if (msk == 1000)
            {
                if (h == 'I')
                {
                    wQuat.x = BitConverter.ToSingle(b, n + 1);
                    n += 3;
                }
                else if (h == 'J')
                {
                    wQuat.y = BitConverter.ToSingle(b, n + 1);
                    n += 3;
                }
                else if (h == 'K')
                {
                    wQuat.z = BitConverter.ToSingle(b, n + 1);
                    n += 3;
                }
                else if (h == 'W')
                {
                    wQuat.w = BitConverter.ToSingle(b, n + 1);
                    n += 3;
                }
            }
            #endregion

            #region Touch
            if (h == 'U')
            {
                wTouch.x = BitConverter.ToInt16(b, n + 1);
                n += 1;
            }
            else if (h == 'S')
            {
                wTouch.y = BitConverter.ToInt16(b, n + 1);
                n += 1;
            }
            else if (h == 'T')
            {
                wTouch.z = BitConverter.ToInt16(b, n + 1);
                n += 1;
            }
            #endregion

            #region Delta Time
            if (msk == 1000)
            {
                if (h == 'M')
                {
                    delT = BitConverter.ToUInt32(b, n + 1);
                    n += 3;
                }
            }
            #endregion
        }

        #region External Calculation Masking
        if (msk > 0)
        {
            wAcc = QuaternionExtensions.gravityCompensation(vizObj.transform.rotation, pAcc);
            if (msk > 1)
            {
                compassHeading = QuaternionExtensions.calculateHeading(wMag);
                if (msk > 2)
                {
                    wQuat = QuaternionExtensions.GetQuaternion(wAcc, wGyro, wMag, wQuat, delT);
                    if (msk > 3)
                    {
                        ypr = QuaternionExtensions.calculateYPR(wQuat);
                    }
                }
            }
        }
        #endregion
    }
    #endregion

    #endregion

    #region Device Locator Extension

    public static void usbDeviceParser()
    {
        SerialPort prt;
        for(int n = 1; n <= 20; n++)
        {
            prt = new SerialPort("COM" + n);

            if (!prt.IsOpen)
            {
                try
                {
                    prt.Open();

                    if (prt.IsOpen)
                    {
                        using (Stream str = prt.BaseStream)
                        {
                            if (str.CanRead && str.CanWrite)
                            {
                                Debug.Log("Can read and write on port " + n);
                            }
                            else if (!str.CanRead && str.CanWrite)
                            {
                                Debug.Log("Cant read on port " + n);
                            }
                            else if (str.CanRead && !str.CanWrite)
                            {
                                Debug.Log("Cant write on port " + n);
                            }
                            else if (!str.CanRead && !str.CanWrite)
                            {
                                Debug.Log("Cant read or write on port " + n);
                            }
                        }
                    }
                }
                catch (Exception e)
                {
                    Debug.Log("Error opening port " + n);
                }
            }
        }
    }

    public static string deviceLocator(string vidPID, int keyLayer)
    {
        string devFound = "";


        List<string> comports = new List<string>();
        //Initial Registry Key
        RegistryKey rk1 = Registry.LocalMachine;
        //Proper Layer 1 Subkey
        RegistryKey rk2 = rk1.OpenSubKey("SYSTEM\\CurrentControlSet\\Enum");
        string temp;
        int blocker = 0;
        //From here, one can iterate through all subkeys to find relevant device data
        foreach (string s3 in rk2.GetSubKeyNames())
        {
            RegistryKey rk3 = rk2.OpenSubKey(s3);
            foreach (string s in rk3.GetSubKeyNames())
            {
                RegistryKey rk4 = rk3.OpenSubKey(s);
                foreach (string s2 in rk4.GetSubKeyNames())
                {
                    RegistryKey rk5 = rk4.OpenSubKey(s2);
                    if (((string)rk5.GetValue("DeviceDesc")).Contains("Bluetooth"))
                    {
                        Debug.Log("Bluetooth subkey located. Descr is: " + (string)rk5.GetValue("DeviceDesc"));
                        string[] vals = rk5.GetValueNames();
                        for (int n = 0; n < vals.Length; n++)
                        {
                            Debug.Log("[" + n + "]:" + vals[n]);
                        }
                        foreach (string s1 in rk5.GetSubKeyNames())
                        {
                            Debug.Log("Layer 1 Subkey Name is: " + s1);
                            if (s1.Contains("Device Parameters") || s1.Contains("Properties"))
                            {
                                try
                                {
                                    RegistryKey rk6 = rk5.OpenSubKey(s1);
                                    vals = rk6.GetValueNames();
                                    for (int n = 0; n < vals.Length; n++)
                                    {
                                        Debug.Log("[" + n + "]:" + vals[n]);
                                    }
                                    foreach (string sn in rk6.GetSubKeyNames())
                                    {
                                        Debug.Log("Layer 2 Subkey name is: " + sn);
                                    }
                                }
                                catch (Exception e)
                                {
                                    Debug.Log("Unable to open subkey " + s1);
                                }
                            }
                        }
                    }
                }
            }
        }

        if (comports.Count > 0)
        {
            foreach (string s in SerialPort.GetPortNames())
            {
                if (comports.Contains(s))
                    return s;
            }
        }

        return devFound;
    }


    public static string deviceLocator1(string vidPID, int keyLayer)
    {
        string devFound = "";


        List<string> comports = new List<string>();
        //Initial Registry Key
        RegistryKey rk1 = Registry.LocalMachine;
        //Proper Layer 1 Subkey
        RegistryKey rk2 = rk1.OpenSubKey("SYSTEM\\CurrentControlSet\\Enum");
        string temp;
        int blocker = 0;
        //From here, one can iterate through all subkeys to find relevant device data
        foreach (string s3 in rk2.GetSubKeyNames())
        {
            RegistryKey rk3 = rk2.OpenSubKey(s3);
            foreach (string s in rk3.GetSubKeyNames())
            {

                if (keyLayer == 3 && s.Contains(vidPID))
                {
                    RegistryKey rk4 = rk3.OpenSubKey(s);
                    foreach (string s2 in rk4.GetSubKeyNames())
                    {
                        Debug.Log("layer2 subkeyname is: " + s2);
                        RegistryKey rk5 = rk4.OpenSubKey(s2);
                        foreach (string s1 in rk5.GetSubKeyNames())
                        {
                            Debug.Log("layer3 subkeyname is: " + s1);
                            RegistryKey rk6 = null;
                            if (!s1.Contains("Properties"))
                            {
                                rk6 = rk5.OpenSubKey(s1);

                                if (s1.Contains("Device Parameters"))
                                {
                                    string[] vals = rk5.GetValueNames();
                                    for (int n = 0; n < vals.Length; n++)
                                    {
                                        Debug.Log("[" + n + "]:" + vals[n]);
                                    }
                                }
                            }
                            else
                            {
                                string[] vals = rk5.GetValueNames();
                                for (int n = 0; n < vals.Length; n++)
                                {
                                    Debug.Log("[" + n + "]:" + vals[n]);
                                }
                                //string dbg = (string)rk5.Get
                            }

                            if (blocker == 10)
                            {
                                string[] vals = rk6.GetValueNames();
                                for (int n = 0; n < vals.Length; n++)
                                {
                                    Debug.Log("[" + n + "]:" + vals[n]);
                                }
                                //string dbg = (string)rk5.GetValue("DeviceDesc");
                                string dbg = "";
                                if (dbg != null && dbg != "")
                                {
                                    Debug.Log("Display Name of device matching supplied VIDPID is: " + dbg);
                                }
                                else if (dbg == null)
                                {
                                    Debug.Log("Display Name returned null");
                                }
                                else if (dbg == "")
                                {
                                    Debug.Log("Display name returned blank");
                                }
                            }
                            else
                            {
                                foreach (string sn in rk6.GetSubKeyNames())
                                {
                                    Debug.Log("layer4 subkeyname is: " + sn);
                                }
                            }
                        }
                    }
                }
                else if (s.Contains("VID") && s.Contains("PID"))
                {
                    RegistryKey rk4 = rk3.OpenSubKey(s);
                    foreach (string s2 in rk4.GetSubKeyNames())
                    {
                        RegistryKey rk5 = rk4.OpenSubKey(s2);
                        if (keyLayer == 5 && (temp = (string)rk5.GetValue("FriendlyName")) != null && temp.Contains("Arduino"))
                        {
                            RegistryKey rk6 = rk5.OpenSubKey("Device Parameters");
                            if (rk6 != null && (temp = (string)rk6.GetValue("PortName")) != null)
                            {
                                comports.Add(temp);
                            }
                        }
                        else if ((temp = (string)rk5.GetValue("FriendlyName")) != null && temp.Contains("Arduino"))
                        {
                            RegistryKey rk6 = rk5.OpenSubKey("Device Parameters");
                            if (rk6 != null && (temp = (string)rk6.GetValue("PortName")) != null)
                            {
                                comports.Add(temp);
                            }
                        }
                    }
                }
            }
        }

        if (comports.Count > 0)
        {
            foreach (string s in SerialPort.GetPortNames())
            {
                if (comports.Contains(s))
                    return s;
            }
        }

        return devFound;
    }
    #endregion

    #region Quaternion Visualizer ENUM

    IEnumerator quatReceive(GameObject g)
    {
        //Quaternion delQuat = Quaternion.identity;
        //Vector3 pAcc = Vector3.zero;
        Rigidbody objRB = vizObj.GetComponent<Rigidbody>();
        byte[] b;
        int tDuration = 0;
        DateTime pTime, cTime;
        cTime = DateTime.Now;
        bool sending = false;
        pTime = cTime;
        tTracking = new List<int>();
        if (port != null && port.IsOpen && port.BaseStream.CanWrite && port.BaseStream.CanRead)
        {
            port.Write("K/");
        }
        else
        {
            OnEnable();
        }
        while (port != null && port.IsOpen)
        {
            while(!sending)
            {
                if (port.BytesToRead > 0)
                {
                    b = new byte[1];
                    b[0] = (byte)port.ReadByte();
                    if (Convert.ToChar(b[0]) == '/')
                    {
                        wQuat = Quaternion.identity;
                        wAcc = new Vector3();
                        wGyro = new Vector3();
                        wMag = new Vector3();
                        wTouch = new Vector3();
                        ypr = new Vector3();
                        compassHeading = 0;
                        delT = 0;
                        VrrktCoroutine.start(quatSend());
                        sending = true;
                        b = new byte[40];
                        worldPos = g.transform.position;
                        worldZero = g.transform.position;
                        locPos = Vector3.zero;
                        locZero = Vector3.zero;
                        velocities = Vector3.zero;
                        displacements = Vector3.zero;
                    }
                }
                yield return true;
            }
            if (port.BytesToRead >= 43)
            {
                b = new byte[43];
                port.Read(b, 0, b.Length);
                variableParser(b);
                cTime = DateTime.Now;
                if (cTime != pTime)
                {
                    tDuration = DateTime.Now.Subtract(pTime).Milliseconds;
                    if (tTracking == null)
                    {
                        tTracking = new List<int>();
                    }
                    tTracking.Add(tDuration);
                    tAVG = 0;
                    for (int n = 0; n < tTracking.Count; n++)
                    {
                        tAVG += tTracking[n];
                    }
                    tAVG = tAVG / tTracking.Count;
                    if (tTracking.Count > 100)
                    {
                        tTracking.RemoveAt(0);
                    }
                }
                pTime = DateTime.Now;
                //Vector3 quatEuler = new Vector3(wQuat.eulerAngles.x, wQuat.eulerAngles.z, wQuat.eulerAngles.y);
                //wQuat = Quaternion.Normalize(wQuat);
                //g.transform.rotation = wQuat;
                //wAcc = QuaternionExtensions.gravityCompensationMethod2(Quaternion.Euler(ypr), wAcc);
                //Vector3 aH = wAcc;
                //wAcc = QuaternionExtensions.gravityCompensation(Quaternion.Euler(ypr), wAcc);
                wAcc = wAcc * 9.8066f;
                wAcc = VectorTransformations.vectorRounding(wAcc, 1);
                float tDur = tDuration / 1000;
               // worldZero = new Vector3(worldPos.x, worldPos.y, worldPos.z);
               // worldPos.x += (velocities.x * tDur) + (.5f * wAcc.y * Mathf.Pow(tDur, 2));
                //worldPos.y += (velocities.y * tDur) + (.5f * wAcc.z * Mathf.Pow(tDur, 2));
                //worldPos.z += (velocities.z * tDur) + (.5f * wAcc.x * Mathf.Pow(tDur, 2));
                //displacements.x = worldPos.x - worldZero.x;
                //displacements.y = worldPos.y - worldZero.y;
                //displacements.z = worldPos.z - worldZero.z;
                
                   // velocities.x = displacements.x / tDur;
                   // velocities.y = displacements.y / tDur;
                   // velocities.z = displacements.z / tDur;
                //Debug.Log("DISPL:" + displacements);
                //Debug.Log("VELOC:" + velocities);
                //g.transform.rotation = Quaternion.Euler(ypr);
                //g.transform.Translate(displacements, Space.World);
                //g.transform.rotation = Quaternion.Euler(ypr);
                try
                {
                    g.transform.SetPositionAndRotation(g.transform.position, Quaternion.Euler(VectorTransformations.vectorRounding(ypr, 2)));
                }
                catch(Exception e)
                {

                }
                //g.transform.rotation = wQuat;
                Repaint();
                
            }
            yield return true;
        }

        OnEnable();
        yield return false;
    }


    IEnumerator quatSend()
    {
        int cTick = 0;
        port.Write("K/");
        Debug.Log("Began Sender");
        while(port != null && port.IsOpen)
        {
            if (cTick >= 1000)
            {
                port.Write("K/");
                //Debug.Log("Sent K/");
                cTick = 0;
            }
            else
            {
                cTick++;
            }
            yield return true;
        }
        yield return false;
    }
    #endregion

    #region Stencil Creation ENUM
    IEnumerator stencilCreator()
    {
        Vector2 boardDims = Vector2.zero;
        Vector2 lineDims = Vector2.zero;
        Vector2 curMMpos = Vector2.zero;
        List<stencilModule> modules = new List<stencilModule>();
        List<stencilPad> modPads = new List<stencilPad>();
        List<bounds> boundings = new List<bounds>();
        List<string> stencilRoutine = new List<string>();
        stencilModule mod = null;
        curLinePos = Vector2.zero;

        #region Load File
        Debug.Log("Loading Routine File");
        cmdCumulation = new List<string>();
        FileStream fL = File.Open(path + files[fSel], FileMode.Open);
        StreamReader r = new StreamReader(fL);
        string lne = "";
        bool withinModule = false;
        int u = 0;
        while (r.Peek() >= 0)
        {
            lne = r.ReadLine();

            if (lne.Length > 0)
            {
                #region Parse Dims
                if (lne.Contains("BW"))
                {
                    string p = lne.Substring(lne.IndexOf("W") + 1);
                    p = p.Substring(0, p.IndexOf("H"));
                    float f = float.Parse(p);
                    boardDims.x = f;
                    p = lne.Substring(lne.IndexOf("H") + 1);
                    p = p.Substring(0, p.IndexOf("/"));
                    f = float.Parse(p);
                    boardDims.y = f;
                    Debug.Log("Parsed Dimensions. W:" + boardDims.x + ", H:" + boardDims.y);
                }
                #endregion

                #region Parse Modules & Pads
                else if (lne.Contains("rOX"))
                {
                    withinModule = true;
                    mod = new stencilModule(lne);
                    Debug.Log("Located Module at line " + u);
                }
                else if (lne.Contains("PX"))
                {
                    if (withinModule)
                    {
                        Debug.Log("Found pad for a module at line " + u);
                        mod.addPad(lne);
                    }
                    else
                    {
                        Debug.LogError("Encountered Pad Outside Of Module!!!!");
                    }
                }
                else if (lne.Contains("eO/"))
                {
                    if (withinModule)
                    {
                        Debug.Log("Found end of module at line " + u);
                        modules.Add(mod);
                        withinModule = false;
                    }
                    else
                    {
                        Debug.LogError("Encountered End of Module Outside Of Module");
                    }
                }
                #endregion
            }
            u++;
            yield return true;
        }
        fL.Close();
        #endregion

        #region Calculate Line Dimensions
        lineDims = new Vector2(boardDims.x * machSperMM.x, boardDims.y * machSperMM.y);
        Debug.Log("Calculate Line Dimensions. W:" + lineDims.x + ", H:" + lineDims.y);
        Debug.Log("Line bit count after delimiter:" + (int)(lineDims.x - 8) + ", = " + (int)((lineDims.x - 8) / 16) + " (16bit) ints available to be encoded. Requiring an offset of " + (int)(16 - ((lineDims.x - 8) % 16)) + " bits at header to round off grid");
        #endregion

        #region Obtain All Pads From Modules
        for (int n = 0; n < modules.Count; n++)
        {
            if (modules[n].pads != null)
            {
                modPads.AddRange(modules[n].pads);
            }
        }
        Debug.Log("Obtained " + modPads.Count + " stencil pads from modules");
        #endregion

        #region Create Bounds
        for(int n = 0; n < modPads.Count; n++)
        {
            boundings.Add(new bounds(modPads[n]));
            Debug.Log("Boundings[" + boundings.Count + "]. xB:" + boundings[boundings.Count - 1].xBound.x + "," + boundings[boundings.Count - 1].xBound.y + "|||| yB:" + boundings[boundings.Count - 1].yBound.x + "," + boundings[boundings.Count - 1].yBound.y);
        }
        //Debug.Log("Created " + boundings.Count + " boundings");
        #endregion

        #region Begin Iteration Sequence
        //Add Sequence Initiator Line
        //Debug.Log("Beginning Iteration Sequence");
        subIteratorLines = new List<List<string>>();
        for(int n = 0; n < lineDims.y; n++)
        {
            cmdCumulation.Add("");
        }
        //Debug.Log("Created " + (lineDims.y / 25) + " Subiterators");
        for(int n = 0; n < lineDims.y / 50; n++)
        {
            Vector2Int rng = new Vector2Int(n * 25, 0);
            if (n < (lineDims.y / 50) - 1)
            {
                rng.y = ((n + 1) * 50) - 1;
            }
            else
            {
                rng.y = (int)lineDims.y;
            }
            subIteratorLines.Add(new List<string>());
            VrrktCoroutine.start(stencilSubiterator(rng, (int)lineDims.x, boundings, n), true);
        }

        //Debug.Log("Awaiting Completion of Subiterators");
        while(curLinePos.y <= lineDims.y)
        {
            Repaint();
            yield return true;
        }
        int bitOffset = (int)((lineDims.x - 8) % 16);
        int intGroupCount = (int)((lineDims.x - 8) / 16);
        if (bitOffset != 0)
        {
            intGroupCount += 1;
        }
        List<BitArray> lba = new List<BitArray>();
        BitArray bitArr = new BitArray(intGroupCount * 16);
        List<byte[]> byteA = new List<byte[]>();
        //cmdCumulation = new List<string>();
        byte[] bo = BitConverter.GetBytes(bitOffset);
        Array.Reverse(bo);
        byte[] bb = new byte[5 + bo.Length];
        bb[0] = (byte)'D';
        bb[1] = (byte)'r';
        bb[2] = (byte)'a';
        bb[3] = (byte)'w';
        for(int n = 4; n < bo.Length; n++)
        {
            bb[n] = bo[n - 4];
        }
        bb[bb.Length - 1] = (byte)'/';
        byteA.Add(bb);
        //cmdCumulation.Add("Draw" + bitOffset + "/");
        //Debug.Log("Subiterator Lines Count:" + subIteratorLines.Count);
        for(int n = 0; n < subIteratorLines.Count; n++)
        {
                for (int j = 0; j < subIteratorLines[n].Count; j++)
            {
                if (subIteratorLines[n].Count == 1)
                {
                    byte[] ba = new byte[1];
                    ba[0] = (byte)'/';
                    byteA.Add(ba);
                }
                else
                {
                    bitArr = new BitArray((intGroupCount * 16));
                    for (int v = 0; v < subIteratorLines[n][j].Length; v++)
                    {
                        if (subIteratorLines[n][j][v] == '0')
                        {
                            bitArr.Set(bitOffset + v, false);
                        }
                        else if (subIteratorLines[n][j][v] == '1')
                        {
                            bitArr.Set(bitOffset + v, true);
                        }
                    }
                    byte[] ia = new byte[(intGroupCount * 2) + 1];
                    BitArray bitsub;
                    for (int v = 0; v < bitArr.Count / 8; v += 8)
                    {
                        bitsub = new BitArray(8);
                        for (int e = 0; e < 8; e++)
                        {
                            bitsub.Set(e, bitArr.Get(v + e));
                        }
                        bitsub.CopyTo(ia, v / 8);
                    }
                    ia[ia.Length - 1] = (byte)'/';
                    byteA.Add(ia);
                }

            }
        }
        byte[] be = { (byte)'E', (byte)'N', (byte)'D', (byte)'/' };
        byteA.Add(be);
        //cmdCumulation.Add("END/");
        #endregion

        #region Save Routine To File
        //Debug.Log("Saving Routine. " + cmdCumulation.Count + " lines");
        string newPath = path + files[fSel];
        if (newPath.Contains("FRONT"))
        {
            newPath = newPath.Replace("FRONT", "STENCILFRONT");
        }
        else if (newPath.Contains("BACK"))
        {
            newPath = newPath.Replace("BACK", "STENCILBACK");
        }

        fL = File.Create(newPath);
        StreamWriter w = new StreamWriter(fL);
        u = 0;
        while (u < byteA.Count)
        {
            int q = 0;
            while(q < byteA[u].Length)
            {
                w.Write(byteA[u][q]);
                q++;
                yield return true;
            }
            u++;
            yield return true;
        }
        fL.Close();

        Debug.Log("Wrote " + byteA.Count + " Lines");
        #endregion

        OnEnable();
        yield return false;
    }
    #endregion

    #region Stencil Creation SubIterator ENUM
    IEnumerator stencilSubiterator(Vector2Int range, int xLim, List<bounds> boundings, int id)
    {
        Debug.Log("Subiterator. Range: {0 - " + (xLim / machSperMM.x) + ", " + (range.x / machSperMM.y) + "-" + (range.y / machSperMM.y) + "}. IDX:" + id);
        Vector2Int curPos = new Vector2Int(0, range.x);
        Vector2 mmPos;
        List<string> subLine = new List<string>();
        string ln = "";

        while (curPos.y <= range.y)
        {
            ln = "";
            while(curPos.x <= xLim)
            {
                mmPos.x = curPos.x  / machSperMM.x;
                mmPos.y = curPos.y  / machSperMM.y;

                for (int n = 0; n < boundings.Count; n++)
                {
                    if (boundings[n].testWithin(mmPos))
                    {
                        ln += "1";
                        break;
                    }
                    else
                    {
                        if (n == boundings.Count - 1)
                        {
                            ln += "0";
                        }
                    }
                }

                if (curPos.x == xLim)
                {
                    if (!ln.Contains("1"))
                    {
                        ln = "/";
                    }
                    else
                    {
                        ln += "/";
                    }
                    subIteratorLines[id].Add(ln);
                }
                curPos.x++;
                yield return true;
            }
            Repaint();
            curPos.x = 0;
            curPos.y++;
            curLinePos.y++;
            yield return true;
        }
        yield return false;
    }
    #endregion

    #region Bitmap Converter ENUM
    IEnumerator bitmapConverter()
    {
        #region Load Image
        bmp = new Texture2D(bW, bH);
        FileStream f = File.Open(path + files[fSel], FileMode.Open);
        StreamReader sr = new StreamReader(f);
        byte[] bs = new byte[f.Length];
        int u = 0;
        while(u < bs.Length)
        {
            bs[u] = (byte)sr.Read();
            u++;
            yield return true;
        }
        bmp.LoadImage(bs);
        #endregion

        string part = "B";
        codeOut = "";
        UnityEngine.Color col;
        #region Construct Code
        u = 0;
        int j = 0;
        while(j < bH)
        {
            while(u < bW)
            {
                col = bmp.GetPixel(u, j);
                if (col == UnityEngine.Color.white)
                {
                    part += "0";
                }
                else
                {
                    part += "1";
                }

                if (part.Length == 9)
                {
                    codeOut += part + ", ";
                    Debug.Log("PART: " + part);
                    part = "B";
                    Repaint();
                }
                u++;
                yield return true;
            }
            u = 0;
            codeOut += "\n";
            j++;
            Repaint();
            yield return true;
        }

        if (codeOut.LastIndexOf(',') > codeOut.LastIndexOf('B'))
        {
            codeOut = codeOut.Substring(0, codeOut.LastIndexOf(','));
        }
        Repaint();
        #endregion

        Debug.Log("Finished Conversion.");
        Debug.Log(codeOut);

        yield return false;
    }
    #endregion

    #region Raster Controller ENUM
    IEnumerator rasterController()
    {
        Vector2 borderDims = Vector2.zero;
        string pt = "";
        string s = "";
        float f = 0;
        List<string> cPP = new List<string>();
        bool borderFound = false;
        List<string> finalized = new List<string>();
         cPos = Vector2.zero;
        Vector2 lPos = Vector2.zero;
        if (objs != null && objs.Count > 0)
        {
            foreach(GameObject o in objs)
            {
                DestroyImmediate(o);
            }
        }

        #region PreParse Strings
        for (int n = 0; n < cmdCumulation.Count; n++)
        {
            if (n >= 1 && n <= 4)
            {
                finalized.Add(cmdCumulation[n]);
            }
            else if (n == 5)
            {
                finalized.Add("RASTER/");
            }
            else
            {
                if (!borderFound)
                {
                    if (cmdCumulation[n].Contains("BW"))
                    {
                        pt = cmdCumulation[n];
                        s = pt;

                        pt = pt.Substring(pt.IndexOf("W") + 1);
                        pt = pt.Substring(0, pt.IndexOf("H"));
                        
                        if (!float.TryParse(pt, out f))
                        {
                            Debug.Log("Failed to parse border width. String is " + pt);
                        }
                        else
                        {
                            borderDims.x = f;
                            s = s.Substring(s.IndexOf("H") + 1);
                            pt = s.Substring(0, s.IndexOf("/"));

                            if (!float.TryParse(pt, out f))
                            {
                                Debug.Log("Failed to parse border height. String is " + pt);
                            }
                            else
                            {
                                borderDims.y = f;
                                borderFound = true;
                                canvasObj.GetComponent<RectTransform>().sizeDelta = borderDims;
                                canvasObj.GetComponent<RectTransform>().SetPositionAndRotation(new Vector3(borderDims.x * .5f, borderDims.y * .5f, 0), Quaternion.Euler(0, 0, 0));
                            }
                        }
                    }
                }

                if (cmdCumulation[n].Contains("rOX") || cmdCumulation[n].Contains("eO") || cmdCumulation[n].Contains("pX") || cmdCumulation[n].Contains("PX") || cmdCumulation[n].Contains("VX"))
                {
                    cPP.Add(cmdCumulation[n]);
                }
            }
            yield return true;
        }
        cmdCumulation = cPP;
        yield return true;
        #endregion

        #region Parse Modules, Pads, & Vias
        rasterParent rP = null;
        List<rasterParent> parents = new List<rasterParent>();
        rasterBoundaries rB = null;
        List<rasterBoundaries> boundaries = new List<rasterBoundaries>();
        GameObject pObj = null;
        GameObject cObj = null;
        objs = new List<GameObject>();
        List<GameObject> pars = new List<GameObject>();

        for(int n = 0; n < cmdCumulation.Count; n++)
        {
            if (cmdCumulation[n].Contains("VX"))
            {
                rB = new rasterBoundaries(cmdCumulation[n]);
                boundaries.Add(rB);
                cObj = Instantiate(circleObj, canvasObj.GetComponent<RectTransform>(), false);
                cObj.GetComponent<RectTransform>().SetPositionAndRotation(new Vector3(rB.p.x, rB.p.y, 0), Quaternion.Euler(0, 0, 0));
                cObj.GetComponent<CircleCollider2D>().radius = rB.dimensions.x;
                //Debug.Log("VIA POSITION:" + cObj.GetComponent<RectTransform>().position);
                objs.Add(cObj);
            }
            else
            {
                if (cmdCumulation[n].Contains("rOX"))
                {
                    rP = new rasterParent(cmdCumulation[n]);
                    pObj = Instantiate(parentObj, canvasObj.GetComponent<RectTransform>(), false);
                    pObj.GetComponent<RectTransform>().SetPositionAndRotation(new Vector3(rP.p.x, rP.p.y, 0), Quaternion.Euler(0, 0, 0));
                    pObj.GetComponent<RectTransform>().rotation = Quaternion.Euler(0, 0, rP.rot);
                    pars.Add(pObj);
                }
                else if (cmdCumulation[n].Contains("eO"))
                {
                    objs.Add(pObj);
                    parents.Add(rP);
                    rP = null;
                    pObj = null;
                }
                else if (rP != null)
                {
                    rB = new rasterBoundaries(cmdCumulation[n]);
                    if (!rB.type)
                    {
                        cObj = Instantiate(quadObj, pObj.GetComponent<RectTransform>(), false);
                        cObj.GetComponent<RectTransform>().SetPositionAndRotation(new Vector3(pObj.GetComponent<RectTransform>().position.x + rB.p.x, pObj.GetComponent<RectTransform>().position.y + rB.p.y, 0), Quaternion.Euler(0, 0, 0));
                        pObj.GetComponent<RectTransform>().Rotate(0, 0, rB.rot - rP.rot, Space.World);
                        cObj.GetComponent<BoxCollider2D>().size = rB.dimensions;
                    }
                    else
                    {
                        cObj = Instantiate(circleObj, pObj.GetComponent<RectTransform>(), false);
                        cObj.GetComponent<RectTransform>().SetPositionAndRotation(new Vector3(pObj.GetComponent<RectTransform>().position.x + rB.p.x, pObj.GetComponent<RectTransform>().position.y + rB.p.y, 0), Quaternion.Euler(0, 0, 0));
                        cObj.GetComponent<CircleCollider2D>().radius = rB.dimensions.x;
                    }
                    objs.Add(cObj);
                    boundaries.Add(rB);
                }
            }
            yield return true;
        }
        
        for(int n = 0; n < pars.Count; n++)
        {
            pars[n].GetComponent<RectTransform>().Rotate(0, 0, 0 - parents[n].rot);
        }
        #endregion

        #region Begin Creating Commands
        //TRUE => laser on. FALSE => laser off.
        //TRUE => moving right. FALSE => moving left
        rDir = true;
        RaycastHit2D hit;
        bool inColl = false;
        Vector2 inCollP = Vector2.zero;

        while (cPos.y <= borderDims.y)
        {
            #region Forward Motion
            if (rDir)
            {
                //Debug.Log("Forward Cycle");

                #region Handle At Edge
                if (cPos.x >= borderDims.x)
                {

                    finalized.Add("mX" + (borderDims.x - inCollP.x) + "Y0/");
                    finalized.Add("dX0Y.1/");
                    cPos.x = borderDims.x;
                    cPos.y += .1f;
                    inCollP = cPos;
                    inColl = false;
                    rDir = false;
                    Debug.Log("Reached Edge FORWARD");
                }
                #endregion

                else
                {
                    hit = Physics2D.Raycast(cPos, Vector2.right, borderDims.x * 2);

                    #region Handle No Hit
                    if (hit.collider == null)
                    {
                        #region No Hit From Edge
                        if (cPos.x == 0)
                        {
                            if (!inColl)
                            {
                                finalized.Add("dX" + borderDims.x + "Y0/");
                                finalized.Add("dX0Y.1/");
                                cPos.x = borderDims.x;
                                cPos.y += .1f;
                                rDir = false;
                                Debug.Log("No Hit While Out Of Collider FORWARD");
                            }
                            else
                            {
                                finalized.Add("mX" + (cPos.x - inCollP.x) + "Y0/");
                                inColl = false;
                                finalized.Add("dX" + (borderDims.x - cPos.x) + "Y0/");
                                finalized.Add("dX0Y.1/");
                                cPos.x = borderDims.x;
                                cPos.y += .1f;
                                rDir = false;
                                Debug.Log("No Hit While In Collider FORWARD");
                            }
                        }
                        #endregion
                    }
                    #endregion

                    #region Handle Hit
                    else
                    {
                        #region Handle Hit While Drawing
                        if (!inColl)
                        {
                            finalized.Add("dX" + (hit.point.x - cPos.x) + "Y0/");
                            cPos.x = hit.point.x + .1f;
                            inColl = true;
                            inCollP = cPos;
                            Debug.Log("Hit while out of collider FORWARD");
                        }
                        #endregion

                        #region Handle Hit While In Collider
                        else
                        {
                            if (hit.point.x == cPos.x)
                            {
                                cPos.x += .1f;
                                Debug.Log("Hit while in collider IDENTICAL FORWARD");
                            }
                            else
                            {
                                finalized.Add("mX" + ((cPos.x - .1f) - inCollP.x) + "Y0/");
                                finalized.Add("dX" + (hit.point.x - cPos.x) + "Y0/");
                                cPos.x = hit.point.x + .1f;
                                inCollP = cPos;
                                Debug.Log("Hit while in collider FORWARD");
                            }
                        }
                        #endregion
                    }
                    #endregion
                }
                Repaint();
                yield return true;
            }
            #endregion
            
            #region  Reverse Motion
            else
            {
                //Debug.Log("Reverse Cycle");

                #region Handle At Edge
                if (cPos.x <= 0)
                {

                    finalized.Add("mX" + (0 - inCollP.x) + "Y0/");
                    finalized.Add("dX0Y.1/");
                    cPos.y += .1f;
                    cPos.x = 0;
                    inCollP = cPos;
                    inColl = false;
                    rDir = true;
                    Debug.Log("Reached Edge BACKWARD");
                }
                #endregion

                else
                {
                    hit = Physics2D.Raycast(cPos, Vector2.left, borderDims.x * 2);

                    #region Handle No Hit
                    if (hit.collider == null)
                    {
                        #region No Hit From Edge
                        if (cPos.x == borderDims.x)
                        {
                            if (!inColl)
                            {
                                finalized.Add("dX-" + borderDims.x + "Y0/");
                                finalized.Add("dX0Y.1/");
                                cPos.x = 0;
                                cPos.y += .1f;
                                rDir = true;
                                Debug.Log("No Hit While out of Collider FORWARD");
                            }
                            else
                            {
                                finalized.Add("mX" + (inCollP.x - cPos.x) + "Y0/");
                                inColl = false;
                                finalized.Add("dX" + (0 - cPos.x) + "Y0/");
                                finalized.Add("dX0Y.1/");
                                cPos.x = 0;
                                cPos.y += .1f;
                                rDir = true;
                                Debug.Log("No Hit While In Collider BACKWARD");
                            }
                        }
                        #endregion
                    }
                    #endregion

                    #region Handle Hit
                    else
                    {
                        #region Handle Hit While Drawing
                        if (!inColl)
                        {
                            finalized.Add("dX" + (cPos.x - hit.point.x) + "Y0/");
                            cPos.x = hit.point.x - .1f;
                            inColl = true;
                            inCollP = cPos;
                            Debug.Log("Hit while out of collider BACKWARD");
                        }
                        #endregion

                        #region Handle Hit While In Collider
                        else
                        {
                            if (hit.point.x == cPos.x)
                            {
                                cPos.x -= .1f;
                                Debug.Log("Hit while in collider IDENTICAL BACKWARD");
                            }
                            else
                            {
                                finalized.Add("mX" + (inCollP.x - (cPos.x + .1f)) + "Y0/");
                                finalized.Add("dX" + (cPos.x - hit.point.x) + "Y0/");
                                cPos.x = hit.point.x - .1f;
                                inCollP = cPos;
                                Debug.Log("Hit while in collider BACKWARD");
                            }
                        }
                        #endregion
                    }
                    #endregion
                }
                Repaint();
                yield return true;
            }
            #endregion
        }
        #endregion

        #region Save Raster Routine
        finalized.Add("END/");
        string pathBase = path + files[fSel];
        pathBase = pathBase.Substring(0, pathBase.IndexOf('.'));
        pt = pathBase + "RASTER";
        pt = pt + ".txt";

        FileStream fL = File.Create(pt);
        StreamWriter w = new StreamWriter(fL);
        
        w = new StreamWriter(fL);
        int u = 0;
        while (u <= finalized.Count - 1)
        {
            //w.Write(cmdCumulation[u]);
            if (finalized[u] != "\n" && finalized[u] != "mX0Y0/" && finalized[u] != "dX0Y0/")
            {
                w.WriteLine(finalized[u]);
                w.Flush();
            }
            u++;
        }
        yield return true;
        Debug.Log("Lines written: " + u + "\n" + "@ " + pt);
        fL.Close();
        #endregion

        foreach(GameObject g in objs)
        {
            DestroyImmediate(g);
        }

        OnEnable();
        yield return false;
    }
    #endregion

    #region Command Routine Rearrangement
    List<string> rearrangeRoutine(List<string> l)
    {
        List<string> p1 = new List<string>();
        List<string> p2 = new List<string>();

        for (int n = 0; n < l.Count; n++)
        {
            if (l[n][0] == 'V' || l[n][0] == 'W' || l[n] == "DRILLS/" || l[n] == "VIAS/")
            {
                p2.Add(l[n]);
            }
            else
            {
                p1.Add(l[n]);
            }
        }

        for(int n = 0; n < p2.Count; n++)
        {
            p1.Add(p2[n]);
        }

        return p1;
    }
    #endregion

    #region Random Chart Creation
    IEnumerator createRandomChart()
    {
        int chartLength = 64;
        int laneCount = 5;
        List<string> chartOut = new List<string>();

        for(int n = 0; n < chartLength; n++)
        {
            string chartLine = "";
            for (int j = 0; j < laneCount; j++)
            {
                int rnd = UnityEngine.Random.Range(0, 20);

                if (rnd % 2 == 0)
                {
                    chartLine += "1";
                }
                else
                {
                    chartLine += "0";
                }
                Repaint();
                yield return true;
            }
            chartOut.Add(chartLine);
            Repaint();
            yield return true;
        }

        string chartDebug = "";
        for(int n = 0; n < chartOut.Count; n++)
        {
            chartDebug += chartOut[n];
            if (n < chartOut.Count - 1)
            {
                chartDebug += "\n";
            }
            Repaint();
            yield return true;
        }
        Debug.Log(chartDebug);

        yield return false;
    }
    #endregion

    #region Controller Control ENUM
    IEnumerator controllerControl()
    {
        while(!endController)
        {
            #region XYZ
            xAxis = Input.GetAxis("7th Axis");

            yAxis = Input.GetAxis("8th Axis");

            if (Input.GetKeyDown("joystick button 6"))
            {
                zAxis = 1;
            }
            else if (Input.GetKeyDown("joystick button 4"))
            {
                zAxis = -1;
            }
            else
            {
                zAxis = 0;
            }
            #endregion

            #region Laser & Spindle
            if (Input.GetKeyDown("joystick button 10"))
            {
                laserOn = !laserOn;
            }
            laserBtn = Input.GetKeyDown("joystick button 10");

            lAxis = Input.GetAxis("X Axis");

            if (Input.GetKeyDown("joystick button 11"))
            {
                spindleOn = !spindleOn;
            }
            spindleBtn = Input.GetKeyDown("joystick button 11");
            #endregion

            #region Other Inputs
            if (Input.GetKeyDown("joystick button 5"))
            {
                stepLength += .01f;
            }
            else if (Input.GetKeyDown("joystick button 7"))
            {
                stepLength -= .01f;

                if (stepLength <= 0)
                {
                    stepLength = .01f;
                }
            }

            homeBtn = Input.GetKeyDown("joystick button 9");

            continuousJog = Input.GetKeyDown("joystick button 1");
            #endregion

            Repaint();
            yield return true;
        }

        yield return false;
    }
    #endregion
}
#endregion

#region Custom Class Objects

#region Version 1 Board Classes

#region Edge Cuts Class
[System.Serializable]
public class edgeCuts
{
    public Vector2Float start;
    public Vector2Float end;
    public Vector2Float dims;
    public Vector2Float minValues;
    public Vector2Float maxValues;
    public string cutLines;

    #region XTOR
    public edgeCuts()
    {
        start = new Vector2Float(0, 0);
        end = new Vector2Float(0, 0);
        dims = new Vector2Float(0, 0);
        minValues = new Vector2Float(0, 0);
        cutLines = "";
    }
    #endregion
    

    #region Processing Function
    public void process(string s, out decimal minX, out decimal minY, out decimal w, out decimal h)
    {
        start = new Vector2Float(0, 0);
        end = new Vector2Float(0, 0);
        dims = new Vector2Float(0, 0);
        minValues = new Vector2Float(0, 0);
        string part = "";
        decimal e = 0, mX = 0, mY = 0;
        s = s.Substring(s.IndexOf("(start ") + 7);
        s = s.Substring(0, s.IndexOf(" (layer "));
        part = s.Substring(0, s.IndexOf(" "));

        if (!decimal.TryParse(part, out e))
        {
            Debug.Log("Error parsing start X. Part:" + part);
            w = -1000;
            h = -1000;
            minX = -1000;
            minY = -1000;
        }
        else
        {
            start.x = e;
            s = s.Substring(s.IndexOf(" ") + 1);
            part = s.Substring(0, s.IndexOf(") (end "));
            if (!decimal.TryParse(part, out e))
            {
                Debug.Log("Error parsing start Y. Part:" + part);
                w = -1000;
                h = -1000;
                minX = -1000;
                minY = -1000;
            }
            else
            {
                start.y = e;
                s = s.Substring(s.IndexOf("("));
                s = s.Replace("(end ", "");
                part = s.Substring(0, s.IndexOf(" "));
                if (!decimal.TryParse(part, out e))
                {
                    Debug.Log("Error parsing end X. Part:" + part);
                    w = -1000;
                    h = -1000;
                    minX = -1000;
                    minY = -1000;
                }
                else
                {
                    end.x = e;
                    s = s.Substring(s.IndexOf(" "));
                    part = s.Substring(0, s.IndexOf(")"));
                    if (!decimal.TryParse(part, out e))
                    {
                        Debug.Log("Error parsing end Y. Part:" + part);
                        w = -1000;
                        h = -1000;
                        minX = -1000;
                        minY = -1000;
                    }
                    else
                    {
                        end.y = e;

                        if (start.x < end.x)
                        {
                            mX = start.x;
                        }
                        else
                        {
                            mX = end.x;
                        }

                        if (start.y < end.y)
                        {
                            mY = start.y;
                        }
                        else
                        {
                            mY = end.y;
                        }

                        w = end.x;
                        h = end.y;
                        minX = mX;
                        minY = mY;
                    }
                }
            }
        }
    }
    #endregion

    #region Processing Function
    public void process()
    {
        start = new Vector2Float(0, 0);
        end = new Vector2Float(0, 0);
        dims = new Vector2Float(0, 0);
        minValues = new Vector2Float(100000, 100000);
        string part = "";
        decimal e = 0;
        String s = "";
            s = "";
            s = cutLines.Substring(cutLines.IndexOf("(start ") + 7);
            s = s.Substring(0, s.IndexOf(" (layer "));
            part = s.Substring(0, s.IndexOf(" "));

            if (!decimal.TryParse(part, out e))
            {
                Debug.Log("Error parsing start X. Part:" + part);
                dims.x = -1000;
                dims.y = -1000;
                minValues.x = -1000;
                minValues.y = -1000;
            }
            else
            {
                start.x = e;
                s = s.Substring(s.IndexOf(" ") + 1);
                part = s.Substring(0, s.IndexOf(") (end "));
                if (!decimal.TryParse(part, out e))
                {
                    Debug.Log("Error parsing start Y. Part:" + part);
                    dims.x = -1000;
                    dims.y = -1000;
                    minValues.x = -1000;
                    minValues.y = -1000;
                }
                else
                {
                    start.y = e;
                    s = s.Substring(s.IndexOf("("));
                    s = s.Replace("(end ", "");
                    part = s.Substring(0, s.IndexOf(" "));
                    if (!decimal.TryParse(part, out e))
                    {
                        Debug.Log("Error parsing end X. Part:" + part);
                        dims.x = -1000;
                        dims.y = -1000;
                        minValues.x = -1000;
                        minValues.y = -1000;
                    }
                    else
                    {
                        end.x = e;
                        s = s.Substring(s.IndexOf(" "));
                        part = s.Substring(0, s.IndexOf(")"));
                        if (!decimal.TryParse(part, out e))
                        {
                            Debug.Log("Error parsing end Y. Part:" + part);
                            dims.x = -1000;
                            dims.x = -1000;
                            minValues.x = -1000;
                            minValues.y = -1000;
                        }
                        else
                        {
                            end.y = e;

                            if (start.x < end.x)
                            {
                                if (start.x < minValues.x)
                                {
                                    minValues.x = start.x;
                                }
                            }
                            else
                            {
                                if (end.x < minValues.x)
                                {
                                    minValues.x = end.x;
                                }
                            }

                            if (start.y < end.y)
                            {
                                if (start.y < minValues.y)
                                {
                                    minValues.y = start.y;
                                }
                            }
                            else
                            {
                                if (end.y < minValues.y)
                                {
                                    minValues.y = end.y;
                                }
                            }
                            
                            if (maxValues.x < end.x)
                            {
                                maxValues.x = end.x;
                            }

                            if (maxValues.y < end.y)
                            {
                                maxValues.y = end.y;
                            }
                            
                            //Debug.Log("Edgecut. Starts, X:" + start.x + ", Y:" + start.y + ", Ends X:" + end.x + ", Y:" + end.y );
                        }
                    }
                }
            }

        dims.x = maxValues.x - minValues.x;
        dims.y = maxValues.y - minValues.y;
        //Debug.Log("Post Processing Dimensions W:" + dims.x + ", H:" + dims.y + ", mX:" + minValues.x + ", mY:" + minValues.y + ", MX:" + maxValues.x + ", MY:" + maxValues.y);
    }
    #endregion

    #region Construct Command Sequence
    public List<string> constructCommand()
    {
        List<string> cmdOut = new List<string>();
        string cmd = "";

        //Debug.Log("Construction Dimensions W:" + dims.x + ", H:" + dims.y);
        cmd = "BW" + dims.x + "H" + dims.y + "/";
        cmdOut.Add(cmd);

        return cmdOut;
    }
    #endregion

    #region Perform Transformations
    public edgeCuts transformations()
    {
        edgeCuts e = new edgeCuts();

        e.start = transformationFunctions.rotationTransformation(start, 270);
        e.end = transformationFunctions.rotationTransformation(end, 270);
        e.dims = transformationFunctions.rotationTransformation(dims, 270);
        e.minValues = transformationFunctions.rotationTransformation(minValues, 270);
        e.start.x = Vector2Float.decimalAbs(e.start.x);
        e.start.y = Vector2Float.decimalAbs(e.start.y);

        e.end.x = Vector2Float.decimalAbs(e.end.x);
        e.end.y = Vector2Float.decimalAbs(e.end.y);

        e.dims.x = Vector2Float.decimalAbs(e.dims.x);
        e.dims.y = Vector2Float.decimalAbs(e.dims.y);

        e.minValues.x = Vector2Float.decimalAbs(e.minValues.x);
        e.minValues.y = Vector2Float.decimalAbs(e.minValues.y);
        return e;
    }
    #endregion

    #region Get Reduction
    public decimal getReduction(out decimal o)
    {
        decimal f = (decimal)minValues.x;
            o = (decimal)minValues.y;
        //Debug.Log("Reduction value is: " + f);
        return f;
    }
    #endregion
    
}
#endregion

#region Board Class
[System.Serializable]
public class board
{
    public boardLayer front;
    public boardLayer back;

    #region XTOR
    public board()
    {

    }
    #endregion

    #region Perform Transformations
    public board transformations()
    {
        board b = new board();

        b = this;
        if (b.front != null)
        {
            b.front.transformations();
        }

        if (b.back != null)
        {
            b.back.transformations();
        }

        return b;
    }
    #endregion

    #region Reduction
    public void reduction()
    {
        if (front != null)
        {
            front.reduction();
        }

        if (back != null)
        {
            back.reduction();
        }
    }
    #endregion
}
#endregion

#region Board Layer Class
[System.Serializable]
public class boardLayer
{
    public List<module> modules;
    public List<trace> traces;
    public edgeCuts borders;
    public Vector2Float offset;
    public List<via> vias;
    public List<string> layerCommandRoutine;

    #region XTOR
    public boardLayer()
    {
        modules = new List<module>();
        traces = new List<trace>();
        vias = new List<via>();
        layerCommandRoutine = new List<string>();
        offset = new Vector2Float(0, 0);
        borders = new edgeCuts();
    }
    #endregion

    #region Processing Function
    public void process()
    {
        foreach(module m in modules)
        {
            m.process();
        }

        foreach(trace t in traces)
        {
            t.process();
        }

        foreach (via v in vias)
        {
            v.process();
        }

        //Debug.Log("Layer contains " + traces.Count + " traces, " + modules.Count + "modules, " + vias.Count + "vias.");
    }
    #endregion

    #region Construct Command Sequence
    public void constructCommand()
    {
        List<string> cmdOut = new List<string>();

        cmdOut.Add("START/");
        cmdOut.Add("O/");
        cmdOut.Add("MX" + offset.x + "Y" + offset.y + "/");
        cmdOut.Add("ZA/");
        
        cmdOut.AddRange(borders.constructCommand());

        //cmdOut.Add("PADS/");
        //Debug.Log("Layer Module Count:" + modules.Count);
        foreach (module m in modules)
        {
            cmdOut.AddRange(m.constructCommand(traces));
        }

        //cmdOut.Add("TRACES/");
        foreach (trace t in traces)
        {
            cmdOut.AddRange(t.constructCommand());
        }

        cmdOut.Add("DRILLS/");
        foreach (via v in vias)
        {
            cmdOut.AddRange(v.constructDrilling());
        }

        foreach(module m in modules)
        {
            cmdOut.AddRange(m.constructDrills());
        }

        //cmdOut.Add("VIAS/");
        foreach (via v in vias)
        {
            cmdOut.AddRange(v.constructCommand());
        }
        
        layerCommandRoutine = cmdOut;
    }
    #endregion

    #region Perform Transformations
    public void transformations()
    {
        List<module> mods = new List<module>();
        List<trace> tracks = new List<trace>();
        edgeCuts cuts = new edgeCuts();
        List<via> v = new List<via>();
        cuts = borders.transformations();

        foreach (module m in modules)
        {
            mods.Add(m.transformations(cuts.dims.x));
        }

        foreach(trace t in traces)
        {
            tracks.Add(t.transformations(cuts.dims.x));
        }
        

        foreach(via a in vias)
        {
            v.Add(a.transformations(cuts.dims.x));
        }
        
    }
    #endregion

    #region Reduction
    public void reduction()
    {
        decimal o = 0;
        decimal r = borders.getReduction(out o);
        for(int n = 0; n < modules.Count; n++)
        {
            modules[n].reduction(r, o);
        }

        for (int n = 0; n < vias.Count; n++)
        {
            vias[n].reduction(r, o);
        }

        for (int n = 0; n < traces.Count; n++)
        {
            traces[n].reduction(r, o);
        }
        
    }
    #endregion

    #region Routine Reduction
    public void routineReduction()
    {
        decimal o = 0;
        decimal b = 0;
        decimal r = (decimal)borders.getReduction(out o);
        string p = "", s = "", end = "", start = "", rdx = "";
        decimal i = 0, j = 0;
        //Debug.Log("Vector2 data type is: " + Vector2.zero.x.GetType().ToString());


        //Debug.Log("Reductions are X:" + r + ", Y:" + o);

        for(int n = 0; n < layerCommandRoutine.Count; n++)
        {
            if (layerCommandRoutine[n].Contains("rOX") || layerCommandRoutine[n].Contains("DX"))
            {
                p = layerCommandRoutine[n];
                s = p;
                s = s.Substring(s.IndexOf("X") + 1);
                s = s.Substring(0, s.IndexOf("Y"));

                //Debug.Log("X prior to parse:" + s);
                if (!decimal.TryParse(s, out b))
                {
                    Debug.Log("Failed parsing X value. S:" + s);
                }
                else
                {
                    //Debug.Log("X after parse: " + b);
                    start = p;
                    start = start.Substring(0, start.IndexOf("X") + 1);
                    i = b - r;
                    //Debug.Log("X after reduction " + i);
                    start += i + "Y";

                    s = p;
                    s = s.Substring(s.IndexOf("Y") + 1);

                    if (p.Contains("U"))
                    {
                        s = s.Substring(0, s.IndexOf("U"));
                        end = p.Substring(p.IndexOf("U"));
                    }
                    else
                    {
                        if (s.Contains("R"))
                        {
                            s = s.Substring(0, s.IndexOf("R"));
                        }
                        else
                        {
                            s = s.Substring(0, s.IndexOf("/"));
                        }

                        if (p.Contains("R"))
                        {
                            end = p.Substring(p.IndexOf("R"));
                        }
                        else
                        {
                            end = p.Substring(p.IndexOf("/"));
                        }
                    }

                    //Debug.Log("Y prior to parse: " + s);
                    if (!decimal.TryParse(s, out b))
                    {
                        Debug.Log("Failed parsing Y value. S:" + s);
                    }
                    else
                    {
                        //Debug.Log("Y after parse:" + b);
                        j = b - o;
                        //Debug.Log("Y after reduction: " + j);
                        start += j;
                        start += end;
                        layerCommandRoutine[n] = start;
                        //Debug.Log("Line pre reduction:" + p);
                        //Debug.Log("Line post reduction:" + start);
                    }
                }
            }
            else if (layerCommandRoutine[n].Contains("MX") && !layerCommandRoutine[n].Contains("MX0Y0"))
            {
                p = layerCommandRoutine[n];
                s = p;
                s = s.Substring(s.IndexOf("X") + 1);
                s = s.Substring(0, s.IndexOf("Y"));

                //Debug.Log("X prior to parse:" + s);
                if (!decimal.TryParse(s, out b))
                {
                    Debug.Log("Failed parsing X value. S:" + s);
                }
                else
                {
                    //Debug.Log("X after parse: " + b);
                    start = p;
                    start = start.Substring(0, start.IndexOf("X") + 1);
                    i = b - r;
                    //Debug.Log("X after reduction " + i);
                    start += i + "Y";

                    s = p;
                    s = s.Substring(s.IndexOf("Y") + 1);

                    if (p.Contains("U"))
                    {
                        s = s.Substring(0, s.IndexOf("U"));
                        end = p.Substring(p.IndexOf("U"));
                    }
                    else
                    {
                        if (s.Contains("R"))
                        {
                            s = s.Substring(0, s.IndexOf("R"));
                        }
                        else
                        {
                            s = s.Substring(0, s.IndexOf("/"));
                        }

                        if (p.Contains("R"))
                        {
                            end = p.Substring(p.IndexOf("R"));
                        }
                        else
                        {
                            end = p.Substring(p.IndexOf("/"));
                        }
                    }

                    //Debug.Log("Y prior to parse: " + s);
                    if (!decimal.TryParse(s, out b))
                    {
                        Debug.Log("Failed parsing Y value. S:" + s);
                    }
                    else
                    {
                        //Debug.Log("Y after parse:" + b);
                        j = b - o;
                        //Debug.Log("Y after reduction: " + j);
                        start += j;
                        start += end;
                        layerCommandRoutine[n] = start;
                        //Debug.Log("Line pre reduction:" + p);
                        //Debug.Log("Line post reduction:" + start);
                    }
                }
            }
            else if (layerCommandRoutine[n].Contains("VX"))
            {
                p = layerCommandRoutine[n];
                s = p;
                s = s.Substring(s.IndexOf("X") + 1);
                s = s.Substring(0, s.IndexOf("Y"));

                if (!decimal.TryParse(s, out b))
                {
                    Debug.Log("Failed parsing X value. S:" + s);
                }
                else
                {
                    //Debug.Log("X after parse: " + b);
                    start = p;
                    start = start.Substring(0, start.IndexOf("X") + 1);
                    i = b - r;
                    //Debug.Log("X after reduction " + i);
                    start += i + "Y";

                    s = p;
                    s = s.Substring(s.IndexOf("Y") + 1);

                    s = s.Substring(0, s.IndexOf("R"));

                    //Debug.Log("Y prior to parse: " + s);
                    if (!decimal.TryParse(s, out b))
                    {
                        Debug.Log("Failed parsing Y value. S:" + s);
                    }
                    else
                    {
                        j = b - o;
                        start += j;
                        s = p.Substring(p.IndexOf("R"));
                        start += s;
                        layerCommandRoutine[n] = start;
                    }
                }
            }
            else if (layerCommandRoutine[n].Contains("WX"))
            {
                p = layerCommandRoutine[n];
                s = p;
                s = s.Substring(s.IndexOf("X") + 1);
                s = s.Substring(0, s.IndexOf("Y"));

                if (!decimal.TryParse(s, out b))
                {
                    Debug.Log("Failed parsing X value. S:" + s);
                }
                else
                {
                    //Debug.Log("X after parse: " + b);
                    start = p;
                    start = start.Substring(0, start.IndexOf("X") + 1);
                    i = b - r;
                    //Debug.Log("X after reduction " + i);
                    start += i + "Y";

                    s = p;
                    s = s.Substring(s.IndexOf("Y") + 1);

                    if (s.IndexOf("R") > 0)
                    {
                        rdx = s.Substring(s.IndexOf("R"));
                        s = s.Substring(0, s.IndexOf("R"));
                    }
                    else
                    {
                        s = s.Substring(0, s.IndexOf("/"));
                    }

                    //Debug.Log("Y prior to parse: " + s);
                    if (!decimal.TryParse(s, out b))
                    {
                        Debug.Log("Failed parsing Y value. S:" + s);
                    }
                    else
                    {
                        j = b - o;
                        start += j;
                        if (rdx != "")
                        {
                            s = rdx;
                        }
                        else
                        {
                            s = p.Substring(p.IndexOf("/"));
                        }
                        start += s;
                        layerCommandRoutine[n] = start;
                    }
                }
            }
        }

        layerCommandRoutine.Insert(0, "?RDCTX" + r + "Y" + o + "/");
    }
    #endregion
}
#endregion

#region Via Class
[System.Serializable]
public class via
{
    public Vector2Float center;
    public Vector2Float radii;
    public string viaLine;
    public string viaCommand;

    #region XTOR
    public via()
    {
        viaLine = "";
    }

    public via(decimal x, decimal y)
    {
        center = new Vector2Float(x, y);
        viaLine = "";
    }

    public via(decimal x, decimal y, decimal i, decimal o)
    {
        center = new Vector2Float(x, y);
        radii = new Vector2Float(i, o);
        viaLine = "";
    }
    #endregion

    #region Processing Function
    public void process()
    {
        string s = "";
        decimal f = 0;


        //Debug.Log("Starting via processing. s:" + viaLine);
        viaLine = viaLine.Substring(viaLine.IndexOf("(at ") + 4);
        s = viaLine.Substring(0, viaLine.IndexOf(" "));

        if (!decimal.TryParse(s, out f))
        {
            Debug.Log("Failed parsing via center X. s:" + s);
        }
        else
        {
            center.x = f;

            viaLine = viaLine.Substring(viaLine.IndexOf(" ") + 1);
            s = viaLine.Substring(0, viaLine.IndexOf(")"));

            if (!decimal.TryParse(s, out f))
            {
                Debug.Log("Failed parsing via center Y. s:" + s);
            }
            else
            {
                center.y = f;

                viaLine = viaLine.Substring(viaLine.IndexOf("(size ") + 6);
                s = viaLine.Substring(0, viaLine.IndexOf(")"));

                if (!decimal.TryParse(s, out f))
                {
                    Debug.Log("Failed parsing via outer radius. s:" + s);
                }
                else
                {
                    radii.x = f;

                    viaLine = viaLine.Substring(viaLine.IndexOf("(drill ") + 7);
                    s = viaLine.Substring(0, viaLine.IndexOf(")"));

                    if (!decimal.TryParse(s, out f))
                    {
                        Debug.Log("Failed parsing via inner radius. s:" + s);
                    }
                    else
                    {
                        radii.y = f;
                        //Debug.Log("Processed Via. C(" + center.x + ", " + center.y + "). R(" + radii.x + ", " + radii.y + ")");
                    }
                }
            }
        }
    }
    #endregion

    #region Construct Command Sequence
    public List<string> constructCommand()
    {
        List<string> cmdOut = new List<string>();

        cmdOut.Add("VX" + center.x + "Y" + center.y + "R" + (radii.x / 2) + "H" + (radii.y / 2) + "/");

        return cmdOut;
    }
    #endregion

    #region Construct Drilling Sequence
    public List<string> constructDrilling()
    {
        List<string> cmdOut = new List<string>();

        cmdOut.Add("WX" + center.x + "Y" + center.y + "R" + (radii.y) + "/");
        //Debug.Log("Via Drill. Drill D:" + radii.y);

        return cmdOut;
    }
    #endregion

    #region Perform Transformations
    public via transformations(decimal h)
    {
        via v = new via();
        Vector2Float vO = new Vector2Float(0, 0);
        
        #region Rotate
        vO = transformationFunctions.rotationTransformation(center, 270);
        v.center = vO;
        vO = transformationFunctions.rotationTransformation(radii, 270);
        v.radii = vO;
        #endregion

        #region Reflect
        v.center = transformationFunctions.reflectionTransformation(v.center, h);
        v.radii = transformationFunctions.reflectionTransformation(v.radii, h);
        #endregion

        return v;
    }
    #endregion

    #region Reduction
    public void reduction(decimal r, decimal o)
    {
        center.x -= r;
        center.y -= o;
    }
    #endregion
}
#endregion

#region Module Class
[System.Serializable]
public class module
{
    public Vector2Float center;
    public decimal rotation;
    public List<pad> pads;
    public List<string> padLines;
    public string moduleLine;
    public List<string> moduleCommand;

    #region XTOR
    public module()
    {
        pads = new List<pad>();
        padLines = new List<string>();
        moduleLine = "";
    }

    public module(Vector2Float c, decimal r)
    {
        center = c;
        rotation = r;
        pads = new List<pad>();
        padLines = new List<string>();
        moduleLine = "";
    }

    public module(decimal x, decimal y, decimal r)
    {
        center = new Vector2Float(x, y);
        rotation = r;
        pads = new List<pad>();
        padLines = new List<string>();
        moduleLine = "";
    }
    #endregion

    #region Processing Function
    public void process()
    {
        string s = "";
        decimal f = 0;

        #region Process Module Line First
        //Debug.Log("Module start line is: " + moduleLine);
        moduleLine = moduleLine.Substring(moduleLine.IndexOf("(at ") + 4);
        s = moduleLine.Substring(0, moduleLine.IndexOf(" "));
        if (!decimal.TryParse(s, out f))
        {
            Debug.Log("Failed parsing module center X. s:" + s);
        }
        else
        {
            center.x = f;
            moduleLine = moduleLine.Substring(moduleLine.IndexOf(" ") + 1);
            if (moduleLine.IndexOf(" ") > 0)
            {
                s = moduleLine.Substring(0, moduleLine.IndexOf(" "));
            }
            else
            {
                s = moduleLine.Substring(0, moduleLine.IndexOf(")"));
            }

            if (!decimal.TryParse(s, out f))
            {
                Debug.Log("Failed parsing module center Y. s:" + s);
            }
            else
            {
                center.y = f;
                if (moduleLine.IndexOf(" ") > 0)
                {
                    moduleLine = moduleLine.Substring(moduleLine.IndexOf(" ") + 1);
                    s = moduleLine.Substring(0, moduleLine.IndexOf(")"));
                    if (!decimal.TryParse(s, out f))
                    {
                        Debug.Log("Failed parsing module rotation. s:" + s);
                    }
                    else
                    {
                        rotation = f;
                    }
                }
                else
                {
                    rotation = 0;
                }
            }
        }
        #endregion

        #region Process Pads
        foreach(pad p in pads)
        {
            p.process();
        }
        #endregion
        
    }
    #endregion

    #region Construct Command Sequence
    public List<string> constructCommand()
    {
        List<string> cmdOut = new List<string>();
        //Debug.Log("Module Pad Count:" + pads.Count);

        cmdOut.Add("rOX" + center.x + "Y" + center.y + "U" + rotation + "/");

        foreach(pad p in pads)
        {
            cmdOut.Add(p.constructCommand());
        }

        cmdOut.Add("eO/");

        return cmdOut;
    }

    public List<string> constructCommand(List<trace> traces)
    {
        List<string> cmdOut = new List<string>();
        //Debug.Log("Module Pad Count:" + pads.Count);

        cmdOut.Add("rOX" + center.x + "Y" + center.y + "U" + rotation + "/");

        foreach (pad p in pads)
        {
            cmdOut.Add(p.constructCommand(new Vector2((float)center.x, (float)center.y), (float)rotation, traces));
        }

        cmdOut.Add("eO/");

        return cmdOut;
    }

    public List<string> constructDrills()
    {
        List<string> lO = new List<string>();

        foreach(pad p in pads)
        {
            if (p.drill)
            {
                lO.Add(p.constructDrills(center, rotation));
            }
        }

        return lO;
    }
    #endregion

    #region Perform Transformations
    public module transformations(decimal h)
    {
        module m = new module();

        #region Perform Rotation
        m.rotation = rotation + 270;
        m.center = transformationFunctions.rotationTransformation(m.center, m.rotation, out m.rotation);

        foreach(pad p in m.pads)
        {
            p.transformations(h);
        }
        #endregion

        #region Perform Reflection
        m.center = transformationFunctions.reflectionTransformation(m.center, h);
        #endregion

        return m;
    }
    #endregion

    #region Reduction
    public void reduction(decimal r, decimal o)
    {
        Debug.Log("Module Center Pre Reduction. X:" + center.x + ", Y:" + center.y);
        center.x -= r;
        center.y -= o;
        Debug.Log("Module Center Post Reduction:" + center.x + ", Y:" + center.y + ". Reduction Values. X:" + r + ", Y:" + o);
    }
    #endregion
}
#endregion

#region Pad Class
[System.Serializable]
public class pad
{
    public Vector2Float center;
    public Vector2Float dimensions;
    public decimal rotation;
    public string padLine;
    public string padCommand;
    //type 0 = smd. type 1 = through hole
    public int padType;
    //shape 0 = square fill. shape 1 = circular. shape 2 = via
    public int padShape;
    public bool drill = false;
    public float drillDiam;

    #region XTOR
    public pad()
    {
        padLine = "";
    }

    public pad(decimal x, decimal y)
    {
        center = new Vector2Float(x, y);
        padLine = "";
    }

    public pad(decimal x, decimal y, decimal w, decimal h, decimal r)
    {
        center = new Vector2Float(x, y);
        dimensions = new Vector2Float(w, h);
        rotation = r;
        padLine = "";
    }
    #endregion

    #region Processing Function
    public void process()
    {
        string lne = "";
        string part = "";
        decimal f = 0;
        lne = padLine;

        if (padLine.Contains("roundrect") || padLine.Contains("rect"))
        {
            padShape = 0;
        }
        else if (padLine.Contains("circle"))
        {
            padShape = 1;
        }

        if (padLine.Contains("smd"))
        {
            padType = 0;
        }

        if (padLine.Contains("(drill "))
        {
            part = padLine.Substring(padLine.IndexOf("(drill ") + 7);
            string pt1 = part;
            part = part.Substring(0, part.IndexOf(") ("));
            string pt2 = part;
            if (!float.TryParse(part, out drillDiam))
            {
                Debug.Log("Padline:" + padLine);
                Debug.Log("Part1:" + pt1);
                Debug.Log("Part2:" + pt2);
                Debug.Log("Failed to parse pad drill diameter. S:" + part);
            }
        }

        padLine = padLine.Substring(padLine.IndexOf("(at ") + 4);
        part = padLine.Substring(0, padLine.IndexOf(" "));

        if (!decimal.TryParse(part, out f))
        {
            Debug.Log("Failed parsing pad center X. s:" + part);
        }
        else
        {
            center.x = f;
            padLine = padLine.Substring(padLine.IndexOf(" ") + 1);


            if (padLine.IndexOf(" ") < padLine.IndexOf(")"))
            {
                part = padLine.Substring(0, padLine.IndexOf(" ") + 1);
            }
            else
            {
                part = padLine.Substring(0, padLine.IndexOf(")"));
                rotation = 0;
            }
            
            if (!decimal.TryParse(part, out f))
            {
                Debug.Log("Failed parsing pad center Y. s:" + part);
            }
            else
            {
                center.y = f;
                if (padLine.IndexOf(" ") < padLine.IndexOf(")"))
                {
                    padLine = padLine.Substring(padLine.IndexOf(" ") + 1);
                    part = padLine.Substring(0, padLine.IndexOf(")"));

                    if (!decimal.TryParse(part, out f))
                    {
                        Debug.Log("Failed parsing pad rotation. s:" + part);
                    }
                    else
                    {
                        rotation = f;
                    }
                }

                padLine = padLine.Substring(padLine.IndexOf("(size ") + 6);
                part = padLine.Substring(0, padLine.IndexOf(" "));

                if (!decimal.TryParse(part, out f))
                {
                    Debug.Log("Failed parsing pad dimension X. s:" + part);
                }
                else
                {
                    dimensions.x = f;

                    padLine = padLine.Substring(padLine.IndexOf(" ") + 1);

                    part = padLine.Substring(0, padLine.IndexOf(")"));

                    if (!decimal.TryParse(part, out f))
                    {
                        Debug.Log("Failed parsing pad dimension Y. s:" + part);
                    }
                    else
                    {
                        dimensions.y = f;
                    }
                }
            }
        }
    }
    #endregion

    #region Construct Command Sequence

    public string constructCommand()
    {
        string s = "";
        s = "PX" + center.x + "Y" + center.y;

        //square
        if (padShape == 0)
        {
            s += "W" + dimensions.x + "H" + dimensions.y + "U" + rotation;
        }
        //circular
        else if (padShape == 1)
        {
            s += "R" + dimensions.x + "U" + rotation;
        }
        //through-hole
        else if (padShape == 2)
        {
            s += "R" + dimensions.x + "H" + dimensions.y + "U" + rotation;
        }

        s += "/";

        return s;
    }

    public string constructCommand(Vector2 mPos, float rot, List<trace> traces)
    {
        string s = "";
        //compute absolute pad location\
        Vector2 tpPos = VectorTransformations.rotate(new Vector2((mPos.x + (float)center.x), (mPos.y + (float)center.y)), (float)rot);
        //Debug.Log("Module Center: " + mPos.x + "," + mPos.y + ". Absolute Pad Center: " + tpPos.x + "," + tpPos.y);
        if (traceConnection(tpPos, traces))
        {
            s = "pX";
        }
        else
        {
            s = "PX";
        }
        s += center.x + "Y" + center.y;

        //square
        if (padShape == 0)
        {
            s += "W" + dimensions.x + "H" + dimensions.y + "U" + rotation;
        }
        //circular
        else if (padShape == 1)
        {
            s += "R" + dimensions.x + "U" + rotation;
        }
        //through-hole
        else if (padShape == 2)
        {
            s += "R" + dimensions.x + "H" + dimensions.y + "U" + rotation;
        }

        s += "/";

        return s;
    }
    #endregion

    #region Trace Connection Checking
    public bool traceConnection(Vector2 pC, List<trace> traces)
    {
        bool rt = false;

        foreach(trace t in traces)
        {
            Vector2 nS, nE;
            getTracePoints(t, out nS, out nE);
            if (Mathf.Abs(Vector2.Distance(nS, pC)) <= .2f)
            {
                return true;
            }

            if (Mathf.Abs(Vector2.Distance(nE, pC)) <= .2f)
            {
                return true;
            }
        }

        return rt;
    }
    #endregion

    #region Obtain Trace Points
    public void getTracePoints(trace t, out Vector2 st, out Vector2 en)
    {
        string p = t.startCommand.Substring(t.startCommand.IndexOf("X") + 1);
        string o = p.Substring(0, p.IndexOf("Y"));
        float f = 0;

        st.x = float.Parse(o);
        p = t.startCommand.Substring(t.startCommand.IndexOf("Y") + 1);
        if (p.Contains("R"))
        {
            o = p.Substring(0, p.IndexOf("R"));
        }
        else
        {
            o = p.Substring(0, p.IndexOf("/"));
        }
        st.y = float.Parse(o);
        
        p = t.endCommand.Substring(t.endCommand.IndexOf("X") + 1);
        o = p.Substring(0, p.IndexOf("Y"));
        en.x = float.Parse(o);

        p = t.endCommand.Substring(t.endCommand.IndexOf("Y") + 1);
        if (p.Contains("R"))
        {
            o = p.Substring(0, p.IndexOf("R"));
        }
        else
        {
            o = p.Substring(0, p.IndexOf("/"));
        }
        en.y = float.Parse(o);
    }
    #endregion

    #region Perform Transformations
    public pad transformations(decimal h)
    {
        pad p = new pad();
        p.rotation = rotation + 270;
        p.center = transformationFunctions.rotationTransformation(center, p.rotation, out p.rotation);
        p.dimensions = transformationFunctions.rotationTransformation(dimensions, p.rotation);

        p.center = transformationFunctions.reflectionTransformation(p.center, h);
        p.dimensions = transformationFunctions.reflectionTransformation(p.dimensions, h);
        return p;
    }
    #endregion

    #region Construct Drills
    public string constructDrills(Vector2Float mPos, decimal mRot)
    {
        string lO = "WX";
        Vector2Float nPos = new Vector2Float(0, 0);

        //Debug.Log("Pad Drill Before Rotation Center CX:" + mPos.x + ", CY:" + mPos.y);

        if (mRot == 0)
        {
            nPos.x = center.x;
            nPos.y = center.y;
        }
        else if (mRot == 90)
        {
            nPos.x = center.y;
            nPos.y = -1 * center.x;
        }
        else if (mRot == 180)
        {
            nPos.x = -1 * center.x;
            nPos.y = -1 * center.y;
        }
        else if (mRot == 270)
        {
            nPos.x = -1 * center.y;
            nPos.y = center.x;
        }
        //Debug.Log("Pad Drill Before Adding Pad Center CX:" + nPos.x + ", CY:" + nPos.y);

        nPos.x += mPos.x;
        nPos.y += mPos.y;

        //Debug.Log("Pad Drill After Adding Pad Center CX:" + nPos.x + ", CY:" + nPos.y);

        lO += nPos.x + "Y" + nPos.y;
        //Debug.Log("Pad. Drill D:" + drillDiam);

        if (drillDiam > 0)
        {
            lO += "R" + drillDiam + "/";
        }
        else
        {
            lO += "/";
        }
        return lO;
    }
    #endregion
}
#endregion

#region Trace Segment Class
[System.Serializable]
public class trace
{
    public Vector2Float start;
    public Vector2Float end;
    public decimal width;
    public string traceLine;
    public string startCommand;
    public string endCommand;

    #region XTOR
    public trace()
    {
        traceLine = "";
    }

    public trace(decimal x, decimal y, decimal i, decimal o)
    {
        start = new Vector2Float(x, y);
        end = new Vector2Float(i, o);
        traceLine = "";
    }
    #endregion

    #region Processing Function
    public void process()
    {
        string pt = "";
        pt = traceLine.Substring(traceLine.IndexOf("(width "));
        pt = pt.Substring(0, pt.IndexOf(")"));
        pt = pt.Substring(7);
        if (pt.Contains(")"))
        {
            pt = pt.Substring(0, pt.IndexOf(")"));
        }
        width = decimal.Parse(pt);

        startCommand = traceLine.Substring(0, traceLine.IndexOf(") (end "));
        endCommand = traceLine.Substring(traceLine.IndexOf(") (end "));
        startCommand = startCommand.Replace("(segment (start ", "MX");
        startCommand = startCommand.Replace(" ", "Y");
        startCommand = startCommand + "R" + width + "/";

        while(startCommand[0] != 'M')
        {
            startCommand = startCommand.Substring(1);
        }

        endCommand = endCommand.Replace(") (end ", "DX");
        endCommand = endCommand.Substring(0, endCommand.IndexOf(")"));
        endCommand = endCommand.Replace(" ", "Y");
        endCommand = endCommand + "R" + width + "/";

        //Debug.Log("STCMD:" + startCommand + "\n EDCMD:" + endCommand);
    }
    #endregion

    #region Construct Command Sequence
    public List<string> constructCommand()
    {
        List<string> cmdOut = new List<string>();

        cmdOut.Add(startCommand);
        cmdOut.Add(endCommand);

        return cmdOut;
    }
    #endregion

    #region Perform Transformations
    public trace transformations(decimal h)
    {
        trace t = new trace();

        t.start = transformationFunctions.rotationTransformation(start, 270);
        t.end = transformationFunctions.rotationTransformation(end, 270);
        t.start = transformationFunctions.reflectionTransformation(t.start, h);
        t.end = transformationFunctions.reflectionTransformation(t.end, h);

        return t;
    }
    #endregion

    #region Reduction
    public void reduction(decimal r, decimal o)
    {
        start.x -= r;
        start.y -= o;
        end.x -= r;
        end.y -= o;
    }
    #endregion
}
#endregion

#region Trace Nodes Class
public class nodeNetwork
{
    public Vector2Float start;
    public List<Vector2Float> nodes;

    #region XTOR
    public nodeNetwork()
    {
        start = new Vector2Float(0, 0);
        nodes = new List<Vector2Float>();
    }

    public nodeNetwork(Vector2Float v)
    {
        start = v;
        nodes = new List<Vector2Float>();
    }
    #endregion

    #region Construct Network
    public List<trace> constructNetwork(List<trace> t)
    {
        List<trace> tracesOut = new List<trace>();
        Vector2Float o;
        bool matchFound = true;
        start = t[0].start;
        nodes.Add(t[0].end);
        t.Remove(t[0]);

        while(matchFound)
        {
            foreach(trace c in t)
            {
                if (c.start == start)
                {
                    nodes.Add(start);
                    start = c.end;
                    t.Remove(c);
                    matchFound = true;
                }
                else if (c.end == start)
                {
                    nodes.Add(start);
                    start = c.start;
                    t.Remove(c);
                    matchFound = true;
                }
            }
        }

        return tracesOut;
    }
    #endregion
}
#endregion

#endregion

#region Version 2 Board Classes

#region Central Board Object Class
[System.Serializable]
public class verrucktBoard
{
    #region Variables
    public verrucktBoardSide topSide;
    public verrucktBoardSide bottomSide;
    public Color32[][] boardImagePixels;
    #endregion

    #region XTOR
    public verrucktBoard()
    {
        return;
    }

    public verrucktBoard(verrucktBoardSide t, verrucktBoardSide b)
    {
        topSide = t;
        bottomSide = b;
        return;
    }
    #endregion
}
#endregion

#region Board Side Class
[System.Serializable]
public class verrucktBoardSide
{
    #region Variables
    public List<verrucktTrace> traces;
    public List<verrucktModule> modules;
    public List<verrucktEdge> edges;
    public List<verrucktVia> vias;
    #endregion

    #region XTOR

    #endregion
}
#endregion

#region Module Class
[System.Serializable]
public class verrucktModule
{
    #region Variables

    #endregion

    #region XTOR

    #endregion
}
#endregion

#region Pad Class
[System.Serializable]
public class verrucktPad
{
    #region Variables

    #endregion

    #region XTOR

    #endregion
}
#endregion

#region Trace Class
[System.Serializable]
public class verrucktTrace
{
    #region Variables

    #endregion

    #region XTOR

    #endregion
}
#endregion

#region Edge Class
[System.Serializable]
public class verrucktEdge
{
    #region Variables

    #endregion

    #region XTOR

    #endregion
}
#endregion

#region Via Class
[System.Serializable]
public class verrucktVia
{
    #region Variables

    #endregion

    #region XTOR

    #endregion
}
#endregion

#endregion

#region Transformation Functions
public static class transformationFunctions
{
    #region Rotation (Vector only return)
    public static Vector2Float rotationTransformation(Vector2Float v, decimal r)
    {
        Vector2Float ret = new Vector2Float(0, 0);

        if (r > 360)
        {
            while (r > 360)
            {
                r -= 360;
            }
        }

        if (r == 0)
        {
            ret.x = v.x;
            ret.y = v.y;
        }
        else if (r == 90)
        {
            ret.x = v.y;
            ret.y = -1 * v.x;
        }
        else if (r == 180)
        {
            ret.x = -1 * v.x;
            ret.y = -1 * v.y;
        }
        else if (r == 270)
        {
            ret.x = -1 * v.y;
            ret.y = v.x;
        }

        return ret;
    }
    #endregion

    #region Rotation (Rotation and Vector Return
    public static Vector2Float rotationTransformation(Vector2Float v, decimal r, out decimal rO)
    {
        Vector2Float ret = new Vector2Float(0, 0);
        rO = r;

        if (rO > 360)
        {
            while (rO > 360)
            {
                rO -= 360;
            }
        }

        if (r == 0)
        {
            ret.x = v.x;
            ret.y = v.y;
        }
        else if (r == 90)
        {
            ret.x = v.y;
            ret.y = -1 * v.x;
        }
        else if (r == 180)
        {
            ret.x = -1 * v.x;
            ret.y = -1 * v.y;
        }
        else if (r == 270)
        {
            ret.x = -1 * v.y;
            ret.y = v.x;
        }

        return ret;
    }
    #endregion

    #region Reflection
    public static Vector2Float reflectionTransformation(Vector2Float v, decimal h)
    {
        Vector2Float o = v;
        o.x = Vector2Float.decimalAbs(h - v.x);
        o.y = Vector2Float.decimalAbs(v.y);

        return v;
    }
    #endregion
}
#endregion

#region Raster Classes

#region Raster Boundaries
public class rasterBoundaries
{
    public Vector2 p;
    //type FALSE => Quadrilateral. True => Circular
    public bool type = false;
    public Vector2 dimensions;
    public float rot = 0;

    #region XTORs
    public rasterBoundaries()
    {
        p = Vector2.zero;
        type = false;
        dimensions = Vector2.zero;
    }

    public rasterBoundaries(Vector2 pI, bool b, Vector2 d)
    {
        p = pI;
        type = b;
        dimensions = d;
    }
    

    public rasterBoundaries(string pd)
    {
        Vector2 padPosition = Vector2.zero;
        float padRotation = 0;
        string pt = pd;
        string s = pt;
        float f = 0;

        #region Created From Raw Via
        if (pd.Contains("VX"))
        {
            pt = pd;
            s = pt;
            pt = pt.Substring(pt.IndexOf("X") + 1);
            pt = pt.Substring(0, pt.IndexOf("Y"));

            pt = pd;
            s = pt;
            pt = pt.Substring(pt.IndexOf("X") + 1);
            pt = pt.Substring(0, pt.IndexOf("Y"));

            if (!float.TryParse(pt, out f))
            {
                Debug.Log("failed to parse x value of via. string is " + pt);
            }
            else
            {
                padPosition.x = f;
                s = s.Substring(s.IndexOf("Y") + 1);
                
                    type = true;
                    pt = s.Substring(0, s.IndexOf("R"));

                    if (!float.TryParse(pt, out f))
                    {
                        Debug.Log("failed to parse y value of via. string is " + pt);
                    }
                    else
                    {
                        padPosition.y = f;
                        s = s.Substring(s.IndexOf("R") + 1);
                        if (s.Contains("H"))
                        {
                            pt = s.Substring(0, s.IndexOf("H"));
                        }
                        else
                        {
                            pt = s.Substring(0, s.IndexOf("U"));
                        }

                        if (!float.TryParse(pt, out f))
                        {
                            Debug.Log("failed to parse radius of via. string is " + pt);
                        }
                        else
                        {
                        p.x = padPosition.x;
                        p.y = padPosition.y;
                            dimensions.x = f;
                            padRotation = 0;
                        }
                    }
                }
            }
        #endregion

        #region Created From Pad
        else
        {
            #region Parse Pad Parameters
            pt = pd;
            s = pt;
            pt = pt.Substring(pt.IndexOf("X") + 1);
            pt = pt.Substring(0, pt.IndexOf("Y"));

            if (!float.TryParse(pt, out f))
            {
                Debug.Log("failed to parse x value of pad. string is " + pt);
            }
            else
            {
                padPosition.x = f;
                s = s.Substring(s.IndexOf("Y") + 1);

                #region Quadrilateral Pad
                if (s.Contains("W"))
                {
                    type = false;
                    pt = s.Substring(0, s.IndexOf("W"));

                    if (!float.TryParse(pt, out f))
                    {
                        Debug.Log("failed to parse y value of pad. string is " + pt);
                    }
                    else
                    {
                        padPosition.y = f;
                        s = s.Substring(s.IndexOf("W") + 1);
                        pt = s.Substring(0, s.IndexOf("H"));

                        if (!float.TryParse(pt, out f))
                        {
                            Debug.Log("failed to parse width of pad. string is " + pt);
                        }
                        else
                        {
                            dimensions.x = f;
                            s = s.Substring(s.IndexOf("H") + 1);
                            pt = s.Substring(0, s.IndexOf("U"));

                            if (!float.TryParse(pt, out f))
                            {
                                Debug.Log("failed to parse height of pad. string is " + pt);
                            }
                            else
                            {
                                dimensions.y = f;
                                s = s.Substring(s.IndexOf("U") + 1);
                                pt = s.Substring(0, s.IndexOf("/"));

                                if (!float.TryParse(pt, out f))
                                {
                                    Debug.Log("failed to parse module rotation. string is " + pt);
                                }
                                else
                                {
                                    p.x = padPosition.x;
                                    p.y = padPosition.y;
                                    padRotation = f;
                                }
                            }
                        }
                    }
                }
                #endregion

                #region Circular Pad
                else if (s.Contains("R"))
                {
                    type = true;
                    pt = s.Substring(0, s.IndexOf("R"));

                    if (!float.TryParse(pt, out f))
                    {
                        Debug.Log("failed to parse y value of pad. string is " + pt);
                    }
                    else
                    {
                        padPosition.y = f;
                        s = s.Substring(s.IndexOf("R") + 1);
                        if (s.Contains("H"))
                        {
                            pt = s.Substring(0, s.IndexOf("H"));
                        }
                        else
                        {
                            pt = s.Substring(0, s.IndexOf("U"));
                        }

                        if (!float.TryParse(pt, out f))
                        {
                            Debug.Log("failed to parse width of pad. string is " + pt);
                        }
                        else
                        {
                            p.x = padPosition.x;
                            p.y = padPosition.y;
                            dimensions.x = f;
                            padRotation = 0;
                        }
                    }
                }
                #endregion
            }
            rot = padRotation;
        }
        #endregion

        #endregion
    }
    #endregion

    #region Within Boundary Check
    public bool isWithinBoundary(Vector2 v)
    {
        if (!type)
        {
            if (v.x >= p.x && v.y >= p.y && v.x <= dimensions.x && v.y <= dimensions.y)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            if (Vector2.Distance(v, p) <= dimensions.x)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }
    #endregion
}

#endregion

#region Raster Parent
public class rasterParent
{
    public Vector2 p;
    public float rot = 0;

    #region XTOR
    public rasterParent(string md)
    {
        string pt = "";
        string s = "";
        float f = 0;
        pt = md;
        s = pt;
        pt = pt.Substring(pt.IndexOf("X") + 1);
        pt = pt.Substring(0, pt.IndexOf("Y"));

        if (!float.TryParse(pt, out f))
        {
            Debug.Log("failed to parse x value of module. string is " + pt);
        }
        else
        {
            p.x = f;
            s = s.Substring(s.IndexOf("Y") + 1);
            pt = s.Substring(0, s.IndexOf("U"));

            if (!float.TryParse(pt, out f))
            {
                Debug.Log("failed to parse y value of module. string is " + pt);
            }
            else
            {
                p.y = f;
                s = s.Substring(s.IndexOf("U") + 1);
                pt = s.Substring(0, s.IndexOf("/"));

                if (!float.TryParse(pt, out f))
                {
                    Debug.Log("failed to parse module rotation. string is " + pt);
                }
                else
                {
                    rot = f;
                }
            }
        }
    }
        #endregion
}
#endregion


#endregion

#region Stencil Classes

#region Stencil Module
public class stencilModule
{
    public Vector2 position;
    public float rotation;
    public List<stencilPad> pads;

    #region XTOR
    public stencilModule()
    {

    }

    public stencilModule(string s)
    {
        //Line is formatted as rOXNYNUN/
        string p = "";
        float f = 0;
        p = s.Substring(s.IndexOf("X") + 1);
        p = p.Substring(0, p.IndexOf("Y"));
        f = float.Parse(p);
        position.x = f;

        p = s.Substring(s.IndexOf("Y") + 1);
        p = p.Substring(0, p.IndexOf("U"));
        f = float.Parse(p);
        position.y = f;

        p = s.Substring(s.IndexOf("U") + 1);
        p = p.Substring(0, p.IndexOf("/"));
        f = float.Parse(p);
        rotation = f;
    }
    #endregion

    #region Functions
    public void addPad(string s)
    {
        if (pads == null)
        {
            pads = new List<stencilPad>();
        }
        stencilPad pad = new stencilPad(s, rotation, position);
        pads.Add(pad);
    }
    #endregion
}
#endregion

#region Stencil Pad
public class stencilPad
{
    public Vector2 position;
    public float rotation;
    public Vector2 dimensions;
    public bool circular = false;

    #region XTOR
    public stencilPad(string s, float mRot, Vector2 mPos)
    {
        //line is formatted as PXNYN(WNHN:RN:RNHN)UN/
        string p = "";
        float f = 0;
        Vector2 prePos;
        Vector2 preDims;

        #region Parse Pre Transformation Values
        p = s.Substring(s.IndexOf("X") + 1);
        p = p.Substring(0, p.IndexOf("Y"));
        f = float.Parse(p);
        prePos.x = f;

        p = s.Substring(s.IndexOf("Y") + 1);
        if (s.Contains("W"))
        {
            p = p.Substring(0, p.IndexOf("W"));
        }
        else if (s.Contains("R"))
        {
            p = p.Substring(0, p.IndexOf("R"));
        }
        f = float.Parse(p);
        prePos.y = f;

        if (s.Contains("W"))
        {
            p = s.Substring(s.IndexOf("W") + 1);
            p = p.Substring(0, p.IndexOf("H"));
            f = float.Parse(p);
            preDims.x = f;

            p = s.Substring(s.IndexOf("H") + 1);
            p = p.Substring(0, p.IndexOf("U"));
            f = float.Parse(p);
            preDims.y = f;
            circular = false;
        }
        else
        {
            p = s.Substring(s.IndexOf("R") + 1);

            if (s.Contains("H"))
            {
                p = p.Substring(0, p.IndexOf("H"));
                f = float.Parse(p);
                preDims.x = f;

                p = s.Substring(s.IndexOf("H") + 1);
                p = p.Substring(p.IndexOf("U"));
                f = float.Parse(p);
                preDims.y = f;
                circular = true;
            }
            else
            {
                circular = true;
                p = p.Substring(0, p.IndexOf("U"));
                f = float.Parse(p);
                preDims.x = f;
                preDims.y = 0;
            }
        }

        p = s.Substring(s.IndexOf("U") + 1);
        p = p.Substring(0, p.IndexOf("/"));
        f = float.Parse(p);
        rotation = f;
        #endregion

        #region PrePosition Transformations
        prePos = VectorTransformations.rotate(prePos, mRot);

        position.x = prePos.x + mPos.x;
        position.y = prePos.y + mPos.y;

        if (!circular)
        {
            preDims = VectorTransformations.rotate(preDims, rotation);
        }
        dimensions = preDims;
        #endregion
    }
    #endregion
}
#endregion

#region Bounds
public class bounds
{
    public Vector2 xBound;
    public Vector2 yBound;
    public bool circular;

    #region XTOR
    public bounds(stencilPad pad)
    {
        circular = pad.circular;

        if (!circular)
        {
            xBound.x = (pad.position.x - (pad.dimensions.x / 2)) + .2f;
            xBound.y = (pad.position.x + (pad.dimensions.x / 2)) - .2f;

            yBound.x = (pad.position.y - (pad.dimensions.y / 2)) + .2f;
            yBound.y = (pad.position.y + (pad.dimensions.y / 2)) - .2f;
        }
        else
        {
            xBound.x = pad.position.x;
            xBound.y = pad.position.y;
            yBound.x = pad.dimensions.x - .2f;
        }
    }
    #endregion

    #region Test If Point Lies Within Bounds
    public bool testWithin(Vector2 v)
    {
        if (!circular)
        {
            //Debug.Log("not circular. position to check:" + v.x + "," + v.y + ". xB:" + xBound.x + "," + xBound.y + ". yB:" + yBound.x + "," + yBound.y);
            if (v.x >= xBound.x && v.x <= xBound.y && v.y >= yBound.x && v.y <= yBound.y)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            if (Vector2.Distance(v, xBound) <= yBound.x)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }
    #endregion
}
#endregion

#endregion

#region Vector 2 Float Structure
public struct Vector2Float
{
    public decimal x;
    public decimal y;

    public Vector2Float(decimal i, decimal o)
    {
        x = i;
        y = o;
    }

    public static bool operator ==(Vector2Float v1, Vector2Float v2)
    {
        if (v1.x == v2.x && v1.y == v2.y)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public static bool operator !=(Vector2Float v1, Vector2Float v2)
    {
        if (v1.x != v2.x || v1.y != v2.y)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public static decimal decimalAbs(decimal d)
    {
        if (d < 0)
        {
            d = d * -1;
        }
        return d;
    }
}
#endregion

#endregion

#region Extensions
public static class QuaternionExtensions
{
    public static float quaternionMagnitude(Quaternion q)
    {
        float f = 0;
        f = Mathf.Sqrt(Mathf.Pow(q.x, 2) + Mathf.Pow(q.y, 2) + Mathf.Pow(q.z, 2) + Mathf.Pow(q.w, 2));
        return f;
    }

    public static Vector3 gravityCompensation(Quaternion q, Vector3 a)
    {
        Vector3 g = new Vector3(2 * (q.y * q.w - q.x * q.z), 2 * (q.x * q.y + q.z * q.w), q.x * q.x - q.y * q.y - q.z * q.z + q.w * q.w);
        return new Vector3((float)Math.Round(a.x - g.x, 1), (float)Math.Round(a.y - g.y, 1), (float)Math.Round(a.z - g.z, 1));
    }

    public static Vector3 gravityCompensationMethod2(Quaternion q, Vector3 a)
    {
        Vector3 g = q * a;
        Vector3 vO;
        vO.x = (float)Math.Round(a.x - g.x, 1);
        vO.y = (float)Math.Round(a.y - g.y, 1);
        vO.z = (float)Math.Round(a.z - g.z, 1);
        return vO;
    }

    public static Vector3 localToWorld(Quaternion q, Vector3 a)
    {
        Vector3 g = new Vector3(2 * (q.y * q.w - q.x * q.z), 2 * (q.x * q.y + q.z * q.w), q.x * q.x - q.y * q.y - q.z * q.z + q.w * q.w);
        return new Vector3((float)Math.Round(a.x - g.x, 3), (float)Math.Round(a.y - g.y, 3), (float)Math.Round(a.z - g.z, 3));
    }

    public static Quaternion GetQuaternion(Vector3 a, Vector3 g, Vector3 m, Quaternion q, ulong dT)
    {
        Quaternion qO = new Quaternion();
        float q1 = q.x, q2 = q.y, q3 = q.z, q4 = q.w;   // short name local variable for readability
        float norm;
        float hx, hy, _2bx, _2bz;
        float s1, s2, s3, s4;
        float qDot1, qDot2, qDot3, qDot4;
        float deltat;
        float beta = Mathf.Sqrt(3.0f / 4.0f) * Mathf.PI * (40.0f / 180.0f);
        float zeta = Mathf.Sqrt(3.0f / 4.0f) * Mathf.PI * (0.0f / 180.0f);

        // Auxiliary variables to avoid repeated arithmetic
        float _2q1mx;
        float _2q1my;
        float _2q1mz;
        float _2q2mx;
        float _4bx;
        float _4bz;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;
        float _2q4 = 2.0f * q4;
        float _2q1q3 = 2.0f * q1 * q3;
        float _2q3q4 = 2.0f * q3 * q4;
        float q1q1 = q1 * q1;
        float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;
        float q1q4 = q1 * q4;
        float q2q2 = q2 * q2;
        float q2q3 = q2 * q3;
        float q2q4 = q2 * q4;
        float q3q3 = q3 * q3;
        float q3q4 = q3 * q4;
        float q4q4 = q4 * q4;
        float gx = g.x, gy = g.y, gz = g.z;
        float ax = a.x, ay = a.y, az = a.z;
        float mx = a.x, my = a.y, mz = a.z;
        gx *= Mathf.PI / 180.0f;
        gy *= Mathf.PI / 180.0f;
        gz *= Mathf.PI / 180.0f;

        // updateTime()
        deltat = (dT / 1000.0f); // set integration time by time elapsed since last filter update

        // Normalise accelerometer measurement
        norm = Mathf.Sqrt(ax * ax + ay * ay + az * az);
        if (norm == 0.0f) return Quaternion.identity; // handle NaN
        norm = 1.0f / norm;
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Normalise magnetometer measurement
        norm = Mathf.Sqrt(mx * mx + my * my + mz * mz);
        if (norm == 0.0f) return Quaternion.identity; // handle NaN
        norm = 1.0f / norm;
        mx *= norm;
        my *= norm;
        mz *= norm;

        // Reference direction of Earth's magnetic field
        _2q1mx = 2.0f * q1 * mx;
        _2q1my = 2.0f * q1 * my;
        _2q1mz = 2.0f * q1 * mz;
        _2q2mx = 2.0f * q2 * mx;
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
        _2bx = Mathf.Sqrt(hx * hx + hy * hy);
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        norm = Mathf.Sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
        norm = 1.0f / norm;
        s1 *= norm;
        s2 *= norm;
        s3 *= norm;
        s4 *= norm;

        // Compute rate of change of quaternion
        qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
        qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
        qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
        qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

        // Integrate to yield quaternion
        q1 += qDot1 * deltat;
        q2 += qDot2 * deltat;
        q3 += qDot3 * deltat;
        q4 += qDot4 * deltat;
        norm = Mathf.Sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
        norm = 1.0f / norm;
        qO = new Quaternion(q1 * norm, q[1] = q2 * norm, q[2] = q3 * norm, q[3] = q4 * norm);
        return qO;
    }

    public static float calculateHeading(Vector3 m)
    {
        float compassHeading = 0;
        if (m.y == 0)
        {
            if (compassHeading == m.x)
            {
                compassHeading = Mathf.PI;
            }
            else
            {
                compassHeading = 0;
            }
        }
        else
        {
            compassHeading = (float)Math.Atan2(m.x, m.y);
        }

        if (compassHeading > Mathf.PI)
        {
            compassHeading -= 2 * Mathf.PI;
        }
        else if (compassHeading < -Mathf.PI)
        {
            compassHeading += 2 * Mathf.PI;
        }
        else if (compassHeading < 0)
        {
            compassHeading += 2 * Mathf.PI;
        }

        compassHeading *= 180.0f / Mathf.PI;

        return compassHeading;
    }

    public static Vector3 calculateYPR(Quaternion q)
    {
        Vector3 ypr = new Vector3();
        float[] matRot = new float[5];
        matRot[0] = 2.0f * (q.y * q.z + q.x * q.w);
        matRot[1] = q.x * q.x + q.y * q.y - q.z * q.z - q.w * q.w;
        matRot[2] = 2.0f * (q.x * q.y + q.z * q.w);
        matRot[3] = 2.0f * (q.y * q.w - q.x * q.z);
        matRot[4] = q.x * q.x - q.y * q.y - q.z * q.z + q.w * q.w;
        ypr.y = -Mathf.Asin(matRot[3]);
        ypr.z = (float)Math.Atan2(matRot[2],  matRot[4]);
        ypr.x = (float)Math.Atan2(matRot[0], matRot[1]);
        ypr.y *= 180.0f / Mathf.PI;
        ypr.z *= 180.0f / Mathf.PI;
        ypr.x *= 180.0f / Mathf.PI;
        ypr.x += 5.483333f;
        if (ypr.x >= +180.0f) ypr.x -= 360.0f;
        else if (ypr.x < -180.0f) ypr.x += 360.0f;

        return ypr;
    }
    
}

public static class VectorTransformations
{
    public static Vector2 rotate(Vector2 vIn, float angle)
    {
        angle = Mathf.Deg2Rad * angle;
        Vector2 vOut = new Vector2();

        vOut.x = (vIn.x * Mathf.Cos(angle)) - (vIn.y * Mathf.Sin(angle));
        vOut.y = (vIn.x * Mathf.Sin(angle)) + (vIn.y * Mathf.Cos(angle));

        return vOut;
    }

    public static Vector3 vectorRounding(Vector3 v, int n)
    {
        return new Vector3((float)Math.Round(v.x, n), (float)Math.Round(v.y, n), (float)Math.Round(v.z, n));
    }
}
#endregion

