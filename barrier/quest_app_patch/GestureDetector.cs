using System;
using System.Collections.Generic;

using UnityEngine;
using UnityEngine.UI;

using NetMQ;
using NetMQ.Sockets;

class GestureDetector : MonoBehaviour
{
    // Hand objects
    public OVRHand LeftHand;
    public OVRHand RightHand;
    public OVRSkeleton LeftHandSkeleton;
    public OVRSkeleton RightHandSkeleton;
    public OVRPassthroughLayer PassthroughLayerManager;
    private List<OVRBone> RightHandFingerBones;

    // Menu and RayCaster GameObjects
    public GameObject MenuButton;
    public GameObject ResolutionButton;
    public GameObject HighResolutionButton;
    public GameObject LowResolutionButton;

    public GameObject WristTracker;
    private GameObject LaserPointer;
    private LineRenderer LineRenderer;


    // Hand Usage indicator
    public RawImage StreamBorder;

    // Stream Enablers
    bool StreamRelativeData = true;
    bool StreamAbsoluteData = false;


    //Newly added
    //bool HighResolution = false;
    //bool LowResolution = false;
    bool StreamResolution= true;
    public HighResolutionButtonController HighResolutionButtonController;
    public LowResolutionButtonController LowResolutionButtonController;




    // Network enablers
    private NetworkManager netConfig;
    private PushSocket client;
    private PushSocket client2;
    private PushSocket client3;
    private string communicationAddress;
    private string ResolutionAddress;

    private string PauseAddress;

    private string state;
    private string pauseState;
    private bool connectionEstablished = false;

    private bool resolutionconnectionEstablished = false;

    private bool ShouldContinueArmTeleop = false;

    private bool PauseEstablished = false;
    private bool resolutioncreated = false;
    private bool PauseCreated = false;

    // Starting the server connection
    public void CreateTCPConnection()
     {
        // Check if communication address is available
        communicationAddress = netConfig.getKeypointAddress();
        bool AddressAvailable = !String.Equals(communicationAddress, "tcp://:");

        if (AddressAvailable)
        {
            // Initiate Push Socket
            client = new PushSocket();
            client.Connect(communicationAddress);
            connectionEstablished = true;
        }

        // Setting color to green to indicate control
        if (connectionEstablished)
        {
            StreamBorder.color = Color.green;
            ToggleMenuButton(false);
        } else
        {
            StreamBorder.color = Color.red;
            ToggleMenuButton(true);
        }
    }

   

   

    public void ToggleMenuButton(bool toggle)
    {
        MenuButton.SetActive(toggle);
        LineRenderer.enabled = toggle;
    }

    public void ToggleResolutionButton(bool toggle)
    {
        ResolutionButton.SetActive(toggle);
        LineRenderer.enabled = toggle;
    }

    public void ToggleHighResolutionButton(bool toggle)
    {
        HighResolutionButton.SetActive(toggle);
        
    }

    public void ToggleLowResolutionButton(bool toggle)
    {
        LowResolutionButton.SetActive(toggle);


    }
    // Start function
    void Start()
     {
        // Getting the Network Config Updater gameobject
        GameObject netConfGameObject = GameObject.Find("NetworkConfigsLoader");
        netConfig = netConfGameObject.GetComponent<NetworkManager>();

        LaserPointer = GameObject.Find("LaserPointer");
        LineRenderer = LaserPointer.GetComponent<LineRenderer>();

        // Initializing the hand skeleton
        RightHandFingerBones = new List<OVRBone>(RightHandSkeleton.Bones);
        
        
    }


    // Function to serialize the Vector3 List
    public static string SerializeVector3List(List<Vector3> gestureData)
    {
        string vectorString = "";
        foreach (Vector3 vec in gestureData)
            vectorString = vectorString + vec.x + "," + vec.y + "," + vec.z + "|";

        // Clipping last element and using a semi colon instead
        if (vectorString.Length > 0)
            vectorString = vectorString.Substring(0, vectorString.Length - 1) + ":";

        return vectorString;
    }

    // [PATCH 2026-10-27] Hand-tracking confidence trailer (see SendHandData).
    // OVRPlugin.TrackingConfidence is { Low = 0, High = 0x3f800000 }: 0x3f800000 is the IEEE-754 bit pattern of 1.0f,
    // i.e. the native runtime reports confidence as a float in [0,1] and the C# wrapper only names the two ends.
    // Reinterpreting the enum bits as float therefore yields the runtime's value unchanged (0.0 / 1.0 on current
    // firmware, intermediate values if the runtime ever reports them).
    private static float ConfToFloat(OVRHand.TrackingConfidence c)
    {
        float f = BitConverter.ToSingle(BitConverter.GetBytes((int)c), 0);
        if (float.IsNaN(f) || f < 0f) f = 0f;
        if (f > 1f) f = 1f;
        return f;
    }

    public string SerializeTrackingState()
    {
        OVRHand h = RightHand;
        string ts = Time.unscaledTime.ToString("F4");
        if (h == null)
            return "0,0.000,0.000,0.000,0.000,0.000,0.000,0,0," + ts;
        return (h.IsTracked ? 1 : 0) + ","
             + ConfToFloat(h.HandConfidence).ToString("F3") + ","
             + ConfToFloat(h.GetFingerConfidence(OVRHand.HandFinger.Thumb)).ToString("F3") + ","
             + ConfToFloat(h.GetFingerConfidence(OVRHand.HandFinger.Index)).ToString("F3") + ","
             + ConfToFloat(h.GetFingerConfidence(OVRHand.HandFinger.Middle)).ToString("F3") + ","
             + ConfToFloat(h.GetFingerConfidence(OVRHand.HandFinger.Ring)).ToString("F3") + ","
             + ConfToFloat(h.GetFingerConfidence(OVRHand.HandFinger.Pinky)).ToString("F3") + ","
             + (h.IsDataValid ? 1 : 0) + ","
             + (h.IsDataHighConfidence ? 1 : 0) + ","
             + ts;
    }

    public void SendHandData(String TypeMarker)
    {
        // Getting bone positional information
        List<Vector3> rightHandGestureData = new List<Vector3>();
        foreach (var bone in RightHandFingerBones)
        {
            Vector3 bonePosition = bone.Transform.position;
            rightHandGestureData.Add(bonePosition);
        }

        // Creating a string from the vectors
        string RightHandDataString = SerializeVector3List(rightHandGestureData);
        RightHandDataString = TypeMarker + ":" + RightHandDataString;

        // [PATCH 2026-10-27] Append hand-tracking confidence after the trailing ':' so receivers can tell
        // real poses from the SDK's extrapolated / lost-tracking poses. Format (one extra field, comma separated):
        //   tracked,handConf,thumbConf,indexConf,middleConf,ringConf,pinkyConf,dataValid,highConf,unityTime
        //   tracked   : 1 if OVRHand.IsTracked (HandTracked status flag), else 0
        //   *Conf     : float 0.000..1.000 (runtime confidence; enum bits reinterpreted as float, see ConfToFloat)
        //   dataValid : OVRHand.IsDataValid, highConf : OVRHand.IsDataHighConfidence (tracked && hand High)
        //   unityTime : Time.unscaledTime (s) when this frame was sampled
        // Backward compatible: Open-Teach's oculus.py uses split(':')[1] and ignores the rest.
        RightHandDataString += SerializeTrackingState();

        client.SendFrame(RightHandDataString);
        byte[] recievedToken = client.ReceiveFrameBytes();
    }


    public void SendResolution()
    {
       
        //printf("Address available", Address)
       
        // client2.SendFrame("None");
        // byte[] recievedToken = client2.ReceiveFrameBytes();
        ResolutionAddress = netConfig.getResolutionAddress();
        bool Available = !String.Equals(ResolutionAddress, "tcp://:");

        if (Available)
        {   if (!resolutioncreated)
            // Initiate Push Socket
            { 
                Debug.Log("Address Available");
                client2 = new PushSocket();
                client2.Connect(ResolutionAddress);
                resolutionconnectionEstablished = true;
                resolutioncreated=true;
            }
            else 
            {
                resolutionconnectionEstablished=true;
            }
        }
        else
        {
            resolutionconnectionEstablished = false;
        }
        
        if (resolutionconnectionEstablished)
        {
            if (HighResolutionButtonController.HighResolution)
            {   
                state="High";
                client2.SendFrame(state);
                Debug.Log("High Button was clicked!");
                //recievedToken = client2.ReceiveFrameBytes();
                
                //byte[] recievedToken2 = client2.ReceiveFrameBytes();
            }

            else if (LowResolutionButtonController.LowResolution)
            {   
                state="Low";
                client2.SendFrame(state);
                Debug.Log("Low Button was clicked!");
                // byte[] recievedToken2 = client2.ReceiveFrameBytes();
                
                
            }

            else 
            {   

                client2.SendFrame("None"); 
                //recievedToken = client2.ReceiveFrameBytes();  
                Debug.Log("No button was pressed");
                //byte[] recievedToken2 = client2.ReceiveFrameBytes();

            }
        }
        else
        {
            client2.SendFrame("None");
        }
    }

    public void SendResetStatus()
    {
       
        //printf("Address available", Address)
       
        // client2.SendFrame("None");
        // byte[] recievedToken = client2.ReceiveFrameBytes();
        PauseAddress = netConfig.getPauseAddress();
        bool PauseAvailable = !String.Equals(PauseAddress, "tcp://:");

        if (PauseAvailable)
        {   if (!PauseCreated)
            // Initiate Push Socket
            { 
                Debug.Log("Address Available");
                client3 = new PushSocket();
                client3.Connect(PauseAddress);
                PauseEstablished = true;
                PauseCreated=true;
            }
            else 
            {
                PauseEstablished=true;
            }
        }
        else
        {
            PauseEstablished = false;
        }
        
        if (PauseEstablished)
        {
            if (ShouldContinueArmTeleop){
                pauseState = "High";
            } else {
                pauseState = "Low";
            }
            client3.SendFrame(pauseState);
        }
        else 
        {
            
            pauseState="None";
            client3.SendFrame(pauseState);
        }
        

    }
    
    public void SendCont()
    {
       
        //printf("Address available", Address)
       
        // client2.SendFrame("None");
        // byte[] recievedToken = client2.ReceiveFrameBytes();
        PauseAddress = netConfig.getPauseAddress();
        bool PauseAvailable = !String.Equals(PauseAddress, "tcp://:");

        if (PauseAvailable)
        {   if (!PauseCreated)
            // Initiate Push Socket
            { 
                Debug.Log("Address Available");
                client3 = new PushSocket();
                client3.Connect(PauseAddress);
                PauseEstablished = true;
                PauseCreated=true;
            }
            else 
            {
                PauseEstablished=true;
            }
        }
        else
        {
            PauseEstablished = false;
        }
        
        if (PauseEstablished)
        {
           
            pauseState="High";
            client3.SendFrame(pauseState);
            
        }
        else 
        {
            
            pauseState="None";
            client3.SendFrame(pauseState);
        }
        

    }

    public void SendPause()
    {
       
        //printf("Address available", Address)
       
        // client2.SendFrame("None");
        // byte[] recievedToken = client2.ReceiveFrameBytes();
        PauseAddress = netConfig.getPauseAddress();
        bool PauseAvailable = !String.Equals(PauseAddress, "tcp://:");

        if (PauseAvailable)
        {   if (!PauseCreated)
            // Initiate Push Socket
            { 
                Debug.Log("Address Available");
                client3 = new PushSocket();
                client3.Connect(PauseAddress);
                PauseEstablished = true;
                PauseCreated=true;
            }
            else 
            {
                PauseEstablished=true;
            }
        }
        else
        {
            PauseEstablished = false;
        }
        
        if (PauseEstablished)
        {
           
            pauseState="Low";
            client3.SendFrame(pauseState);
            
        }
        else 
        {
            
            pauseState="None";
            client3.SendFrame(pauseState);
        }
        

    }
    

    public void StreamPauser()
    {
        // Switching from Right hand control
        if (LeftHand.GetFingerIsPinching(OVRHand.HandFinger.Middle))
        {
            StreamRelativeData = false;
            StreamAbsoluteData = true;
            StreamResolution= false;
            StreamBorder.color = Color.blue; // Blue for left hand stream
            ToggleMenuButton(false);
            ToggleResolutionButton(false);
            WristTracker.SetActive(true);
            ShouldContinueArmTeleop = true;
            // SendCont();
            
        }

        // Switching from Left hand control
        if (LeftHand.GetFingerIsPinching(OVRHand.HandFinger.Index))
        {
            StreamRelativeData = true;
            StreamAbsoluteData = false;
            StreamResolution = false;
            StreamBorder.color = Color.green; // Green for right hand stream
            ToggleMenuButton(false);
            ToggleResolutionButton(false);
            WristTracker.SetActive(false);
            ShouldContinueArmTeleop = false;
            // SendPause();
            
        }

        // Pausing Stream
        if (LeftHand.GetFingerIsPinching(OVRHand.HandFinger.Ring))
        {
            StreamRelativeData = false;
            StreamAbsoluteData = false;
            StreamResolution = false;
            StreamBorder.color = Color.red; // Red color for no stream 
            ToggleMenuButton(true);
            //ToggleResolutionButton(false);
            WristTracker.SetActive(false);
            ShouldContinueArmTeleop = false;
            
        }

        if (LeftHand.GetFingerIsPinching(OVRHand.HandFinger.Pinky))
        {
            StreamRelativeData = false;
            StreamAbsoluteData = false;
            StreamResolution = true;
            StreamBorder.color = Color.black; // Black color for resolution
            ShouldContinueArmTeleop = false;
            ToggleMenuButton(false);
            ToggleResolutionButton(true);
            WristTracker.SetActive(false);
            // SendPause();

        }
    }

    void Update()
    {
        if (connectionEstablished)
        
        {   
            
            SendResolution();
            SendResetStatus();
            if (String.Equals(communicationAddress, netConfig.getKeypointAddress()))
            {   

                StreamPauser();

                

                if (StreamAbsoluteData)
                {   SendHandData("absolute");
                    ToggleResolutionButton(false);
                    // SendCont();
                   
                }

                if (StreamRelativeData)
                {
                    SendHandData("relative");
                    ToggleResolutionButton(false);
                    // SendPause();
                    
                }

                if (StreamResolution)
                {   
                    ToggleHighResolutionButton(true);
                    ToggleLowResolutionButton(true);
                    // SendPause();
                   
                } 
              
            }
            else
            {
                connectionEstablished = false;
            }
        
        } else
        {
            StreamBorder.color = Color.red;
            ToggleMenuButton(true);
            //ToggleResolutionButton(false);
            CreateTCPConnection();
            
            
        }
        //SendResolution();
    }
}