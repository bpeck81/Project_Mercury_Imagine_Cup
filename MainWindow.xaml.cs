//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------
//-Brandon Peck 4/15/2015

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using System.Runtime.InteropServices;
    using Microsoft.Kinect;
    using Microsoft.Speech.AudioFormat;
    using Microsoft.Samples;
    using Microsoft.Speech.Recognition;
    using System.Windows.Documents;
    using System.Speech.Synthesis;
    using System.Linq;
    using System.Runtime.InteropServices.WindowsRuntime;
    using System.Windows.Controls;
    using System.Threading.Tasks;
    using Microsoft.WindowsAzure.Storage.Blob;
    using Microsoft.WindowsAzure.Storage.Auth;
    using Twilio;
    using System.Net.Mail;
    using Microsoft.WindowsAzure.Storage;



    public enum DisplayFrameType
    {
        bodyTracking,
        Color
    }




    public partial class MainWindow : Window, INotifyPropertyChanged
    {

        #region AddedMembers
        CameraSpacePoint[] floorPlanePoints;
        int theCount;
        List<CameraSpacePoint> headPoints;
        CameraSpacePoint[] headPts;
        public float fallConfidence;
        Stopwatch watch;
        Plane floorPlane;
        Plane theFloor;
        float startedTrackingState;
        int theInnerCount;
        int fallDetectedLoopCount;
        Boolean fallDetected;
        public double elapsedFallTime;
        string username;
        int healthScanCount;
        Boolean hitEdge;


        SpeechRecognitionEngine speechEngine;
        KinectAudioStream convertStream;

        int colorRunCount;

        public int[] boundedBox;
        private WriteableBitmap colorBitmap;
        WriteableBitmap colorInstantImage;
        public byte[] colorArray;
        double maxAveVal, minAveVal;
        byte[] colorPixels;
        ColorFrameReader colorReader;
        private readonly int bytePerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;
        SpeechSynthesizer synth;
        int responseToFallCount;
        Stopwatch incapacitatedStopWatch;
        Boolean scanningHealthMode;
        Boolean userIncapacitated;

        private const DisplayFrameType DEFAULT_DISPLAYFRAMETYPE = DisplayFrameType.bodyTracking;
        private FrameDescription currentFrameDescription;
        private DisplayFrameType currentDisplayFrameType;
        private MultiSourceFrameReader multiSourceFrameReader = null;

        #endregion




        private const double HandSize = 30;

        private const double JointThickness = 3;

        private const double ClipBoundsThickness = 10;

        private const float InferredZPositionClamp = 0.1f;

        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        private readonly Brush inferredJointBrush = Brushes.Yellow;

        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        private DrawingGroup drawingGroup;

        private DrawingImage imageSource;

        private KinectSensor kinectSensor = null;

        private CoordinateMapper coordinateMapper = null;

        private BodyFrameReader bodyFrameReader = null;

        private Body[] bodies = null;

        private List<Tuple<JointType, JointType>> bones;

        private int displayWidth;

        private int displayHeight;

        private List<Pen> bodyColors;

        private string statusText = null;



        public MainWindow()
        {
            #region AddedMembersInit
            this.username = App.UserName;
            headPoints = new List<CameraSpacePoint>();
            headPts = new CameraSpacePoint[2];
            floorPlanePoints = new CameraSpacePoint[3];
            theCount = 0;
            hitEdge = false;
            fallDetectedLoopCount = 0;
            watch = new Stopwatch();
            watch.Start();
            theInnerCount = 0;
            fallDetected = false;
            responseToFallCount = 0;
            userIncapacitated = false;
            incapacitatedStopWatch = new Stopwatch();
            maxAveVal = 0;
            colorRunCount = 0;
            minAveVal = 0;
            healthScanCount = 0;
            scanningHealthMode = false;


            this.getStaticBox();



            #endregion


            this.kinectSensor = KinectSensor.GetDefault();

            #region speech Region
            synth = new SpeechSynthesizer();
            synth.Volume = 100;
            synth.SelectVoiceByHints(VoiceGender.Female);
            synth.Rate = 1;

            #endregion

            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            this.multiSourceFrameReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Body | FrameSourceTypes.Color);

            this.multiSourceFrameReader.MultiSourceFrameArrived += this.Reader_MultiSourceFrameArrived;

            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            this.kinectSensor.Open();
             SetupCurrentDisplay(DEFAULT_DISPLAYFRAMETYPE);



            this.drawingGroup = new DrawingGroup();

            this.imageSource = new DrawingImage(this.drawingGroup);

            this.DataContext = this;

            this.InitializeComponent();


        }


        private void SetupCurrentDisplay(DisplayFrameType newDisplayFrameType)
        {
            currentDisplayFrameType = newDisplayFrameType;
            switch (currentDisplayFrameType)
            {
                case DisplayFrameType.bodyTracking:
                     this.currentFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
                    this.displayWidth = this.currentFrameDescription.Width;
                    this.displayHeight = this.currentFrameDescription.Height;
                    this.setupBodyJoints();
                    break;
                case DisplayFrameType.Color:
                    if (this.kinectSensor.ColorFrameSource.FrameDescription != null)
                    {
                        this.currentFrameDescription = this.kinectSensor.ColorFrameSource.FrameDescription;
                    }
                   // this.currentFrameDescription = colorFrameDescription;
                    this.colorBitmap = new WriteableBitmap(this.currentFrameDescription.Width, this.currentFrameDescription.Height, 96, 96, PixelFormats.Bgr32, null);
                    colorPixels = new byte[this.currentFrameDescription.Width * this.currentFrameDescription.Height * bytePerPixel];
                    break;

                default:
                    break;
            }

        }
        private void setupBodyJoints()
        {
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));
        }


        public event PropertyChangedEventHandler PropertyChanged;


        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }



        }
        public void getStaticBox()
        {
            this.boundedBox = new int[1038720];
            int count = 0;
            for (int j = (1920 * 4 * 270) - (1200 * 4); j < (1920 * 810 * 4); j += (1920 * 4)) //starts at beginning of box  goes to end of box
            {
                for (int i = 0; i < 480 * 4; i++)
                {
                    this.boundedBox[count] = j + i;
                    count++;
                }


            }
        }




        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            this.setUpAudio();


        }
        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();
            if (multiSourceFrame == null)
            {
                return;
            }
            switch(currentDisplayFrameType)
            {
                case DisplayFrameType.bodyTracking:
                    using(BodyFrame bodyFrame = multiSourceFrame.BodyFrameReference.AcquireFrame()){
                                    bool dataReceived = false;

                        showBodyFrame(bodyFrame, dataReceived);
                    }

                    break;
                case DisplayFrameType.Color:
                    using (ColorFrame colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame())
                    {
                        if (colorFrame != null)
                        {
                            this.showColorFrame(colorFrame);
                        }
                    }

                    break;
                default:
                    break;
            }

        }

        private void setUpAudio()
        {
            IReadOnlyList<AudioBeam> audioBeamList = this.kinectSensor.AudioSource.AudioBeams;
            System.IO.Stream audioStream = audioBeamList[0].OpenInputStream();
            this.convertStream = new KinectAudioStream(audioStream);
            RecognizerInfo ri = MainWindow.GetKinectRecognizer();
            if (ri != null)
            {
                this.speechEngine = new SpeechRecognitionEngine(ri.Id);
                var choices = new Choices();

                choices.Add(new SemanticResultValue("I am okay", "OKAY"));
                choices.Add(new SemanticResultValue("I'm okay", "OKAY"));
                choices.Add(new SemanticResultValue("I'm fine", "OKAY"));
                choices.Add(new SemanticResultValue("yes", "OKAY"));



                choices.Add(new SemanticResultValue("I want a Health Scan", "SCAN"));
                choices.Add(new SemanticResultValue("Turn off body", "BODYOFF"));



                choices.Add(new SemanticResultValue("Help", "HELP"));
                choices.Add(new SemanticResultValue("call", "HELP"));
                choices.Add(new SemanticResultValue("Help me", "HELP"));
                choices.Add(new SemanticResultValue("no", "HELP"));




                var gb = new GrammarBuilder { Culture = ri.Culture };
                gb.Append(choices);
                var g = new Grammar(gb);
                this.speechEngine.LoadGrammar(g);
                this.speechEngine.SpeechRecognized += this.SpeechRecognized;
                this.speechEngine.SpeechRecognitionRejected += this.SpeechRejected;
                this.convertStream.SpeechActive = true;
                this.speechEngine.SetInputToAudioStream(
                this.convertStream, new SpeechAudioFormatInfo(EncodingFormat.Pcm, 16000, 16, 1, 32000, 2, null));
                this.speechEngine.RecognizeAsync(RecognizeMode.Multiple);
            }

        }

        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            Console.Write("wuddup");
            if (this.multiSourceFrameReader != null)
            {
                this.multiSourceFrameReader.Dispose();
                this.multiSourceFrameReader = null;
            }
           

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }
        public void openColorCamera()
        {
            this.SetupCurrentDisplay(DisplayFrameType.Color);

           // this.currentDisplayFrameType = DisplayFrameType.Color;




        }
        public void closeColorCamera()
        {
            colorRunCount = 0;
            this.SetupCurrentDisplay(DisplayFrameType.bodyTracking);

          //  this.currentDisplayFrameType = DisplayFrameType.bodyTracking;
          //  this.returnToIdle();


        }
        private void returnToIdle()
        {
            healthScanCount = 0;
            this.scanningHealthMode = false;
            this.fallDetected = false;
            this.userIncapacitated = false;
        }


        private static RecognizerInfo GetKinectRecognizer()
        {
            IEnumerable<RecognizerInfo> recognizers;
            try
            {
                recognizers = SpeechRecognitionEngine.InstalledRecognizers();
            }
            catch (COMException)
            {
                return null;
            }

            foreach (RecognizerInfo recognizer in recognizers)
            {
                string value;
                recognizer.AdditionalInfo.TryGetValue("Kinect", out value);
                if ("True".Equals(value, StringComparison.OrdinalIgnoreCase) && "en-US".Equals(recognizer.Culture.Name, StringComparison.OrdinalIgnoreCase))
                {
                    return recognizer;
                }
            }
            return null;
        }

        private void SpeechRecognized(object sender, SpeechRecognizedEventArgs e)
        {
            double upperConfidenceThreshold = .3;
            double lowerConfidenceThereshold = .2;

          //  Console.WriteLine(e.Result.Confidence);
            if (fallDetected == true && userIncapacitated == false)                      
            {

                if (e.Result.Confidence >= upperConfidenceThreshold)
                {
                    String response = e.Result.Semantics.Value.ToString();
                    if (response.Equals("OKAY"))
                    {
                        this.healthScan();

                    }
                    else if (response.Equals("HELP"))
                    {
                        
                        this.incapacitationDetected();
                    }
                }

            }
            //else if (fallDetected == false)
            //{
            //    if (e.Result.Confidence >= upperConfidenceThreshold)
            //    {
            //        String response = e.Result.Semantics.Value.ToString();
            //        if (response.Equals("INQUIRY"))
            //        {
            //            synth.Speak("What can I help you with?");
            //        }
            //        if (response.Equals("SCAN"))
            //        {
            //            this.healthScan();
            //        }
            //    }
            //}
        }
        private void SpeechRejected(object sender, SpeechRecognitionRejectedEventArgs e)
        {

        }
        
        public void healthScan()
        {   if(healthScanCount ==0){
            scanningHealthMode = true;
            this.incapacitatedStopWatch.Stop();
            this.incapacitatedStopWatch.Reset();

            //switch to color camera
          //  synth.SpeakAsync("Let's double check with a health scan. Please match the box that is bounded around your head with the stationary box on screen.");
           
            synth.SpeakAsync("Good to hear that you're okay. Please be careful!");
            
        }
        healthScanCount++;

       //     this.returnToIdle();
            //if match call eulerian

        }
        public byte[] alterBitmap(byte[] colorPixels, int height, int width) // 1920 /1080
        {

            //alter here
            eulerian(colorPixels);




            return colorPixels;
        }
        public void eulerian(byte[] colorPixels)
        {

            int count = 0;
            byte[] relevantcolorStreamValues = new byte[boundedBox.Length];
            for (int i = 0; i < colorPixels.Length; i++)
            {
                if (count < boundedBox.Length)
                {
                    if (i == boundedBox[count])
                    {
                        relevantcolorStreamValues[count] = colorPixels[i];
                        count++;
                    }
                }

            }
            int[] relevantRedValues = new int[boundedBox.Length / 4];
            count = 0;
            for (int i = 2; i < relevantcolorStreamValues.Length; i += 4)
            {
                relevantRedValues[count] = (int)relevantcolorStreamValues[i];
                count++;
            }

            var averageVal = relevantRedValues.Average();

            if (colorRunCount == 5)
            {
                maxAveVal = averageVal;
                minAveVal = averageVal;
            }
            if (averageVal > maxAveVal) maxAveVal = averageVal;
            if (averageVal < minAveVal) minAveVal = averageVal;
            double deltaVal = maxAveVal - minAveVal;

            //    Console.WriteLine(deltaVal);

        }
        public void incapacitationDetected()
        {
            userIncapacitated = true;
            incapacitatedStopWatch.Stop();
            this.incapacitatedStopWatch.Reset();
            
            synth.SpeakAsync("I have confirmed your fall. I am now contacting Emergency Responders. Dont worry, help is on the way" + App.UserName);
            
            this.openColorCamera();


            //send text

        }
        private void showColorFrame(ColorFrame colorFrame)
        {             
            //FrameDescription desc = colorFrame.FrameDescription;
                
                if (this.currentFrameDescription.Width == this.colorBitmap.PixelWidth && this.currentFrameDescription.Height == colorBitmap.PixelHeight)
                {
                    if (colorFrame.RawColorImageFormat == ColorImageFormat.Bgra)
                    {
                        colorFrame.CopyRawFrameDataToArray(colorPixels);
                    }
                    else colorFrame.CopyConvertedFrameDataToArray(colorPixels, ColorImageFormat.Bgra);

                    colorPixels = this.alterBitmap(colorPixels, colorBitmap.PixelHeight, colorBitmap.PixelWidth);

                    colorBitmap.WritePixels(new Int32Rect(0, 0, this.currentFrameDescription.Width, this.currentFrameDescription.Height), colorPixels, this.currentFrameDescription.Width * bytePerPixel, 0);
                    if (colorRunCount == 1)
                    {
                        //this.saveImage("mercpics/mercpic.png", this.colorBitmap);
                        if (App.UserNumber.Length == 10)
                        {
                            this.SendTextWithImage();
                         //   this.connectPhoneCall();

                        }

                        this.closeColorCamera();
                      //  this.colorInstantImage = this.colorBitmap;
                    }




                }
                colorRunCount++;

            }

        

        private void showBodyFrame(BodyFrame bodyFrame, bool dataReceived)
        {
            if (fallDetected)
            {
                elapsedFallTime = this.incapacitatedStopWatch.ElapsedMilliseconds * .001;
                if (elapsedFallTime >= 8 && responseToFallCount ==0)
                {
                    incapacitatedStopWatch.Stop();
                    synth.SpeakAsync("No response detected");
                    this.incapacitationDetected();
                }
                
            }
            if (bodyFrame != null)
            {
                this.theFloor = new Plane(bodyFrame.FloorClipPlane); //get floor plane

                if (this.bodies == null)
                {
                    this.bodies = new Body[bodyFrame.BodyCount];
                }

                bodyFrame.GetAndRefreshBodyData(this.bodies);
                dataReceived = true;
            }


            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];
                        FrameEdges clippedEdges = body.ClippedEdges;

                            if (body.IsTracked)
                            {


                                this.DrawClippedEdges(body, dc);
                                IReadOnlyDictionary<JointType, Joint> joints = body.Joints;



                                detectFall(body, theFloor,clippedEdges);



                                Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();


                                foreach (JointType jointType in joints.Keys)
                                {

                                    CameraSpacePoint position = joints[jointType].Position;
                                    if (position.Z < 0)
                                    {
                                        position.Z = InferredZPositionClamp;
                                    }

                                    DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);


                                    jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                                }


                                this.DrawBody(joints, jointPoints, dc, drawPen);

                                this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                                this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                                theCount++;
                            }
                        
                    }

                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }
            


        private void detectFall(Body body, Plane theFloor, FrameEdges clippedEdges)
        {
            //condifdence proportional to head off ground
            //factor1 distance of head off ground
            if (this.hitEdge == true) { return; }
            
            var distanceToGround = this.headOffFloor(body.Joints[JointType.Head].Position, this.theFloor);

            float fallVelocity = 0;

            if (theCount % 1 == 0) //changing intervals of velocity detection
            {
                watch.Stop();

                headPoints.Add(body.Joints[JointType.Head].Position); //needs clean up
                if (headPoints.Count > 2)
                {
                    fallVelocity = (float)(2 * ((headPoints[headPoints.Count - 1].Y) - (headPoints[headPoints.Count - 2].Y)) / (watch.ElapsedMilliseconds * .001));
                    //will implement velocity and confidence as an additional factor
                }


                watch.Reset();
                watch.Start();
            }
            //  Console.WriteLine(fallVelocity);

            //  Console.WriteLine(distanceToGround); 

            if (distanceToGround <= .5f) //60cm threshold for detection
            {
                if (fallDetected != true)
                {
                    this.fallDetected = true;
                    incapacitatedStopWatch.Reset();
                    incapacitatedStopWatch.Start();

                    
                        String question = "I have detected a fall. " + App.UserName + "Are you alright?";
                        synth.SpeakAsync(question);
                 
                        fallDetectedLoopCount++;



                }
            }

            if (this.scanningHealthMode == true && distanceToGround > 1f)
            {
                Console.WriteLine("Stood up");
               

               // scanningHealthMode = false;
                this.returnToIdle();
            }
            if (this.userIncapacitated == true && distanceToGround > 1f)
            {
                Console.WriteLine("Stood up");
                this.returnToIdle();
            }
            {

            }

        }


        public void getfloorPlane(Body body)
        {
            if (body.Joints[JointType.FootLeft] != null && body.Joints[JointType.FootRight] != null && theInnerCount == 0)
            {
                Joint leftFoot = body.Joints[JointType.FootLeft];
                Joint rightFoot = body.Joints[JointType.FootRight];
                leftFoot.Position.X += .5f;
                leftFoot.Position.Y += .5f;
                rightFoot.Position.X += .5f;
                rightFoot.Position.Y += .5f;

                floorPlanePoints[0] = leftFoot.Position;
                floorPlanePoints[1] = rightFoot.Position;
                startedTrackingState = theCount;
                theInnerCount++;
            }

            CameraSpacePoint leftFootPosState = body.Joints[JointType.FootLeft].Position;
            leftFootPosState.X += .5f;
            leftFootPosState.Y += .5f;
            if (leftFootPosState.Equals(floorPlanePoints[0]) == false)
            {
                Joint leftFoot2 = body.Joints[JointType.FootLeft];
                leftFoot2.Position.X += .5f;
                leftFoot2.Position.Y += .5f;
                floorPlanePoints[2] = leftFoot2.Position;
                floorPlane = new Plane(floorPlanePoints);
            }

            Joint head = body.Joints[JointType.Head];
            head.Position.X += .5f;
            head.Position.Y += .5f; //.5 sets positive axis' to bottom left corner

            if (floorPlane != null)
            {
                if (floorPlane.checkPoint(head.Position))
                {
                    Console.WriteLine("RED ALERT!");
                }
            }
        }



        public float headOffFloor(CameraSpacePoint headPoint, Plane floorPlane)
        {
            var z = (-1 * floorPlane.definedPlane.W) / floorPlane.definedPlane.Z;
            CameraSpacePoint randoPP;
            randoPP.X = 0; randoPP.Y = 0; randoPP.Z = z;
            Vector3 PPtoHP = new Vector3(randoPP, headPoint);
            var distance = Vector3.compProjection(PPtoHP, floorPlane.normal);

            return distance;
        }




        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }


        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }                                                                                      


        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }
  


        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
           //     this.hitEdge = true;
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
             //   this.hitEdge = true;

                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
               // this.hitEdge = true;

                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
               // this.hitEdge = true;

                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
            if (clippedEdges.HasFlag(FrameEdges.None))
            {
                this.hitEdge = false;
            }
        }

        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {

        }

        public void saveImage(string filename, BitmapSource image5)
        {
            if (filename != string.Empty)
            {
                using (FileStream stream5 = new FileStream(filename, FileMode.Create))
                {
                    PngBitmapEncoder encoder5 = new PngBitmapEncoder();
                    encoder5.Frames.Add(BitmapFrame.Create(image5));
                    encoder5.Save(stream5);
                    stream5.Close();
                }
            }
        }
        private string[] saveCurrentImage()
        {
            if (this.colorBitmap != null)
            {
                string[] PathAndTime = new string[2];
                // create a png bitmap encoder which knows how to save a .png file
                BitmapEncoder encoder = new PngBitmapEncoder();

                // create frame from the writable bitmap and add to encoder
                encoder.Frames.Add(BitmapFrame.Create(this.colorBitmap));

                string time = System.DateTime.Now.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

                string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);

                string path = Path.Combine(myPhotos, "KinectScreenshot-Color-" + time + ".png");
                PathAndTime[0] = path;
                PathAndTime[1] = time;
                // write the new file to disk
                try
                {
                    // FileStream is IDisposable
                    using (FileStream fs = new FileStream(path, FileMode.Create))
                    {
                        encoder.Save(fs);
                    }


                }
                catch (IOException)
                {
                    return PathAndTime;
                }
                return PathAndTime;
            }
            return new string[1];
        }
        private void SendTextWithImage()
        {

            var PathAndTime = saveCurrentImage();

            string filename = "KinectScreenshot-Color-" + PathAndTime[1] + ".png";

            var credentials = new StorageCredentials("projectmercury",
                                        "nwfOOBW8N0SvFeTFhZ4E673xVpqV8q6q71gy6f/9K24c5008+tJd2GjjU9V7ydAeVrj5ZYPL4j9YkMU61Lxx5Q==");
            var client = new CloudBlobClient(new Uri("http://projectmercury.blob.core.windows.net/"), credentials);

            // Retrieve a reference to a container. (You need to create one using the mangement portal, or call container.CreateIfNotExists())
            var container = client.GetContainerReference("photos");

            // Retrieve reference to a blob named "myfile.gif".
            var blockBlob = container.GetBlockBlobReference(String.Format("{0}", filename));

            // Create or overwrite the "myblob" blob with contents from a local file.
            using (var fileStream = System.IO.File.OpenRead(@PathAndTime[0]))
            {
                blockBlob.UploadFromStream(fileStream);
            }

            string AccountSid = "Removed For Privacy Purposes";
            string AuthToken = "Removed For Privacy Purposes";

            var twilio = new TwilioRestClient(AccountSid, AuthToken);
            var message = twilio.SendMessage("Removed For Privacy Purposes"+1" + App.UserNumber, App.UserName +" has fallen and requires assitance!", new string[] { "http://projectmercury.blob.core.windows.net/photos/" + filename });

            Console.WriteLine("message sent");
            //this.fallDetected = false;

        }
        private void connectPhoneCall()
        {
            string AccountSid = "Removed For Privacy Purposes";
            string AuthToken = "Removed For Privacy Purposes";
            var phonenumber = "Removed For Privacy Purposes"
            var client = new TwilioRestClient(AccountSid, AuthToken);
            var call = client.InitiateOutboundCall(phonenumber, App.UserNumber, "http:.//demo.twilio.com/welcome/voice/");
            if (call.RestException == null)
            {
             //   synth.SpeakAsync("Call not recieved");
            }
        }
        private void reset_clicked(object sender, RoutedEventArgs e)
        {  
            this.Close();

        }
    }

}
