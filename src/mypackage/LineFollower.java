package mypackage;

import java.io.IOException;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.List;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
 
public class LineFollower {
	public static void main(String[] args) throws Exception {
        
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);        
        //Variables in matrix type to be used for image processing 
        Mat mat = new Mat();
        Mat grayScale = new Mat();
        Mat thresholdImg = new Mat();
        Mat notused = new Mat();
        Mat dilateImg = new Mat();
        Mat dilate = new Mat();  
        Mat gaussianBlur = new Mat();
        
        //Variables to be used for line following operation
        int horiMid = 0;
        int frameCount = 0;
        int roiMiddle = 80;
        int offset = 0;
        int tachoCount = 0;
        int countWidth = 0;
        int cnt = 0;
        float distance = 0;
        
        //Open the default camera for capturing image sequences. Parameter 0 is used to open the default camera if only one camera is plugged in
        VideoCapture vid = new VideoCapture(0);
        
        //Set the frame dimension to be used for image processing
        vid.set(Highgui.CV_CAP_PROP_FRAME_WIDTH, 160);
        vid.set(Highgui.CV_CAP_PROP_FRAME_HEIGHT, 120);
        
        //Open capturing device for video capturing
        vid.open(0);        
        LCD.drawString("Camera Open", 1, 0);  
        
        //Get a port number in this case 8080, for server socket binding to make a TCP connection
        ServerSocket ss = new ServerSocket(8080);
        Socket sock = ss.accept();
        Delay.msDelay(2000);
        LCD.drawString("Socket Connected", 1, 1);   
        String boundary = "Thats it folks!";
        writeHeader(sock.getOutputStream(), boundary);
            
        //Setup the motor connected to their respective port in the EV3 brick. Driving motor is connected to port A and steering motor is connected to port B
        NXTRegulatedMotor steerMotor = Motor.B;
        NXTRegulatedMotor driveMotor = Motor.A;
        
        //Set the acceleration and speed of drive motor 
        driveMotor.setAcceleration(30);
        driveMotor.setSpeed(100);
      
        LCD.clear(0);
        @SuppressWarnings("resource")
        
        //Setup the ultrasonic sensor connnected to sensor port 3 to be used for obstacle detection
        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S3); 
        
        //Get the system current time to derive frame per second 
        long stime = System.currentTimeMillis();
        
        //Enter the line detection and following process until down button is pressed to exit 
        while (!Button.ESCAPE.isDown()) {
        	//Get the number of frame captured to derive frame per second
        	frameCount+= 1;  
        	
        	//Measure the distance to an object in front of the sensor
        	final SampleProvider sp = ultrasonicSensor.getDistanceMode();       	
        	float [] sample = new float[sp.sampleSize()];
		    sp.fetchSample(sample, 0);
		    
		    //Convert default unit type from meter to cm
		    distance = sample[0] * 100;
		    
		    //Keep the line follower running if object infront is not within the range of 25cm
		    if(distance >25) {
		    	driveMotor.forward();	        	
				
	        	//set the initial deviation of vehicle from the center of line to 0
	        	offset = 0;
	        	horiMid = 0;
	        	
	        	//Grab, decodes the data and return the image frame into a matrix
	            vid.read(mat);
	             
	            //Region of interest to apply image processing
	            Mat roi = new Mat(mat, new Rect( 0, 2*mat.rows()/3, 160, mat.rows()/12));	                        		
	            Core.rectangle(mat, new Point(0,80), new Point(160,90), new Scalar(0, 255, 0));
	            
	            //To convert colour image to grayscale image
	            Imgproc.cvtColor(roi, grayScale, Imgproc.COLOR_BGR2GRAY);
	            
	            //To remove noise in image using gaussian blur function and kernel size of 9 by 9 for smoothing operation
	            Imgproc.GaussianBlur(grayScale, gaussianBlur, new Size(9, 9), 0, 0);
	            
	            //Perform image thresholding for image segmentation. To remove shadow and foreign objects that might affect the detection of line on surface
	            Imgproc.threshold(gaussianBlur, thresholdImg, 100, 255, Imgproc.THRESH_BINARY_INV);
	            
	            //Morphological function to fill in incomplete and breaks in line
	            Imgproc.dilate(thresholdImg, dilateImg, dilate);
	          	            
	            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
	            
	            //Find the presence of contours in images
				Imgproc.findContours(dilateImg, contours, notused, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
				Imgproc.drawContours(roi, contours, -1, new Scalar(0,255,255), 1);
				
				//If the presence of contours, executes the line following procedure
				for(MatOfPoint cont : contours)
			    { 			
					//Get the weighted average of image pixels intensities 
					Moments mu = Imgproc.moments(cont, false);
			        
					//To keep the camera at a certain height while detecting for line
			        if (mu.get_m00() > 5)
			        {			            		            
			            //Create a bounding boxes for contour detected
			        	Rect boundingRect = Imgproc.boundingRect(cont);			         
			       		
			        	//Get the mid point of the bounding rectangle
			            horiMid = boundingRect.x + boundingRect.width/2;
			            
			            Core.line(roi, new Point(horiMid,boundingRect.y), new Point(horiMid,boundingRect.y + boundingRect.height), new Scalar(0,0,255));
			           
			           	//Update the steer motor to center the position of vehicle with the line detected during every 7th frame
			            if(frameCount == 7) {			           					           					           		
			           					           		
			           		//calculate the distance the vehicle is deviating from the center of line
			           		offset = horiMid - roiMiddle;
			           		
			           		//Rotate the steerMotor corresponding to the deviation to keep the vehicle centered
			           		if((offset < -1 && offset > -80) || (offset > 1 && offset < 80)) {
			           			steerMotor.rotateTo((int) -(offset * 2.3 ));
			           		}
			           		
			           		//Reset the frame count to 0 so that the steerMotor can be updated again in the 7th frame 
				           	frameCount = 0;
			           	}
			           	
			            //Check if there is line on surface with a width of more than 20 pixels that indicates a temporary stop
			           	if (boundingRect.width > 20) {
			           		//Count to keep track of the length of wide line, only wide line that is long enough indicates a temporary stop
			           		countWidth += 1;
			           	} else {
			           		//Count set to 0 if no wide line is detected
			           		countWidth = 0;
			           	}
			           	
			           	//Stop the driveMotor if the wide line marking is long enough that indicates a stop
			           	if (countWidth == 15) {
		           			driveMotor.stop();
		           			
		           			//Wait for 5 seconds and continue moving forward
		           			Thread.sleep(5000);
		           			
		           			//reset the wide line detected
		           			countWidth = 0;		           			
		           		}
			           	
			           	//Get the values for steerMotor rotation angle to be displayed on EV3 brick
			           	tachoCount = steerMotor.getTachoCount();				            
			        }		               		        		      	        		        
			    }
				// to clear the LCD display screen on EV3 brick before printing new values
				LCD.clear();
				
				//to display the frame per second for performance analysis
				LCD.drawString("fps: " + (++cnt * 1000f/(System.currentTimeMillis() - stime)), 1,0);
				
				//to print the labels of each attributes
				printLabels(roiMiddle, offset, horiMid, tachoCount, distance);	
				
				//convert images to Jpeg format and sends it to PC to be displayed
		        writeJpg(sock.getOutputStream(), mat, boundary);
		    } else {
		    	//Stop the drive motor upon detecting obstacle within the range in 25cm
		    	Motor.A.stop();
		    }       		       
        }
        //calibrate steering angle to center when the down button is pressed
        calibrateSteeringAngle(); 
        
        //close to server socket 
        sock.close();
        ss.close();    
    }
    
	//function to write URL header
    private static void writeHeader(OutputStream stream, String boundary) throws IOException {
        stream.write(("HTTP/1.0 200 OK\r\n" +
                "Connection: close\r\n" +
                "Max-Age: 0\r\n" +
                "Expires: 0\r\n" +
                "Cache-Control: no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0\r\n" +
                "Pragma: no-cache\r\n" + 
                "Content-Type: multipart/x-mixed-replace; " +
                "boundary=" + boundary + "\r\n" +
                "\r\n" +
                "--" + boundary + "\r\n").getBytes());
    }
    
    //function to write image capture to Jpeg format to be displayed in the form of continuous images at the web browser
    private static void writeJpg(OutputStream stream, Mat img, String boundary) throws IOException {
        MatOfByte buf = new MatOfByte();
        Highgui.imencode(".jpg", img, buf);
        byte[] imageBytes = buf.toArray();
        stream.write(("Content-type: image/jpeg\r\n" +
                "Content-Length: " + imageBytes.length + "\r\n" +
                "\r\n").getBytes());
        stream.write(imageBytes);
        stream.write(("\r\n--" + boundary + "\r\n").getBytes());
    }
    
    //function to print labels on the EV3 brick LCD display
    private static void printLabels(int roiMiddle, int offset, int boundingRectPosition, int tachoCount, float distance) {  	
    	LCD.drawString("roiMiddle: " + roiMiddle, 1, 1);   	
		LCD.drawString("Offset: " +  offset, 1,2);
		LCD.drawString("Bound Rect: " + boundingRectPosition, 1, 3);
		LCD.drawString("Tacho Count: " + tachoCount, 1, 4);
		LCD.drawString("Distance:" + distance, 1, 5);
    }
    
    //function to calibrate steering angle
    private static void calibrateSteeringAngle() {
       	 Motor.B.rotateTo(0);
    }
 }