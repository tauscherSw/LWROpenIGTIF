/*=========================================================================

  Program:   LWRVisualizationInterface
  Language:  java
  Web page: http://www.slicer.org/slicerWiki/index.php/Documentation/Nightly/Extensions/LightWeightRobotIGT

  Copyright (c) Sebastian Tauscher, Institute of Mechatronics System, Leibniz Universitaet Hannover. All rights reserved.

	See License.txt for more information
	
	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
	A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
	CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
	EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
	PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
	PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
	LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
	NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
	=========================================================================*/
package LWROpenIGTIF;

import java.io.IOException;
import java.io.OutputStream;
import java.math.BigInteger;
import java.net.ServerSocket;
import java.nio.ByteBuffer;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;


import javax.net.ServerSocketFactory;
import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Rotation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;


import LWROpenIGTIF.IGTMessageHandler;
import OpenIGTLink.swig.*;

/**
 * This Class for the Communication with a Visualization system using the opnIGTLink protocol is based on the igtlink4j class developed at the WPI.
 * @author Sebastian Tauscher
 * @see SimpleStateExample
 */
public class LWRVisualizationInterface extends Thread {
	
	/**
	 * Load SWIG igtlutil library (Default Library folder is "..\OpenIGTLinkLib\swig\"
	 */
	static 
	{
		System.loadLibrary("SWIGigtlutil");
	}

	
	/**
	 * Time step for statistic timing of the Visualization Interface thread
	 */
	private OneTimeStep aStep;
	
	/**
	 * Statistic Timer for the Visualization Interface Thread.
	 */
	public StatisticTimer Visualtiming = new StatisticTimer();
	
	/**
	 * OpenIGTLink Client socket - socket of the connected Client
	 */
	 private java.net.Socket openIGTClient = null;
	 
	 /**
	  * openIGTLink visualization server socket
	  */
     private ServerSocket openIGTServer;
     
     /**
 	 * Error Message Handler which takes care of the time consuming Error output in a separate thread
 	 */
     private IGTMessageHandler ErrorHandler;
     
     /**
      * Output stream for sending the currant transformation or joint angles to visualization software
      */
     private OutputStream outstr;
     
     /**
      * Flag to indicate if an Error occurred during the last cycle
      */
     @SuppressWarnings("unused")
	private boolean ErrorFlag= false;
     
     /**
      * Enum for the client status. Possible states are connected and disconnected
      *
      */
     private static enum ClientStatus {CONNECTED, DISCONNECTED }; //possible client states
     
     /**
      * current client status. Possible states are connected and disconnected
      *
      */
     @SuppressWarnings("unused")
	private ClientStatus currentStatus = ClientStatus.DISCONNECTED; //start as stopped status
     /**
      * Enum for the type of date requested from the visualization Software (Image space, roboter base COF, joint space)
      *       
      */
     public static enum VisualIFDatatypes {IMAGESPACE, ROBOTBASE,JOINTSPACE}; //possible client states
     
     /**
      * Current selected data type to be send to the robot.
      */
     public VisualIFDatatypes datatype = VisualIFDatatypes.ROBOTBASE; //start as stopped status
     
     /**
      * Flag to indicate if the Visualization interface is set active or not.
      */
	public boolean VisualActive = false;
	 /**
     * Flag to indicate if the Visualization interface is running or if the thread is stopped.
     */
	public boolean VisualRun = false;
	/**
	 * Current Cartesian position in robot base coordinate system of the robot
	 */
	public MatrixTransformation cartPose_StateM= null;
	/**
	 * Transformation from robot base coordinate system to image space coordinate system
	 */
	public MatrixTransformation T_IMGBASE_StateM= null;
	
	/**
	 * Current Joint positions
	 */
	public JointPosition jntPose_StateM = null;
	
	/**
	 * Working copy of the transformation from robot base coordinate system to image space coordinate system.
	 */
	public MatrixTransformation T_IMGBASE= MatrixTransformation.IDENTITY;
	
	
	/**
	 * Working copy of the Current Cartesian position in robot base coordinate system of the robot
	 */
	private MatrixTransformation cartPose= null;
	
	/**
	 * Working copy of the Current Joint6 position of the robot
	 */
	private JointPosition jntPose = null;

   /**
    * Error code
    */
	public int ErrorCode=0;
	
	/**
	 * Semaphore for secure acces to the shared variables
	 */
	public Semaphore VisualSemaphore = new Semaphore(1,true);

	/**
     *Port number for the communication with visualization software e.g. 3D Slicer. Possible ports 49001 - 49005 
     */
	public int port = 49002;

	/**
	 * Flag to indicate if the Debug Information should be printed or not.
	 */
	public boolean DebugInfos=false;

	/**
	 * Flag to indicate if force at the tool center point should be sent or not.
	 */
	public boolean SendTCPForce = false;
	
	
	/**
	 * UID of the current pose sent to the visualization software
	 */
	public int PoseUID=0;
	
	/**
	 * UID of the last pose sent to the visualization software
	 */
	public int PoseUIDOldCount = 0;
	
	/**
	 * Working copy of UID
	 */
	private int PoseUIDLocal = 0;
	
	/**
	 * Working copy of old UID
	 */
	private int PoseUIDOldLocal = -1;
	
    /**
     * This integer is set to true if an connection error occurs.
     */
	private int ConnectionError =0;
	
	/**
	 * cycle time of the visualization interface thread. Default value is 25 ms
	 */
	public int millisectoSleep = 25;

	/**
	 * Array of Matrix Transformation used to save the 8 Transformation from the robot base coordinate frame to the nth coordinate system
	 */
	private MatrixTransformation [] T_DH = new MatrixTransformation [8];

	/** 
	 * Vector containing the force estimated at the tool center point by the internal torque sensors
	 */
	public Vector TCPForce;
	
    /**
     * Starts the listening server on the defined port.
     * @param port the port for the communication with state control
     * @throws IOException
     */
	private void ConnectServer(int port) throws IOException{
    	stopServer();
        try {
        	ServerSocketFactory serverSocketFactory = ServerSocketFactory.getDefault();
        	openIGTServer = serverSocketFactory.createServerSocket(this.port);
        	openIGTServer.setReuseAddress(true);
        	System.out.println("Visualization interface server socket succesfully created (port " + this.port + ")");
          
        } catch (IOException e) {
        	System.out.println("Could not Connect to Visualization interface server");
                throw e;
        }
    }
	  
    /**
     * Stops the listening OpenIGTLink server
     * 
     */
	 private void stopServer(){
			if(openIGTServer!= null){
				try {
					openIGTServer.close();
					openIGTServer= null;
					openIGTClient.close();
					openIGTClient = null;
					System.out.println("Visualization interface server stopped");
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
			//socket = null;
			//currentStatus = ServerStatus.STOPPED;
		}
	 public LWRVisualizationInterface(){
		 setDaemon(true);
	 }
	 
	  public void finalize(){
	    	if(openIGTServer!= null){
				try {
					openIGTServer.close();
					openIGTServer= null;
					openIGTClient.close();
					openIGTClient = null;
					System.out.println("Visualization interface server stopped");
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
	    	
	    }
	 /**
	  * In this function the Matrixtransformation for each Joint is calculated from the set of denavit hardenberg parameter
	  */
	private void SetDHTransformation(JointPosition q){
		 MatrixTransformation T_1 = MatrixTransformation.of(Vector.of(0, 0, 160)     , Matrix.ofRowFirst(Math.cos(q.get(0)), -Math.sin(q.get(0)), 0, Math.sin(q.get(0)), Math.cos(q.get(0)) , 0, 0, 0, 1));

	        MatrixTransformation T_2 = MatrixTransformation.of(Vector.of(0, 0, 180)	 , Matrix.ofRowFirst(Math.cos(q.get(1)), 0, Math.sin(q.get(1)), 0, 1, 0, -Math.sin(q.get(1)), 0 , Math.cos(q.get(1))));

	        MatrixTransformation T_3 = MatrixTransformation.of(Vector.of(0, 0, 180)     , Matrix.ofRowFirst(Math.cos(q.get(2)), -Math.sin(q.get(2)), 0, Math.sin(q.get(2)), Math.cos(q.get(2)) , 0, 0, 0, 1));

	        MatrixTransformation T_4 = MatrixTransformation.of(Vector.of(0, 0, 220)	 , Matrix.ofRowFirst(Math.cos(-q.get(3)), 0, Math.sin(-q.get(3)), 0, 1, 0, -Math.sin(-q.get(3)), 0 , Math.cos(-q.get(3))));

	        MatrixTransformation T_5 = MatrixTransformation.of(Vector.of(0, 0, 180)     , Matrix.ofRowFirst(Math.cos(q.get(4)), -Math.sin(q.get(4)), 0, Math.sin(q.get(4)), Math.cos(q.get(4)) , 0, 0, 0, 1));

	        MatrixTransformation T_6 = MatrixTransformation.of(Vector.of(0, 0, 220)	 , Matrix.ofRowFirst(Math.cos(q.get(5)), 0, Math.sin(q.get(5)), 0, 1, 0, -Math.sin(q.get(5)), 0 , Math.cos(q.get(5))));

	        MatrixTransformation T_7 = MatrixTransformation.of(Vector.of(0, 0, 80)      , Matrix.ofRowFirst(Math.cos(q.get(6)), -Math.sin(q.get(6)), 0, Math.sin(q.get(6)), Math.cos(q.get(6)) , 0, 0, 0, 1));

	        MatrixTransformation T_8 = MatrixTransformation.of(Vector.of( 0, 0, 50)	 , Matrix.ofRowFirst(1, 0,0, 0, 1, 0, 0, 0, 1));
	        T_DH[0]= T_1;
	        T_DH[1]=  T_DH[0].compose(T_2);
	        T_DH[2]=  T_DH[1].compose(T_3);
	        T_DH[3]=  T_DH[2].compose(T_4);
	        T_DH[4]=  T_DH[3].compose(T_5);
	        T_DH[5]=  T_DH[4].compose(T_6);
	        T_DH[6]=  T_DH[5].compose(T_7);
	        T_DH[7]=  T_DH[6].compose(T_8);

			 
			 
		 }
		 
		 
	    /**
	    * Main function of the Visualization Interface. In this function the server is initialized and a packet handler is started.
	    * In a loop with a cycle time of 20 ms the new Command String is received and the Acknowledgment String send.
	    **/
	    public void run() {
			// TODO Automatisch generierter Methodenstub
	    	
	    	ErrorHandler = new IGTMessageHandler();
	    	ErrorHandler.setPriority(2);
	    	ErrorHandler.Sendername = "Visualization Interface:";
	    	ErrorHandler.DebugInfos = DebugInfos;
	    	ErrorHandler.start();
	    	
			
			//Initializing the Communication with the Visualization Software
			try {
				//Set up server
				ConnectServer(port);
				VisualRun = true;
				VisualActive = true;
				openIGTClient = openIGTServer.accept();
				openIGTClient.setTcpNoDelay(true);
				openIGTClient.setSoTimeout(10*millisectoSleep);
		        this.outstr =openIGTClient.getOutputStream();
	            this.currentStatus = ClientStatus.CONNECTED; 
	            System.out.println("Visualization interface client connected ( " + openIGTClient.getInetAddress()+ ", " + openIGTClient.getPort() +")");
				
			}catch (Exception e) {
				// TODO Auto-generated catch block
				ErrorFlag = true;
				ErrorHandler.ErrorMessage = "Couldn't connect to Visualization interface server!";
			}
			
			
			while(VisualRun){
			    long startTimeStamp = (long) (System.nanoTime());
				aStep = Visualtiming.newTimeStep();
				if(VisualActive){
					//Get new data from State machine
					try {
						VisualSemaphore.acquire();
						jntPose = jntPose_StateM;
						cartPose= cartPose_StateM;
						PoseUIDLocal = PoseUID;
						if(datatype.equals(VisualIFDatatypes.IMAGESPACE)){
							T_IMGBASE =T_IMGBASE_StateM;
						}
						VisualSemaphore.release();
						//Send the transform to Visualization
						
					} catch (InterruptedException e) {
						ErrorFlag = true;
						ErrorHandler.ErrorMessage = "Unable to Acquire Visual Semaphore";
					}
					
					if( !openIGTClient.isClosed() && ConnectionError <100){
						SendTransform(cartPose, jntPose);
					}else{
						RestartIGTServer();
					}
					

					
				}
				if(PoseUIDLocal == PoseUIDOldLocal){
					ErrorFlag=true;
					ErrorHandler.ErrorMessage = "Visual IF: Getting Old Data from State Machine Thread";
					PoseUIDOldCount++;
				}
				PoseUIDOldLocal = PoseUIDLocal;
				
				//Set the Module in Sleep mode for stability enhancement
				long curTime = (long) ((System.nanoTime() - startTimeStamp));
				long curTime_millis = (long) curTime/1000000;
				int curTime_nanos = (int) (curTime%1000000);
				if(curTime_millis<millisectoSleep-2){
					try {
						Thread.sleep(millisectoSleep-2-curTime_millis, 999999-curTime_nanos);
					} catch (InterruptedException e) {
						ErrorFlag = true;
						ErrorHandler.ErrorMessage ="Visual Thread Sleep failed!!";
					}
				}
				  aStep.end();
				  if(Visualtiming.getMaxTimeMillis() >(double) 3* millisectoSleep || Visualtiming.getMeanTimeMillis() >(double) 2*millisectoSleep ){
						ErrorFlag = true;
						ErrorHandler.ErrorMessage = "VisualIF: Warning bad communication quality!";
						
					}
			}
		  
		}

	/**
	 * In this function the tranform message is packed and send to the openIGTClient by calling the sendMessage function.
	 * @param deviceName - Device Name of the open IGTLink Transform message send to the visualization software
	 * @param t - the transformation to be send
	 */
		
	    /***************************************************************************
	     * Sends bytes
	     * <p>
	     * 
	     * @throws IOException
	     *             - Exception in I/O.
	     *             <p>
	     * @param bytes
	     *            - byte[] array.
	     **************************************************************************/
	    final public synchronized void sendBytes(byte[] bytes) throws IOException {
	            outstr.write(bytes);
	            outstr.flush();
	    }
	    
	    /**
	    * In this function the transform message is packed using the SWIGGED igtl_util classes and send to the openIGTClient by calling the sendMessage function.
	    * @param DeviceName - Device Name of the open IGTLink Transform message send to the visualization software
	    * @param transform - the transformation to be send
	    */
	   		
	public boolean SendIGTLTransform(String DeviceName, float [] transform){
		byte [] BodyByte = new byte [IGTLtransform.IGTL_TRANSFORM_SIZE];
		byte [] HeaderByte = new byte [IGTLheader.IGTL_HEADER_SIZE];
		igtl_header header = new  igtl_header();
		boolean check = false;
		header.setVersion(IGTLheader.IGTL_HEADER_VERSION);
        header.setBody_size( (BigInteger.valueOf(IGTLtransform.IGTL_TRANSFORM_SIZE)));
        header.setName("TRANSFORM"); 
        header.setDevice_name(DeviceName);   /* Device name */
        header.setTimestamp(BigInteger.valueOf(System.nanoTime()));
		 //BodyBuffer = ByteBuffer.wrap(BodyByte);
		 ByteBuffer BodyBuffer = ByteBuffer.allocate(IGTLtransform.IGTL_TRANSFORM_SIZE);
		 for(int i =0; i<12; i++){
		    	BodyBuffer.putFloat(transform[i]);
		    }

		BodyByte = BodyBuffer.array();
	    ByteArr BodyArr =new ByteArr(BodyByte.length);
        for(int i =0;  i<BodyByte.length; i++){
        	BodyArr.setitem(i,BodyByte[i]);
        }
        
        ByteArr HeaderArr = ByteArr.frompointer(IGTLheader.PackHeader(header, BodyArr.cast()));
        for(int i =0; i<IGTLheader.IGTL_HEADER_SIZE; i++){
        	HeaderByte[i] = (byte) HeaderArr.getitem(i);
        }
        
       try {
			sendBytes(HeaderByte);
			sendBytes(BodyByte);
			check= true;
		} catch (IOException e) {
			check= false;
		}
       return check;
		
		
	}


	    
	    /**
	     * Function to restart the IGTLink Server and reinitialize the connection
	     */
	    private void RestartIGTServer(){
	    	
	    	ErrorFlag = true;
			try {
				ErrorHandler.MessageSemaphore.tryAcquire(2, TimeUnit.MILLISECONDS);
				ErrorHandler.ErrorMessage = "StateMachineIF: Lost Connection to Client. Try to reconnect...";
				ErrorHandler.MessageSemaphore.release();
			} catch (InterruptedException e) {

			}
			stopServer();
			try {
				//Set up server
				ConnectServer(port);
				VisualRun = true;
				openIGTClient = openIGTServer.accept();
				openIGTClient.setTcpNoDelay(true);
				openIGTClient.setSoTimeout(1*millisectoSleep);
		        this.outstr =openIGTClient.getOutputStream();
	            this.currentStatus = ClientStatus.CONNECTED; 
	            ErrorHandler.ErrorMessage = "Visual interface client connected ( " + openIGTClient.getInetAddress()+ ", " + openIGTClient.getPort() +")";
				ConnectionError=0;
				ErrorCode = 0;
			}catch (Exception e) {
				ErrorHandler.ErrorMessage ="Couldn't connect to visualisation interface server!";
				ErrorCode = 18;
			}

	    	
	    }
   
	
	/**
	 * Sending the Cartesian or Joint position of the robot.
	 * @param T_curPose the current position of the robot
	 */
	
	private void SendTransform(MatrixTransformation T_curPose, JointPosition curJntPose){

		float[] T_tmp = new float [12];
		T_tmp[9]=(float) T_curPose.getTranslation().getX();
		T_tmp[10]= (float) T_curPose.getTranslation().getY();
		T_tmp[11]= (float) T_curPose.getTranslation().getZ();

		if(SendTCPForce){
			T_tmp[9]= 	(float) T_curPose.getTranslation().getX();
			T_tmp[10]=  (float) T_curPose.getTranslation().getY();
			T_tmp[11]=  (float) T_curPose.getTranslation().getZ();
			 double theta = 0;
			double phi = 0;
			theta = -Math.asin(TCPForce.normalize().getX());
			phi = Math.atan2(TCPForce.normalize().getY(), TCPForce.normalize().getZ());

			 Rotation rot = Rotation.ofRad(0, 0 , theta).compose(Rotation.ofRad(0, phi , 0 ));
			for(int i=0; i<3; i++){
				for(int j=0; j<3; j++){
					T_tmp[i +3*j]=(float) (TCPForce.length()*rot.getMatrix().get(i, j));
				}
			}
			
			SendIGTLTransform("TCPForce", T_tmp);
						
		}
		
		// Checking what data type was requested
		if(datatype.name().contentEquals((VisualIFDatatypes.JOINTSPACE.name()))){ //if joint space data was requested use the first seven values for the joint angles
			
			SetDHTransformation(jntPose);
			for(int njoint = 0; njoint<9; njoint++){
				if(njoint == 8){
					T_tmp[9]= 	(float) T_curPose.getTranslation().getX();
					T_tmp[10]=  (float) T_curPose.getTranslation().getY();
					T_tmp[11]=  (float) T_curPose.getTranslation().getZ();
					for(int i=0; i<3; i++){
						for(int j=0; j<3; j++){
							T_tmp[i +3*j]=(float) T_curPose.getRotationMatrix().get(i, j);
						}
					}
					SendIGTLTransform("T_EE", T_tmp);
					
				}else{
					T_tmp[9]= 	(float)T_DH[njoint].getTranslation().getX();
					T_tmp[10]= 	(float)T_DH[njoint].getTranslation().getY();
					T_tmp[11]= 	(float)T_DH[njoint].getTranslation().getZ();
					 for(int i=0; i<3; i++){
							for(int j=0; j<3; j++){
								T_tmp[i +3*j]=(float)T_DH[njoint].getRotationMatrix().get(i, j);
							}
					}
					 SendIGTLTransform("T_" + 0 + (njoint+1), T_tmp);
				}
				
	
			}
	
		}else if (datatype.name().contentEquals((VisualIFDatatypes.IMAGESPACE.name()))){// if imagespace data was requested the current robot position in image space is calculated and send 
			MatrixTransformation T = T_IMGBASE.compose(T_curPose);
			T_tmp[9]= 	(float)T.getTranslation().getX();
			T_tmp[10]= 	(float)T.getTranslation().getY();
			T_tmp[11]= 	(float)T.getTranslation().getZ();
			for(int i=0; i<3; i++){
				for(int j=0; j<3; j++){
					T_tmp[i +3*j]=(float) T.getRotationMatrix().get(i, j);
				}
			}
			SendIGTLTransform("T_EE", T_tmp);
			
		}else{//if the robot base pose was requested the current position in robot space is send
			for(int i=0; i<3; i++){
				for(int j=0; j<3; j++){
					T_tmp[i +3*j] =(float) T_curPose.getRotationMatrix().get(i, j);
				}
			}
			SendIGTLTransform("T_EE", T_tmp);
		}
		
	}
	
	
}
