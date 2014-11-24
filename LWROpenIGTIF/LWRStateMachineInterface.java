/*=========================================================================

  Program:   LWRStateMachineInterface
  Language:  java
  Web page: http://www.slicer.org/slicerWiki/index.php/Documentation/Nightly/Extensions/LightWeightRobotIGT

  Copyright (c) Sebastian Tauscher. Institute of Mechatronics System, Leibniz Universitaet Hannover. All rights reserved.

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


import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.math.BigInteger;
import java.net.ServerSocket;
import java.nio.ByteBuffer;

import javax.net.ServerSocketFactory;



import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;

import OpenIGTLink.swig.ByteArr;
import OpenIGTLink.swig.IGTLheader;
import OpenIGTLink.swig.igtl_header;

/**
 * This Class for the Communication with a control system using the opnIGTLink protocol is based on the igtlink4j class 
 * developed at the WPI. For Further information see WPI website.
 * The LWRStateMachineInterface Thread is operated with a 20ms cycle. An Example of the use of this class for the 
 * communication with a State Control Software e.g. 3D Slicer see imesStateApllication. According of the data type of the OpenIGTMessage
 * the data is packed and send to the state control. Supported data types are "STATUS", "STRING" and "TRANSFORM". Later on "POINT" will be supported.
 * @author Sebastian Tauscher
 * @see SimpleStateExample
 */
public class LWRStateMachineInterface extends Thread {
	
	/**
	 * Load SWIG igtlutil library (Default Library folder is "..\OpenIGTLinkLib\swig\"
	 */
	static 
	{
		System.load("C:/KRC/ApplicationServer/Git/IGTBasicStateMachine/OpenIGTLinkLib/SWIGigtlutil.dll");
	}
	/**
	 * String containing the data type of the received OpenIGTLink message
	 */
	public String IGTLdatatype = "STRING";

	/**
	 * Time step for statistic timing of the State Machine Interface thread
	 */
	private OneTimeStep aStep;

	/**
	 * Statistic Timer for the State Machine Interface Thread.
	 */
	public StatisticTimer SMtiming = new StatisticTimer();
	
	/**
	 * OpenIGTLink Client socket - socket of the connected Client
	 */
	 private java.net.Socket openIGTClient = null;
	 
	 /**
	  * Server socket for the communication with state control interface 
	  */
     private ServerSocket openIGTServer;
     
     /**
      * output stream of the socket communication
      */
     private OutputStream outstr;
     /**
      * input stream of the socket communication
      */
     private InputStream instr;
     
     /**
      *error flag to indicate if an error occurred
      */
     @SuppressWarnings("unused")
	private boolean ErrorFlag= false;
     
     /**
      * Enum for the current client status {CONNECTED, DISCONNECTED }possible client states
      */
     private static enum ClientStatus {CONNECTED, DISCONNECTED }; //possible client states
     
     /**
      * current status of the client status
      */
     @SuppressWarnings("unused")
	private ClientStatus currentStatus = ClientStatus.DISCONNECTED; //start as stopped status
     
     /**
      * Current unified Identification number
      */
	public long UID = 0;
	 /**
     * Current unified Identification number (working copy)
     */
	private long UID_local = 0;
	
	/**
	 * Old current unified identification number
	 */
	private long UID_old = 0;
	/**
	 * Number of detected repetitions of the unified identification number
	 */
	public int UIDrepeatNum=0;
	
	/**
	 * Number of maximun repetitions in a row
	 */
	public int UIDrepeat_max=0;
	
	/**
	 * Command OpenIGTLink Message used for data transfer to the state machine default message is "IDLE;"
	 */
	public String CMD_StateM = "IDLE;";
	
	/**
	 * Acknowledgement OpenIGTLink Message used for data transfer to the state machine default message is "IDLE;"
	 */
	public String ACK_StateM  =  "IDLE;";
	
	/**
	 * Acknowledgement OpenIGTLink Message working copy
	 */
 	private String ACKmessage;
 	
 	/**
	 * Command OpenIGTLink Message working copy
	 */
	private String CMDmessage = "IDLE;";
	
	/**
	 * Error Message Handler which takes care of the time consuming Error output in a separate thread
	 */
	private IGTMessageHandler ErrorHandler;
	
	/**
	 * Semaphore for save reading and writing of the the variables e.g. CMD_StateM.
	 */
    public Semaphore ControlSemaphore = new Semaphore(1,true);
    
    /**
	 * cycle time of the state control interface thread. Default value is 20 ms
	 */
	public int millisectoSleep = 50;
	
    /**
     * port number for the communication with state control. Supported ports: 49001 - 49005 
     */
	public int port = 49001;
	
	/**
	 * Flag to indicate if the Debug Information should be printed or not.
	 */
	public boolean DebugInfos=false;
	/**
	 * Flag to indicate if the transform from image space to robot base coordinate is received or not 
	 */
	public boolean TransformRecieved= false;
	
	/**
	 * Flag to indicate if the communication interface is running or not
	 */
    public boolean ControlRun = false;
    
    /**
	 * Number of missed UID numbers in total
	 */
    public long UIDmiss = 0;
    
    /**
     * This integer is set to true if an connection error occurs.
     */
    private int ConnectionError=0;
    /**
	 * Number loops getting the same UID number (in a row)
	 */
    private int UIDrepeat = 0;
    
    /**
   	 * delay loops between receiving and sending a packet with the same UID (should be 0)
   	 */
    private long UIDdelay = 0;

   /**
    * Transformation matrix from image space to robot base coordinate system.
    */
   public MatrixTransformation TransformImageRobot;
    
   /**
    * Error code
    */
   public int ErrorCode = 0;

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
        	System.out.println("State machine interface server Socket succesfully created (port " + this.port + ")");
        } catch (IOException e) {
        	System.out.println("Could not Connect to port :" + this.port + ")");
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
				System.out.println("State machine interface server stopped");
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}
    
/**
 * Constructor
 */
    public LWRStateMachineInterface() {
    	setDaemon(true);
    };
    
    /**
     * Finalize method of this class
     */
    public void finalize(){
    	if(openIGTServer!= null){
			try {
				openIGTServer.close();
				openIGTServer= null;
				openIGTClient.close();
				openIGTClient = null;
				System.out.println("State machine interface server stopped");
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
    	
    }

/**
*main/run method  function of the State control Interface. In this function the server is initialized and a packet handler is started.
* In a loop with a cycle time of 20 ms the new Command String is received and the Acknowledgment String send to the state control (e.g. 3d Slicer).
**/
    public void run() {
		// Init the ErrorHandler 
    	ErrorHandler = new IGTMessageHandler();
    	ErrorHandler.setPriority(2);
    	ErrorHandler.Sendername = "State Control Interface:";
    	ErrorHandler.DebugInfos = DebugInfos;
    	ErrorHandler.start();
 

		
		//Initializing the Communication with the Visualization Software
		try {
			//Set up server
			ConnectServer(port);
			ControlRun = true;
			openIGTClient = openIGTServer.accept();
			openIGTClient.setTcpNoDelay(true);
			openIGTClient.setSoTimeout(millisectoSleep);
	        this.outstr =openIGTClient.getOutputStream();
	        this.instr = openIGTClient.getInputStream();
            this.currentStatus = ClientStatus.CONNECTED; 
			System.out.println("State machine interface client connected ( " + openIGTClient.getInetAddress()+ ", " + openIGTClient.getPort() +")");
			
			//Sending first Acknowledgment String to start the cyclic communication
			try {
				
				ControlSemaphore.acquire();
				ACKmessage =ACK_StateM ;
				ControlSemaphore.release();
			} catch (InterruptedException e) {
				ErrorFlag = true;
				ErrorHandler.ErrorMessage = "StateMachineIF: Unable to Acquire Control Semaphore";
			}
			try {
				if( !openIGTClient.isClosed() ){
					sendIGTStringMessage(ACKmessage + UID_local +";");
				}
			} catch (Exception e1) {
				ErrorFlag = true;
				ErrorHandler.ErrorMessage = "StateMachineIF: Couldn't Send ACk data";
			}
		}catch (Exception e) {
			ErrorHandler.ErrorMessage = "Couldn't connect to state machine interface server!";
		}
		
		//Entering Loop for Communication - the loop is stopped if ControlRun is set to false
		while(ControlRun){
			//Starting Time 
			long startTimeStamp = (long) (System.nanoTime());
		    
			aStep = SMtiming.newTimeStep();
			
			if( !openIGTClient.isClosed() &&ConnectionError <100){
				 ErrorFlag= false;
				try {
						//Receive Message from State Control
						receiveMessage();
						
						// Write data into the public String CMD_StateM
						try {
							ControlSemaphore.acquire();
							CMD_StateM =CMDmessage;
							UID = UID_local;
							ControlSemaphore.release();
						} catch (InterruptedException e) {
							ErrorFlag = true;
							try {
								ErrorHandler.MessageSemaphore.tryAcquire(2, TimeUnit.MILLISECONDS);
								ErrorHandler.ErrorMessage = "StateMachineIF:Unable to Acquire Control Semaphore";
								ErrorHandler.MessageSemaphore.release();
							} catch (InterruptedException e1) {
								// TODO Automatisch generierter Erfassungsblock
								e1.printStackTrace();
							}
							
						}
						ConnectionError=0;
				} catch (IOException e1) {
					// TODO Automatisch generierter Erfassungsblock
					ErrorFlag = true;
					try {
						ErrorHandler.MessageSemaphore.tryAcquire(2, TimeUnit.MILLISECONDS);
						ErrorHandler.ErrorMessage = "StateMachineIF: Receive data timeout!!";
						ErrorHandler.MessageSemaphore.release();
					} catch (InterruptedException e) {
		
					}
		
					ConnectionError++;
				}
			}else{ // If there is an connection error stop listening server and restart the server
				RestartIGTServer();
			}
					
			try {
				Thread.sleep((long) millisectoSleep/2 +1);
			} catch (InterruptedException e) {
				// TODO
				ErrorFlag = true;
				 ErrorHandler.ErrorMessage = "StateMachineIF: Failed thread sleep!";
			}
			
			try {
				ControlSemaphore.acquire();
				ACKmessage =ACK_StateM ;
				ControlSemaphore.release();
			} catch (InterruptedException e) {
				// TODO 
				ErrorFlag = true;
				 ErrorHandler.ErrorMessage = "StateMachineIF: Unable to Acquire Control Semaphore";
			}
			if( !openIGTClient.isClosed() && ConnectionError <100 ){
					try {
						sendIGTStringMessage(ACKmessage + UID_local +";");
						ConnectionError =0;
					} catch (Exception e1) {
						ConnectionError++;
						ErrorFlag = true;
						ErrorHandler.ErrorMessage = "StateMachineIF: Couldn't Send ACk data";
					}
			}else{
				RestartIGTServer();
			}
				
			//Set the Module in Sleep mode for stability enhancement
			long curTime = (long) ((System.nanoTime() - startTimeStamp));
			long curTime_millis = (long) curTime/1000000;
			int curTime_nanos = (int) (curTime%1000000);
			if(curTime_millis<millisectoSleep-2){
				//ThreadUtil.milliSleep((long) Math.floor((millisectoSleep-1 -  curTime)));
				try {
					Thread.sleep(millisectoSleep-2-curTime_millis, 999999-curTime_nanos);
				} catch (InterruptedException e) {
					
					ErrorHandler.ErrorMessage ="Thread Sleep failed!";
				}
			}
			
			aStep.end();
			
				
			if(SMtiming.getMaxTimeMillis() >=  10* millisectoSleep || SMtiming.getMeanTimeMillis() >= 2*millisectoSleep){
				 ErrorHandler.ErrorMessage = "StateMachineIF: Attention! Bad communication quality robot changes state to Error!";
				ErrorCode = 18;
			}else if( (SMtiming.getMaxTimeMillis() >  3.0* millisectoSleep && SMtiming.getMaxTimeMillis() < 10.0* millisectoSleep) ||  (SMtiming.getMeanTimeMillis() > millisectoSleep + 5 && SMtiming.getMeanTimeMillis() < 2*millisectoSleep)){
				ErrorFlag = true;
				 ErrorHandler.ErrorMessage ="StateMachineIF: Warning bad communication quality!";
			}
		}//end while
	}
    
    /**
     * Function to restart the IGTLink Server and reinitialize the connection. This function is used if the connection to the state control client got lost.
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
			ControlRun = true;
			openIGTClient = openIGTServer.accept();
			openIGTClient.setTcpNoDelay(true);
			openIGTClient.setSoTimeout(millisectoSleep);
	        this.outstr =openIGTClient.getOutputStream();
	        this.instr = openIGTClient.getInputStream();
            this.currentStatus = ClientStatus.CONNECTED; 
            ErrorHandler.ErrorMessage = "State machine interface client connected ( " + openIGTClient.getInetAddress()+ ", " + openIGTClient.getPort() +")";
			ConnectionError=0;
			ErrorCode = 0;
		}catch (Exception e) {
			ErrorHandler.ErrorMessage ="Couldn't connect to state machine interface server!";
			ErrorCode = 18;
		}

    	
    }
		
    /**
  	 * In this function a message from the Client Socket is received and according to the OpenIGTLink datatype the recieved data is saved in the member variable CMDmessage.
  	 * If the data type is neither Transform nor String an ErrorMessage is created. (Error Message not used yet...)
  	 * 
  	 * @throws IOException
  	 */
      public void receiveMessage() throws IOException{
  	    int ret_read = 0;
  	    byte[] HeaderByte = new byte[IGTLheader.IGTL_HEADER_SIZE];
  	    boolean ReceivedNewData = false;
  	    String messageType = null;
  	    String UID_String = "";
  	    long UID_max = 999999999999L;
  	    int BodySize= 0;
  	    String DeviceName = null;
  	    byte[] bodyBytes = null;
  	    if(UID_old == UID_max){
	    	UID_old = -1;
	    }
	    UID_old = UID_local;

   	   while(!ReceivedNewData /*&& instr.available()>0*/){
	  	    ret_read = instr.read(HeaderByte);
	  	    int enddata = 0;
	  	    byte[] TmpName = new byte [IGTLheader.IGTL_HEADER_TYPE_SIZE];
	  	    int k =0;
	  	    for(int i =2; i<2 + IGTLheader.IGTL_HEADER_TYPE_SIZE  && enddata == 0; i++){
	  		   TmpName[k] = HeaderByte[i];
	  		   if (HeaderByte[i] == 0){
	  	    		enddata = k;
	  	    	}
	  		   k++;
	  	    }
	  	    messageType = new String(TmpName).substring(0, enddata);
	  	    IGTLdatatype = messageType;
	  	    byte[] TmpDeviceName = new byte[IGTLheader.IGTL_HEADER_DEVSIZE];
	  	    int l =0;
	  	    enddata = 0;
	  	    for(int j =14; j<14 + IGTLheader.IGTL_HEADER_DEVSIZE && enddata == 0; j++){
	  	    	TmpDeviceName[l] = HeaderByte[j];
	  	    	if (HeaderByte[j] == 0){
	  	    		enddata = l;
	  	    	}
	  	    	l++;
	  		   }
	  	    DeviceName = new String(TmpDeviceName).substring(0, enddata); 
	  	   
	  	    byte[] TmpBodySize = new byte[8];
	  	    int m =0;
	  	    for(int h = 42; h< 42 + 8/*IGTLheader.IGTL_HEADER_NAME_SIZE*/; h++){
	  	    	TmpBodySize[m] = HeaderByte[h];
	  	    	m++;
	  		   }
	  	   //int BodySize =  Integer.parseInt(new String(TmpBodySize));
	  	    BigInteger BI = new BigInteger(TmpBodySize);
	  	    BodySize = BI.intValue();


	  	    if (ret_read > 0) {
  	           
  	            bodyBytes = new byte[BodySize];
  	           
  	            if ( BodySize > 0) {
  	                    ret_read = instr.read(bodyBytes);
  	                    if (ret_read !=BodySize) {
  	                            //errorManager.error("ServerThread bodyBuf in ServerThread ret_read = " + ret_read, new Exception("Abnormal return from reading"), ErrorManager.SERVERTHREAD_ABNORMAL_ANSWER);
  	                    }
  	            }
		          if(messageType.equalsIgnoreCase("STRING") && DeviceName.split("_").length >= 2 ){
			           UID_String = DeviceName.split("_")[1];
				       UID_local = Integer.parseInt(UID_String);
				      
			        }else if(messageType.equals("TRANSFORM") ){
			        	ReceivedNewData = true;
			        }else{
			    	  ErrorHandler.ErrorMessage = "State machine interface: Unexpected command name structure - expected is CMD_UID!!";
			    	  ErrorFlag = true;
			    	  
			        }
				    if(UID_local>UID_old){
			      	  ReceivedNewData = true;
			        }
	  	    } 
   	   }

      if (messageType.equalsIgnoreCase("STRING")){
          UIDdelay = UID_local-UID_old;
          if(UIDdelay ==0){
        	  if(UIDrepeat ==0){
        		  UIDrepeatNum++;
        	  }
        	  UIDrepeat ++;
        	  if(UIDrepeat_max<UIDrepeat){
        		  UIDrepeat_max=UIDrepeat;
        	  }
        	  if(UIDrepeat>=4){
        		  ErrorHandler.ErrorMessage = "State machine interface: UID has not changed for the " + UIDrepeat + ". time!! Check state control!";
	        	  ErrorCode = 18;
	        	  ErrorFlag = true;
        	  }
          }else if(UIDdelay>1){
        	  UIDmiss= UIDmiss + UIDdelay -1;
        	  ErrorHandler.ErrorMessage = "State machine interface: missed UID!!(miss count: " + UIDmiss + ")";
        	  ErrorFlag = true;
        	  
          }else if(UIDdelay == 1){
        	  UIDrepeat = 0;
          }
         
          
    	  byte [] TmpString = new byte [BodySize-4];
    	  int p =0;
    	 for(int z = 4; z< BodySize; z++){
      	    	TmpString[p] = bodyBytes[z];
      	    	p++;
      		   }
    	  try {
			String CMDString =new String(TmpString);
			CMDmessage = CMDString;
    	  } catch (Exception e) {
			// TODO Automatisch generierter Erfassungsblock
    		  ErrorHandler.ErrorMessage =" Couldn't generate new OpenIGTLink String Message!!";
    		  ErrorFlag = true;
    	  }
             
  
      }else if (messageType.equalsIgnoreCase("TRANSFORM")){
    	  try {
    		  ByteBuffer bodyBuff = ByteBuffer.wrap(bodyBytes);
    		  	double [] R_tmp = new double [9];
    		  	double [] t_tmp = new double [3];
    		  	for (int i = 0; i < 12; i++) {
    		  		if(i<9){
    		  			R_tmp[i] = bodyBuff.getDouble(i * 8);
    		  		}else if(i>=9 && i <12){
    		  				t_tmp[i-9] = bodyBuff.getDouble(i * 8);
    		  		}
    		  	}
    		  	TransformImageRobot = MatrixTransformation.of(Vector.of(t_tmp[0], t_tmp[1], t_tmp[2]), Matrix.ofRowFirst(R_tmp[0], R_tmp[1], R_tmp[2], R_tmp[3], R_tmp[4], R_tmp[5], R_tmp[6], R_tmp[7], R_tmp[8]));
    		  	IGTLdatatype = "TRANSFORM";
    		  	ErrorHandler.ErrorMessage = "Transform to Image space succesfully received:" + TransformImageRobot;
				TransformRecieved = true;
				
        	  } catch (Exception e) {
        		  ErrorHandler.ErrorMessage =" Couldn't generate new OpenIGTLink Transform Message!!";
        		  ErrorFlag = true;
        	  }
        	  
      }else{
    	  ErrorHandler.ErrorMessage =  "State machine interface: Unexpected Data type received!!";
    	  ErrorFlag = true;
      }
  	       
      UID_old = UID_local;

      }
	
	
    /**
     * Sends an OpenIGTlink String message
     * @param message
     * @throws Exception
     */

	
	public void sendIGTStringMessage(String message) throws Exception {
		byte [] BodyByte = new byte [message.length() + 4];
		byte [] HeaderByte = new byte [IGTLheader.IGTL_HEADER_SIZE];
		igtl_header header = new  igtl_header();
		 header.setVersion(IGTLheader.IGTL_HEADER_VERSION);
        header.setBody_size( (BigInteger.valueOf(message.length() + 4)));
        header.setName("STRING"); 
        header.setDevice_name("ACK");   /* Device name */
        header.setTimestamp(BigInteger.valueOf(System.nanoTime()));
		 ByteBuffer BodyBuffer = ByteBuffer.allocate(message.length() + 4);
		 BodyBuffer.putShort((short) 3);
		 BodyBuffer.putShort((short) message.length());
		 BodyBuffer.put(message.getBytes());


		BodyByte = BodyBuffer.array();
	    ByteArr BodyArr =new ByteArr(message.length() + 4);
        for(int i =0;  i<message.length() + 4; i++){
        	BodyArr.setitem(i,BodyByte[i]);
        }
        ByteArr HeaderArr = ByteArr.frompointer(IGTLheader.PackHeader(header, BodyArr.cast()));
        for(int i =0; i<IGTLheader.IGTL_HEADER_SIZE; i++){
        	HeaderByte[i] = (byte) HeaderArr.getitem(i);
        }
        
       try {
			sendBytes(HeaderByte);
			sendBytes(BodyByte);
		} catch (IOException e) {
			// TODO Automatisch generierter Erfassungsblock
		}
  
			
	}


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



}
