/*=========================================================================

  Program:   SimpleStateExample
  Language:  java
  Web page: http://www.slicer.org/slicerWiki/index.php/Documentation/Nightly/Extensions/LightWeightRobotIGT

  Portions (c) Sebastian Tauscher, Institute of Mechatronics Systems, Leibniz Universität Hannover. All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:
	
	  * Redistributions of source code must retain the above copyright notice,
	    this list of conditions and the following disclaimer.
	
	  * Redistributions in binary form must reproduce the above copyright notice,
	    this list of conditions and the following disclaimer in the documentation 
	    and/or other materials provided with the distribution.
	
	  * Neither the name of the Insight Software Consortium nor the names of its 
	    contributors may be used to endorse or promote products derived from this 
	    software without specific prior written permission.
	
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

package application;


import java.util.concurrent.TimeUnit;


import LWROpenIGTIF.LWRStateMachineInterface;
import LWROpenIGTIF.LWRVisualizationInterface;
import LWROpenIGTIF.StateMachine.LWRStatemachine;

import com.kuka.common.StatisticTimer;
import com.kuka.common.ThreadUtil;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.PhysicalObject;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Rotation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;
/**
 * This is an example robot application for an LBR 4 and a Sunrise controller using the LWRVisualization and LWRStatemachineInterface. 
 * For the Communication with the robot the SmartServo interface is used. For further information on this topic see the SmartServo documentation.
 * To use the Visualization- and StateControlInterface class you need to declare the object in the main program of your robotAPI e.g the runRealtimeMotion(IMotionControlMode controlMode)
 * function.
 * <pre>
 * <code>
 * {@code
 *   //Flag to indicate if the Visualization is active or not
 *   boolean VisualON=false;	
 *  
 *  // To set the Visualization active at the start of the program just set the StartVisual flag of the lwrStatemachine Object imesStatemachine to true,
 *  // e.g.  imesStatemachine.StartVisual=true. Else this flag is set if the StateControl sends the Command "Visual;true;img/rob/jnt"
 *  //imesStatemachine.StartVisual=true;
 *
 *  // Declaration of a LWRStateMachineInterface Object for the communication with a State Control using OpenIGTLink
 *  LWRStateMachineInterface SlicerControlIf = new LWRStateMachineInterface();
 *	
 * 	//Declaration of a LWRVisualizationInterface Object for the communication with Visualization using OpenIGTLink e.g. 3D Slicer OpenIGTIF
 *	LWRVisualizationInterface SlicerVisualIF = new LWRVisualizationInterface();	
 *	
 *	//Setting the port for the Control Interface supported ports are 49001 to 49005. Default Value is 49001
 *   SlicerControlIf.port =49001;
 *   
 *   //Setting the port for the Visualization Interface supported ports are 49001 to 49005. Default Value is 49002
 *	SlicerVisualIF.port = 49002;
 *}
 *</code>
 * </pre>
 * After this the SmartServo Motion needs to be initialized (see SmartServo Documentation) and the SlicerControl thread is started.
* <pre>
* {@code
*  	//initializing and starting of the AliveThread for the Communication with the  State controller
*	SlicerControlIf.start();
*	}
 * </pre>
 * After the current position of the robot was read from the SmartServoRuntime the current Position of the imesStatemachine is initialized with these values.
 * * <pre>
* {@code
* 	//Setting the imes State machine member variables such as the control Mode
*	imesStatemachine.curPose= MatrixTransformation.of(SmartServoRuntime.getCartFrmMsrOnController());
*	imesStatemachine.controlMode = controlMode;
*	imesStatemachine.cmdPose = MatrixTransformation.of(SmartServoRuntime.getCartFrmMsrOnController());
*		
*	//Initialize some of the Visualization Interface member variables
*	SlicerVisualIF.jntPose_StateM = SmartServoRuntime.getAxisQMsrOnController();
*	SlicerVisualIF.cartPose_StateM = imesStatemachine.curPose;
*	}
*</pre>
*When these initializing routine is done the main loop is entered. The loop stopps when:
*	- the command Shutdown/End/Quit were received from the state control
*	- when there was no packet received from the state control for _numRuns loops
*
 *
 * @author Sebastian Tauscher
 * @version 0.2
 */
public class SimpleStateExample extends RoboticsAPIApplication {
	@SuppressWarnings("unused")
	private Controller lbrController;
	private LBR imesLBR;
	private PhysicalObject ImesTool;
	private ISmartServoRuntime SmartServoRuntime;
	/**
	 * Object of the state machine interface class for the communication with a state control software using the OpenIGTLink protocol
	 * @see LWRStateMachineInterface
	 */
	private LWRStateMachineInterface SlicerControlIf;
	
	/**
	 * Object of the visualization interface class for the communication with a visualization software using the OpenIGTLink protocol
	 * @see LWRVisualizationInterface
	 */
	private LWRVisualizationInterface SlicerVisualIF ;
 	/**
 	 * Object of the State machine class.
 	 * @see LWRStatemachine
 	 */
	private LWRStatemachine imesStatemachine = new LWRStatemachine();
	/**
	 * number of loops to run with out any communication with the state control.
	 */
    private static int _numRuns = 500;
    
    /**
     * Cyclic time of each loop of the main (state machine) thread.
     */
    int millisectoSleep = 10;
    
    public void dispose(){
    	
    	if (SlicerControlIf != null){
    		SlicerControlIf.finalize();
    	}
    	if (SlicerVisualIF != null){
    		SlicerVisualIF.finalize();
    		
    	}
		
    }
    @Override
    /**
     * In this function the robot, tool etc are initialized
     * 
     **/
    public void initialize()
    {
    	  System.out.println("Initializing Tool and Validate load for SmartServo.");
		// Locate the "first" Lightweight Robot in the system
	    imesLBR = (LBR) ServoMotionUtilities.locateLBR(getContext());
		// FIXME: Set proper Weights or use the plugin feature
	    //The Translation to the Tool Tip in mm
		final double translationOfTool[] ={ -40, 10, 207 };
		//{ 54.5, 0.1, 211.6 };
		
	//and the mass in kg
		final double mass = 0.6;
		
		//First rough guess of the Center of Mass
		final double centerOfMassInMillimeter[] =
		{ -5, 0, 50 };
		
		ImesTool = ServoMotionUtilities.createTool(imesLBR,
	                "ImesTool", translationOfTool, mass,
	                centerOfMassInMillimeter);
		ImesTool.attachTo(imesLBR.getFlange());
        //ServoMotionUtilities.validateCurrentLoadSetting(imesLBR);
		// Reset the Controller on the LBR completely
		// WARNING:: THIS WILL KILL ALL MOTIONS -
		// AND WILL DISTURB OTHER APPLICATIONS RUNNING ON THE VERY SAME
		// CONTROLLER
		// DONT CALL THIS ROUTINE, IF YOU ARE NOT ALONE ON THE SYSTEM
		ServoMotionUtilities.resetControllerAndKILLALLMOTIONS( imesLBR);
		// You may call instead
		// If you just like to acknowledge pending errors
		ServoMotionUtilities.acknowledgeError( imesLBR);

    }
    /**
     * Function to parse a Frame into a MatrixTransformation.
     * @param Pos
     * @return the Matrix Transformation containing the same information as the Frame
     */
    public MatrixTransformation FrameToMatrixTransformation(Frame Pos)
    {

    	Vector Trans = Vector.of(Pos.getX(), Pos.getY(), Pos.getZ());
    	Rotation Rot = Rotation.ofRad(Pos.getAlphaRad(), Pos.getBetaRad(), Pos.getGammaRad());
    	MatrixTransformation T = MatrixTransformation.of(Trans, Rot);
    	return T;

    	
    }

    /**
     * Move to an initial Position WARNING: MAKE SHURE, THAT the pose is
     * collision free
     */
    public void moveToInitialPosition()
    {
    	System.out.println("Move to initial start Position before the State machine Application starts");
		ImesTool.move(ptp(0.0, Math.PI / 180 * 30., 0., -Math.PI / 180 * 60., 0.,
				Math.PI / 180 * 90., 0.).setJointVelocityRel(0.2));
		
		FrameToMatrixTransformation(imesLBR.getCurrentCartesianPosition(ImesTool.getDefaultMotionFrame()));
		/*
		 * Note: The Validation itself justifies, that in this very time
		 * instance, the load parameter setting was sufficient. This does not
		 * mean by far, that the parameter setting is valid in the sequel or
		 * lifetime of this program
		 */
		if (SmartServo.validateForImpedanceMode( imesLBR) != true)
		{
		    System.out
			    .println("Validation of Torque Model failed - correct your mass property settings");
		    System.out
			    .println("RealtimePTP will be available for position controlled mode only, until validation is performed");
		}
    }


/**
 * In this function the communication with the robot via RealTimePTP, 
 * the communication with the Visualization and Control Software (e.g. 3D Slicer, Matlab) 
 * and the State machine it self are operated. 
     * 
     */


    public void runRealtimeMotion(IMotionControlMode controlMode)
    {
    	boolean DebugOut = true;
    String LastPrintedError = "";
    String ErrorMessage = "";
    //Flag to indicate if the Visualization is active or not
    boolean VisualON=false;	
    
    // To set the Visualization active at the start of the program just set the StartVisual flag of the lwrStatemachine Object imesStatemachine to true,
    // e.g.  imesStatemachine.StartVisual=true. Else this flag is set if the StateControl sends the Command "Visual;true;img/rob/jnt"
    imesStatemachine.StartVisual=true;

    // Declaration of a LWRStateMachineInterface Object for the communication with a State Control using OpenIGTLink
    SlicerControlIf = new LWRStateMachineInterface();
	SlicerControlIf.setPriority(6);
	SlicerControlIf.DebugInfos = true;
	//Setting the port for the Control Interface supported ports are 49001 to 49005. Default Value is 49001
    SlicerControlIf.port =49001;
    SlicerControlIf.millisectoSleep = 20;
    
    //Declaration of a LWRVisualizationInterface Object for the communication with Visualization using OpenIGTLink e.g. 3D Slicer OpenIGTIF
   	SlicerVisualIF = new LWRVisualizationInterface();	
   	SlicerVisualIF.setPriority(5);
   	SlicerVisualIF.datatype = LWRVisualizationInterface.VisualIFDatatypes.JOINTSPACE;
   	imesStatemachine.currentVisualIFDatatype=3;
   	SlicerVisualIF.DebugInfos = true;
    //Setting the port for the Visualization Interface supported ports are 49001 to 49005. Default Value is 49002
	SlicerVisualIF.port = 49002;
	SlicerVisualIF.millisectoSleep = 25;

	boolean StatemachineRun = true;
	int i=0;
	
	long curTime=0;
	int curTime_nanos=0;
	long startTimeStamp = System.nanoTime();
	
	
	JointPosition initialPosition = new JointPosition(
		imesLBR.getCurrentJointPosition());
	
	//Initializing the SmartServo
	SmartServo aRealtimeMotion = new SmartServo(initialPosition);
	aRealtimeMotion.useTrace(true);
	// Set the motion properties to 10% of the systems abilities
	aRealtimeMotion.setJointAccelerationRel(1);
	aRealtimeMotion.setJointVelocityRel(1);

	System.out.println("Starting SmartServo Realtime Motion in "
		+ controlMode.getClass().getName());

	// Set the control mode as member of the realtime motion
	ImesTool.getDefaultMotionFrame().moveAsync(	aRealtimeMotion.setMode(controlMode));

	// Fetch the Runtime of the Motion part
	// NOTE: the Runtime will exist AFTER motion command was issued
	SmartServoRuntime  = aRealtimeMotion
		.getRuntime();
	SmartServoRuntime.setMinimumTrajectoryExecutionTime(5e-3);

	
	
	long curTime_millis=0;
	
	//initializing and starting of the AliveThread for the Communication with the  State controller
	System.out.println("Starting Thread for state control communication ");
	SlicerControlIf.start();
	

	try
	{
		//Reading the current a couple of times for safety reasons
		SmartServoRuntime.updateWithRealtimeSystem();
		ThreadUtil.milliSleep(millisectoSleep);
		SmartServoRuntime.updateWithRealtimeSystem();
		ThreadUtil.milliSleep(millisectoSleep);
		
		
		//Setting the imes State machine member variables such as the control Mode
		imesStatemachine.curPose= FrameToMatrixTransformation(SmartServoRuntime.getCurrentCartesianPosition(ImesTool.getDefaultMotionFrame()));
		imesStatemachine.controlMode = controlMode;
		imesStatemachine.cmdPose = FrameToMatrixTransformation(SmartServoRuntime.getCurrentCartesianPosition(ImesTool.getDefaultMotionFrame()));
		
		//Initialize some of the Visualization Interface member variables
		SlicerVisualIF.jntPose_StateM = SmartServoRuntime.getAxisQMsrOnController();
		SlicerVisualIF.cartPose_StateM = imesStatemachine.curPose;
		
	
		//Know entering the main loop. In this loop the Command from the State Control is read, interpreted, 
		//the new parameters calculated and sent to the Robot via SmartServo and the ACknowlegment String is send back to the State Control.
		//Therefore, the LWRStatemachine, the LWRStateMachineInterface and the LWRVisualization objects are used
		
		StatisticTimer timing = new StatisticTimer();
		
	    while (StatemachineRun && i<_numRuns)
	    {
	    //Loop Starting time for statistics
	    startTimeStamp = (long) (System.nanoTime());
	    
		// Timing - draw one step
		OneTimeStep aStep = timing.newTimeStep();
		//Update with Realtime System (LWR)
		try{
			SmartServoRuntime.updateWithRealtimeSystem();
			// Get the measured position in cartesian pose
			imesStatemachine.curPose =FrameToMatrixTransformation(SmartServoRuntime.getCurrentCartesianPosition(ImesTool.getDefaultMotionFrame()));
			// and the measured joint angles
			imesStatemachine.curJntPose =  SmartServoRuntime.getAxisQMsrOnController();
			imesStatemachine.TCPForce = SmartServoRuntime.getExtForceVector();
			imesStatemachine.PoseUID++;
		}catch(Exception e){
			ErrorMessage = "Error: Failed Update with RealtimeSystem!!";
			
		}
		
		
		//if the Visualization Interface is active sending the Current Position to the Visualization
		if (SlicerVisualIF.VisualRun){

			try{
				SlicerVisualIF.VisualSemaphore.tryAcquire(1, TimeUnit.MILLISECONDS);
				SlicerVisualIF.PoseUID = imesStatemachine.PoseUID;
				SlicerVisualIF.TCPForce = imesStatemachine.TCPForce;
				SlicerVisualIF.SendTCPForce = true;

				if(imesStatemachine.currentVisualIFDatatype==1){
					SlicerVisualIF.datatype = LWRVisualizationInterface.VisualIFDatatypes.IMAGESPACE;
					SlicerVisualIF.cartPose_StateM = imesStatemachine.curPose;
					if(imesStatemachine.TransformRecieved){
						SlicerVisualIF.T_IMGBASE_StateM= imesStatemachine.TransformRobotImage;
					}
				}else if(imesStatemachine.currentVisualIFDatatype==2){
					SlicerVisualIF.datatype = LWRVisualizationInterface.VisualIFDatatypes.ROBOTBASE;
					SlicerVisualIF.cartPose_StateM = imesStatemachine.curPose;
				}else if(imesStatemachine.currentVisualIFDatatype==3){
					SlicerVisualIF.datatype = LWRVisualizationInterface.VisualIFDatatypes.JOINTSPACE;
					SlicerVisualIF.jntPose_StateM = imesStatemachine.curJntPose;
					SlicerVisualIF.cartPose_StateM = imesStatemachine.curPose;
				}
				SlicerVisualIF.VisualSemaphore.release();
			}catch(InterruptedException e){
				ErrorMessage = "Error: Couldn't acquire VisualIF Semaphore!!";
			}
	    }
		
		//If flags are set and VisualIF is not running yet start the Visual Thread
		if (imesStatemachine.StartVisual && VisualON ==false){	
			//Initialize the necessary member variables first
			SlicerVisualIF.jntPose_StateM = initialPosition;
			SlicerVisualIF.cartPose_StateM = imesStatemachine.curPose;
			if(DebugOut){
				System.out.println("Setting datatype");	
			}
			if(imesStatemachine.currentVisualIFDatatype==1){
				SlicerVisualIF.datatype = LWRVisualizationInterface.VisualIFDatatypes.IMAGESPACE;
			}else if(imesStatemachine.currentVisualIFDatatype==2){
				SlicerVisualIF.datatype = LWRVisualizationInterface.VisualIFDatatypes.ROBOTBASE;
			}else if(imesStatemachine.currentVisualIFDatatype==3){
				SlicerVisualIF.datatype = LWRVisualizationInterface.VisualIFDatatypes.JOINTSPACE;
				
			}
			if(imesStatemachine.TransformRecieved){
				SlicerVisualIF.T_IMGBASE_StateM= imesStatemachine.TransformRobotImage;
			}
			
			SlicerVisualIF.VisualActive = true;
			//Start the Visualization thread
			SlicerVisualIF.start();
			VisualON = true;
		}else if (imesStatemachine.StartVisual && VisualON  && !SlicerVisualIF.VisualActive){//if Viuslaization interface is started, not active but is set active
			//Change VisualActive to true. Thereby, the pose is send to the visualization
			SlicerVisualIF.VisualActive = true;
			
		}else if(!imesStatemachine.StartVisual && /*SlicerVisualIF.isAlive() &&*/ SlicerVisualIF.VisualActive){//if the visualization  is running and the the Start Visual flag is false and the interface is still active 
			//Set the VisualACtive flag to false - thereby, no more data is send to the visualization
			SlicerVisualIF.VisualActive = false;

		}
		
		//If SlicerControl Interface Thread is running...
		if( SlicerControlIf.ControlRun){
			
			i =0;
			//Try to read new command String from SlicerControl (Alive) Thread
			try{
				SlicerControlIf.ControlSemaphore.tryAcquire(1, TimeUnit.MILLISECONDS);
				imesStatemachine.CmdIGTmessage= SlicerControlIf.CMD_StateM;
				imesStatemachine.IGTLdatatype = SlicerControlIf.IGTLdatatype;
				imesStatemachine.UID = SlicerControlIf.UID;
				if(SlicerControlIf.TransformRecieved && !imesStatemachine.TransformRecieved){
					imesStatemachine.TransformRobotImage = SlicerControlIf.TransformImageRobot;
					imesStatemachine.TransformRecieved = true;
				}
				 SlicerControlIf.ControlSemaphore.release();
				 
			}catch(InterruptedException e){
				ErrorMessage = "Couldn't acquire Semaphore!!";			
			}
		}else{ //if it is not to Error handling
			imesStatemachine.ErrorCode = 2;
			ErrorMessage = "Slicer Control Interface not Alive...";
			i++;
		}
		//Check if there is a Transition Request and in that case Change the state and interpret the command parameters by calling the function InterpretCommandString of the Current State
		imesStatemachine.CheckTransitionRequest();
		
	
		//If the State has changed print the new State
		if(imesStatemachine.InitFlag ){
			System.out.println("Robot State has Changed to:" + imesStatemachine.RobotState.name() );

		}
		
		//Check on Communication Quality
		if(SlicerControlIf.ErrorCode == 18){ //Bad Communication Quality!!
			imesStatemachine.ErrorCode = 18;
			imesStatemachine.ErrorMessage = "ERROR: State Control Interface Bad Communication quality setting robot State to IDLE";
			imesStatemachine.ErrorFlag = true;
			SlicerControlIf.ErrorCode = 0;
			
		}
		
		//Print Error messages if there where any Errors
		imesStatemachine.ErrorHandler(DebugOut);
		
		//Calculating the new control Param and Change the parameters
		imesStatemachine.CalcControlParam();
		
		//Change the control mode settings of the robot and send a new Destination pose
		try{
			SmartServoRuntime
				.changeControlModeSettings(imesStatemachine.controlMode);
			SmartServoRuntime
			.setDestination(imesStatemachine.cmdPose);
		}	catch (Exception e){
		    ErrorMessage = "Error: Failed to change Realtime Settings!!";
		}

		
		//Defining the Acknowledgement String for Control Interface
		imesStatemachine.SetACKPacket();
		

		if(SlicerControlIf.ControlRun){
			try{
				SlicerControlIf.ControlSemaphore.tryAcquire(1, TimeUnit.MILLISECONDS);
				//try to update the ACK String for the ControlIF Thread
				SlicerControlIf.ACK_StateM = imesStatemachine.AckIGTmessage;
				SlicerControlIf.ControlSemaphore.release();
			}catch(InterruptedException e){
				ErrorMessage = "Error: Couldn't Acquire ControlIF Semaphore!!";
			}
		}
		
		
		
		if (!ErrorMessage.equals(LastPrintedError)){
		if(DebugOut){
			//System.out.println(ErrorMessage);
		}
			LastPrintedError = ErrorMessage;
		}
		
		//Set the Module in Sleep mode for stability enhancement
		curTime = (long) ((System.nanoTime() - startTimeStamp));
		curTime_millis = (long) curTime/1000000;
		curTime_nanos = (int) (curTime%1000000);
		if(curTime_millis<millisectoSleep-2){
			Thread.sleep(millisectoSleep-2-curTime_millis, 999999-curTime_nanos);
			
		}
		
		// Overall timing end
		aStep.end();
		if (imesStatemachine.End){
			StatemachineRun =false;
			
		}
		
	    } // end while
	    
		SlicerControlIf.ControlRun = false;
		SlicerVisualIF.VisualRun = false;
		
	  //Print the timing statistics
		System.out.println("Statistic Timing of Statemachine interface thread " + SlicerControlIf.SMtiming);
		System.out.println("UID miss: " + SlicerControlIf.UIDmiss + " UIDrepeats: " + SlicerControlIf.UIDrepeatNum + "(max: " + SlicerControlIf.UIDrepeat_max + ")");
		System.out.println("Statistic Timing of Visualisation interface thread " + SlicerVisualIF.Visualtiming);
		System.out.println("PoseUID miss: " + SlicerVisualIF.PoseUIDOldCount);
		System.out.println("Statistic Timing of Statemachine Mean:" + timing);
		ThreadUtil.milliSleep((long) (1000));

		// /////////////////////////////////////////////////
		// Do or die: print statistics and parameters of the motion
		System.out.println("Displaying final states after loop "
			+ controlMode.getClass().getName());
		SmartServoRuntime.setDetailedOutput(1);
		// Stop the motion
		SmartServoRuntime.stopMotion();
		

		
		if (timing.getMeanTimeMillis() > millisectoSleep + 5)
		{
		    System.out
			    .println("Statistic Timing is unexpected slow, you should try to optimize TCP/IP Transfer");
		    System.out
			    .println("Under Windows, you should play with the registry, see the e.g. the RealtimePTP Class javaDoc for details");
		}
	}
	

	 // Stopping the Control Interface thread and the visualization thread
	
	catch (Exception e)
	{
	    System.out.println(e);
	    e.printStackTrace();
		SlicerControlIf.ControlRun = false;
		SlicerVisualIF.VisualRun = false;
	}
	

	
	ThreadUtil.milliSleep((long) (1000));
	
    }


    protected CartesianImpedanceControlMode createCartImp()
    {
	CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
	cartImp.parametrize(CartDOF.TRANSL).setStiffness(5000.);
	cartImp.parametrize(CartDOF.ROT).setStiffness(300.);
	cartImp.setNullSpaceStiffness(5000.);
	// For your own safety, shrink the motion abilities to useful limits
	cartImp.setMaxPathDeviation(500, 500, 500, 10, 10, 10);

	return cartImp;
    }


	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		SimpleStateExample app = new SimpleStateExample();
		app.runApplication();
	}

	@Override
	public void run() {
		// TODO Automatisch generierter Methodenstub
		//Initiliaze "instanz" of the RealtimePTP
		// ///////////////////////////////////////////////////////////////////////////////////////////////
		// Cartesian impedance sample
		// ///////////////////////////////////////////////////////////////////////////////////////////////
		moveToInitialPosition();

		// Initialize Cartesian impedance mode
		CartesianImpedanceControlMode cartImp = createCartImp();
		// run the realtime motion, as before done in the SimpleJointMotion
		// sample.
		// the only difference - pass on the InteractionControlStrategy...
		runRealtimeMotion(cartImp);
	}
}
