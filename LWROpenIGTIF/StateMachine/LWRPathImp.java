/*=========================================================================

  Program:   LWRPathImp
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
package LWROpenIGTIF.StateMachine;

import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
/**
 * In This State the LWR can be moved along a linear Path in one direction to a specified position in Cartesian space. Therefore, the stiffness is set according to the distance towards the defined path.
 * Therefor, the current pose of the robot at the beginning of this state (statemachine InitFlag ==true) is used as the start pose. 
 * @author Sebastian Tauscher
 * @version 0.1
 */
public class LWRPathImp implements LWRState {
	
	/**
	 * Flag indicates if the Position is send in Image or robot base coordinate system
	 */
	public boolean ImageSpace=false;
	
	/**
	 * TargetPosition in Image or robot base coordinate system due to ImageSpace flagS
	 */
	public Vector TargetPosition=null;
	
	/**
	 * Target Orientation as rotation matrix in Image or robot base coordinate system due to ImageSpace flag
	 * HINT: Not used YET!!!
	 */
	public MatrixTransformation TargetOrientation=null;
	
	/**
	 * Flag indicates if the target pose is reached
	 */
	private boolean EndofPath = false;
	
	/**
	 * vector to define the line path from start to End point.
	 * g: x= ap + lambda*u
	 */
	private Vector ap= null;
	
	/**
	 * directional vector of the line 
	 * g: x= ap + lambda*u
	 */
	private Vector u=  null;
	
	/**
	 * directional vector of the line 
	 * g: x= ap + lambda*u
	 */
	private double lambda_end = 0;
	
	/**
	 * Allowed euclidean distance from current robot position to the path from start to end pose
	 */
	private double tolerance=10.0;
	private double distance = 0.0;
	/**
	 * In this Function the Stiffness value is calculated
	 * @param	dist the minimum distance to path
	 * @param	tolerance the maximum allowed distance from the path
	 */
	
	public double get_stiffness_value( double dist, double tolerance)	//Berechnung der Dämpfung bei Annäherung an die Grenze
	 {
		double max_stiff=5000;
		if (dist>=tolerance) return max_stiff;
		else if (dist<tolerance && dist>0)
		{
			double y;
			double m= max_stiff/Math.pow(tolerance,2);
			y =m*Math.pow(dist,2); 
			return y;
		}
		else return 0;
	 }
	@Override
	/**
	 * In this Function control Mode Parameters are set and the commanded pose are calculated due the current LWR State.
	 * During the PathImp State the Cartesian Stiffness is set according to the distance to defined Path, the NullSpaceStiffness is set to zero and the Pose is set to the closest point at the path plus an offset in the desired direction.
	 * When the End point of the path is reached the robot holds his position and the boolean EndPath is set true.
	 * @param lwrStatemachine The operated state machine
	 * @see LWRState
	 */
	public void CalcControlParam(LWRStatemachine lwrStatemachine) {
		
		double d = 0.0;
		distance = 0.0;
		Vector curPosition = null;
		double lambda = 0.0;
		Vector aim = null;
		double aRotStiffVal =300;
		double StiffVal=3000;
		Vector aTransStiffVal=null;
		int[] NewStiffness = {0, 0, 0,  0, 0, 0 };
		
		// TODO Automatisch generierter Methodenstub
		if(lwrStatemachine.InitFlag){
			aim = Vector.of(0, 0, 0);
			TargetOrientation = MatrixTransformation.of(Vector.of(0, 0, 0), lwrStatemachine.curPose.getRotation());
			ap=Vector.of(lwrStatemachine.curPose.getTranslation().getX(),lwrStatemachine.curPose.getTranslation().getY(),lwrStatemachine.curPose.getTranslation().getZ()) ;
			lambda_end = TargetPosition.subtract(ap).length();
			u=TargetPosition.subtract(ap).normalize();
			lwrStatemachine.InitFlag=false;
		}
		curPosition =lwrStatemachine.curPose.getTranslation();
		
		
		if(curPosition.subtract(TargetPosition).length()<tolerance){
			StiffVal = 3000 -1000*curPosition.subtract(TargetPosition).length()/tolerance ;
			aTransStiffVal = Vector.of(StiffVal,StiffVal,StiffVal);
			lwrStatemachine.cmdPose = MatrixTransformation.of(TargetOrientation.withTranslation(TargetPosition));
			if(curPosition.subtract(TargetPosition).length()<10){
				StiffVal = 5000;
				aTransStiffVal = Vector.of(StiffVal,StiffVal,StiffVal);
				lwrStatemachine.cmdPose = MatrixTransformation.of(TargetOrientation.withTranslation(TargetPosition));
				if(!EndofPath){
					System.out.println("Path Impedance: Reached End of path (" + TargetPosition +")");
				}
				EndofPath = true;
			}
		}else{
			
		
			d = curPosition.dotProduct(u);
			lambda = d -u.dotProduct(ap);



			if (lambda>= 0 && lambda<=lambda_end){
				ap = ap.add(u.multiply(lambda));
			}
			distance = ap.subtract(curPosition).length();
			
			// Beginn der Impedanzregelung für Pfad
			
			 
			 if(distance>tolerance) //Bahngrenze übertreten
			 {
				 aTransStiffVal = Vector.of(5000,5000,5000);
				 aim = ap.subtract(curPosition).normalize().multiply(distance-tolerance);
				 //System.out.println("Out of bounds!!!!!!! ");
			 }else{
				 StiffVal = get_stiffness_value( distance, tolerance);
				 aTransStiffVal = Vector.of(StiffVal,StiffVal,StiffVal);
				 aim =Vector.of(0,0,0);
			 }
			 
			 lwrStatemachine.cmdPose = MatrixTransformation.of(ap.add(aim), TargetOrientation.getRotation());
		}
		CartesianImpedanceControlMode cartImp = (CartesianImpedanceControlMode) lwrStatemachine.controlMode;
		
		if(EndofPath){
			 lwrStatemachine.cmdPose = MatrixTransformation.of(TargetPosition, TargetOrientation.getRotation());
			cartImp.parametrize(CartDOF.TRANSL).setStiffness(5000);
			cartImp.parametrize(CartDOF.ROT).setStiffness(300);
			NewStiffness[0] = 5000;
			NewStiffness[1] = 5000;
			NewStiffness[2] = 5000;
			NewStiffness[3] =  300;
			NewStiffness[3] =  300;
			NewStiffness[3] =  300;
		}else{
			cartImp.parametrize(CartDOF.X).setStiffness(aTransStiffVal.getX());
			cartImp.parametrize(CartDOF.Y).setStiffness(aTransStiffVal.getY());
			cartImp.parametrize(CartDOF.Z).setStiffness(aTransStiffVal.getZ());
			cartImp.parametrize(CartDOF.ROT).setStiffness(aRotStiffVal);
			cartImp.setNullSpaceStiffness(0.0);
			NewStiffness[0] = (int) aTransStiffVal.getX();
			NewStiffness[1] = (int) aTransStiffVal.getY();
			NewStiffness[2] = (int) aTransStiffVal.getZ();
			NewStiffness[3] =  (int) aRotStiffVal;
			NewStiffness[3] =  (int) aRotStiffVal;
			NewStiffness[3] =  (int) aRotStiffVal;
	

		}
		// Send the new Stiffness settings down to the
		// controller
		lwrStatemachine.curCartStiffness = NewStiffness;
		lwrStatemachine.controlMode = cartImp;
	}
	/**
	 * In this Function the Acknowledge IGTMessage which is send to the State Control is defined due the current LWR State.
	 * In the PathImp State the IGTMessageString is Set to "PathImp;true;" or "PathImp;false,"according to the value of the boolean PathImp.
	 * 
	 * @param lwrStatemachine The operated Statemachine
	 */
	@Override
	public void SetACKPacket(LWRStatemachine lwrStatemachine) {
		// TODO Automatisch generierter Methodenstub
		String ACK;
		if(EndofPath){
			ACK="PathImp;0;true;";
		}else{
			if(distance> tolerance){
				ACK="PathImp;2;false;";
			}else if(distance == 0){
				ACK="PathImp;0;false;";
			}else{
				ACK="PathImp;1;false;";
			}
		}

		lwrStatemachine.AckIGTmessage = ACK;// TODO Automatisch generierter Methodenstub
	}
	/**
	 * In this Function the CommandIGTMessage which is received from the State Control is read and interpreted due to the Current State and if requested and allowed the State is Changed. 
	 * In The VirtualFixtures State the allowed state transitions are:<br>
	 * 	- IDLE (transition condition: none)<br>
	 * 	- GravComp (transition condition: none)<br>
	 *  - VirtualFixtures (transition condition: RegistrationFinished AND DataSend AND TransformRecieved)<br>
	 *  - MoveToPose (transition condition: RegistrationFinished AND DataSend AND TransformRecieved)<br>
	 * If a transition request to another State is requested or some the number of Parameters is incorrect the state is set to Error and the Error code is set to 1 = Transition not allowed. 
	 * @param lwrStatemachine - The operated state machine
	 * @see LWRState
	 */
	@Override
	public void InterpretCMDPacket(LWRStatemachine lwrStatemachine) {
		// TODO Automatisch generierter Methodenstub
		boolean Error=false;
		if(lwrStatemachine.IGTLdatatype.equals("STRING")){
			String CMD_String;
			CMD_String = lwrStatemachine.CmdIGTmessage;
			lwrStatemachine.ParameterString = CMD_String.substring(CMD_String.indexOf(";"));
			String[] CMD_Array =CMD_String.split(";");
				if(CMD_Array[1].contentEquals("img")){
					this.ImageSpace = true;
					System.out.println("Image space is set to true...");
				}else if(CMD_Array[1].contentEquals("rob")){
					this.ImageSpace = false;
				}else{
					Error = true;
				}
				
				this.TargetPosition = Vector.of(Double.parseDouble(CMD_Array[2]), Double.parseDouble(CMD_Array[3]), Double.parseDouble(CMD_Array[4]));
				System.out.println("Targetposition...: " + this.TargetPosition);
				//newState.TargetOrientation = MatrixTransformation.of(Vector.of(0, 0, 0), Rotation.ofRad(Double.parseDouble(CMD_Array[5])*Math.PI/180, Double.parseDouble(CMD_Array[6])*Math.PI/180, Double.parseDouble(CMD_Array[7])*Math.PI/180));
				if(this.ImageSpace && lwrStatemachine.TransformRecieved){
					Vector Tmp ;
					Tmp = lwrStatemachine.TransformRobotImage.invert().applyTo(TargetPosition).add(lwrStatemachine.TransformRobotImage.invert().getTranslation());
					this.TargetPosition = Tmp;
					System.out.println("After Transformation...: " + this.TargetPosition);
					
				}
				if(Error){
					lwrStatemachine.ErrorCode = 10;//Configuration error
					lwrStatemachine.ChangeLWRState(new LWRIdle());
				}
	
		}else{
			lwrStatemachine.ErrorCode = 12; //Illegal/unknown instruction
			lwrStatemachine.ErrorMessage = "Unexpected Messagetype recieved! Expected STRING"; 
		}
	}

}
