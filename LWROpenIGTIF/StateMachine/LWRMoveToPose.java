/*=========================================================================

  Program:   LWRMoveToPose
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
import com.kuka.roboticsAPI.geometricModel.math.Rotation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
/**
 * In This State the LWR is moving to a specified position in Cartesian space. Therefore, the stiffness is set to the maximum value.
 * 
 * @author Sebastian Tauscher
 * @version 0.1
 */
public class LWRMoveToPose implements LWRState {
	private boolean EndofPath=false;
	public boolean ImageSpace=false;
	public Vector TargetPosition=null;
	public MatrixTransformation TargetOrientation=null;
	
	private double lambda_end=0.0;
	double lambda_null=0.0;
	private Vector u = null;
	private Vector ap=null;
	private Vector ap_null=null;
	
	
	
	@Override
	/**
	 * In this Function control Mode Parameters are set and the commanded pose are calculated due the current LWR State.
	 * During the MoveToPose State the Cartesian Stiffness is set to the maximum value of 5000 and the Pose is set to the closest point at the path plus an offset in the desired direction.
	 * When the End point of the path is reached the robot holds his position and the boolean EndPath is set true.
	 * @param lwrStatemachine The operated state machine
	 * @see LWRState
	 */
	public void CalcControlParam(LWRStatemachine lwrStatemachine) {
		// TODO Automatisch generierter Methodenstub
		
		
		Vector curPosition = null;
		Vector aim=null;
		double d = 0.0;
		double lambda = 0.0;
		
		CartesianImpedanceControlMode cartImp = (CartesianImpedanceControlMode) lwrStatemachine.controlMode;
		
		
		if(lwrStatemachine.InitFlag){
			cartImp.parametrize(CartDOF.TRANSL).setStiffness(5000);
			cartImp.parametrize(CartDOF.ROT).setStiffness(300);
			cartImp.setNullSpaceStiffness(0);
			int[] NewStiffness = {5000, 5000, 5000, 300, 300, 300 };
			lwrStatemachine.curCartStiffness = NewStiffness;
			ap=Vector.of(lwrStatemachine.curPose.getTranslation().getX(),lwrStatemachine.curPose.getTranslation().getY(),lwrStatemachine.curPose.getTranslation().getZ()) ;
			ap_null = ap;
			u=TargetPosition.subtract(ap).normalize();
			lambda_end =TargetPosition.subtract(ap).length();
			lwrStatemachine.InitFlag=false;
		}
		curPosition =lwrStatemachine.curPose.getTranslation();
		if(!EndofPath){
			if(curPosition.subtract(TargetPosition).length()<10){
				lwrStatemachine.cmdPose = MatrixTransformation.of(TargetPosition,TargetOrientation.getRotationMatrix());
				EndofPath = true;
			}else{
				
				d = curPosition.dotProduct(u);
				lambda = d -u.dotProduct(ap);
				lambda_null = d -u.dotProduct(ap_null);
				aim = u.multiply(1);
				if (lambda_null>= 0 && lambda_null<=lambda_end){
					ap = ap.add(u.multiply(lambda));
					
				}
				aim = u.multiply(10);

				
	
				lwrStatemachine.cmdPose = MatrixTransformation.of(ap.add(aim), TargetOrientation.getRotationMatrix());
			}
		}else{
			lwrStatemachine.cmdPose = MatrixTransformation.of(TargetPosition,TargetOrientation.getRotationMatrix());
		}
			
			
			// Send the new Stiffness settings down to the
			// controller
			lwrStatemachine.controlMode = cartImp;

	}

	/**
	 * In this Function the Acknowledge String which is send to the State Control is defined due the current LWR State.
	 * In the MovetoPose State the String is Set to "MovetoPose;true;" or "MoveToPose;false;" according to teh value of the boolean ReachedPose.
	 * 
	 * @param lwrStatemachine The operated Statemachine
	 */
	@Override
	public void SetACKPacket(LWRStatemachine lwrStatemachine) {
		// TODO Automatisch generierter Methodenstub
		String ACK;
		if(EndofPath){
			ACK="MoveToPose;true;";
		}else{
			ACK="MoveToPose;false;";
		}
		lwrStatemachine.AckIGTmessage = ACK;// TODO Automatisch generierter Methodenstub
	}
	/**
	 * In this Function the Command String which is received from the State Control is read and interpreted due to the Current State and if requested and allowed the State is Changed. 
	 * In The MoveToPose State the allowed state transitions are:<br>
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
		if(lwrStatemachine.IGTLdatatype.equals("STRING")){
			String CMD_String;
					CMD_String = lwrStatemachine.CmdIGTmessage;
					lwrStatemachine.ParameterString = CMD_String.substring(CMD_String.indexOf(";"));
					String[] CMD_Array = CMD_String.split(";");
					if(CMD_Array[1].contentEquals("img")){
						this.ImageSpace = true;
					}else if(CMD_Array[1].contentEquals("rob")){
						this.ImageSpace = false;
					}else{
						lwrStatemachine.ErrorMessage = ("Unexpected coordinate system (supported are img or plane)");
						lwrStatemachine.ErrorCode = 10;
						lwrStatemachine.ErrorFlag = true;
					}
					this.TargetPosition = Vector.of(Double.parseDouble(CMD_Array[2]), Double.parseDouble(CMD_Array[3]),Double.parseDouble(CMD_Array[4]));
					this.TargetOrientation = MatrixTransformation.of(Vector.of(0, 0, 0), Rotation.ofRad(Double.parseDouble(CMD_Array[5])*Math.PI/180, Double.parseDouble(CMD_Array[6])*Math.PI/180, Double.parseDouble(CMD_Array[7])*Math.PI/180));
					if(this.ImageSpace && lwrStatemachine.TransformRecieved){
						MatrixTransformation Tmp =MatrixTransformation.of(TargetPosition, TargetOrientation.getRotationMatrix());
						Tmp = lwrStatemachine.TransformRobotImage.invert().compose(Tmp);
						this.TargetPosition = Tmp.getTranslation();
						this.TargetOrientation = Tmp.withTranslation(Vector.of(0, 0, 0));
						
					}
					
		}else{
			lwrStatemachine.ErrorCode = 12; //Illegal/unknown instruction
			lwrStatemachine.ErrorMessage = "Unexpected Messagetype recieved! Expected STRING"; 
		}
		
	}

}
