/*=========================================================================

  Program:   LWRIdle
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
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
/**
 * In This State the LWR is holding its position with a maximum stiffness. This an save mode the robot could always change too.
 * 
 * @author Sebastian Tauscher
 * @version 0.1
 */
class LWRIdle implements LWRState{
	boolean IncreaseStiffness = false;
	/**
	 * In this Function control Mode Parameters are set and the commanded pose are calculated due the current LWR State.
	 * During the Idle State the Cartesian Stiffness is set to the maximum value of 5000, the NullSpaceStiffness is set to zero and the Pose is set to the measured pose.
	 * Because the Values are static whiles this State is active the Values are just set when the InitFlag of the Statemachine is true.
	 * 
	 * @param lwrStatemachine The operated state machine
	 * @see LWRState
	 */
	@Override
	public void CalcControlParam(LWRStatemachine lwrStatemachine) {
		double aTransStiffVal =5000;
		double aRotStiffVal = 300;
		CartesianImpedanceControlMode cartImp = (CartesianImpedanceControlMode) lwrStatemachine.controlMode;
		int [] DeltaStiffness = new int[6];
		int[] NewStiffness = {(int) aTransStiffVal, (int) aTransStiffVal, (int) aTransStiffVal,  (int) aRotStiffVal, (int) aRotStiffVal, (int) aRotStiffVal };
		for(int i = 0; i<6; i++){
			DeltaStiffness[i] = NewStiffness[i] - lwrStatemachine.curCartStiffness[i];
		}
		Vector DeltaStiffnessTrans = Vector.of(DeltaStiffness[0], DeltaStiffness[1], DeltaStiffness[2]);
		Vector DeltaStiffnessRot = Vector.of(DeltaStiffness[3], DeltaStiffness[4], DeltaStiffness[5]);
		
		if (lwrStatemachine.InitFlag ==true){
					
			if(DeltaStiffnessTrans.length()<=100 && DeltaStiffnessRot.length()<=30){
				cartImp.parametrize(CartDOF.TRANSL).setStiffness(aTransStiffVal);
				cartImp.parametrize(CartDOF.ROT).setStiffness(aRotStiffVal);
				cartImp.setNullSpaceStiffness(0.);
				lwrStatemachine.controlMode = cartImp;
				lwrStatemachine.cmdPose = lwrStatemachine.curPose;
				IncreaseStiffness=false;
				lwrStatemachine.curCartStiffness = NewStiffness;
			}else{
				IncreaseStiffness = true;
				
			}
			lwrStatemachine.InitFlag = false;
		}
		if(IncreaseStiffness){
			boolean IncreaseTrans = true;
			boolean IncreaseRot = true;
			if(DeltaStiffnessTrans.length()>100){
				for(int k = 0; k<3; k++){
					if(lwrStatemachine.curCartStiffness[k]<=4950){
						NewStiffness[k] = lwrStatemachine.curCartStiffness[k] + 50;
					}else{
						NewStiffness[k] = 5000;
					}
					lwrStatemachine.curCartStiffness[k] = NewStiffness[k];						
				}
				cartImp.parametrize(CartDOF.X).setStiffness(NewStiffness[0]);
				cartImp.parametrize(CartDOF.Y).setStiffness(NewStiffness[1]);
				cartImp.parametrize(CartDOF.Z).setStiffness(NewStiffness[2]);
				IncreaseTrans = true;
			}else{
				cartImp.parametrize(CartDOF.TRANSL).setStiffness(aTransStiffVal);
				IncreaseTrans = false;
			}
			if(DeltaStiffnessRot.length()>30){
				for(int k = 3; k<6; k++){
					if(lwrStatemachine.curCartStiffness[k]<=300-15){
						NewStiffness[k] = lwrStatemachine.curCartStiffness[k] + 15;
						
					}else{
						NewStiffness[k] = 300;
					}
					lwrStatemachine.curCartStiffness[k] = NewStiffness[k];
				}
				cartImp.parametrize(CartDOF.A).setStiffness(NewStiffness[3]);
				cartImp.parametrize(CartDOF.B).setStiffness(NewStiffness[4]);
				cartImp.parametrize(CartDOF.C).setStiffness(NewStiffness[5]);
				IncreaseRot = true;
			}else{
				cartImp.parametrize(CartDOF.ROT).setStiffness(aRotStiffVal);
				IncreaseRot = false;
			}
			lwrStatemachine.curCartStiffness = NewStiffness;
			cartImp.setNullSpaceStiffness(0.);
			lwrStatemachine.controlMode = cartImp;
			if(!IncreaseRot && !IncreaseTrans){
				IncreaseStiffness = false;
			}
		}
	}

	/**
	 * In this Function the Acknowledge String which is send to the State Control is defined due the current LWR State.
	 * In the Idle State the String is Set to "IDLE;" or "SendData;".
	 * 
	 * @param lwrStatemachine The operated Statemachine
	 */
	@Override
	public void SetACKPacket(LWRStatemachine lwrStatemachine) {
		// TODO Automatisch generierter Methodenstub
		String ACK;
		ACK = "IDLE;";
		//Send the string to StateControl
		if(lwrStatemachine.End){
			lwrStatemachine.AckIGTmessage = "SHUTDOWN;";
		}else{
			lwrStatemachine.AckIGTmessage = ACK;// TODO Automatisch generierter Methodenstub
		}
	}
	/**
	 * In this Function the Command String which is received from the State Control is interpreted and the parameters are set. It is just called after a State transition 
	 * For the LWRState LWRIdle this is empty because no Parameters are send from the state control.
	 * @param lwrStatemachine - The operated state machine
	 * @see LWRState
	 */
	
	@Override
	public void InterpretCMDPacket(LWRStatemachine lwrStatemachine){
		// TODO Automatisch generierter Methodenstub
		if(lwrStatemachine.IGTLdatatype.equals("STRING")){
			String CMD_String;

			CMD_String = lwrStatemachine.CmdIGTmessage;
			lwrStatemachine.ParameterString = CMD_String.substring(CMD_String.indexOf(";"));
			
		}
	}

}
