/*=========================================================================

  Program:   LWRGravComp
  Language:  java
  Web page: http://www.slicer.org/slicerWiki/index.php/Documentation/Nightly/Extensions/LightWeightRobotIGT

  Copyright (c) Sebastian Tauscher, Institute of Mechatronics Systems, Leibniz Universität Hannover. All rights reserved.

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
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

/**
 * In this state the LWR is set to gravitation Compensation mode so that robot can be moved manually and freely without constraints. The GravComp/Free mode can be used for e.g., for registration purposes
 * 
 * @author Sebastian Tauscher
 * @version 0.1
 */
public class LWRGravComp implements LWRState {

/**
 * In this Function control mode parameters are set and the command pose is calculated due to the current LWR State.
 * In the GravComp State the translational and rotational Stiffness is set to zero and the command pose to the current measured Pose.
 * Because the values are not changing during this State the values are just set when the InitFlag of the state machine is true.
 * @param lwrStatemachine - The operated state machine
 * @see LWRState
 */
	@Override
	public void CalcControlParam(LWRStatemachine lwrStatemachine) {
			
		
		if (lwrStatemachine.InitFlag ==true){
			// We are in CartImp Mode,
			// Modify the settings:
			// NOTE: YOU HAVE TO REMAIN POSITIVE SEMI-DEFINITE !!
			// NOTE: DONT CHANGE TOO FAST THE SETTINGS, ELSE YOU
			// WILL DESTABILIZE THE CONTROLLER
			int[] NewStiffness = {0, 0, 0,  0, 0, 0 };
			CartesianImpedanceControlMode cartImp = (CartesianImpedanceControlMode) lwrStatemachine.controlMode;
			cartImp.parametrize(CartDOF.ALL).setStiffness(0.0);
			cartImp.setNullSpaceStiffness(0.);
			lwrStatemachine.curCartStiffness = NewStiffness;
			lwrStatemachine.controlMode = cartImp;
			lwrStatemachine.InitFlag = false;
		}
		if(lwrStatemachine.cmdPose.getTranslation().subtract(lwrStatemachine.curPose.getTranslation()).length()>100){
			
			System.out.println("Difference to big!!!!!!!!!!!!");
		}
		
		lwrStatemachine.cmdPose = lwrStatemachine.curPose;

	}
	
	
	/**
	 * In this function the acknowledge string, which is send to the state control is defined due the current LWR State.
	 * In the GravComp State the String is Set to "GravComp;".
	 * 
	 * @param lwrStatemachine The operated state machine
	 */
	@Override
	public void SetACKPacket(LWRStatemachine lwrStatemachine) {
		
		String ACK;
		ACK = "GravComp;";
		lwrStatemachine.AckIGTmessage = ACK;// TODO Automatisch generierter Methodenstub
		
	}

	/**
	 * In this function the command string which is received from the state control is interpreted and the parameters are set. It is only called after a State transition.
	 * For the LWRState LWRGravComp because no Parameters are send from the state control.
	 * @param lwrStatemachine - The operated state machine
	 * @see LWRState
	 */
	
	@Override
	public void InterpretCMDPacket(LWRStatemachine lwrStatemachine) {
		if(lwrStatemachine.IGTLdatatype.equals("STRING")){
			String CMD_String;
			CMD_String = lwrStatemachine.CmdIGTmessage;
			lwrStatemachine.ParameterString = CMD_String.substring(CMD_String.indexOf(";"));
		}
	}
}
