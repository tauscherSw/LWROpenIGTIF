/*=========================================================================

  Program:   LWRVirtualFixtures
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
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

/**
 * In this State two different kinds of Virtuall Fixtures, a plane and a cone are definied and the Stiffness and
 * Damping values are calculated due to there distance to the defined Fixtures. This is one of the States defined for the LWRStatemachine see LWRState. There are three different areas defined: the Awareness area, the free area and the forbidden area.
 * In the first area the stiffness values are increasing when the robot is getting closer to the Fixtures and decreasing if the distance increases. In the second can move freely 
 * and in the third area the stiffness values are set to  a maximum value and the commanded Position is the closest point on the Fixture.
 * 
 * @author Sebastian Tauscher
 * @version 0.1
 */

public class LWRVirtualFixtures implements LWRState {

	public Vector VirtualFixture_ap;
	public Vector VirtualFixture_n;
    public double  VirtualFixture_phi;
	public int VFtype=0;
    public boolean ImageSpace=false;
	//Some Variables for Stiffness Control
	private double distance;
	private double awaredist = 20;
	private double last_distance=0.0;
	private double max_stiff=5000;
	private MatrixTransformation T_Base_cone;
	private Vector normvec = Vector.of(0, 0, 1);
	private Vector normvec_old = Vector.of(0, 0, 1);
	private Vector normvec_old2 = Vector.of(0, 0, 1);
	private Vector normvec_old3 = Vector.of(0, 0, 1);
	private Vector normvec_old4 = Vector.of(0, 0, 1);
	private Vector normvec_old5 = Vector.of(0, 0, 1);
	private Vector normvec_new = Vector.of(0, 0, 1);;
	int i=0;
	
	boolean ConeTip = false;
	boolean EndPoint = false;
	/**
	 * In this Function the Stiffness value for the case that the robot is getting closer to a virtual fixture is calculated
	 * @param	dist the minimum distance to the Virtual Fixture
	 * @param	awaredist the minimum distance from where on the stiffness is changed
	 */
	

	public double get_stiffness_value_approach( double dist, double awaredist)	//Berechnung der Dämpfung bei Annäherung an die Grenze
	 {
		if (dist>=awaredist) return 0.01;
		else if (dist<awaredist && dist>0)
		{
			double y;
			double m= max_stiff/Math.pow(awaredist,2);
			y =m*Math.pow(awaredist-dist,2); 
			return y;
		}
		else return max_stiff;
	 }
	
	

	/**
	 * In this Function the Stiffness value for the case that the robot is getting further away from a virtual fixture is calculated
	 * @param dist the minimum distance to the Virtual Fixture
	 * @param awaredist the minimum distance from where on the stiffness is changed

	 */
	 public double get_stiffness_value_remove( double dist, double awaredist)	
	 { 	

		
		double zero_point=awaredist/2;
		if (dist>=zero_point) return 0.01;
		else if (dist<zero_point && dist>0)
		{
			double y;
			double m= max_stiff/Math.pow(zero_point,3);
			y =m*Math.pow(zero_point-dist,3); 
			return y;
		}
		else return max_stiff;
	 }
	 /**
		 * In this Function the Stiffness value for the case that the robot is getting further away from a virtual fixture is calculated
		 * @param dist the minimum distance to the Virtual Fixture
		 * @param awaredist the minimum distance from where on the stiffness is changed

		 */
		 public double get_stiffness_value_ConeTip( double dist, double awaredist)	
		 { 	

			double max =max_stiff/2;
			if (dist< awaredist && dist>0)
			{
				double y;
				double m= max/Math.pow( awaredist,3);
				y =max_stiff - m*Math.pow( awaredist-dist,3); 
				return y;
			}
			else return max;
		 }

	/**
	 * In this Function control Mode Parameters are set and the commanded pose are calculated due the current LWR State.
	 * During the VirtualFixtures State the Cartesian Stiffness is set according to the distance towards the Fixtures and the movement (approaching, removing), the NullSpaceStiffness is set to zero and the Pose is set to the measured pose.
	 *  
	 * @param lwrStatemachine The operated state machine
	 * @see LWRState
	 */
	@Override
	public void CalcControlParam(LWRStatemachine lwrStatemachine) {
		int[] NewStiffness = {0, 0, 0,  0, 0, 0 };
		double aDampVal=0.0, StiffVal =0.0 ;
		if(lwrStatemachine.InitFlag){

			ConeTip = false;
			EndPoint = false;
			if(this.VFtype==2){
				awaredist = 10;
				lwrStatemachine.InitFlag = false;
				
			}else{
				awaredist = 20;
				lwrStatemachine.InitFlag = false;
			}
		}
		 if(this.VFtype == 1){
			Vector dv = lwrStatemachine.curPose.getTranslation().subtract(VirtualFixture_ap);
			distance = VirtualFixture_n.dotProduct(dv);
			normvec = VirtualFixture_n;
		
		
		 }else if(VFtype==2){
			 
			Vector x_axis;
			Vector y_axis;
			double n;
			 if( VirtualFixture_n.getX()!=0 ){
		            n=-(VirtualFixture_n.getY()+VirtualFixture_n.getZ())/VirtualFixture_n.getX();
		            x_axis=Vector.of(n,1,1).normalize();
		            y_axis=VirtualFixture_n.crossProduct(x_axis);
			 }else if(VirtualFixture_n.getY()!=0 ){
		            n=-(VirtualFixture_n.getX()+VirtualFixture_n.getZ())/VirtualFixture_n.getY();
		            y_axis=Vector.of(1,n,1).normalize();
		            x_axis=y_axis.crossProduct(VirtualFixture_n);
			 }else{
		            x_axis=Vector.of(1, 0, 0);
		            y_axis=Vector.of(0, 1, 0);
			 }
			
			T_Base_cone = MatrixTransformation.of(VirtualFixture_ap, Matrix.ofRowFirst(1, 0, 0, 0, 1, 0, 0, 0, 1));
		
			Vector curCartPoseCOcone = T_Base_cone.getRotationMatrix().multiply(lwrStatemachine.curPose.getTranslation().subtract(VirtualFixture_ap));
			
			
			
			double diagonale = Math.sqrt(Math.pow(curCartPoseCOcone.getX(),2)+Math.pow(curCartPoseCOcone.getY(),2));
			
			distance = ((Math.sin((VirtualFixture_phi)/2)*curCartPoseCOcone.getZ()-diagonale)*Math.cos(VirtualFixture_phi/2));
			double sign = Math.signum(distance);
			Vector v_temp = curCartPoseCOcone.withZ(0);
		
			double z = Math.abs(distance)*Math.sin(VirtualFixture_phi/2);

			i++;
			
			if(curCartPoseCOcone.length() < (awaredist/2) && curCartPoseCOcone.length()>10)
			{
				
				ConeTip = true; 
				normvec_new= curCartPoseCOcone.invert().normalize();
				distance = curCartPoseCOcone.length();
			}else if (curCartPoseCOcone.length()<=10){
				EndPoint = true; 
				normvec_new= curCartPoseCOcone.invert().normalize();
			}else{ 
				if(sign == 1){
					v_temp = v_temp.invert().multiply(distance/diagonale);
				}else{
					v_temp = v_temp.multiply(distance/diagonale);
				}
				v_temp = v_temp.withZ(Math.abs(z));
				MatrixTransformation T_cone_Base = T_Base_cone.invert();
				normvec_new = T_cone_Base.getRotation().applyTo(v_temp.normalize()).normalize(); 
			}
			if(lwrStatemachine.InitFlag){
				normvec_old = normvec_new;
				normvec_old2 = normvec_old;
				normvec_old3 = normvec_old2;
				normvec_old4 = normvec_old3;
				normvec_old5 = normvec_old4;
				lwrStatemachine.InitFlag = false;
			}
			
			normvec = normvec_new.add(normvec_old).add(normvec_old2).add(normvec_old3).add(normvec_old4).add(normvec_old5).normalize();
			normvec_old5 = normvec_old4;
			normvec_old4 = normvec_old3;
			normvec_old3 = normvec_old2;
			normvec_old2 = normvec_old;
			normvec_old = normvec;
			
		 }
		if (distance < awaredist)
		{
			aDampVal = 0.7;
			
			if(ConeTip){
				if(EndPoint){
					StiffVal = max_stiff;
				}else{
					StiffVal = get_stiffness_value_ConeTip(distance, awaredist);
				}
				lwrStatemachine.cmdPose = MatrixTransformation
						.of(VirtualFixture_ap, lwrStatemachine.curPose.getRotation());
				
			}else if (distance >=0)
			{ 	
				
				
				if (distance-last_distance <= 0)	
				{
					StiffVal = get_stiffness_value_approach(distance, awaredist);
					
				}	
				else 							
				{
					StiffVal = get_stiffness_value_approach(distance, awaredist);
				}
				lwrStatemachine.cmdPose = MatrixTransformation
						.of(lwrStatemachine.curPose.getTranslation(), lwrStatemachine.curPose.getRotation());
			}
			else 				
			{
				StiffVal = max_stiff;
				lwrStatemachine.cmdPose = MatrixTransformation
						.of(lwrStatemachine.curPose.getTranslation().subtract(normvec.multiply(distance)), lwrStatemachine.curPose.getRotation());
			}
		}
		else 
		{	
			StiffVal = 0.01;
				aDampVal = 0.7;
				lwrStatemachine.cmdPose = MatrixTransformation
						.of(lwrStatemachine.curPose.getTranslation(), lwrStatemachine.curPose.getRotation());
		}
		Vector aTransStiffVal;
		if(ConeTip){
			aTransStiffVal = Vector.of(StiffVal, StiffVal,StiffVal) ;
		}else{
			aTransStiffVal = Vector.of(Math.abs(normvec.getX()*StiffVal), Math.abs(normvec.getY()*StiffVal), Math.abs(normvec.getZ()*StiffVal));
		}		
		double aRotStiffVal = StiffVal*150/5000;
		double aNullStiffVal = 0;
		 
			
		last_distance = distance;

   		// We are in CartImp Mode,
		// Modify the settings:
		// NOTE: YOU HAVE TO REMAIN POSITIVE SEMI-DEFINITE !!
		// NOTE: DONT CHANGE TOO FAST THE SETTINGS, ELSE YOU
		// WILL DESTABILIZE THE CONTROLLER
		CartesianImpedanceControlMode cartImp = (CartesianImpedanceControlMode) lwrStatemachine.controlMode;
		
		cartImp.parametrize(CartDOF.X).setStiffness(aTransStiffVal.getX());
		cartImp.parametrize(CartDOF.Y).setStiffness( aTransStiffVal.getY());
		cartImp.parametrize(CartDOF.Z).setStiffness( aTransStiffVal.getZ());
		cartImp.parametrize(CartDOF.ROT).setStiffness(aRotStiffVal);
		NewStiffness[0] = (int) aTransStiffVal.getX();
		NewStiffness[1] = (int) aTransStiffVal.getY();
		NewStiffness[2] = (int) aTransStiffVal.getZ();
		NewStiffness[3] =  (int) aRotStiffVal;
		NewStiffness[3] =  (int) aRotStiffVal;
		NewStiffness[3] =  (int) aRotStiffVal;
		cartImp.setNullSpaceStiffness(aNullStiffVal);
		cartImp.parametrize(CartDOF.ALL).setDamping(aDampVal);
		// Send the new Stiffness settings down to the
		// controller
		lwrStatemachine.controlMode = cartImp;
		lwrStatemachine.curCartStiffness = NewStiffness;
	}
	/**
	 * In this Function the Acknowledge IGTMessage which is send to the State Control is defined due the current LWR State.
	 * In the Virtual Fixtures State IGTString is Set to "VirtualFixtures;plane;"/"VirtualFixtures;cone;" or "VirtualFixtures;none;" dependent on the selected Virtual Fixtures type.
	 * @param lwrStatemachine The operated Statemachine
	 */
	@Override
	public void SetACKPacket(LWRStatemachine lwrStatemachine) {
		String ACK;
		if (VFtype == 1){
			if(distance>=awaredist){
				ACK = "VirtualFixtures;plane;0;";
			}else if(distance<awaredist && distance>0){
				ACK = "VirtualFixtures;plane;1;";
			}else{
				ACK = "VirtualFixtrues;plane;2;";
			}
		}else if(VFtype==2){
			if(distance>=awaredist){
				ACK = "VirtualFixtures;cone;0;";
			}else if(distance<awaredist && distance>0){
				ACK = "VirtualFixtures;cone;1;";
			}else{
				ACK = "VirtualFixtrues;cone;2;";
			}
		}else{
			ACK = "VirtualFixtures;none;";
		}
		lwrStatemachine.AckIGTmessage = ACK;// TODO Automatisch generierter Methodenstub
	}
	
	/**
	 * In this Function the CommandIGTmessage which is received from the State Control is read and interpreted due to the Current State and if requested 
	 * and allowed the State is Changed. Furthermore the parameter of the state are set and the class of the LWRState Object is changed according to the state change
	 * In The VirtualFixtures State the allowed state transitions are:<br>
	 * 	- IDLE (transition condition: none)<br>
	 * 	- GravComp (transition condition: none)<br>
	 *  - PathImp (transition condition: RegistrationFinished AND DataSend AND TransformRecieved)<br>
	 *  - MoveToPose (transition condition: RegistrationFinished AND DataSend AND TransformRecieved)<br>
	 * If a transition request to another State is requested or the number of Parameters is incorrect, the state is set to LWRError and the Error code is set to 2= Transition not allowed. 
	 * @param lwrStatemachine - The operated state machine
	 * @see LWRState
	 */
	@Override
	public void InterpretCMDPacket(LWRStatemachine lwrStatemachine) {
		// TODO Automatisch generierter Methodenstub
		if(lwrStatemachine.IGTLdatatype.equals("STRING")){
			String CMD_String;
			
				CMD_String = lwrStatemachine.CmdIGTmessage;
				lwrStatemachine.ParameterString = CMD_String.substring(CMD_String.indexOf(";"));
				String[] CMD_Array =CMD_String.split(";");

						lwrStatemachine.InitFlag =true;
						if(CMD_Array[1].contentEquals("img")){
							this.ImageSpace = true;
						}else if(CMD_Array[1].contentEquals("rob")){
							this.ImageSpace = false;
						}else{
							lwrStatemachine.ErrorMessage = ("Unexpected coordinate system (supported are img or plane)");
							lwrStatemachine.ErrorFlag = true;
						}
						if(CMD_Array[2].contentEquals("plane")){
							this.VFtype=1;
						}else if (CMD_Array[2].contentEquals("cone")){
							this.VFtype=2;
						}else{
							lwrStatemachine.ErrorMessage = ("Unexpected VF type (supported are plane or cone)");
							lwrStatemachine.ErrorFlag = true;
						}
						
						
						this.VirtualFixture_ap = Vector.of(Double.parseDouble(CMD_Array[3]), Double.parseDouble(CMD_Array[4]), Double.parseDouble(CMD_Array[5]));
						this.VirtualFixture_n = Vector.of(Double.parseDouble(CMD_Array[6]), Double.parseDouble(CMD_Array[7]), Double.parseDouble(CMD_Array[8]));
						
						if(this.ImageSpace && lwrStatemachine.TransformRecieved){
							this.VirtualFixture_ap = lwrStatemachine.TransformRobotImage.applyTo(this.VirtualFixture_ap);
							this.VirtualFixture_n = lwrStatemachine.TransformRobotImage.applyTo(this.VirtualFixture_n);
							
						}
						if (this.VFtype == 2){
							//this.VirtualFixture_phi = Double.parseDouble(CMD_Array[9])*Math.PI/180;
							this.VirtualFixture_phi = 135 * Math.PI/180;
						}
						System.out.println( "Virtaul Fixture ( ap =" + this.VirtualFixture_ap + ", n " + this.VirtualFixture_n + ", type " + CMD_Array[2] +") is now active!" );

		}else{
			lwrStatemachine.ErrorCode = 12; //Illegal/unknown instruction
			lwrStatemachine.ErrorMessage = "Unexpected Messagetype recieved! Expected STRING"; 
		}
	}
	
	

}
