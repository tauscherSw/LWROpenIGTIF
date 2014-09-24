/*=========================================================================

  Program:   IGTMessageHandler
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

/**
 * This Class is handling the display output on the KUKA SmartPad.
 * @author Sebastian Tauscher
 * @see LWRVisualizationInterface
 * @see LWRStateMachineInterface
 * 
 */
public class IGTMessageHandler extends Thread {
	
     
	/**
	 * Error Message String for error in the State machine interface
	 */
	public String ErrorMessage="";
	/**
	 * Name of the thread using this Message handler
	 */
	public String Sendername = "";
	/**
	 * in this String the last printed error message is saved to check if it is error message has already been printed
	 */
	private String LastPrintedError = "";
	
	/**
	 * Semaphore for save reading and writing the variables 
	 */
    public Semaphore MessageSemaphore = new Semaphore(1,true);
    /**
	 * cycle time of the state control interface thread. Default value is 20 ms
	 */
	public int millisectoSleep = 1000;
	/**
	 * Flag indicating if the message handler is active
	 */
	public boolean MessageHandlerRun = true;

	/**
	 * Flag to indicate if this message handler is displaying the errors at the smartPad or not
	 */
	public boolean DebugInfos = false;
   
  
   
    public IGTMessageHandler(){
    	setDaemon(true);
    };
    
    public void finalize(){
 
    	
    }

    /**
    * Initialize function of the State control Interface. In this function the server is initialized and a packet handler is started.
    * In a loop with a cycle time of 20 ms the new Command String is received and the Acknowledgment String send.
    **/
    public void run() {
		// TODO Automatisch generierter Methodenstub
    
		while(MessageHandlerRun){
			
			long startTimeStamp = (long) (System.nanoTime());
		    try {
				MessageSemaphore.acquire();
			  if (!this.ErrorMessage.equals(this.LastPrintedError)){
					  if(DebugInfos){
							System.out.println("From: " + Sendername +": " + this.ErrorMessage);
					  }
					this.LastPrintedError = this.ErrorMessage;
				}
		    }catch(InterruptedException e){
		    	
		    }
			MessageSemaphore.release();
			//Set the Module in Sleep mode for stability enhancement
			long curTime = (long) ((System.nanoTime() - startTimeStamp));
			long curTime_millis = (long) curTime/1000000;
			int curTime_nanos = (int) (curTime%1000000);
			if(curTime_millis<millisectoSleep){
				//ThreadUtil.milliSleep((long) Math.floor((millisectoSleep-1 -  curTime)));
				try {
					Thread.sleep(millisectoSleep-1-curTime_millis, 999999-curTime_nanos);
				} catch (InterruptedException e) {
					// TODO Automatisch generierter Erfassungsblock
					//e.printStackTrace();
				}
			}
		}
	}
		
	
	
    
	

}
