/**********************************************
 * 
 * Author: Yigiter Yuksel
 * 
 * 01.2010
 * instk.org
 * 
 */

package org.instk.demo_1001;

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.view.Gravity;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.ViewGroup.LayoutParams;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;

public class main extends Activity implements OnClickListener, SensorEventListener{
	
	private final float N2S=1000000000f;
	
	//Initial Camera Pos/Att
	private float[] INIPos={0,0,5};
	private float[] INIVel={0,0,0};
	private float[] INICbn={1,0,0,0,1,0,0,0,1};
	
	private GLSurfaceView mGLSView;
	private SensorManager mSMan;
	private MyCube mCube;
	private INS mINS;
	private Kalman mKalman;
	
	//Debug Text View
	TextView etv;
	
	//Mode of the program 
	//0 = 3Dof (Acc+Compass)
	//1 = 3Dof (Gyro)
	//2 = 6Dof
	//3 = Bias Removal
	int mode=0;
	boolean ZFlag=false, CFlag=false;
	
	//Most recent sensor data and its timestamp
	float[] dAcc=new float[3];
	float[] dGyro=new float[3];
	float[] dMag=new float[3];
	long etime=0; //Time for the latest sensor data
	long ptAcc=0,ptGyro=0,ptMag=0;	//previous Sample Time
	
	//Sensor Calibration values
	float[] cBAcc=new float[3];
	float[] cBGyro=new float[3];
	
	//Periods
	private final long PERBR=(long) (1*N2S);	//Data collection period for bias removal (Nanosec)
	private long tBR=0;
	private final long PERPROP=(long) (1*N2S);	//KF maximum covariance propagation period (Nanosec)
	private long ptProp=0, tProp=0;
	
	//Temporary variables to be used (used to redeuce the load of GC)
	float[] vr_a=new float[3];
	float[] vr_b=new float[3];
	float[] mx_a=new float[9];
	
	public main() {
		mCube=new MyCube(INIPos, INICbn);
		mINS=new INS(INIPos, INIVel, INICbn);
		mKalman=new Kalman();
	}
	
	
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        
        //Get a reference to sensor manager
        mSMan = (SensorManager)getSystemService(Context.SENSOR_SERVICE);
        
        //Set the OpenGl View
        mGLSView = new GLSurfaceView(this);
        mGLSView.setRenderer(mCube);
        setContentView(mGLSView);
        
        //Add button and text elements to the view
        LinearLayout ll = new LinearLayout(this);
        ll.setOrientation(LinearLayout.VERTICAL);
        ll.setGravity(Gravity.CENTER_HORIZONTAL | Gravity.BOTTOM);
        
        LinearLayout llb1 = new LinearLayout(this);
        llb1.setOrientation(LinearLayout.HORIZONTAL);
        llb1.setLayoutParams(new LayoutParams(LayoutParams.WRAP_CONTENT, LayoutParams.WRAP_CONTENT));
        llb1.setGravity(Gravity.CENTER);
        
        LinearLayout llb2 = new LinearLayout(this);
        llb2.setOrientation(LinearLayout.HORIZONTAL);
        llb2.setLayoutParams(new LayoutParams(LayoutParams.WRAP_CONTENT, LayoutParams.WRAP_CONTENT));
        llb2.setGravity(Gravity.CENTER);
        
        Button mbut0 = new Button(this);
        mbut0.setText("3Dof(A+C)");
        mbut0.setId(0);
        Button mbut1 = new Button(this);
        mbut1.setText("3DoF(Gyro)");
        mbut1.setId(1);
        Button mbut3 = new Button(this);
        mbut3.setText("6DoF");
        mbut3.setId(3);
        
        Button mbut2 = new Button(this);
        mbut2.setText("Bias Rem.");
        mbut2.setId(2);
        Button mbut4 = new Button(this);
        mbut4.setText("ZUPT");
        mbut4.setId(4);
        Button mbut5 = new Button(this);
        mbut5.setText("CUPT");
        mbut5.setId(5);
        Button mbut6 = new Button(this);
        mbut6.setText("Reset");
        mbut6.setId(6);
        llb1.addView(mbut0);
        llb1.addView(mbut1);
        llb1.addView(mbut3);
        llb2.addView(mbut2);
        llb2.addView(mbut4);
        llb2.addView(mbut5);
        llb2.addView(mbut6);
        
        etv= new TextView(this);
        etv.setText("Default: Acc+Compass");
        
        //ll.addView(etv);
        ll.addView(llb1);
        ll.addView(llb2);
        this.addContentView(etv, new LayoutParams(LayoutParams.FILL_PARENT, LayoutParams.FILL_PARENT));
        this.addContentView(ll, new LayoutParams(LayoutParams.FILL_PARENT, LayoutParams.FILL_PARENT));
        
        mbut0.setOnClickListener(this);
        mbut1.setOnClickListener(this);
        mbut2.setOnClickListener(this);
        mbut3.setOnClickListener(this);
        mbut4.setOnClickListener(this);
        mbut5.setOnClickListener(this);
        mbut6.setOnClickListener(this);
    }
    
    @Override
    protected void onResume() {
        super.onResume();
        //OpenGl View
        mGLSView.onResume();
        
        //Sensors
        Sensor sAcc = mSMan.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        Sensor sMag = mSMan.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        Sensor sGyro = mSMan.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        
        int rate=SensorManager.SENSOR_DELAY_FASTEST;
        
        mSMan.registerListener(this, sAcc, rate);
        if (sMag!=null) mSMan.registerListener(this, sMag, rate);
        if (sGyro!=null)
        	mSMan.registerListener(this, sGyro, rate);
        else
        	etv.setText("## You don't have a gyro ##");
    }

    @Override
    protected void onPause() {
        // Ideally a game should implement onResume() and onPause()
        // to take appropriate action when the activity looses focus
        super.onPause();
        mGLSView.onPause();
        mSMan.unregisterListener(this);
    }
    
    @Override
	public void onClick(View v) {
    	int cmd=v.getId();	
    	flow_control(cmd);
	}
    
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    	//Do nothing
    }

    public void onSensorChanged(SensorEvent event) {
    	int etype = event.sensor.getType();
    	etime = event.timestamp;
    	float dt=0;
    	
    	
    	//Recod the value and time
    	switch (etype) {
    	case Sensor.TYPE_ACCELEROMETER:
    		dAcc[0]=event.values[0]-cBAcc[0];
    		dAcc[1]=event.values[1]-cBAcc[1];
    		dAcc[2]=event.values[2]-cBAcc[2];
    		if (ptAcc!=0) dt=(etime-ptAcc)/N2S;
    		ptAcc=etime;
    		break;
    	case Sensor.TYPE_GYROSCOPE:
    		dGyro[0]=event.values[0]-cBGyro[0];
    		dGyro[1]=event.values[1]-cBGyro[1];
    		dGyro[2]=event.values[2]-cBGyro[2];
    		if (ptGyro!=0) dt=(etime-ptGyro)/N2S;
    		ptGyro=etime;
    		break;
    	case Sensor.TYPE_MAGNETIC_FIELD:
    		dMag[0]=event.values[0];dMag[1]=event.values[1];dMag[2]=event.values[2];
    		if (ptMag!=0) dt=(etime-ptMag)/N2S;
    		ptMag=etime;
    		
    		break;
    	}
    	
    	if (mode==0) { //3dof based on Acc+Compass
    		//We have to be sure that neither Acc nor Compass is zero.
    		if ((dAcc[0]*dAcc[0]+dAcc[1]*dAcc[1]+dAcc[2]*dAcc[2])<80)
    			return;
    		else if ((dMag[0]*dMag[0]+dMag[1]*dMag[1]+dMag[2]*dMag[2])<400) //30ut*30ut
    			return;
    		
    		//I am going to use android's built function for body vector to dcm transformation (see the development notes for the criticism of this method)
    		SensorManager.getRotationMatrix(mx_a, null, dAcc, dMag);
    		
    		//Set the new orientation in INS and Cube classes
    		mCube.set_dcm(mx_a);
    		mINS.set_dcm(mx_a);
    	}
    	else if (mode==1) { //3Dof Gyro (mINS must have an initial orientation)
    		if (etype==Sensor.TYPE_GYROSCOPE && dt>0) {
    			//Update attitude
    			mINS.update_attI(dGyro, dt);
    			
    			//Set the camera orientation for cube
    			mCube.set_dcm(mINS.get_dcm());
    		}
    		
    	}
    	else if (mode==2) { //Bias removal under LEVEL and STATIONARY conditions
    		if (etype==Sensor.TYPE_ACCELEROMETER)
    			mINS.accum.addacc(dAcc);
    		
    		if (etype==Sensor.TYPE_GYROSCOPE)
    			mINS.accum.addgyro(dGyro);
    		
    		if (etime>tBR) {
    			int i;
    			//Set the bias values
    			mINS.get_gravity(vr_a);
    			mINS.accum.avacc(cBAcc);
    			for (i=0;i<3;i++)
    				cBAcc[i]+=vr_a[i];	//Gravity is in ned. That is why we add them.
    			
    			mINS.accum.avgyro(cBGyro);
    			
    			//Reset the accumulators and time
    			mINS.accum.clear();
    			tBR=0;
    			
    			//Set the mode to 0 automatically after this routine
    			flow_control(0);
    			etv.setText("Abias :" + cBAcc[0] + "\n" + cBAcc[1] + "\n"+ cBAcc[2] + "\n" +
    					    "Gbias :" + cBGyro[0] + "\n" + cBGyro[1] + "\n"+ cBGyro[2]);
    		}	
    	}
    	else if (mode==3) {	//6Dof Calculations
    		if (etype==Sensor.TYPE_ACCELEROMETER) { //Update velocity and Pos
    			//Update pos and vel   			
    			mINS.update_velI(dAcc, dt);
        		mINS.update_posII(dt);
        		
        		//Update acc accum
        		mINS.accum.addacc(dAcc);
    		}
    		
    		if (etype==Sensor.TYPE_GYROSCOPE) { //Update velocity and Pos
    			//Update pos and vel
    			mINS.update_attI(dGyro, dt);
    			mINS.update_velII(dGyro, dt);
        		mINS.update_posI(dGyro, dt);
        		
        		//Update acc accum
        		mINS.accum.addgyro(dGyro);
    		}
    		
    		//Set the camera pos & orientation for cube
    		mCube.set_dcm(mINS.get_dcm());
    		mCube.set_pos(mINS.get_pos());
    		
    		//State Updates and Covariance propagation
    		if (etime>tProp || ZFlag || CFlag) {
    			//First update (propagate the covariance to the current time) 
    			dt=(etime-ptProp)/N2S;
    			ptProp=etime;
    			
    			//Propagate the covariance
    			mKalman.Propagate(mINS, dt);
    			
    			//Clear sensor data accumulators
    			mINS.accum.clear();
    			
    			//Next Covaraince update time
    			tProp=etime+PERPROP;
    			
    			//Debug screen
        		etv.setText("Pos :" + mINS.Pos_b.data[0] + "\n" + mINS.Pos_b.data[1] + "\n"+ mINS.Pos_b.data[2] + "\n" +
    				    "Vel :" + mINS.Vel_b.data[0] + "\n" + mINS.Vel_b.data[1] + "\n"+ mINS.Vel_b.data[2]);
    			//etv.setText("Abias :" + cBAcc[0] + "\n" + cBAcc[1] + "\n"+ cBAcc[2] + "\n" +
				//	    "Gbias :" + cBGyro[0] + "\n" + cBGyro[1] + "\n"+ cBGyro[2]);
        		
        		//flow_control(4);
        		
        		//Check if there is an update request
        		if (ZFlag) { //Process Zupt request
        			mKalman.applyZupt(mINS, cBAcc, cBGyro);
        			ZFlag=false;
        		}
        		if (CFlag) { //Process Cupt Request
        			mKalman.applyCupt(mINS, cBAcc, cBGyro, INIPos);
        			CFlag=false;
        		}
        		
    		}
    		
    			
    	}
    }
    
    
    //Decides what to do
    private void flow_control(int cmd) {
    	if (cmd==0){
    		mode=0;
    		
    		//Reset the pos and velocity (attitude will be set at each update)
    		mINS.set_pos(INIPos);
    		mINS.set_vel(INIVel);
    		
    		mCube.set_pos(mINS.get_pos());
    	}
    	else if (cmd==1) {
    		mode=1;
    		
    		//Reset the pos and velocity (gyro data updates the previous attitude)
    		mINS.set_pos(INIPos);
    		mINS.set_vel(INIVel);
    		
    		mCube.set_pos(mINS.get_pos());
    		
    		//Debug
    		etv.setText("Switched to Gyro mode");
    	}
    	else if (cmd==2) {	//Bias Removal
    		if (etime>0) {
    			mode=2;
    			//Reset the previous bias estimates and sensor data accumulators
    			for (int i=0;i<3;i++) {
    				cBAcc[i]=0;
    				cBGyro[i]=0;
    			}
    			mINS.accum.clear();
    			
    			//Maximum time to stay in this mode
	    		tBR=etime+PERBR;
    		}
    	}
    	else if (cmd==3) {	//6Dof
    		mode=3;

    		//Covariance propagation times
    		ptProp=etime;	//Implicitly assumes that there were at least one sensor sample before coming to this point
    		tProp=ptProp+PERPROP;
    		
    		//Clear accumulators
    		mINS.accum.clear();
    		
    		//Clear Update Flags
    		ZFlag=false;
    		CFlag=false;
    		
    		//Reset the covariance
    		mKalman.initP();
    	}
    	
    	else if (cmd==4) {	//Zupt
    		if (mode==3)
    			ZFlag=true;
    		else
    			etv.setText("Zupt can only be applied in 6DoF mode");
    	}
    	
    	else if (cmd==5) {	//Cupt for the initial Position
    		if (mode==3)	
    			CFlag=true;
    		else
    			etv.setText("Zupt can only be applied in 6DoF mode");
    	}
    	
    	else if (cmd==6) {	//Reset the states
    		//Reset the INS
    		mINS.set_pos(INIPos);
    		mINS.set_vel(INIVel);
    		mINS.set_dcm(INICbn);
    		for (int i=0;i<3;i++) {
    			cBAcc[i]=0;
    			cBGyro[i]=0;
    		}
    		mINS.accum.clear();
    		
    		//Set the camera pos & orientation for cube
    		mCube.set_dcm(mINS.get_dcm());
    		mCube.set_pos(mINS.get_pos());
    		
    		//Reset Kalman
    		ZFlag=false;
    		CFlag=false;
    		mKalman.initP();
    		
    		//change mode to 0
    		mode=0;
    	}
    }
    
}