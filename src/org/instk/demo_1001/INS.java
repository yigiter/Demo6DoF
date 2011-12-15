/**********************************************
 * 
 * Author: Yigiter Yuksel
 * 
 * 01.2010
 * instk.org
 * 
 */

package org.instk.demo_1001;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;

import android.hardware.SensorManager;	//Just for the value of gravity

public class INS {
	public DenseMatrix64F Pos_b=new DenseMatrix64F(3,1);
	public DenseMatrix64F Vel_b=new DenseMatrix64F(3,1);
	public DenseMatrix64F Cbn=new DenseMatrix64F(3,3);
	
	public DenseMatrix64F gravity=new DenseMatrix64F(3,1);
	
	public DataAccum accum=new DataAccum();		//sensor data accumulator
	
	//Temporary variables (Save the GC, save the world)
	private DenseMatrix64F acc=new DenseMatrix64F(3,1);
	private DenseMatrix64F gyro=new DenseMatrix64F(3,1);
	DenseMatrix64F rotv=new DenseMatrix64F(3,1);
	DenseMatrix64F mx_a=new DenseMatrix64F(3,3);
	DenseMatrix64F mx_b=new DenseMatrix64F(3,3);
	DenseMatrix64F vr_a=new DenseMatrix64F(3,1);
	
	public INS(float[] pos, float[] vel, float[] dcm) {
		int i;
		for (i=0;i<3;i++) {
			Pos_b.set(i,pos[i]);
			Vel_b.set(i,vel[i]);
		}
		
		for (i=0;i<9;i++) {
			Cbn.set(i,dcm[i]);
		}
		
		//Assume a fixed gravity
		gravity.set(2,-SensorManager.GRAVITY_EARTH); //Gravity in NED (not ENU) 
	}
	
	//Setters and getters
	public void set_dcm(float[] dcm) {
		for (int i=0;i<9;i++) {
			Cbn.set(i,dcm[i]);
		}
	}
	
	public void set_pos(float[] pos) {
		for (int i=0;i<3;i++)
			Pos_b.set(i,pos[i]);
	}
	
	public void set_vel(float[] vel) {
		for (int i=0;i<3;i++)
			Vel_b.set(i,vel[i]);
	}
	
	public double[] get_dcm() {
		return Cbn.data;
	}
	
	public double[] get_pos() {
		return Pos_b.data;
	}
	
	public void get_posn(float[] out) {
		CommonOps.mult(Cbn, Pos_b, vr_a);
		out[0]=(float) vr_a.get(0);
		out[1]=(float) vr_a.get(1);
		out[2]=(float) vr_a.get(2);
	}
	
	public void get_gravity(float[] out) {
		out[0]=(float) gravity.get(0);
		out[1]=(float) gravity.get(1);
		out[2]=(float) gravity.get(2);
	}
	
	//Algorithms
	public void update_attI(float[] gdat, float dt) {
		rotv.set(0,gdat[0]*dt);
		rotv.set(1,gdat[1]*dt);
		rotv.set(2,gdat[2]*dt);
		
		//Convert rotation vector to dcm
		rot2dcm(rotv, mx_a);
    	
		//Update the dcm
		CommonOps.mult(Cbn, mx_a, mx_b);
		Cbn.set(mx_b);
	}
	
	public void rot2dcm(DenseMatrix64F rotvec, DenseMatrix64F dcm) {
		double[] rot=rotvec.data;
		double rot_norm=NormOps.fastNormF(rotv);
		
		if (rot_norm>0) {
    	    double sr_a=Math.sin(rot_norm)/rot_norm;
    	    double sr_b=(1-Math.cos(rot_norm))/(rot_norm*rot_norm);
    	    
    	    //Dcm =eye(3)+sr_a*skew(rot)+sr_b*skew(rot)*skew(rot);
    	    dcm.set(0,0,1+sr_b*(-rot[2]*rot[2]-rot[1]*rot[1]));
    	    dcm.set(0,1,sr_b*(rot[1]*rot[0])+sr_a*(-rot[2]));
    	    dcm.set(0,2,sr_b*(rot[2]*rot[0])+sr_a*(rot[1]));
    	    dcm.set(1,0,sr_b*(rot[0]*rot[1])+sr_a*(rot[2]));
    	    dcm.set(1,1,1+sr_b*(-rot[2]*rot[2]-rot[0]*rot[0]));
    	    dcm.set(1,2,sr_b*(rot[2]*rot[1])+sr_a*(-rot[0]));
    	    dcm.set(2,0,sr_b*(rot[2]*rot[0])+sr_a*(-rot[1]));
    	    dcm.set(2,1,sr_b*(rot[2]*rot[1])+sr_a*(rot[0]));
    	    dcm.set(2,2,1+sr_b*(-rot[1]*rot[1]-rot[0]*rot[0]));
    	}
	}
	
	public void update_velI(float[] adat, float dt) {
		acc.set(0, adat[0]);
		acc.set(1, adat[1]);
		acc.set(2, adat[2]);
		
		//Specific force  (acc=acc+Cbn'*gravity)
		CommonOps.multAddTransA(Cbn, gravity, acc);
		
		//Update vel with specific force (Vel_b=Vel_b+dt*acc)
		CommonOps.addEquals(Vel_b, dt, acc);
	}
	
	public void update_velII(float[] gdat, float dt) {
		gyro.set(0, gdat[0]);
		gyro.set(1, gdat[1]);
		gyro.set(2, gdat[2]);
		
	    //vel_inc=(cross(Vb,(Cbn'*wie_n)+w))*dt;
		skew(Vel_b, mx_a);
		CommonOps.multAdd(dt, mx_a, gyro, Vel_b); //Vel_b=Vel_b+dt*mx_a*gyro
	}
	
	public void update_posI(float[] gdat, float dt) {
		gyro.set(0, gdat[0]);
		gyro.set(1, gdat[1]);
		gyro.set(2, gdat[2]);
		
    	//Update for the rotation (Pos_b=Pos_b+cross(Pos_b,gyro)
		skew(Pos_b, mx_a);
		CommonOps.multAdd(dt, mx_a, gyro, Pos_b);
    }
	
	public void update_posII(float dt) {
		//Update pos with body vel (Pos_b=Pos_b+dt*Vel_b)
		CommonOps.addEquals(Pos_b, dt, Vel_b);
	}
	
	public static void skew(DenseMatrix64F vec, DenseMatrix64F smat) {
		smat.zero();
    	smat.set(0,1,-vec.get(2));
    	smat.set(0,2,vec.get(1));
    	smat.set(1,0,vec.get(2));
    	smat.set(1,2,-vec.get(0));
    	smat.set(2,0,-vec.get(1));
    	smat.set(2,1,vec.get(0));
	}
	
	public void update(DenseMatrix64F dx) {
		
		double sr_a;
		for (int i=0;i<3;i++) {
			sr_a=Pos_b.get(i)-dx.get(i);
			Pos_b.set(i,sr_a);
			sr_a=Vel_b.get(i)-dx.get(i+3);
			Vel_b.set(i,sr_a);
			
			vr_a.set(i, dx.get(i+6));
		}
		
		skew(vr_a,mx_a);
		CommonOps.mult(mx_a, Cbn, mx_b);
		CommonOps.addEquals(Cbn, mx_b);
	}
	
	///////////////////////////////////////////////////////////
	//Sensor Data Accumulator
	class DataAccum {
		private DenseMatrix64F acacc=new DenseMatrix64F(3,1);
		private DenseMatrix64F acgyro=new DenseMatrix64F(3,1);
		private int acina=0, acing=0;	//Accumulator indexes
		
		//Temporary variables
		private DenseMatrix64F vr_a=new DenseMatrix64F(3,1);
		
		public void clear() { //Clears the sensor data accumulators
	    	acacc.zero();
	    	acgyro.zero();
			acina=0;
			acing=0;
	    }
				
		public void addacc(float[] dat) {	//Updates the acc accumulator
			acc.set(0, dat[0]);
			acc.set(1, dat[1]);
			acc.set(2, dat[2]);
			
			CommonOps.addEquals(acacc, acc);
			acina++;
		}
		
		public void addgyro(float[] dat) {	//Updates the gyro accumulator
			gyro.set(0, dat[0]);
			gyro.set(1, dat[1]);
			gyro.set(2, dat[2]);
			
			CommonOps.addEquals(acgyro, gyro);
			acing++;
		}
		
		public void avacc(float[] out) { //Avarage of acc
			if (acina>0) 
				for (int i=0;i<3;i++)
					out[i]=(float) acacc.get(i)/acina;
			else
				{out[0]=0;out[1]=0;out[2]=0;}
		}
		
		public void avgyro(float[] out) { //Avarage of gyro
			if (acing>0) 
				for (int i=0;i<3;i++)
					out[i]=(float) acgyro.get(i)/acing;
			else
				{out[0]=0;out[1]=0;out[2]=0;}
		}
		
		//Note: Below the return values are the skewed matrices!!!
		public void avacc(DenseMatrix64F out) { //Avarage of acc
			if (acina>0) {
				CommonOps.scale(1/acina, acacc, vr_a);
				skew(vr_a, out);
			}
			else
				out.zero();
		}
		
		public void avgyro(DenseMatrix64F out) { //Avarage of gyro
			if (acing>0) {
				CommonOps.scale(1/acing, acgyro, vr_a);
				skew(vr_a, out);
			}
			else
				out.zero();
		}
	};
}