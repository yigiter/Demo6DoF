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

public class Kalman {
	DenseMatrix64F P=new DenseMatrix64F(15,15);
	DenseMatrix64F A=new DenseMatrix64F(15,15);
	DenseMatrix64F N=new DenseMatrix64F(9,6);
	DenseMatrix64F STM=new DenseMatrix64F(15,15);
	DenseMatrix64F Q=new DenseMatrix64F(15,15);
	DenseMatrix64F Qrw=new DenseMatrix64F(6,6);
	DenseMatrix64F H=new DenseMatrix64F(3,15);
	DenseMatrix64F K=new DenseMatrix64F(15,3);
	DenseMatrix64F dz=new DenseMatrix64F(3,1);
	DenseMatrix64F dx=new DenseMatrix64F(15,1);
	DenseMatrix64F R=new DenseMatrix64F(3,3);
	
	//Temporary Variables
	DenseMatrix64F I=new DenseMatrix64F(3,3);
	DenseMatrix64F mx_a=new DenseMatrix64F(3,3);
	DenseMatrix64F mx_b=new DenseMatrix64F(3,3);
	DenseMatrix64F mx_c=new DenseMatrix64F(9,6);
	DenseMatrix64F mx_d=new DenseMatrix64F(9,9);
	DenseMatrix64F mx_e=new DenseMatrix64F(15,3);
	DenseMatrix64F mx_f=new DenseMatrix64F(3,15);
	
	
	private final double VRW=0.2, ARW=0.04; //Random Walk values: Just made these up
	
	public Kalman() {
		initP();
		CommonOps.setIdentity(I);
	}

	//Initialize the covariance
	public void initP() {
		//States pos,vel, att, acc, gyro
    	//Pos
    	P.set(0,0,0);
    	P.set(1,1,0);
    	P.set(2,2,0);
    	
    	//Vel
    	P.set(3,3,0.05*0.05);
    	P.set(4,4,0.05*0.05);
    	P.set(5,5,0.05*0.05);
    	
    	//Attitude (the initial attitude are determined from acc. Therefore, these states must be correlated with imu errors. However, ....)
    	P.set(6,6,0.3*0.3);
    	P.set(7,7,0.3*0.3);
    	P.set(8,8,0.1*0.1);
    	 
    	//Acc (biases) (simply made up)
    	P.set(9,9,0.5*0.5);
    	P.set(10,10,0.5*0.5);
    	P.set(11,11,0.5*0.5);
    	
    	//Gyro (biases) (made up)
    	P.set(12,12,0.01*0.01);
    	P.set(13,13,0.01*0.01);
    	P.set(14,14,0.01*0.01);
    	
    	//if anyone sends me some stationary imu data for the phone, I will gladly perform modelling on your behalf.
	}
	
	public void Propagate(INS mINS, float dt) {
		//Compute the system matrix
		A.zero();
		N.zero();
		sys_bd_dcm(A,N, mINS);
		
		//Discretize A
		CommonOps.setIdentity(STM);
		CommonOps.addEquals(STM,dt,A);
		
		//Discrete time Q matrix
		Q.zero();
		Qrw.zero();
		setsub(Qrw,0,0,I,VRW*dt);
		setsub(Qrw,3,3,I,ARW*dt);
		CommonOps.mult(N,Qrw,mx_c);
		CommonOps.multTransB(mx_c,N,mx_d);
    	setsub(Q,0,0,mx_d,1);
    	
    	//Propagate P (use A as the temporary matrix) P=STM*P*STM'+Q;
    	CommonOps.mult(STM,P,A);
		CommonOps.multTransB(A,STM,P);
		CommonOps.addEquals(P, Q);
	}
	
	private void sys_bd_dcm(DenseMatrix64F fA, DenseMatrix64F fN, INS mINS) {	
		////Input errors (N)
    	//Attitude N(6:8,3,5)=-Cbn
		setsub(fN,6,3,mINS.Cbn,-1);
		
		//Velocity  N(4:6,1:3)=eye(3);  N(4:6,4:6)=skew(vel_x);
		setsub(fN,3,0,I,1);
		skew(mINS.Vel_b,mx_a);
    	setsub(fN,3,3,mx_a,1);
    	
    	//Position
    	skew(mINS.Pos_b,mx_a);
    	setsub(fN,0,3,mx_a,1);
		
    	////Characteristics (A)
    	//Attitude (no dynamics)
    	
    	//Velocity F(4:6,4:6)=-skew(gyro);     F(4:6,7:9)=Cbn'*skew(-gravity);
    	mINS.accum.avgyro(mx_a);
    	setsub(fA, 3,3, mx_a,-1);
    	skew(mINS.gravity, mx_a);
    	CommonOps.multTransA(mINS.Cbn, mx_a, mx_b);
    	setsub(fA, 3, 6, mx_b, -1);
    	
    	//Position
    	mINS.accum.avgyro(mx_a);
    	setsub(fA,0,0,mx_a,-1);
    	setsub(fA,0,3,I,1);
    	
    	//Imu stability errors (random constant)
    	/*
    	for (i=9;i<15;i++)
    		A.set(i,i,0);
    	*/
    	
    	//Effect of imu errors on nav states
    	setsub(fA,0,9,fN,1);
	}
	
	
	private void setsub(DenseMatrix64F A, int rs, int cs, DenseMatrix64F S, double c) {
    	int i,k;
    	for (i=0;i<S.numRows;i++) {
    		for (k=0;k<S.numCols;k++) {
    			A.set(i+rs,k+cs,c*S.get(i,k));
    		}
    	}
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
	
	public void applyZupt(INS mINS, float[] BAcc, float[] BGyro) {
		//Observation matrix
		H.zero();
		setsub(H,0,3,I,1);
		
		R.zero();
		setsub(R,0,0, I, 0.01*0.01);
		
		dz.set(mINS.Vel_b);
		
		update(P, H, R, dz, dx);
		
		//Update INS and bias estimates
		mINS.update(dx);
		for (int i=0;i<3;i++) {
			BAcc[i]=BAcc[i]+(float)dx.get(9+i);
			BGyro[i]=BGyro[i]+(float)dx.get(12+i);	
		}
	}
	
	public void applyCupt(INS mINS, float[] BAcc, float[] BGyro, float[] UPos) {
		//Observation matrix
		H.zero();
		setsub(H,0,0,I,1);
		
		R.zero();
		setsub(R,0,0, I, 0.2*0.2);
		
		dz.set(mINS.Pos_b);
		for (int i=0;i<3;i++)
			dz.set(i,dz.get(i)-UPos[i]);
		
		update(P, H, R, dz, dx);
		
		//Update INS and bias estimates
		mINS.update(dx);
		for (int i=0;i<3;i++) {
			BAcc[i]=BAcc[i]+(float)dx.get(9+i);
			BGyro[i]=BGyro[i]+(float)dx.get(12+i);	
		}
	}
	
	private void update(DenseMatrix64F sP, DenseMatrix64F sH, DenseMatrix64F sR, DenseMatrix64F sdz, DenseMatrix64F sdx) {
		
		//Q=H*P*H'+R
		CommonOps.multTransB(sP,sH,mx_e); //Note:mx_e=PH'
		CommonOps.mult(sH,mx_e,mx_a);
        
        CommonOps.addEquals(mx_a,sR);
        
        //mx_b=inv(H*P*H'+R);
        CommonOps.invert(mx_a,mx_b);
        
        //K=P*H'*inv(H*P*H'+R);
        CommonOps.mult(mx_e,mx_b,K);
        
        //dx=K*dz;
        CommonOps.mult(K,sdz,sdx);
        		    	
    	//Updated Covariance
        CommonOps.mult(H,P,mx_f);	//mx_f=H*P
        CommonOps.mult(K,mx_f,A);	//A=KHP; A is temprorary variable
        CommonOps.subEquals(sP,A); //P=P-KHP
	}
}
