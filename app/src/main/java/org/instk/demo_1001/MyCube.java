package org.instk.demo_1001;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import android.opengl.GLSurfaceView.Renderer;


public class MyCube implements Renderer{
	//Camera Coordinates
	private float[] Pos_b=new float[3];
	private float[] Cbn=new float[16];
	
	//Cube Coordinates
	private final int sz=1;
	private final float[][] cCoords = new float[][] {
		new float[] {sz, sz,-sz, -sz, sz,-sz, -sz, sz, sz, sz, sz, sz}, // top
		new float[] {sz,-sz, sz, -sz,-sz, sz, -sz,-sz,-sz, sz,-sz,-sz}, // bottom
		new float[] {sz, sz, sz, -sz, sz, sz, -sz,-sz, sz, sz,-sz, sz}, // front
		new float[] {sz,-sz,-sz, -sz,-sz,-sz, -sz, sz,-sz, sz, sz,-sz}, // back
		new float[] {-sz, sz, sz, -sz, sz,-sz, -sz,-sz,-sz, -sz,-sz, sz},// left
		new float[] {sz, sz,-sz, sz, sz, sz, sz,-sz, sz, sz,-sz,-sz} // right
	};

	private final float[] cColors = new float[] {
		0,1,0,1,
		1,0.5f,0,1,
		1,0,0,1,
		1,1,0,1,
		0,0,1,1,
		1,0,1,1		
	};
		
	private FloatBuffer[] vertexBuffers;
	
	public void set_pos(float[] pos){
		Pos_b[0]=pos[0];
		Pos_b[1]=pos[1];
		Pos_b[2]=pos[2];
	}
	
	public void set_pos(double[] pos){
		Pos_b[0]=(float) pos[0];
		Pos_b[1]=(float) pos[1];
		Pos_b[2]=(float) pos[2];
	}
	
	public void set_dcm(float[] dcm){
		for (int i=0;i<3;i++)
    		for (int k=0;k<3;k++)
    			Cbn[(i*4)+k]=dcm[i*3+k];
    	Cbn[15]=1;
	}
	
	public void set_dcm(double[] dcm){
		for (int i=0;i<3;i++)
    		for (int k=0;k<3;k++)
    			Cbn[(i*4)+k]=(float) dcm[i*3+k];
    	Cbn[15]=1;
	}
	
    private static FloatBuffer makeDirectFloatBuffer(float[] vals) {
        FloatBuffer returnVal = null;
        int SIZE_OF_FLOAT = 4; // size of float in bytes
        ByteBuffer vbb = ByteBuffer.allocateDirect(vals.length * SIZE_OF_FLOAT);
        vbb.order(ByteOrder.nativeOrder());
        returnVal = vbb.asFloatBuffer();
        returnVal.put(vals);
        returnVal.position(0);
        return returnVal;
    }

	public MyCube(float[] pos, float[] dcm) {
        //Set initial camera coordinates
        set_pos(pos);
        set_dcm(dcm);

        initializeVertexBuf(cCoords);
	}

	private void initializeVertexBuf(float[][] cCoords) {
        vertexBuffers = new FloatBuffer[cCoords.length];
        for (int index = 0; index < vertexBuffers.length; index++) {
                vertexBuffers[index] = (makeDirectFloatBuffer(cCoords[index]));
        }
	}        
	
	//Main Shape draw routine
	public void draw(GL10 gl) {
		for (int i = 0; i < 6; i++)
		{
			gl.glEnableClientState(GL10.GL_COLOR_ARRAY);
			gl.glColor4f(cColors[4*i+0], cColors[4*i+1], cColors[4*i+2], cColors[4*i+3]);
			gl.glDisableClientState(GL10.GL_COLOR_ARRAY);
			
			gl.glEnableClientState(GL10.GL_VERTEX_ARRAY);
			gl.glVertexPointer(3, GL10.GL_FLOAT, 0, vertexBuffers[i]);
			gl.glDrawArrays(GL10.GL_TRIANGLE_FAN, 0, 4);
			gl.glDisableClientState(GL10.GL_VERTEX_ARRAY);
		}
	};
	
	//Renderer implementation
	public void onDrawFrame(GL10 gl) {
		//Clear the screen
        gl.glClear(GL10.GL_COLOR_BUFFER_BIT | GL10.GL_DEPTH_BUFFER_BIT);
        
        //Set camera position
        gl.glMatrixMode(GL10.GL_MODELVIEW);
        gl.glLoadIdentity();
        gl.glTranslatef(-Pos_b[0], -Pos_b[1], -Pos_b[2]);	//Position
        gl.glMultMatrixf(Cbn, 0);	//Orientation Note:Effectively Cnb!!!
        
        //Draw the cube
        this.draw(gl);
    }

    public void onSurfaceChanged(GL10 gl, int width, int height) {
        gl.glViewport(0, 0, width, height);
        float ratio = (float) width / height;
        gl.glMatrixMode(GL10.GL_PROJECTION);
        gl.glLoadIdentity();
        gl.glFrustumf(-ratio, ratio, -1, 1, 1, 10);
    }

    public void onSurfaceCreated(GL10 gl, EGLConfig config) {
        //improve performance
        gl.glDisable(GL10.GL_DITHER);
        gl.glDisable(GL10.GL_MULTISAMPLE);
        
        //Initialization (not sure why)
        gl.glClearColor(1,1,1,1);
        gl.glEnable(GL10.GL_CULL_FACE);
        gl.glShadeModel(GL10.GL_SMOOTH);
        gl.glEnable(GL10.GL_DEPTH_TEST);
    }

}