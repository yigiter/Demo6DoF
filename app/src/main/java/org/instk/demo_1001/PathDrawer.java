//This class can be used to draw 3d trajectory.
//To do so, MyCube should be switched with this one and content of the view should be set to this class.
//Also, some complimentary changes are required in the code (note this class needs pon_n not pos_b)

//Currently this class is not used at all.

/**********************************************
 * 
 * Author: Yigiter Yuksel
 * 
 * 01.2010
 * instk.org
 * 
 */

package org.instk.demo_1001;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import android.opengl.GLSurfaceView.Renderer;


public class PathDrawer implements Renderer{
	//Camera Coordinates
	private final float[] cam_pos={0,0,3};
	
	//Buffer for position coords
	private FloatBuffer VertexBuf;
	private final int MAXVER=1500;	//Maximum number of vertexes
	int nVer=1;		//Number of existing vertex
	
	public void add_vertex(float[] coord){
		if (nVer<MAXVER) {
			VertexBuf.position(nVer*3);
			VertexBuf.put(coord);
			VertexBuf.position(0);
			nVer++;
		}
		else {
			VertexBuf.position(0);
			VertexBuf.put(coord);
			VertexBuf.position(0);
			nVer=1;
		}
	}
	
	public void reset() {
		VertexBuf.position(0);
		VertexBuf.put(0);
    	VertexBuf.put(0);
    	VertexBuf.put(0);
    	VertexBuf.position(0);
    	nVer=1;
	}
	
	public PathDrawer() {
		ByteBuffer vbb = ByteBuffer.allocateDirect(MAXVER * 3 * 4);
    	vbb.order(ByteOrder.nativeOrder());
    	VertexBuf = vbb.asFloatBuffer();
    	
    	//Assure that there is always a vertex at origin
    	VertexBuf.put(0);
    	VertexBuf.put(0);
    	VertexBuf.put(0);
    	VertexBuf.position(0);
	}
	
	//Main Shape draw routine
	public void draw(GL10 gl) {
		gl.glEnableClientState(GL10.GL_VERTEX_ARRAY);
		gl.glVertexPointer(3, GL10.GL_FLOAT, 0, VertexBuf);
		gl.glDrawArrays(GL10.GL_LINE_STRIP, 0, nVer);
		gl.glDisableClientState(GL10.GL_VERTEX_ARRAY);
	};
	
	//Renderer implementation
	public void onDrawFrame(GL10 gl) {
		//Clear the screen
        gl.glClear(GL10.GL_COLOR_BUFFER_BIT | GL10.GL_DEPTH_BUFFER_BIT);
        
        //Set camera position
        gl.glMatrixMode(GL10.GL_MODELVIEW);
        gl.glLoadIdentity();
        gl.glTranslatef(-cam_pos[0], -cam_pos[1], -cam_pos[2]);	//Position
        
        //Draw the cube
        this.draw(gl);
    }

    public void onSurfaceChanged(GL10 gl, int width, int height) {
        gl.glViewport(0, 0, width, height);
        float ratio = (float) width / height;
        gl.glMatrixMode(GL10.GL_PROJECTION);
        gl.glLoadIdentity();
        gl.glFrustumf(-ratio, ratio, -1, 1, 1, 3);
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
        
        //Line Color
        gl.glEnableClientState(GL10.GL_COLOR_ARRAY);
		gl.glColor4f(0, 0, 0, 1); //Black
		gl.glDisableClientState(GL10.GL_COLOR_ARRAY);
		gl.glLineWidthx(20);
    }

}
