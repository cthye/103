using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

public class FVM : MonoBehaviour
{
	float dt 			= 0.003f;
    float mass 			= 1;
	float stiffness_0	= 20000.0f;
    float stiffness_1 	= 5000.0f;
    float damp			= 0.999f;
	Vector3 gravity = new Vector3(0, -9.8f, 0);
	
	int[] 		Tet;
	int tet_number;			//The number of tetrahedra

	Vector3[] 	Force;
	Vector3[] 	V;
	Vector3[] 	X;
	int number;				//The number of vertices

	Matrix4x4[] inv_Dm;

	//For Laplacian smoothing.
	Vector3[]   V_sum;
	int[]		V_num;

	SVD svd = new SVD();

	// floor plane and normal
	Vector3 floor_plane = new Vector3(0, -3.0f, 0);
	Vector3 floor_normal = new Vector3(0, 1, 0);
	float restitution 	= 0.5f;					// for collision, 反弹系数
	float frition = 0.5f; 						// for collision, 摩擦系数

    // Start is called before the first frame update
    void Start()
    {
    	// FILO IO: Read the house model from files.
    	// The model is from Jonathan Schewchuk's Stellar lib.
    	{
    		string fileContent = File.ReadAllText("Assets/house2.ele");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		
    		tet_number=int.Parse(Strings[0]);
        	Tet = new int[tet_number*4];

    		for(int tet=0; tet<tet_number; tet++)
    		{
				Tet[tet*4+0]=int.Parse(Strings[tet*5+4])-1;
				Tet[tet*4+1]=int.Parse(Strings[tet*5+5])-1;
				Tet[tet*4+2]=int.Parse(Strings[tet*5+6])-1;
				Tet[tet*4+3]=int.Parse(Strings[tet*5+7])-1;
			}
    	}
    	{
			string fileContent = File.ReadAllText("Assets/house2.node");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		number = int.Parse(Strings[0]);
    		X = new Vector3[number];
       		for(int i=0; i<number; i++)
       		{
       			X[i].x=float.Parse(Strings[i*5+5])*0.4f;
       			X[i].y=float.Parse(Strings[i*5+6])*0.4f;
       			X[i].z=float.Parse(Strings[i*5+7])*0.4f;
       		}
    		//Centralize the model.
	    	Vector3 center=Vector3.zero;
	    	for(int i=0; i<number; i++)		center+=X[i];
	    	center=center/number;
	    	for(int i=0; i<number; i++)
	    	{
	    		X[i]-=center;
	    		float temp=X[i].y;
	    		X[i].y=X[i].z;
	    		X[i].z=temp;
	    	}
		}
        /*
		tet_number=1;
        Tet = new int[tet_number*4];
        Tet[0]=0;
        Tet[1]=1;
        Tet[2]=2;
        Tet[3]=3;

        number=4;
        X = new Vector3[number];
        V = new Vector3[number];
        Force = new Vector3[number];
        X[0]= new Vector3(0, 0, 0);
        X[1]= new Vector3(1, 0, 0);
        X[2]= new Vector3(0, 1, 0);
        X[3]= new Vector3(0, 0, 1);
		*/


        //Create triangle mesh.
       	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];

        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }

        int[] triangles = new int[tet_number*12];
        for(int t=0; t<tet_number*4; t++)
        {
        	triangles[t*3+0]=t*3+0;
        	triangles[t*3+1]=t*3+1;
        	triangles[t*3+2]=t*3+2;
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.triangles = triangles;
		mesh.RecalculateNormals ();


		V 	  = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];

		//TODO: Need to allocate and assign inv_Dm
		inv_Dm = new Matrix4x4[tet_number];
		for (int t = 0; t < tet_number; t++) {
			inv_Dm[t] = Build_Edge_Matrix(t).inverse;
		}
    }

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
    	Matrix4x4 ret=Matrix4x4.zero;
    	//TODO: Need to build edge matrix here.
		Vector3 x0 = X[Tet[tet*4+0]];
		Vector3 x1 = X[Tet[tet*4+1]];
		Vector3 x2 = X[Tet[tet*4+2]];
		Vector3 x3 = X[Tet[tet*4+3]];

		// edge x10
		ret[0, 0] = x1.x - x0.x;
		ret[1, 0] = x1.y - x0.y; 
		ret[2, 0] = x1.z - x0.z; 

		// edge x20
		ret[0, 1] = x2.x - x0.x; 
		ret[1, 1] = x2.y - x0.y; 
		ret[2, 1] = x2.z - x0.z; 

		// edge x30
		ret[0, 2] = x3.x - x0.x; 
		ret[1, 2] = x3.y - x0.y; 
		ret[2, 2] = x3.z - x0.z; 

		ret[3, 3] = 1;

		return ret;
    }

    void _Update()
    {
    	// Jump up.
		if(Input.GetKeyDown(KeyCode.Space))
    	{
    		for(int i=0; i<number; i++)
    			V[i].y+=0.2f;
    	}

    	for(int i=0 ;i<number; i++)
    	{
    		//TODO: update Force with gravity
			Force[i] = gravity * mass;
    	}


    	for(int tet=0; tet<tet_number; tet++)
    	{
    		//TODO: Deformation Gradient
			Matrix4x4 deformedMatrix = Build_Edge_Matrix(tet);
			// Debug.Log("dm" + tet+ " " + deformedMatrix);

			Matrix4x4 F = deformedMatrix * inv_Dm[tet]; // deformation gradient
			// Debug.Log("F" + tet+ " " + F);

    		
    		//TODO: Green Strain

			Matrix4x4 G = Matrix_Multiply_With_Float(Matrix_Subtraction(F.transpose * F,  Matrix4x4.identity), 0.5f);
			// Debug.Log("G" + tet+ " " + G);


    		//TODO: Second PK Stress

			Matrix4x4 S = Matrix_Add(Matrix_Multiply_With_Float(G, 2.0f * stiffness_1), Matrix_Multiply_With_Float(Matrix4x4.identity, stiffness_0 * trace(G)));
			// Debug.Log("S" + tet+ " " + S);

			Matrix4x4 P = F * S;
			// Debug.Log("P" + tet+ " " + P);

    		//TODO: Elastic Force
			Matrix4x4 Forces = Matrix_Multiply_With_Float(P * inv_Dm[tet].transpose, - 1.0f / (6.0f * inv_Dm[tet].determinant)) ;
			// Debug.Log("Forces" + tet+ " " + Forces);
			
			Force[Tet[tet*4+1]] += (Vector3)Forces.GetColumn(0);
			Force[Tet[tet*4+2]] += (Vector3)Forces.GetColumn(1);
			Force[Tet[tet*4+3]] += (Vector3)Forces.GetColumn(2);
			Force[Tet[tet*4+0]] += - Force[Tet[tet*4+1]] - Force[Tet[tet*4+2]] - Force[Tet[tet*4+3]];
			// Debug.Log("update Force" + Tet[tet*4+0] + " " + Force[Tet[tet*4+0]]);
			// Debug.Log("update Force" + Tet[tet*4+1] + " " + Force[Tet[tet*4+1]]);
			// Debug.Log("update Force" + Tet[tet*4+2] + " " + Force[Tet[tet*4+2]]);
			// Debug.Log("update Force" + Tet[tet*4+3] + " " + Force[Tet[tet*4+3]]);

    	}

		Laplacian_Smooth();

    	for(int i=0; i<number; i++)
    	{
    		//TODO: Update X and V here.
			// Debug.Log("prev v" + i + " " + V[i]);
			V[i] += dt * Force[i] / mass;
			V[i] *= damp;

			X[i] += dt * V[i];

    		//TODO: (Particle) collision with floor.
				
			if ((Vector3.Dot((X[i] - floor_plane), floor_normal)) < 0 && Vector3.Dot(V[i], floor_normal) < 0) {
				Vector3 v_ni = Vector3.Dot(V[i], floor_normal) * floor_normal;
				Vector3 v_ti = V[i] - v_ni;
				// reduce oscillation
				restitution = Mathf.Max(restitution - 0.001f, 0); 
				float a = Mathf.Max(0, 1.0f - frition * (1 + restitution) * v_ni.magnitude / v_ti.magnitude);
				v_ni = -restitution * v_ni;
				v_ti = a * v_ti;
				V[i] = v_ni + v_ti;
				X[i] -= Vector3.Dot((X[i] - floor_plane), floor_normal) * floor_normal;
			}
			// Debug.Log("update v" + i + " " + V[i]);
			// Debug.Log("update x" + i + " " + X[i]);
    	}

    }

    // Update is called once per frame
    void Update()
    {
    	for(int l=0; l<10; l++)
    		 _Update();

    	// Dump the vertex array for rendering.
    	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.RecalculateNormals ();
    }

	float trace(Matrix4x4 mat) {
		return mat[0, 0] + mat[1, 1] + mat[2, 2];
	}

	void Laplacian_Smooth() {
		for (int i = 0; i < number; i++){
            V_sum[i] = new Vector3(0, 0, 0);
            V_num[i] = 0;
		}
		
		for (int tet = 0; tet < tet_number; tet++) {
			Vector3 sum = V[Tet[tet*4+0]] + V[Tet[tet*4+1]] + V[Tet[tet*4+2]] + V[Tet[tet*4+3]];
			V_sum[Tet[tet*4+0]] += sum;
			V_sum[Tet[tet*4+1]] += sum;
			V_sum[Tet[tet*4+2]] += sum;
			V_sum[Tet[tet*4+3]] += sum;

			V_num[Tet[tet*4+0]] += 4;
			V_num[Tet[tet*4+1]] += 4;
			V_num[Tet[tet*4+2]] += 4;
			V_num[Tet[tet*4+3]] += 4;
		}

		for (int i = 0; i < number; i++) {
			V[i] = V[i] * 0.9f + V_sum[i] / V_num[i] * 0.1f;
			// Debug.Log("update laplacian v" + i + " " + V[i]);
		}
	}

	Matrix4x4 Matrix_Subtraction(Matrix4x4 a, Matrix4x4 b) {
		Matrix4x4 ret = Matrix4x4.zero;
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				ret[i, j] = a[i, j] - b[i, j];
			}
		}
		return ret;
	}

	Matrix4x4 Matrix_Multiply_With_Float(Matrix4x4 a, float f) {
		Matrix4x4 ret = Matrix4x4.zero;
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				ret[i, j] = a[i, j] * f;
			}
		}
		return ret;
	}

	Matrix4x4 Matrix_Add(Matrix4x4 a, Matrix4x4 b) {
		Matrix4x4 ret = Matrix4x4.zero;
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				ret[i, j] = a[i, j] + b[i, j];
			}
		}
		return ret;
	}
}