using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;					// for collision, 反弹系数
	float frition = 0.5f; 						// for collision, 摩擦系数


	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices; // local coordinate

		float m=1;
		mass=0;
		//I = ∑ m_i (r_i^2 δ_ij - r_i r_j)？ sigma is 1 when i==j or 0 otherwises
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		// (Rri) x => (Rri) *
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	Quaternion  QuaternionAddition (Quaternion  a, Quaternion  b) {
		//(a + i b + j c + k d)+(e + i f + j g + k h) = (a+e) + i ( b+f) + j ( c+g) + k ( d+h)
		return new Quaternion (a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
	}

	Matrix4x4 Matrix4x4Substraction(Matrix4x4 a, Matrix4x4 b) {
		Matrix4x4 res = new Matrix4x4();
		for (int r = 0; r < 4; r++) {
			for (int c = 0; c < 4; c++) {
				res[r, c] = a[r, c] - b[r, c];
			}
		}
		return res;
	}

	// In this function, update v and w by the impulse due to the collision with
	// a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		Vector3 collision_point = new Vector3(0, 0, 0);
		int num_collision = 0;
		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		// !important, otherwises it would be super slow to refer to mesh every time
		Vector3[] vertices = mesh.vertices; 
		for (int i = 0; i < vertices.Length; i++) {
			Vector3 xi = transform.position + R.MultiplyVector(vertices[i]);
			if ((Vector3.Dot((xi - P), N)) < 0) {
				collision_point += vertices[i];
				num_collision ++;
			}
		}
		Debug.Log("num collide" + num_collision);
		if(num_collision == 0) return;
		Vector3 ri = collision_point / num_collision;
		Vector3 Rri = R.MultiplyVector(ri);
		Vector3 vi = v + Vector3.Cross(w, Rri);

		if(Vector3.Dot(vi, N) >= 0) return;
		//compute the wanted vi_new
		Vector3 v_ni = Vector3.Dot(vi, N) * N;
		Vector3 v_ti = vi - v_ni;
		// reduce oscillation
		restitution = Mathf.Max(restitution - 0.001f, 0); 
		float a = Mathf.Max(0, 1.0f - frition * (1 + restitution) * v_ni.magnitude / v_ti.magnitude);
		v_ni = -restitution * v_ni;
		v_ti = a * v_ti;
		Vector3 vi_new = v_ni + v_ti;

		//compute the impulse
		Matrix4x4 I = R * I_ref * R.transpose;
		Matrix4x4 MRri = Get_Cross_Matrix(Rri);
		Matrix4x4 mass_inverse = new Matrix4x4();
		mass_inverse[0, 0] = 1.0f / mass;
		mass_inverse[1, 1] = 1.0f / mass;
		mass_inverse[2, 2] = 1.0f / mass;
		mass_inverse[3, 3] = 1.0f / mass;
		Matrix4x4 K = Matrix4x4Substraction(mass_inverse, MRri * I.inverse * MRri);
		Vector3 j = K.inverse.MultiplyVector(vi_new - vi);

		//update the v and w
		v = v + 1.0f / mass * j;
		w = w + (I.inverse).MultiplyVector(Vector3.Cross(Rri, j));
		Debug.Log("Collision v" + v);
		Debug.Log("Collision w" + w);

	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			launched=true;
		}

		if(launched) {
			// Part I: Update velocities
			//* update translation velocity
			float g = 9.8f;
			// 4*4 matrix muliplied with Vector3, convert the Vector3 into Vector4 first (w=0)
			// implicit Vector4 convert to Vector3, omitting the w
			Vector3 fa = new Vector3 (0.0f, - g, 0.0f);
			Vector3 dv = dt * fa;
			v = linear_decay * (v + dv); 

			//* update rotation velocity
			w *= angular_decay;

			// Part II: Collision Impulse
			Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0)); //ground
			Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0)); // backwall

			// Part III: Update position & orientation
			//Update linear status
			Vector3 x    = transform.position;
			//Update angular status
			Quaternion q = transform.rotation;
			
			//implicit euler
			x = x + dt * v; 
			// Debug.Log(v);
			// derive the formular q' = 1/2 w dt + q 
			// https://gamedev.stackexchange.com/questions/108920/applying-angular-velocity-to-quaternion
			Quaternion det_q = new Quaternion(0.5f * dt * w.x, 0.5f * dt * w.y, 0.5f * dt * w.z, 0.0f);
			q = QuaternionAddition(q, det_q * q);
			q = q.normalized;

			// Part IV: Assign to the object
			transform.position = x;
			transform.rotation = q;
		}
	}
}