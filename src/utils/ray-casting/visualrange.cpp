#include "mex.h"
#include "math.h"
#define NULL 0
#define INFISM 0.0000001

void crossPro(double a[3], double b[3], double c[3])
{
	c[0] = a[1]*b[2] - a[2]*b[1];
	c[1] = a[2]*b[0] - a[0]*b[2];
	c[2] = a[0]*b[1] - a[1]*b[0];
}
double dotPro(double a[3], double b[3])
{
	double c = 0;
	for(int ii = 0; ii < 3; ii++)
	{
		c += a[ii]*b[ii];
	}
	return c;
}
int intersLineTria(double stp[3], double endp[3], double v0[3], double v1[3], double v2[3])
{
	double e1[3], e2[3], dir[3], det = 0;
	for(int ii = 0; ii < 3; ii++)
	{
		e1[ii] = v1[ii] - v0[ii];
		e2[ii] = v2[ii] - v0[ii];
        dir[ii] = endp[ii] - stp[ii];
	}
	double p[3];
    crossPro(dir, e2, p);
	det = dotPro(e1, p);
	double T[3];
	if(det > 0)
	{
	for(int ii = 0; ii<3; ii++){T[ii] = stp[ii] - v0[ii];}
	}
	else{for(int ii =0; ii<3; ii++){T[ii] = v0[ii] - stp[ii];det = -det;}}
	double u = dotPro(T, p);
	if(u < 0 || u > det) return 0;
	double Q[3];
    crossPro(T, e1, Q);
	double v = dotPro(dir, Q);
	if(v < 0 ||u + v > det) return 0;
	double t = dotPro(e2, Q);
	if(t < INFISM)
		return 0;
    else if(t > det*(1-INFISM))
        return 0;
	return 1;
}

void mexFunction(int nlhs, mxArray*plhs[],int nrhs, const mxArray *prhs[])
{
// get the pointers of the input arguments
	
	double *endo_coord = mxGetPr(prhs[0]);
	double *vertices = mxGetPr(prhs[1]);
	double *seen_ind = mxGetPr(prhs[2]);
	double *trianglesTemp = mxGetPr(prhs[3]);
    int numColumnvert = mxGetN(prhs[2]);
    int numColumntris = mxGetN(prhs[3]);
    int* triangles = new int[3*numColumntris];
    for (int ii=0;ii<3*numColumntris;ii++)
        triangles[ii] = (int) trianglesTemp[ii];
    int intersect = 0;
	double vertex[3], v0[3], v1[3], v2[3];

	for(int ii = 0; ii < numColumnvert; ii++)
	{
        intersect = 1;
		if(fabs(seen_ind[ii]-1) < INFISM)
		{
			intersect = 0;
            vertex[0] = vertices[3*ii];
			vertex[1] = vertices[3*ii+1];
			vertex[2] = vertices[3*ii+2];// not sure whether indexing of the matrix elements is correct
			for(int jj = 0;jj < numColumntris;jj++)
			{
				for(int kk = 0;kk < 3;kk++)
				{
					v0[kk] = vertices[3*triangles[3*jj] + kk];
					v1[kk] = vertices[3*triangles[3*jj+1] + kk];
					v2[kk] = vertices[3*triangles[3*jj+2] + kk];
				}
				intersect = intersLineTria(endo_coord, vertex, v0, v1, v2);
 				if(abs(intersect-1) < INFISM)
 					break;
			}
		}
		seen_ind[ii] = (double)!intersect;
	}
    delete []triangles;
	plhs[0] = mxCreateDoubleMatrix(1,numColumnvert,mxREAL);
	double *output = mxGetPr(plhs[0]);
	for(int ii = 0;ii < numColumnvert;ii++)
			output[ii] = (double) seen_ind[ii];
}
