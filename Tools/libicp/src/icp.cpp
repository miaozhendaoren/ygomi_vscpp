/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

Authors: Andreas Geiger

libicp is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libicp is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libicp; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

#include "icp.h"

using namespace std;

Icp::Icp (double *M,const int32_t M_num,const int32_t dim) :
  dim(dim), max_iter(200), min_delta(1e-4) {
  
  // check for correct dimensionality
  if (dim!=2 && dim!=3) {
    //cout << "ERROR: LIBICP works only for data of dimensionality 2 or 3" << endl;
    M_tree = 0;
    return;
  }
  
  // check for minimum number of points
  if (M_num<MIN_NUM_POINT) {
    //cout << "ERROR: LIBICP works only with at least 5 model points" << endl;
    M_tree = 0;
    return;
  }

  // copy model points to M_data
  M_data.resize(boost::extents[M_num][dim]);
  for (int32_t m=0; m<M_num; m++)
    for (int32_t n=0; n<dim; n++)
      M_data[m][n] = (float)M[m*dim+n];

  // build a kd tree from the model point cloud
  M_tree = new kdtree::KDTree(M_data);
}

Icp::~Icp () {
  if (M_tree)
    delete M_tree;
}

void Icp::fit (double *T,const int32_t T_num,Matrix &R,Matrix &t,const double indist,
	          std::vector<int32_t> &active,std::vector<int32_t> &matchIdx, std::vector<int32_t> &closeIdx, double *minDist, double *meanDist) {
  
  // make sure we have a model tree
  if (!M_tree) {
    cout << "ERROR: No model available." << endl;
    return;
  }
  
  // check for minimum number of points
  if (T_num<MIN_NUM_POINT) {
    //cout << "ERROR: Icp works only with at least 5 template points" << endl;
    *minDist = 10000000;
    *meanDist = 10000000;
    return;
  }
  
  // set active points
  active.clear();
  matchIdx.clear();
  closeIdx.clear();

  if (indist<=0) {
    for (int32_t i=0; i<T_num; i++)
      active.push_back(i);
  } else {
    active = getInliers(T,T_num,R,t,indist);
  }
  
  // run icp
  fitIterate(T,T_num,R,t,active,matchIdx,closeIdx,minDist,meanDist);
}

void Icp::fitIterate(double *T,const int32_t T_num,Matrix &R,Matrix &t,const std::vector<int32_t> &active,std::vector<int32_t> &matchIdx, std::vector<int32_t> &closeIdx, double *minDist, double *meanDist) {
  
  // check if we have at least 5 active points
  if (active.size()<MIN_NUM_POINT) {
  	//cout << "ERROR: Icp works only with at least 5 active points." << endl;
    *minDist = 10000000;
    *meanDist = 10000000;
    return;
  }
  
  // iterate until convergence
  for (int32_t iter=0; iter<max_iter; iter++)
  {
    if (fitStep(T,T_num,R,t,active)<min_delta)
    {
      break;
    }
  }

  matchPoints(T,T_num,R,t,active,matchIdx, closeIdx, minDist, meanDist);
}

void Icp::matchPoints (double *T,const int32_t T_num,Matrix &R,Matrix &t,const std::vector<int32_t> &active, std::vector<int32_t> &matchIdx, std::vector<int32_t> &closeIdx, double *minDist, double *meanDist) {
  
  int i;
  int nact = (int)active.size();
  matchIdx.clear();
  std::vector<float> matchDist;
  float minDistLoc = 10000000;
  *minDist = 10000000;
  *meanDist = 10000000;

  // dimensionality 2
  if (dim==2) {
    
    // extract matrix and translation vector
    double r00 = R.val[0][0]; double r01 = R.val[0][1];
    double r10 = R.val[1][0]; double r11 = R.val[1][1];
    double t0  = t.val[0][0]; double t1  = t.val[1][0];
    double mum0 = 0.0, mum1 = 0.0;
    double mut0 = 0.0, mut1 = 0.0;

    // establish correspondences
#pragma omp parallel for private(i) default(none) shared(T,active,nact,p_m,p_t,r00,r01,r10,r11,t0,t1) reduction(+:mum0,mum1, mut0,mut1) // schedule (dynamic,2)
    for (i=0; i<nact; i++) {
      // kd tree query + result
      std::vector<float>         query(dim);
      kdtree::KDTreeResultVector result;
  
      // get index of active point
      int32_t idx = active[i];

      // transform point according to R|t
      query[0] = (float)(r00*T[idx*2+0] + r01*T[idx*2+1] + t0);
      query[1] = (float)(r10*T[idx*2+0] + r11*T[idx*2+1] + t1);

      // search nearest neighbor
      M_tree->n_nearest(query,1,result);

      // check if match or not
      int32_t resultIdx = result[0].idx;
	  float   resultDis = result[0].dis;
	  bool isMinDist = true;
      for (int outIdx = 0; outIdx < matchIdx.size(); outIdx ++) {
      	if (resultIdx == matchIdx[outIdx]) {
		  if (resultDis < matchDist[outIdx]) {
			matchIdx[outIdx] = -1;
			break;
		  } else {
			isMinDist = false;
			break;
		  }
      	}
      }

	  if (isMinDist) {
	    matchIdx.push_back(resultIdx);
	  }else {
	    matchIdx.push_back(-1);
      }

      closeIdx.push_back(resultIdx);

	  matchDist.push_back(resultDis);

	  // Find minimun distance for output
	  if (minDistLoc > resultDis) {
		minDistLoc = resultDis;
  	  }

	  *minDist = (double) minDistLoc;
    }

    // Find mean distance of closest points
    int numDist = 0;
    float sumDist = 0.0;
    for (int outIdx = 0; outIdx < matchIdx.size(); outIdx ++) {
      if (-1 != matchIdx[outIdx]) {
		sumDist += matchDist[outIdx];
        ++numDist;
      }
    }
    if (0 == numDist){
        *meanDist = 1000000;
    }else{
        *meanDist = (double) sumDist / numDist;
    }
    
  // dimensionality 3
  } else {
	cout << "ERROR: Do not support dim = 3." << endl;
  }
  
}

