#include "setting.h"
#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

using namespace mto;

float target_x_2=0,target_y_2=0;

bool reverse2=false;

Obstacle mto::square(float start_x,float start_y,float end_x,float end_y,float width,float height){

    float x=current.x,y=current.y;

    float theta=atan(height/width);

    std::vector<float> vector_obstacle={(end_x-start_x)/hypotf(end_x-start_x, end_y-start_y),(end_y-start_y)/hypotf(end_x-start_x, end_y-start_y)};//BD向量

    std::vector<float> A={start_x+float((vector_obstacle[0]*std::cos(M_PI/2.0-theta)-vector_obstacle[1]*std::sin(M_PI/2.0-theta))*height),start_y+float((vector_obstacle[0]*std::sin(M_PI/2.0-theta)+vector_obstacle[1]*std::cos(M_PI/2.0-theta))*height)};

    std::vector<float> B={start_x,start_y};

    std::vector<float> C={start_x+((vector_obstacle[0]*std::cos(theta)+vector_obstacle[1]*std::cos(theta))*width),start_y+((vector_obstacle[0]*-std::sin(theta)+vector_obstacle[1]*std::cos(theta))*width)};

    std::vector<float> D={end_x,end_y};

    std::vector<float> M={(start_x+end_x)/2,(start_y+end_y)/2};//障礙物中心點

    std::vector<float> p_dir={target_x_2-x/hypotf(target_x_2, target_y_2),target_y_2-y/hypotf(target_x_2, target_y_2)};//現在車子到終點向量

    std::vector<float> BA={A[0]-B[0],A[1]-B[1]},CB={B[0]-C[0],B[1]-C[1]},DC={C[0]-D[0],C[1]-D[1]},AD={D[0]-A[0],D[1]-A[1]};

    std::vector<float> BA_N={(-A[1]+B[1])/hypotf(BA[0],BA[1]),(A[0]-B[0])/hypotf(BA[0],BA[1])};//BA法向量(單位向量)

    std::vector<float> CB_N={(-B[1]+C[1])/hypotf(CB[0],CB[1]),(B[0]-C[0])/hypotf(CB[0],CB[1])};//CB法向量(單位向量)

    std::vector<float> DC_N={(-C[1]+D[1])/hypotf(DC[0],DC[1]),(C[0]-D[0])/hypotf(DC[0],DC[1])};//DC法向量(單位向量)

    std::vector<float> AD_N={(-D[1]+A[1])/hypotf(AD[0],AD[1]),(D[0]-A[0])/hypotf(AD[0],AD[1])};//AD法向量(單位向量)

    std::vector<float> V={x-start_x,y-start_y};//障礙物開始點到現在車子點的向量

    float 
    dotBA= (V[0]*BA_N[0]+V[1]*BA_N[1]),
    dotCB= (V[0]*CB_N[0]+V[1]*CB_N[1]),
    dotDC= (V[0]*DC_N[0]+V[1]*DC_N[1]),
    dotAD= (V[0]*AD_N[0]+V[1]*AD_N[1]);

    float length = std::sqrt(pow(dotBA*(dotBA>0),2)+pow(dotCB*(dotCB>0),2)+pow((dotDC-width)*((dotDC-width)>0),2)+pow((dotAD-height)*((dotAD-height)>0),2));

    float
    dirBA=p_dir[0]*BA[0]+(p_dir[1]*BA[1]),
    dirCB=p_dir[0]*CB[0]+(p_dir[1]*CB[1]),
    dirDC=p_dir[0]*DC[0]+(p_dir[1]*DC[1]),
    dirAD=p_dir[0]*AD[0]+(p_dir[1]*AD[1]);

    float angle=std::atan2(M[0]-x,M[1]-y)*180.0/M_PI-heading2(reverse2);

    int direction = -1+(2*(((dotBA>0)*dirBA)+((dotCB>0)*dirCB)+((dotDC>0)*dirDC)+((dotAD>0)*dirAD)>0));

    return Obstacle(length*(length<(motor().max_width+10)),direction*(90-fabs(angle))*(length<(motor().max_width+10))*((90-fabs(angle)>0)),angle,length);


}

Obstacle mto::circle (float center_x,float center_y,float diameter){

    float x=current.x,y=current.y;

    std::vector<float> p_direction={target_x_2-x/hypotf(target_x_2 ,target_y_2),target_y_2-y/hypotf(target_x_2 ,target_y_2)};

    std::vector<float> p2center={center_x-x,center_y-y};

    float length=hypotf(p2center[0],p2center[1])-(diameter/2.0);

    float direction=(-1+(2*((p_direction[0]*-p2center[1]+(p_direction[1]*p2center[0]))>0)));

    float angle=std::atan2(p2center[0],p2center[1])*180.0/M_PI-heading2(reverse2);

    return Obstacle(length*(length<(motor().max_width+10)),direction*(90-fabs(angle))*(length<(motor().max_width+10))*((90-fabs(angle)>0)),angle,length);

}

Obstacle mto::sum(const Obstacle& accum,const Obstacle& s){

    return Obstacle(accum.error+s.error,accum.error_angle+s.error_angle,accum.angle+s.angle,accum.length+s.length);

}