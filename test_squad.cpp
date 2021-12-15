#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/impl/point_types.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

#include "./src/Quaternion.h"
#include "./src/Bezier.h"
#include "./src/Spline.h"
#include "./src/Slerp.h"

using namespace std;

/* 
    Proposition 12: 任意的单位四元数都可以写成 [cos(theta), sin(theta)*v'] 的形式
    Definition 14: 单位四元数 q = [cos(theta), sin(theta)*v'], log(q) ≡ [0，theta*v]
    Definition 15: 单位四元数 q = [0，theta*v], exp(q) ≡ [cos(theta), sin(theta)*v']
    Corollary 4: 任何围绕单位向量 n 旋转 theta 角度的旋转都可以通过一个四元数来表示，
                 这个四元数的表示方式为：q = [cos(theta/2), sin(theta/2)*n]
 */
Eigen::Quaterniond expq(const Eigen::Vector4d& q)
{
    Eigen::Quaterniond res(cos(q(0)), sin(q(0))*q(1), sin(q(0))*q(2), sin(q(0))*q(3));
    return res; 
}

Eigen::Vector4d logq(const Eigen::Quaterniond& q)
{
    Eigen::AngleAxisd aa(q.toRotationMatrix());
    Eigen::Vector4d res(aa.angle()/2.0, aa.axis()(0), aa.axis()(1), aa.axis()(2));
    return res; 
}

int main() {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    viewer->addCoordinateSystem(100.0);
    Eigen::Quaterniond q1(1,  0, 0,  0.0);
    // Eigen::Quaterniond q1(0,  0.6, 0.8,  0.0);
    Eigen::Quaterniond q2(0,  0.0, 0.6,  0.8);
    Eigen::Quaterniond q3(0,  0.6, 0.0,  0.8);
    Eigen::Quaterniond q4(0, -0.8, 0.0, -0.6);
    Eigen::Quaterniond s1 = q1*expq(-(logq(q1.inverse()*q2)+logq(q1.inverse()*q1)/4));
    Eigen::Quaterniond s2 = q2*expq(-(logq(q2.inverse()*q3)+logq(q2.inverse()*q1)/4));
    Eigen::Quaterniond s3 = q3*expq(-(logq(q3.inverse()*q4)+logq(q3.inverse()*q2)/4));
    Eigen::Quaterniond s4 = q4*expq(-(logq(q4.inverse()*q4)+logq(q4.inverse()*q3)/4));
    // std::cout << s1.inverse().coeffs().transpose() << std::endl;
    // std::cout << s2.inverse().coeffs().transpose() << std::endl;

    // std::cout << q1.inverse().coeffs().transpose() << std::endl;
    std::vector<Eigen::Quaterniond> q_vec;
    for ( double i = 0; i <= 1; i = i + 0.01 ) {
        q_vec.push_back(q1.slerp(i,q2).slerp(2*i*(1-i), s1.slerp(i,s2)));
    }
    std::vector<Eigen::Quaterniond> q_vec2;
    for ( double i = 0; i <= 1; i = i + 0.01 ) {
        q_vec2.push_back(q2.slerp(i,q3).slerp(2*i*(1-i), s2.slerp(i,s3)));
    }
    std::vector<Eigen::Quaterniond> q_vec3;
    for ( double i = 0; i <= 1; i = i + 0.01 ) {
        q_vec3.push_back(q3.slerp(i,q4).slerp(2*i*(1-i), s3.slerp(i,s4)));
    }
    // for ( int i = 0; i < q_vec.size(); i++ ) {
    //     std::cout << q_vec[i].coeffs().transpose() << std::endl;
    // }
/*  // Calculate speed
    std::vector<Eigen::Quaterniond> q_vec_all;
    q_vec_all.insert(q_vec_all.end(),q_vec.begin(),q_vec.end());
    q_vec_all.insert(q_vec_all.end(),q_vec2.begin(),q_vec2.end());
    q_vec_all.insert(q_vec_all.end(),q_vec3.begin(),q_vec3.end());
    std::vector<double> diff;
    for (int i = 0; i < (q_vec_all.size() - 2); i++) {
        diff.push_back(((q_vec_all[i+1].coeffs()-q_vec_all[i].coeffs()).norm()+(q_vec_all[i+1].coeffs()-q_vec_all[i+2].coeffs()).norm())/2);
    }
    for (int i = 0; i < diff.size(); i++) {
        std::cout << diff[i] << std::endl;
    }
 */
    Eigen::Vector3d * vs = new Eigen::Vector3d[4];
    vs[0] = Eigen::Vector3d(0,0,0);
    vs[1] = Eigen::Vector3d(200,0,0);
    vs[2] = Eigen::Vector3d(200,200,200);
    vs[3] = Eigen::Vector3d(0,0,200);
    // for (int i = 0; i < 4; i++) {
    //     Eigen::Affine3d mat = Eigen::Translation3d(vs[i])*Eigen::Quaterniond(qs[i].real(),qs[i].imag_i(),qs[i].imag_j(),qs[i].imag_k());
    //     viewer->addCoordinateSystem(50, mat.cast<float>());
    // }
    
    // vector<Eigen::Vector3d> pos;
    // for (int j = 0; j < 3; j++) {
    //     for (int i = 0; i <10; i++) {
    //         pos.push_back(vs[j+1]-vs[j]);
    //     }
    // }
    uint size = q_vec.size();
    // std::cout << q_vec.size() << std::endl;
    for ( int i = 0; i < size; i++ ) {
        Eigen::Affine3d mat = Eigen::Translation3d(vs[0]+((double)i/(double)size)*(vs[1]-vs[0]))*Eigen::Quaterniond(q_vec[i].coeffs()[3],q_vec[i].coeffs()[0],q_vec[i].coeffs()[1],q_vec[i].coeffs()[2]);
        // pcl::PointXYZ p(100*q_vec(i).imag_i(), 100*q_vec(i).imag_j(), 100*q_vec(i).imag_k());
        // viewer->addSphere(p, 5, to_string(i));
        viewer->addCoordinateSystem(20, mat.cast<float>());
    }
    for ( int i = 0; i < size; i++ ) {
        Eigen::Affine3d mat = Eigen::Translation3d(vs[1]+((double)i/(double)size)*(vs[2]-vs[1]))*Eigen::Quaterniond(q_vec2[i].coeffs()[3],q_vec2[i].coeffs()[0],q_vec2[i].coeffs()[1],q_vec2[i].coeffs()[2]);
        // pcl::PointXYZ p(100*q_vec(i).imag_i(), 100*q_vec(i).imag_j(), 100*q_vec(i).imag_k());
        // viewer->addSphere(p, 5, to_string(i));
        viewer->addCoordinateSystem(20, mat.cast<float>());
    }
    for ( int i = 0; i < size; i++ ) {
        Eigen::Affine3d mat = Eigen::Translation3d(vs[2]+((double)i/(double)size)*(vs[3]-vs[2]))*Eigen::Quaterniond(q_vec3[i].coeffs()[3],q_vec3[i].coeffs()[0],q_vec3[i].coeffs()[1],q_vec3[i].coeffs()[2]);
        // pcl::PointXYZ p(100*q_vec(i).imag_i(), 100*q_vec(i).imag_j(), 100*q_vec(i).imag_k());
        // viewer->addSphere(p, 5, to_string(i));
        viewer->addCoordinateSystem(20, mat.cast<float>());
    }
    
    // Eigen::Vector3d * vs = new Eigen::Vector3d[4];
    // vs[0] = Eigen::Vector3d(0,0,0);
    // vs[1] = Eigen::Vector3d(200,0,0);
    // vs[2] = Eigen::Vector3d(200,200,200);
    // vs[3] = Eigen::Vector3d(0,0,200);
    // for (int i = 0; i < 4; i++) {
    //     Eigen::Affine3d mat = Eigen::Translation3d(vs[i])*Eigen::Quaterniond(qs[i].real(),qs[i].imag_i(),qs[i].imag_j(),qs[i].imag_k());
    //     viewer->addCoordinateSystem(50, mat.cast<float>());
    // }
    
    // vector<Eigen::Vector3d> pos;
    // for (int j = 0; j < 3; j++) {
    //     for (int i = 0; i <10; i++) {
    //         pos.push_back(vs[j+1]-vs[j]);
    //     }
    // }
    // //     cout << pos.size() << endl;
    // // for ( int i = 0; i < pos.size(); i++ ) {
    // //     cout << pos[i] << endl;
    // // }
    
    // Spline<Quaternion<double>, double> spln( qs, 4 ); 
    // // Bezier<Quaternion<double>, double> bex( qs, 4 );  // TODO: segmentation fault (core dumped)

    // // for ( int i = 0; i <= 30; ++i ) {
    // //         cout << i << '\t' << spln.value( 0.1*i ) << endl;
    // // }

    // for ( int i = 0; i < 10; ++i ) {
    //     // if(i == 0 or i == 10 or i == 20 or i == 30){
    //     //     Eigen::Affine3d mat = Eigen::Translation3d(vs[0]+0.1*i*pos[i])*Eigen::Quaterniond(spln.value(0.1*i).real(),spln.value(0.1*i).imag_i(),spln.value(0.1*i).imag_j(),spln.value(0.1*i).imag_k());
    //     //     // mat = mat.translation(0.1*i*pos[i]);
    //     //     viewer->addCoordinateSystem(50, mat.cast<float>());
    //     // } else {
    //         pcl::PointXYZ p(100*spln.value(0.1*i).imag_i(),100*spln.value(0.1*i).imag_j(),100*spln.value(0.1*i).imag_k());
    //         viewer->addSphere(p, 5, to_string(i));
    //         Eigen::Affine3d mat = Eigen::Translation3d(vs[0]+0.1*i*pos[i])*Eigen::Quaterniond(spln.value(0.1*i).real(),spln.value(0.1*i).imag_i(),spln.value(0.1*i).imag_j(),spln.value(0.1*i).imag_k());
    //         viewer->addCoordinateSystem(20, mat.cast<float>());
    //     // }
    //     // cout << 0.1*i << '\t' << spln.value( 0.1*i ) << endl;
    // }
    // for ( int i = 10; i < 20; ++i ) {
    //     // if(i == 0 or i == 10 or i == 20 or i == 30){
    //     //     Eigen::Affine3d mat = Eigen::Translation3d(vs[1]+0.1*(i-10)*pos[i])*Eigen::Quaterniond(spln.value(0.1*i).real(),spln.value(0.1*i).imag_i(),spln.value(0.1*i).imag_j(),spln.value(0.1*i).imag_k());
    //     //     // mat = mat.translation(0.1*i*pos[i]);
    //     //     viewer->addCoordinateSystem(50, mat.cast<float>());
    //     // } else {
    //         Eigen::Affine3d mat = Eigen::Translation3d(vs[1]+0.1*(i-10)*pos[i])*Eigen::Quaterniond(spln.value(0.1*i).real(),spln.value(0.1*i).imag_i(),spln.value(0.1*i).imag_j(),spln.value(0.1*i).imag_k());
    //         pcl::PointXYZ p(100*spln.value(0.1*i).imag_i(),100*spln.value(0.1*i).imag_j(),100*spln.value(0.1*i).imag_k());
    //         viewer->addSphere(p, 5, to_string(i));
    //         viewer->addCoordinateSystem(20, mat.cast<float>());
    //     // }
    //     // cout << 0.1*i << '\t' << spln.value( 0.1*i ) << endl;
    // }

    // for ( int i = 20; i < 30; ++i ) {
    //     // if(i == 0 or i == 10 or i == 20 or i == 30){
    //     //     Eigen::Affine3d mat = Eigen::Translation3d(vs[2]+0.1*(i-20)*pos[i])*Eigen::Quaterniond(spln.value(0.1*i).real(),spln.value(0.1*i).imag_i(),spln.value(0.1*i).imag_j(),spln.value(0.1*i).imag_k());
    //     //     // mat = mat.translation(0.1*i*pos[i]);
    //     //     viewer->addCoordinateSystem(50, mat.cast<float>());
    //     // } else {
    //         Eigen::Affine3d mat = Eigen::Translation3d(vs[2]+0.1*(i-20)*pos[i])*Eigen::Quaterniond(spln.value(0.1*i).real(),spln.value(0.1*i).imag_i(),spln.value(0.1*i).imag_j(),spln.value(0.1*i).imag_k());
    //         pcl::PointXYZ p(100*spln.value(0.1*i).imag_i(),100*spln.value(0.1*i).imag_j(),100*spln.value(0.1*i).imag_k());
    //         viewer->addSphere(p, 5, to_string(i));
    //         viewer->addCoordinateSystem(20, mat.cast<float>());
    //     // }
    //     // cout << 0.1*i << '\t' << spln.value( 0.1*i ) << endl;
    // }


    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
    viewer->resetStoppedFlag();
    viewer->close();
    return 0;
}