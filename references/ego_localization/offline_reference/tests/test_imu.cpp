#include "src/ceres_fusion.cpp"
#include <iostream>

using namespace offline_fusion;
void check(bool value, const char* name) {
    if (!value) throw std::runtime_error(name);
    std::cout << "PASS " << name << '\n';
}
int main() {
    ImuMatrix15 noise = ImuMatrix15::Zero();
    noise.block<3,3>(3,3).diagonal().setConstant(0.04);
    noise.block<3,3>(6,6).diagonal().setConstant(0.0004);
    noise.block<3,3>(9,9).diagonal().setConstant(0.0025);
    noise.block<3,3>(12,12).diagonal().setConstant(0.000025);
    const ImuMatrix15 f = imu_error_dynamics({}, {}, {});
    for (double dt : {1e-6, 0.001, 0.1, 1.0}) {
        ImuMatrix15 p = ImuMatrix15::Zero();
        int steps = std::max(1, int(std::ceil(dt/0.005)));
        for (int i=0; i<steps; ++i) propagate_imu_covariance(p, f, noise, dt/steps);
        const double pp = 0.04*std::pow(dt,3)/3 + 0.0025*std::pow(dt,5)/20;
        const double pv = 0.04*dt*dt/2 + 0.0025*std::pow(dt,4)/8;
        const double vv = 0.04*dt + 0.0025*std::pow(dt,3)/3;
        check(std::abs(p(0,0)/pp-1)<1e-8 && std::abs(p(0,3)/pv-1)<1e-8 && std::abs(p(3,3)/vv-1)<1e-8,
              "analytic acceleration and bias covariance at variable dt");
        check(std::abs(p(3,9) + 0.0025*dt*dt/2)<1e-12, "bias and velocity cross covariance sign");
        const ImuMatrix15 w = imu_sqrt_information(p);
        check((w*p*w.transpose()-ImuMatrix15::Identity()).norm()<1e-8,
              "whitening including very short intervals");
    }
    ImuMatrix15 p1=ImuMatrix15::Zero(), p2=ImuMatrix15::Zero(), whole=ImuMatrix15::Zero();
    for (int i=0;i<200;++i) propagate_imu_covariance(whole,f,noise,0.005);
    for (int i=0;i<74;++i) propagate_imu_covariance(p1,f,noise,0.005);
    for (int i=0;i<126;++i) propagate_imu_covariance(p2,f,noise,0.005);
    const ImuMatrix15 phi=imu_transition(f,0.63);
    check((whole-(phi*p1*phi.transpose()+p2)).norm()<1e-10,
          "split interval composes to full covariance");
    Dataset dataset;
    for (int i=0;i<=100;++i) {
        ImuSample s;
        s.t=i*0.01; s.accel={0.2,0.1,-9.793}; s.gyro={0.1,-0.2,0.3};
        dataset.imu.push_back(s);
    }
    FusionOptions options;
    const auto all=integrate_imu_interval(dataset,0,1,{},{},&options,true);
    const auto a=integrate_imu_interval(dataset,0,0.373,{},{},&options,true);
    const auto b=integrate_imu_interval(dataset,0.373,1,{},{},&options,true);
    Vec3 pdiff = all.delta_p - (a.delta_p+a.delta_v*b.dt+rotate(a.delta_q,b.delta_p));
    Vec3 vdiff = all.delta_v - (a.delta_v+rotate(a.delta_q,b.delta_v));
    check(std::abs(pdiff.x)+std::abs(pdiff.y)+std::abs(pdiff.z)<1e-9 &&
          std::abs(vdiff.x)+std::abs(vdiff.y)+std::abs(vdiff.z)<1e-9,
          "rotating nominal integration is invariant to non-sample split");
    const auto pi=preintegrate_with_bias_jacobians(dataset,0,0.1,&options,{0.01,0.02,0.03},{0.001,0.002,0.003});
    check(pi.covariance.allFinite() && pi.sqrt_information.allFinite(), "dynamic preintegration is finite");
    bool threw=false;
    try { integrate_imu_interval(dataset,0,1.1,{},{},&options,true); } catch (const std::runtime_error&) {threw=true;}
    check(threw,"uncovered IMU interval is rejected");
    options.imu_gyro_noise_density=0;
    threw=false;
    try { preintegrate_with_bias_jacobians(dataset,0,0.1,&options); } catch (const std::runtime_error&) {threw=true;}
    check(threw,"invalid noise density rejected");

    double p0[3]={0,0,0},v0[3]={0.1,0.2,0.3},q0[4]={1,0,0,0},ba0[3]={0.01,0.02,0.03},bg0[3]={0.001,0.002,0.003};
    double p3[3]={0.02,0.03,0.04},v3[3]={0.11,0.21,0.31},q3[4]={1,0,0,0},ba3[3]={0.011,0.021,0.031},bg3[3]={0.0011,0.0021,0.0031};
    ceres::AutoDiffCostFunction<ImuPreintegrationCost,15,3,3,4,3,3,3,3,4,3,3> cost(
        new ImuPreintegrationCost{pi,9.793,1,1,1,1,1});
    double* blocks[]={p0,v0,q0,ba0,bg0,p3,v3,q3,ba3,bg3};
    const double* inputs[]={p0,v0,q0,ba0,bg0,p3,v3,q3,ba3,bg3};
    int sizes[]={3,3,4,3,3,3,3,4,3,3};
    double storage[10][60],r[15]; double* js[10];
    for(int i=0;i<10;++i)js[i]=storage[i];
    check(cost.Evaluate(inputs,r,js),"whitened IMU cost evaluation");
    for(int block=0;block<10;++block)for(int k=0;k<sizes[block];++k){
        double old=blocks[block][k], plus[15],minus[15],eps=1e-6;
        blocks[block][k]=old+eps; cost.Evaluate(inputs,plus,nullptr);
        blocks[block][k]=old-eps; cost.Evaluate(inputs,minus,nullptr);
        blocks[block][k]=old;
        for(int row=0;row<15;++row){
            double numeric=(plus[row]-minus[row])/(2*eps);
            if(std::abs(numeric-js[block][row*sizes[block]+k])>=1e-4*(1+std::abs(numeric)))
                throw std::runtime_error("Jacobian component mismatch");
        }
    }
    std::cout<<"PASS all whitened IMU Jacobians\n";
}
