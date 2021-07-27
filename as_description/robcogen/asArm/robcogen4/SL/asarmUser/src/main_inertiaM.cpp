#include <iostream>
#include <fstream>
#include <ctime>

#include <iit/rbd/rbd.h>
#include <iit/rbd/utils.h>
#include <iit/robots/asarm/jsim.h>

#include "SL.h"
#include "SL_user.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"

using namespace std;
using namespace [iit];

static SL_Jstate currentState[N_ROBOT_DOFS];
static SL_endeff endeffector[N_ROBOT_ENDEFFECTORS];
static SL_Cstate basePosition;
static SL_quat baseOrient;
static SL_uext   ux[N_DOFS+1];
static Matrix rbdM;
static Vector rbdCG;

static void fillState(asArm::JointState& q, SL_Jstate* SLState);
static void SL_init();

/* This main is supposed to be used to test the joint space inertia matrix routines */
int main(int argc, char**argv)
{
    asArm::JointState q;

    SL_init();
    std::srand(std::time(NULL)); // initialize random number generator
    fillState(q, currentState);
    SL_ForDynComp(currentState, &basePosition, &baseOrient, ux, endeff, rbdM, rbdCG);
    asArm::dyn::jsim(q);

    asArm::dyn::JSIM SLM;

    // Copies the matrix of SL into an Eigen matrix, to make it easier to print, compare, etc.
    // Copy the joint-space part of the matrix.
    // Cannot use for loops because the joint ordering might be different in SL
    SLM(asArm::XARMJOINT1+6,asArm::XARMJOINT1+6) = rbdM[xarmjoint1][xarmjoint1];
    SLM(asArm::XARMJOINT1+6,asArm::XARMJOINT2+6) = rbdM[xarmjoint1][xarmjoint2];
    SLM(asArm::XARMJOINT1+6,asArm::XARMJOINT3+6) = rbdM[xarmjoint1][xarmjoint3];
    SLM(asArm::XARMJOINT1+6,asArm::XARMJOINT4+6) = rbdM[xarmjoint1][xarmjoint4];
    SLM(asArm::XARMJOINT1+6,asArm::XARMJOINT5+6) = rbdM[xarmjoint1][xarmjoint5];
    SLM(asArm::XARMJOINT1+6,asArm::XARMJOINT6+6) = rbdM[xarmjoint1][xarmjoint6];
    SLM(asArm::XARMJOINT2+6,asArm::XARMJOINT1+6) = rbdM[xarmjoint2][xarmjoint1];
    SLM(asArm::XARMJOINT2+6,asArm::XARMJOINT2+6) = rbdM[xarmjoint2][xarmjoint2];
    SLM(asArm::XARMJOINT2+6,asArm::XARMJOINT3+6) = rbdM[xarmjoint2][xarmjoint3];
    SLM(asArm::XARMJOINT2+6,asArm::XARMJOINT4+6) = rbdM[xarmjoint2][xarmjoint4];
    SLM(asArm::XARMJOINT2+6,asArm::XARMJOINT5+6) = rbdM[xarmjoint2][xarmjoint5];
    SLM(asArm::XARMJOINT2+6,asArm::XARMJOINT6+6) = rbdM[xarmjoint2][xarmjoint6];
    SLM(asArm::XARMJOINT3+6,asArm::XARMJOINT1+6) = rbdM[xarmjoint3][xarmjoint1];
    SLM(asArm::XARMJOINT3+6,asArm::XARMJOINT2+6) = rbdM[xarmjoint3][xarmjoint2];
    SLM(asArm::XARMJOINT3+6,asArm::XARMJOINT3+6) = rbdM[xarmjoint3][xarmjoint3];
    SLM(asArm::XARMJOINT3+6,asArm::XARMJOINT4+6) = rbdM[xarmjoint3][xarmjoint4];
    SLM(asArm::XARMJOINT3+6,asArm::XARMJOINT5+6) = rbdM[xarmjoint3][xarmjoint5];
    SLM(asArm::XARMJOINT3+6,asArm::XARMJOINT6+6) = rbdM[xarmjoint3][xarmjoint6];
    SLM(asArm::XARMJOINT4+6,asArm::XARMJOINT1+6) = rbdM[xarmjoint4][xarmjoint1];
    SLM(asArm::XARMJOINT4+6,asArm::XARMJOINT2+6) = rbdM[xarmjoint4][xarmjoint2];
    SLM(asArm::XARMJOINT4+6,asArm::XARMJOINT3+6) = rbdM[xarmjoint4][xarmjoint3];
    SLM(asArm::XARMJOINT4+6,asArm::XARMJOINT4+6) = rbdM[xarmjoint4][xarmjoint4];
    SLM(asArm::XARMJOINT4+6,asArm::XARMJOINT5+6) = rbdM[xarmjoint4][xarmjoint5];
    SLM(asArm::XARMJOINT4+6,asArm::XARMJOINT6+6) = rbdM[xarmjoint4][xarmjoint6];
    SLM(asArm::XARMJOINT5+6,asArm::XARMJOINT1+6) = rbdM[xarmjoint5][xarmjoint1];
    SLM(asArm::XARMJOINT5+6,asArm::XARMJOINT2+6) = rbdM[xarmjoint5][xarmjoint2];
    SLM(asArm::XARMJOINT5+6,asArm::XARMJOINT3+6) = rbdM[xarmjoint5][xarmjoint3];
    SLM(asArm::XARMJOINT5+6,asArm::XARMJOINT4+6) = rbdM[xarmjoint5][xarmjoint4];
    SLM(asArm::XARMJOINT5+6,asArm::XARMJOINT5+6) = rbdM[xarmjoint5][xarmjoint5];
    SLM(asArm::XARMJOINT5+6,asArm::XARMJOINT6+6) = rbdM[xarmjoint5][xarmjoint6];
    SLM(asArm::XARMJOINT6+6,asArm::XARMJOINT1+6) = rbdM[xarmjoint6][xarmjoint1];
    SLM(asArm::XARMJOINT6+6,asArm::XARMJOINT2+6) = rbdM[xarmjoint6][xarmjoint2];
    SLM(asArm::XARMJOINT6+6,asArm::XARMJOINT3+6) = rbdM[xarmjoint6][xarmjoint3];
    SLM(asArm::XARMJOINT6+6,asArm::XARMJOINT4+6) = rbdM[xarmjoint6][xarmjoint4];
    SLM(asArm::XARMJOINT6+6,asArm::XARMJOINT5+6) = rbdM[xarmjoint6][xarmjoint5];
    SLM(asArm::XARMJOINT6+6,asArm::XARMJOINT6+6) = rbdM[xarmjoint6][xarmjoint6];
    // Copy the 6x6 block with the composite inertia of the whole robot:
    int r,c;
    for(r=0; r<6; r++) {
        for(c=0; c<6; c++) {
            SLM(r,c) = rbdM[r+1+6][c+1+6];
        }
    }
    // re-arrange blocks to match the convention of the generated dynamics code
    iit::rbd::Matrix33d temp;
    temp = SLM.block<3,3>(0,0);
    SLM.block<3,3>(0,0) = SLM.block<3,3>(3,3);
    SLM.block<3,3>(3,3) = temp;

    SLM.block<3,3>(0,3) = SLM.block<3,3>(3,0);
    SLM.block<3,3>(3,0) = SLM.block<3,3>(0,3).transpose();

    // Copy the remaining blocks:
    for(r=0; r<6; r++) {
        for(c=6; c<6; c++) {
            SLM(r,c) = rbdM[r+1+6][c+1-6];
        }
    }
    for(r=0; r<6; r++) {
        SLM(r,asArm::XARMJOINT1+6) = rbdM[r+1+6][xarmjoint1];
        SLM(r,asArm::XARMJOINT2+6) = rbdM[r+1+6][xarmjoint2];
        SLM(r,asArm::XARMJOINT3+6) = rbdM[r+1+6][xarmjoint3];
        SLM(r,asArm::XARMJOINT4+6) = rbdM[r+1+6][xarmjoint4];
        SLM(r,asArm::XARMJOINT5+6) = rbdM[r+1+6][xarmjoint5];
        SLM(r,asArm::XARMJOINT6+6) = rbdM[r+1+6][xarmjoint6];
    }
    // Deal with different convention about spatial vectors (linear/angular part)
    Eigen::Matrix<double,3,6> tempF;
    tempF = SLM.block<3,6>(0,6);
    SLM.block<3,6>(0,6) = SLM.block<3,6>(3,6);
    SLM.block<3,6>(3,6) = tempF;
    // F and F^T blocks
    SLM.block<6,6>(6,0) = SLM.block<6,6>(0,6).transpose();

    rbd::Utils::CwiseAlmostZeroOp<asArm::dyn::JSIM::Scalar> almostZero(1E-4);

    cout << "SL:" << endl << SLM.unaryExpr(almostZero) << endl;
    cout << "Me:" << endl << asArm::dyn::jsim.unaryExpr(almostZero)  << endl;

    //asArm::dyn::JSIM::MatrixType diff = SLM - asArm::dyn::jsim;
    //cout << "difference:" << endl << diff.unaryExpr(almostZero) << endl;

    //cout << SLM.block<6,6>(0,0).unaryExpr(almostZero) << endl;
    //cout << asArm::dyn::jsim.block<6,6>(0,0).unaryExpr(almostZero) << endl;
    return TRUE;
}


void fillState(asArm::JointState& q, SL_Jstate* SLState) {
    static const double max = 12.3;
    q(0) = ( ((double)std::rand()) / RAND_MAX) * max;
    SLState[xarmjoint1].th = q(0);
    q(1) = ( ((double)std::rand()) / RAND_MAX) * max;
    SLState[xarmjoint2].th = q(1);
    q(2) = ( ((double)std::rand()) / RAND_MAX) * max;
    SLState[xarmjoint3].th = q(2);
    q(3) = ( ((double)std::rand()) / RAND_MAX) * max;
    SLState[xarmjoint4].th = q(3);
    q(4) = ( ((double)std::rand()) / RAND_MAX) * max;
    SLState[xarmjoint5].th = q(4);
    q(5) = ( ((double)std::rand()) / RAND_MAX) * max;
    SLState[xarmjoint6].th = q(5);
}

static void SL_init() {
    init_kinematics();
    init_dynamics();

    bzero((void *)&basePosition,sizeof(basePosition));
    bzero((void *)&baseOrient,sizeof(baseOrient));
    bzero((void *)ux,sizeof(SL_uext)*N_DOFS+1);
    setDefaultEndeffector(); // the the default end-effector parameters

    baseOrient.q[_Q0_] = 1;
    baseOrient.q[_Q1_] = 0;
    baseOrient.q[_Q2_] = 0;
    baseOrient.q[_Q3_] = 0;

    rbdM = my_matrix(1,N_DOFS+6,1,N_DOFS+6);
    rbdCG = my_vector(1,N_DOFS+6);
    mat_zero(rbdM);
    vec_zero(rbdCG);

    freeze_base = TRUE;//
}
