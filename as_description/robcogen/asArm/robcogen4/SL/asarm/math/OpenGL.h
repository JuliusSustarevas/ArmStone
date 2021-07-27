#define RAD2DEG (57.3)

static double x,y,z; // support vars

// The state of the base
glPushMatrix();
glTranslated((GLdouble)basec[0].x[1],(GLdouble)basec[0].x[2],(GLdouble)basec[0].x[3]);
glRotated((GLdouble)114.5916*ArcCos(baseo[0].q[1]),(GLdouble)baseo[0].q[2],(GLdouble)baseo[0].q[3],(GLdouble)baseo[0].q[4]);

// Joint xarmjoint1

glPushMatrix();
glPushMatrix();
// Align the Z axis along the direction between the two joints, to display
//  the link correctly ('myDrawGLElement()' draws along the Z axis)
glRotated((GLdouble)(RAD2DEG*std::acos( 0.21400000154972076/(std::sqrt(0.17499999701976776*0.17499999701976776 + 0.0*0.0 + 0.21400000154972076*0.21400000154972076)) )), (GLdouble)-(0.0), (GLdouble)0.17499999701976776, (GLdouble)0.0);
myDrawGLElement(::XARMJOINT1, 0.27644348359149146, 1);
glPopMatrix();

// move to the next joint, the same parameters as in the kinematics model file
glTranslated((GLdouble)0.17499999701976776, (GLdouble)0.0, (GLdouble)0.21400000154972076);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)1.0, (GLdouble)0.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)1.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// move according to the joint state
glRotated((GLdouble)RAD2DEG*state[::XARMJOINT1].th,(GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// Joint xarmjoint2

glPushMatrix();
glPushMatrix();
// Align the Z axis along the direction between the two joints, to display
//  the link correctly ('myDrawGLElement()' draws along the Z axis)
// nothing to do
myDrawGLElement(::XARMJOINT2, 0.0, 1);
glPopMatrix();

// move to the next joint, the same parameters as in the kinematics model file
glTranslated((GLdouble)0.0, (GLdouble)0.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*-1.5707999467849731), (GLdouble)1.0, (GLdouble)0.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)1.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// move according to the joint state
glRotated((GLdouble)RAD2DEG*state[::XARMJOINT2].th,(GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// Joint xarmjoint3

glPushMatrix();
glPushMatrix();
// Align the Z axis along the direction between the two joints, to display
//  the link correctly ('myDrawGLElement()' draws along the Z axis)
glRotated((GLdouble)(RAD2DEG*std::acos( 0.0/(std::sqrt(0.05350000038743019*0.05350000038743019 + -0.28450000286102295*-0.28450000286102295 + 0.0*0.0)) )), (GLdouble)-(-0.28450000286102295), (GLdouble)0.05350000038743019, (GLdouble)0.0);
myDrawGLElement(::XARMJOINT3, 0.2894866174270878, 1);
glPopMatrix();

// move to the next joint, the same parameters as in the kinematics model file
glTranslated((GLdouble)0.05350000038743019, (GLdouble)-0.28450000286102295, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)1.0, (GLdouble)0.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)1.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// move according to the joint state
glRotated((GLdouble)RAD2DEG*state[::XARMJOINT3].th,(GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// Joint xarmjoint4

glPushMatrix();
glPushMatrix();
// Align the Z axis along the direction between the two joints, to display
//  the link correctly ('myDrawGLElement()' draws along the Z axis)
glRotated((GLdouble)(RAD2DEG*std::acos( -0.0/(std::sqrt(0.07750000059604645*0.07750000059604645 + 0.3425000011920929*0.3425000011920929 + -0.0*-0.0)) )), (GLdouble)-(0.3425000011920929), (GLdouble)0.07750000059604645, (GLdouble)0.0);
myDrawGLElement(::XARMJOINT4, 0.35115879728261234, 1);
glPopMatrix();

// move to the next joint, the same parameters as in the kinematics model file
glTranslated((GLdouble)0.07750000059604645, (GLdouble)0.3425000011920929, (GLdouble)-0.0);
glRotated((GLdouble)(RAD2DEG*-1.5708004236221313), (GLdouble)1.0, (GLdouble)0.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)1.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// move according to the joint state
glRotated((GLdouble)RAD2DEG*state[::XARMJOINT4].th,(GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// Joint xarmjoint5

glPushMatrix();
glPushMatrix();
// Align the Z axis along the direction between the two joints, to display
//  the link correctly ('myDrawGLElement()' draws along the Z axis)
// nothing to do
myDrawGLElement(::XARMJOINT5, 0.0, 1);
glPopMatrix();

// move to the next joint, the same parameters as in the kinematics model file
glTranslated((GLdouble)0.0, (GLdouble)0.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*1.5707995891571045), (GLdouble)1.0, (GLdouble)0.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)1.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// move according to the joint state
glRotated((GLdouble)RAD2DEG*state[::XARMJOINT5].th,(GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// Joint xarmjoint6

glPushMatrix();
glPushMatrix();
// Align the Z axis along the direction between the two joints, to display
//  the link correctly ('myDrawGLElement()' draws along the Z axis)
glRotated((GLdouble)(RAD2DEG*std::acos( -0.0/(std::sqrt(0.07599999755620956*0.07599999755620956 + 0.09700000286102295*0.09700000286102295 + -0.0*-0.0)) )), (GLdouble)-(0.09700000286102295), (GLdouble)0.07599999755620956, (GLdouble)0.0);
myDrawGLElement(::XARMJOINT6, 0.12322743275578828, 1);
glPopMatrix();

// move to the next joint, the same parameters as in the kinematics model file
glTranslated((GLdouble)0.07599999755620956, (GLdouble)0.09700000286102295, (GLdouble)-0.0);
glRotated((GLdouble)(RAD2DEG*-1.5708004236221313), (GLdouble)1.0, (GLdouble)0.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)1.0, (GLdouble)0.0);
glRotated((GLdouble)(RAD2DEG*0.0), (GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// move according to the joint state
glRotated((GLdouble)RAD2DEG*state[::XARMJOINT6].th,(GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

// Draw the end effector
glPushMatrix();
x = eff[1].x[_X_];
y = eff[1].x[_Y_];
z = eff[1].x[_Z_];
glRotated(
        (GLdouble)RAD2DEG*acos(z),(GLdouble)-y,(GLdouble)x,(GLdouble)0);
myDrawGLElement(101, (double)Sqrt(x*x + y*y + z*z), 0);
glPopMatrix();

glPopMatrix();

glPopMatrix();

glPopMatrix();

glPopMatrix();

glPopMatrix();

glPopMatrix();

// pops the first matrix related to the state of the base
glPopMatrix();
