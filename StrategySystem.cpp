#include "StdAfx.h"
#include "StrategySystem.h"

IMPLEMENT_DYNAMIC(CStrategySystem, CObject)

extern int nKick;

#define  BALL_WIDTH		78
#define  BALL_LENGTH	156 
#define  BALL_DIS	    26 
#define  CORNER         115

//Quadrant labels
#define UP_RIGHT 1
#define DOWN_LEFT -1
#define UP_LEFT 2
#define DOWN_RIGHT -2



CStrategySystem::CStrategySystem(int id)
{
	m_OurTeam=id;
	boundRect.SetRect(65,95,965,723);
	if(id == 1)
		m_nGameArea=GAME_LEFT;
	else
		m_nGameArea=GAME_RIGHT;
	for (int i = 0; i < 35; i++)
		command[i] = 0;
	C_Home1.Data.Lv=0;
	C_Home1.Data.Rv=0;
	C_Home1.Data.Command=C_STOP;
	C_Home2.Data.Lv=0;
	C_Home2.Data.Rv=0;
	C_Home2.Data.Command=C_STOP;
	C_Home3.Data.Lv=0;
	C_Home3.Data.Rv=0;
	C_Home3.Data.Command=C_STOP;
	C_Home4.Data.Lv=0;
	C_Home4.Data.Rv=0;
	C_Home4.Data.Command=C_STOP;
	C_Home5.Data.Lv=0;
	C_Home5.Data.Rv=0;
	C_Home5.Data.Command=C_STOP;
	C_Home6.Data.Lv=0;
	C_Home6.Data.Rv=0;
	C_Home6.Data.Command=C_STOP;
	C_Home7.Data.Lv=0;
	C_Home7.Data.Rv=0;
	C_Home7.Data.Command=C_STOP;
	C_Home8.Data.Lv=0;
	C_Home8.Data.Rv=0;
	C_Home8.Data.Command=C_STOP;
	C_Home9.Data.Lv=0;
	C_Home9.Data.Rv=0;
	C_Home9.Data.Command=C_STOP;
	C_Home10.Data.Lv=0;
	C_Home10.Data.Rv=0;
	C_Home10.Data.Command=C_STOP;
	C_Home11.Data.Lv=0;
	C_Home11.Data.Rv=0;
	C_Home11.Data.Command=C_STOP;

	m_nStrategy=id;//left=1,right=0
	nKick=0;
	nKick2=0;
	nShoot=0;
	nSweep=0;
}




CStrategySystem::~CStrategySystem()
{

}


void CStrategySystem::Action()
{
	Think();
}


void CStrategySystem::Think()
{ 
	KickOff();
	/*static int state_switch = 0;
	if (ball.position.x == 520 && ball.position.y == 407 && state_switch == 0)
	{
		KickOff();
	}

	if (ball.position.x != 520 && ball.position.y != 407 && state_switch == 0)
	{
		state_switch++;
	}

	if (state_switch != 0)
	{
		if (ball.position.x > 520)
		{
			Attack1();
		}
		else
		{
			Attack1();
		}
	}*/
}


void CStrategySystem::KickOff()
{
	CPoint goal = { 1040,311 };
	smartPosition(HOME1, goal, 127);
	Position(HOME3, goal, 127);
}

void CStrategySystem::Attack1()
{
	Velocity(HOME2, -127, -127);
	Velocity(HOME1, -127, -127);
	smartPosition(HOME3, ball.position, 20);
}


void CStrategySystem::Defense1()
{
	
}


void CStrategySystem::NormalGame3()
{

}


void CStrategySystem::NormalGame4()
{
	
}


void CStrategySystem::Angle(int which, int desired_angle)
{
	Robot2 *robot;
	int theta_e, vL, vR; 
	
	switch(which){
	case HOME1:	
		robot = &home1;
		break;
	case HOME2:            
		robot = &home2;
		break;
	case HOME3:	
		robot = &home3;
		break;
	case HOME4:	
		robot = &home4;
		break;
	case HOME5:	
		robot = &home5;
		break;
	case HOME6:	
		robot = &home6;
		break;
	case HOME7:	
		robot = &home7;
		break;
	case HOME8:	
		robot = &home8;
		break;
	case HOME9:	
		robot = &home9;
		break;
	case HOME10:	
		robot = &home10;
		break;
	case HGOALIE:      
		robot = &hgoalie;
		break;
	}

	theta_e = robot->angle - desired_angle;
	while (theta_e > 180)
		theta_e -= 360;
	while (theta_e < -180)
		theta_e += 360;                                      
	if(theta_e < -90)  
		theta_e += 180; 
	else if(theta_e > 90)  
		theta_e -= 180;		
	vL = (int)(-7.0/10.0*theta_e);  
	vR = (int)(7.0/10.0*theta_e);
	Velocity(which, vL, vR);
}


void CStrategySystem::Velocity(int which, int vL, int vR)
{
	if(vL < -127)	vL = -127;
	if(vL > 127)	vL = 127;

	if(vR < -127)	vR = -127;
	if(vR > 127)	vR = 127;
	
	int buffer;

	buffer = vR;
	vR = -vL;
	vL = -buffer;

	switch(which){
	case HOME1:
		command[2] = vL;
		command[3] = vR;
		command[4] = C_GO;
		break;
	case HOME2:           
		command[5] = vL;
		command[6] = vR;
		command[7] = C_GO;	
		break;
	case HOME3:           
		command[8] = vL;
		command[9] = vR;
		command[10] = C_GO;	
		break;
	case HOME4:           
		command[11] = vL;
		command[12] = vR;
		command[13] = C_GO;	
		break;
	case HOME5:           
		command[14] = vL;
		command[15] = vR;
		command[16] = C_GO;	
		break;
	case HOME6:           
		command[17] = vL;
		command[18] = vR;
		command[19] = C_GO;	
		break;
	case HOME7:           
		command[20] = vL;
		command[21] = vR;
		command[22] = C_GO;	
		break;
	case HOME8:           
		command[23] = vL;
		command[24] = vR;
		command[25] = C_GO;	
		break;
	case HOME9:           
		command[26] = vL;
		command[27] = vR;
		command[28] = C_GO;	
		break;
	case HOME10:           
		command[29] = vL;
		command[30] = vR;
		command[31] = C_GO;	
		break;
	case HGOALIE:         
		command[32] = vL;
		command[33] = vR;
		command[34] = C_GO;	
		break;
	}
}


void CStrategySystem::Position(int which, CPoint point, int max_v)
{
	Robot2 *robot;
	double distance_e;

	switch (which) {
	case HOME1:
		robot = &home1;
		break;
	case HOME2:
		robot = &home2;
		break;
	case HOME3:
		robot = &home3;
		break;
	case HOME4:
		robot = &home4;
		break;
	case HOME5:
		robot = &home5;
		break;
	case HOME6:
		robot = &home6;
		break;
	case HOME7:
		robot = &home7;
		break;
	case HOME8:
		robot = &home8;
		break;
	case HOME9:
		robot = &home9;
		break;
	case HOME10:
		robot = &home10;
		break;
	case HGOALIE:
		robot = &hgoalie;
		break;
	}

	int desired_angle = 0, theta_e = 0, vL, vR, vO = max_v, buffer;
	double dx, dy, d_e, Ka = 0.111;

	//manage vO
	if (vO > 127)	vO = 127;
	if (vO < -127)	vO = -127;

	//calculate distance error
	d_e = computeDistance(robot->position, point);
	if (d_e > 100)	    Ka = 0.188;
	else if (d_e > 50)  Ka = 0.211;
	else if (d_e > 30)	Ka = 0.233;
	else if (d_e > 20)	Ka = 0.255;
	else				Ka = 0.277;

	//calculate the desired angle
	desired_angle = computeAngle(robot->position, point);
	//calculate angle error,theta_e
	theta_e = robot->angle - desired_angle;
	//normalize angle error
	if (theta_e > 180)	theta_e -= 360;
	if (theta_e < -180) theta_e += 360;


	if (theta_e > 95 || theta_e < -95)
	{
		//switch heading direction
		theta_e += 180;
		if (theta_e > 180)theta_e -= 360;

		//manage exception
		if (theta_e < -80)theta_e = -80;
		if (d_e < 5.0&&abs(theta_e) < 40) Ka = 0.1;

		//calculate vl and vr
		vR = (int)((-vO * (1.0 / 1.0 + exp(-3 * d_e)) - 0.3) + (Ka * theta_e));
		vL = (int)((-vO * (1.0 / 1.0 + exp(-3 * d_e)) - 0.3) - (Ka * theta_e));
	}

	else if (theta_e<85 && theta_e>-85)
	{
		//manage exception
		if (d_e < 5.0&&abs(theta_e) < 40) Ka = 0.1;

		//calculate vL and VR
		vR = (int)((vO*(1.0 / (1.0 + exp(-3 * d_e)) - 0.3)) + (Ka * theta_e));
		vL = (int)((vO*(1.0 / (1.0 + exp(-3 * d_e)) - 0.3)) - (Ka * theta_e));

	}
	//magnitude of angle error is within [90-5,90+5]
	else
	{
		//calculate vL and vR
		vR = (int)(+0.17*theta_e);
		vL = (int)(-0.17*theta_e);
	}

	Velocity(which, vL, vR);

}

#define ANGLE_BOUND 60
#define BOUND_BOUND 12
#define G_OFFSET		20
#define G_ANGLE_BOUND	60
#define G_BOUND_BOUND	10


void CStrategySystem::ReceiveData(Robot1 bal,Robot2 ho1,Robot2 ho2,Robot2 ho3,Robot2 ho4,
		             Robot2 ho5,Robot2 ho6,Robot2 ho7,Robot2 ho8,Robot2 ho9,
					 Robot2 ho10,Robot2 hgo,Robot3 opp)
{
	if(m_nGameArea==GAME_LEFT)
	{
		ball.position=bal.position;   
		//ball.oldPosition=bal.oldPosition;
		
		home1.position=ho1.position;      
		home1.angle=ho1.angle;
		
		home2.position=ho2.position;       
		home2.angle=ho2.angle;
		
		home3.position=ho3.position;
		home3.angle=ho3.angle;

		home4.position=ho4.position;
		home4.angle=ho4.angle;

		home5.position=ho5.position;
		home5.angle=ho5.angle;

		home6.position=ho6.position;
		home6.angle=ho6.angle;

		home7.position=ho7.position;
		home7.angle=ho7.angle;

		home8.position=ho8.position;
		home8.angle=ho8.angle;

		home9.position=ho9.position;
		home9.angle=ho9.angle;

		home10.position=ho10.position;
		home10.angle=ho10.angle;

		hgoalie.position=hgo.position;    
		hgoalie.angle=hgo.angle;
		
		opponent.position1=opp.position1;   
		opponent.position2=opp.position2;
		opponent.position3=opp.position3;
		opponent.position4=opp.position4;
		opponent.position5=opp.position5;
		opponent.position6=opp.position6;
		opponent.position7=opp.position7;
		opponent.position8=opp.position8;
		opponent.position9=opp.position9;
		opponent.position10=opp.position10;
		opponent.position11=opp.position11;
	
	}
	else  
	{
		ball.position.x = 1040 - bal.position.x;
		ball.position.y = bal.position.y;
		//ball.oldPosition = bal.oldPosition;

		home1.position.x = 1040 - ho1.position.x;
		home1.position.y = 814 - ho1.position.y;
		if (ho1.angle >= 0)	home1.angle = ho1.angle - 180;
		if (ho1.angle < 0)	home1.angle = ho1.angle + 180;
		//home1.angle = -home1.angle;

		home2.position.x = 1040 - ho2.position.x;
		home2.position.y = 814 - ho2.position.y;
		if (ho2.angle >= 0)	home2.angle = ho2.angle - 180;
		if (ho2.angle < 0)	home2.angle = ho2.angle + 180;
		//home2.angle = -home2.angle;

		home3.position.x = 1040 - ho3.position.x;
		home3.position.y = 814 - ho3.position.y;
		if (ho3.angle >= 0)	home3.angle = ho3.angle - 180;
		if (ho3.angle < 0)	home3.angle = ho3.angle + 180;
		//home3.angle = -home3.angle;

		home4.position.x = 1040 - ho4.position.x;
		home4.position.y = 814 - ho4.position.y;
		if (ho4.angle >= 0)	home4.angle = ho4.angle - 180;
		if (ho4.angle < 0)	home4.angle = ho4.angle + 180;
		//home4.angle = -home4.angle;

		home5.position.x = 1040 - ho5.position.x;
		home5.position.y = 814 - ho5.position.y;
		if (ho5.angle >= 0)	home5.angle = ho5.angle - 180;
		if (ho5.angle < 0)	home5.angle = ho5.angle + 180;
		//home5.angle = -home5.angle;

		home6.position.x = 1040 - ho6.position.x;
		home6.position.y = 814 - ho6.position.y;
		if (ho6.angle >= 0)	home6.angle = ho6.angle - 180;
		if (ho6.angle < 0)	home6.angle = ho6.angle + 180;
		//home6.angle = -home6.angle;

		home7.position.x = 1040 - ho7.position.x;
		home7.position.y = 814 - ho7.position.y;
		if (ho7.angle >= 0)	home7.angle = ho7.angle - 180;
		if (ho7.angle < 0)	home7.angle = ho7.angle + 180;
		//home7.angle = -home7.angle;

		home8.position.x = 1040 - ho8.position.x;
		home8.position.y = 814 - ho8.position.y;
		if (ho8.angle >= 0)	home8.angle = ho8.angle - 180;
		if (ho8.angle < 0)	home8.angle = ho8.angle + 180;
		//home8.angle = -home8.angle;

		home9.position.x = 1040 - ho9.position.x;
		home9.position.y = 814 - ho9.position.y;
		if (ho9.angle >= 0)	home9.angle = ho9.angle - 180;
		if (ho9.angle < 0)	home9.angle = ho9.angle + 180;
		//home9.angle = -home9.angle;

		home10.position.x = 1040 - ho10.position.x;
		home10.position.y = 814 - ho10.position.y;
		if (ho10.angle >= 0)	home10.angle = ho10.angle - 180;
		if (ho10.angle < 0)	home10.angle = ho10.angle + 180;
		//home10.angle = -home10.angle;

		hgoalie.position.x = 1040 - hgo.position.x;
		hgoalie.position.y = 814 - hgo.position.y;
		if (hgo.angle >= 0)	hgoalie.angle = hgo.angle - 180;
		if (hgo.angle < 0)	hgoalie.angle = hgo.angle - 180;
		//hgoalie.angle = -hgoalie.angle;

		opponent.position1.y = 814 - opp.position1.y;
		opponent.position2.y = 814 - opp.position2.y;
		opponent.position3.y = 814 - opp.position3.y;
		opponent.position4.y = 814 - opp.position4.y;
		opponent.position5.y = 814 -  opp.position5.y;
		opponent.position6.y = 814 - opp.position6.y;
		opponent.position7.y = 814 - opp.position7.y;
		opponent.position8.y = 814 - opp.position8.y;
		opponent.position9.y = 814 - opp.position9.y;
		opponent.position10.y = 814 - opp.position10.y;
		opponent.position11.y = 814 - opp.position11.y;

		opponent.position1.x = 1040 - opp.position1.x;
		opponent.position2.x = 1040 - opp.position2.x;
		opponent.position3.x = 1040 - opp.position3.x;
		opponent.position4.x = 1040 - opp.position4.x;
		opponent.position5.x = 1040 - opp.position5.x;
		opponent.position6.x = 1040 - opp.position6.x;
		opponent.position7.x = 1040 - opp.position7.x;
		opponent.position8.x = 1040 - opp.position8.x;
		opponent.position9.x = 1040 - opp.position9.x;
		opponent.position10.x = 1040 - opp.position10.x;
		opponent.position11.x = 1040 - opp.position11.x;
	}
}

void CStrategySystem::Stop(int which)
{
	int vL,vR;
	vL=vR=0;
	Velocity(which,vL,vR);
}


void CStrategySystem::Goalie(int which)
{
	Robot2 *robot;
	CPoint target; 
	int dx, dy;
	
	switch(which){
	case HOME1:	
		robot = &home1;
		break;
	case HOME2:            
		robot = &home2;
		break;
	case HOME3:	
		robot = &home3;
		break;
	case HOME4:	
		robot = &home4;
		break;
	case HGOALIE:      
		robot = &hgoalie;
		break;
	}

	target.y = ball.position.y;
	target.x = boundRect.right - 10;

	if(target.y < boundRect.top + (boundRect.bottom-boundRect.top)/3)
	{
		target.y = boundRect.top + (boundRect.bottom-boundRect.top)/3;
	}
	else if(target.y > boundRect.bottom - (boundRect.bottom-boundRect.top)/3)
	{
		target.y = boundRect.bottom - (boundRect.bottom-boundRect.top)/3;
	}

	dx = robot->position.x - target.x ;
	dy = robot->position.y - target.y; 

	if(dx*dx+dy*dy > 10)  
		Position(which, CPoint(target.x,target.y),127);
	else 
		Angle(which, 90);
}


void CStrategySystem::faceBall(int which)
{
	Robot2 *robot;
	double dy, dx;
	int desired_angle;

	switch (which) {
	case HOME1:
		robot = &home1;
		break;
	case HOME2:
		robot = &home2;
		break;
	case HOME3:
		robot = &home3;
		break;
	case HOME4:
		robot = &home4;
		break;
	case HOME5:
		robot = &home5;
		break;
	case HOME6:
		robot = &home6;
		break;
	case HOME7:
		robot = &home7;
		break;
	case HOME8:
		robot = &home8;
		break;
	case HOME9:
		robot = &home9;
		break;
	case HOME10:
		robot = &home10;
		break;
	case HGOALIE:
		robot = &hgoalie;
		break;
	}

	dx = ball.position.x - robot->position.x;
	dy = ball.position.y - robot->position.y;

	if (dx == 0 && dy == 0)
		desired_angle = 90;
	else
		desired_angle = (180.0 / M_PI * atan2((double)(dy), (double)(dx)));

	if (desired_angle >= 0)	desired_angle += 180;
	if (desired_angle < 0)	desired_angle -= 180;

	Angle(which, desired_angle);
}


// Kick the ball with a velocity.
void CStrategySystem::kickBall(int which, int velocity)
{
	// TODO: Add your implementation code here.
	static int state = 0;
	Robot2 *robot;
	int dy, dx, distance, desired_angle, robotAngle;
	int theta;
	CPoint destination;

	switch (which) {
	case HOME1:
		robot = &home1;
		break;
	case HOME2:
		robot = &home2;
		break;
	case HOME3:
		robot = &home3;
		break;
	case HOME4:
		robot = &home4;
		break;
	case HOME5:
		robot = &home5;
		break;
	case HOME6:
		robot = &home6;
		break;
	case HOME7:
		robot = &home7;
		break;
	case HOME8:
		robot = &home8;
		break;
	case HOME9:
		robot = &home9;
		break;
	case HOME10:
		robot = &home10;
		break;
	case HGOALIE:
		robot = &hgoalie;
		break;
	}

	dy = ball.position.y - robot->position.y;
	dx = ball.position.x - robot->position.x;

	if (dx == 0 && dy == 0)
		desired_angle = 90;
	else
		desired_angle = (180.0 / M_PI * atan2((double)(dy), (double)(dx)));

	robotAngle = robot->angle;

	theta = desired_angle - robot->angle;

	if (theta > 180)
		theta -= 360;
	if (theta < -180)
		theta += 360;

	distance = sqrt(dx * dx + dy * dy);
	
	if (state == 0)
	{
		if (abs(computeAngle(robot->position, ball.position) - robot->angle) > 20)
			faceBall(which);
		else
			state = 1;
	}

	if (state == 1)
	{
		if (distance > 20)
			Position(which, ball.position,127);
		else
			state = 2;
	}

	if (state == 2)
	{
		desired_angle = computeAngle(robot->position, ball.position);
		
		if (abs((int)(desired_angle - robot->angle)) > 20)
			faceBall(which);
		else
			state = 3;
	}

	if (state == 3)
	{
		Velocity(which, velocity, velocity);
	}
}


int CStrategySystem::computeDistance(CPoint source, CPoint target)
{
	// TODO: Add your implementation code here.
	int distance, dy, dx;
	dy = target.y - source.y;
	dx = target.x - source.x;
	distance = sqrt(dy * dy + dx * dx);
	return distance;
}


int CStrategySystem::computeAngle(CPoint source, CPoint target)
{
	// TODO: Add your implementation code here.
	int theta, dy, dx;
	dy = source.y - target.y;
	dx = source.x - target.x;
	if (dx == 0 && dy == 0)
		theta = 90;
	else
		theta = (int)(180.0 / M_PI * atan2((double)(dy), (double)(dx)));

	if (theta > 180)
		theta -= 360;
	if (theta < -180)
		theta += 360;

	return theta;
}


int CStrategySystem::faceToPoint(int which, CPoint target)
{
	// TODO: Add your implementation code here.
	Robot2 *robot;
	double dy, dx;
	int desired_angle;

	switch (which) {
	case HOME1:
		robot = &home1;
		break;
	case HOME2:
		robot = &home2;
		break;
	case HOME3:
		robot = &home3;
		break;
	case HOME4:
		robot = &home4;
		break;
	case HOME5:
		robot = &home5;
		break;
	case HOME6:
		robot = &home6;
		break;
	case HOME7:
		robot = &home7;
		break;
	case HOME8:
		robot = &home8;
		break;
	case HOME9:
		robot = &home9;
		break;
	case HOME10:
		robot = &home10;
		break;
	case HGOALIE:
		robot = &hgoalie;
		break;
	}

	dx = target.x - robot->position.x;
	dy = target.y - robot->position.y;

	if (dx == 0 && dy == 0)
		desired_angle = 90;
	else
		desired_angle = (180.0 / M_PI * atan2((double)(dy), (double)(dx)));

	if (desired_angle != robot->angle)
		Angle(which, desired_angle);
	return 0;
}


int CStrategySystem::reportQuadrant(CPoint source, CPoint centre)
{
	// TODO: Add your implementation code here.
	int dy, dx, quadrant;

	dy = source.y - centre.y;
	dx = source.x - centre.x;

	if (dy > 0) //Down
	{
		if (dx > 0) //Right
			quadrant = DOWN_RIGHT;
		else
			quadrant = DOWN_LEFT;
	}
	else //Up
	{
		if (dx > 0) //Right
			quadrant = UP_RIGHT;
		else
			quadrant = UP_LEFT;
	}

	return quadrant;
}



/*int CStrategySystem::moveAroundPoint(int which, CPoint centre)
{
	Robot2 *robot;

	switch (which) {
	case HOME1:
		robot = &home1;
		break;
	case HOME2:
		robot = &home2;
		break;
	case HOME3:
		robot = &home3;
		break;
	case HOME4:
		robot = &home4;
		break;
	case HOME5:
		robot = &home5;
		break;
	case HOME6:
		robot = &home6;
		break;
	case HOME7:
		robot = &home7;
		break;
	case HOME8:
		robot = &home8;
		break;
	case HOME9:
		robot = &home9;
		break;
	case HOME10:
		robot = &home10;
		break;
	case HGOALIE:
		robot = &hgoalie;
		break;
	}

	static int state = 0;
	int currentTheta, faceAngle, thetaDiff, vL, vR, r;
	double omega;
	r = computeDistance(robot->position, centre);

	//Get within range of 20
	if (state == 0)
	{
		//Outside range
		if (r > 20)		Position(which, centre);
		//Within range
		else			state = 1;
	}

	//Adjust so facing perpendicular to centre
	if (state == 1)
	{
		
		currentTheta = computeAngle(robot->position, centre);
		if (currentTheta == 180)	currentTheta = -180;
		thetaDiff = robot->angle - currentTheta;
		if (thetaDiff > 180)		thetaDiff -= 360;
		else if (thetaDiff < -180)	thetaDiff += 360;

		if ((thetaDiff > 91 || thetaDiff < 89) || (thetaDiff < -91 || thetaDiff > -89))
		{
			//Check for position
			//If azimuth
			if (currentTheta == 0)			faceAngle = 90;
			else if (currentTheta == 90)	faceAngle = -180;
			else if (currentTheta == -180)	faceAngle = -90;
			else if (currentTheta == -90)	faceAngle = 0;
			//Not azimuth
			else	faceAngle = currentTheta + 90;
			//Correction
			if (faceAngle > 180)		faceAngle -= 360;
			else if (faceAngle < -180)	faceAngle += 360;
			//Act
			Angle(which, faceAngle);
		}
		else
		{
			state = 2;
		}
	}

	//Move along the circular path
	if (state == 2)
	{
		r = computeDistance(robot->position, centre);
		omega =(double)(127 / (r + 5));
		vR = omega * (r + 5);
		vL = omega * (r - 5);
		Velocity(which, vL, vR);
	}

	return 0;
}*/


// Positioning with collision and wall avoidance
int CStrategySystem::smartPosition(int which, CPoint target, int max_velocity)
{
	Robot2 *robot;
	switch (which) {
	case HOME1:
		robot = &home1;
		break;
	case HOME2:
		robot = &home2;
		break;
	case HOME3:
		robot = &home3;
		break;
	case HOME4:
		robot = &home4;
		break;
	case HOME5:
		robot = &home5;
		break;
	case HOME6:
		robot = &home6;
		break;
	case HOME7:
		robot = &home7;
		break;
	case HOME8:
		robot = &home8;
		break;
	case HOME9:
		robot = &home9;
		break;
	case HOME10:
		robot = &home10;
		break;
	case HGOALIE:
		robot = &hgoalie;
		break;
	}
	// TODO: Add your implementation code here.
	static int state = 0;
	int dist_closest, threshold = 30, dy, k, theta, theta_w, x, y, dist, dxd, dx1, dx2, wall_id = 0, dxw, dyw;
	CPoint coor1, coor2, interim, interim_w;

	//check for wall
	if (robot->position.y < 20 || robot->position.y > 794 || robot->position.x < 20 || robot->position.x > 1020)
	{
		//Manage exceptions
		if (!((robot->position.x < 20 || robot->position.x > 1020) && (robot->position.y > 311 || robot->position.y < 509)))
			state = 2;
	}

	if (state = 0)
	{
		Position(which, target, max_velocity);
		//Check for closest enemy
		dist_closest = reportEnemy(which).distance;
		
		if (dist_closest < threshold)	state = 1;
	}

	//state=1 when closest enemy >= threshold
	if (state = 1)
	{
		//Check for closest coordinates
		coor1 = reportEnemy(which).coor1;
		coor2 = reportEnemy(which).coor2;
		dy = coor2.y - robot->position.y;
		if (dy < 0)
		{
			theta = -90;

		}
		else
		{
			theta = 90;
		}
		
		dist = computeDistance(robot->position, coor1);
		interim.x = coor1.x + dist  * cos(theta * PI/180);
		interim.y = coor1.y + dist  * sin(theta * PI/180);

		if (dy < 0)
		{
			interim.y += 13;
		}
		else
		{
			interim.y -= 13;
		}

		dx2 = target.x - coor1.x;

		if (dx2 > 0)
		{
			interim.x += 10;
		}
		else
		{
			interim.x -= 10;
		}

		dx1 = target.x - robot->position.x;
		dx2 = target.x - coor1.x;
		dxd = dx1 - dx2;

		if (dxd > 0)
		{
			Position(which, interim, max_velocity);
		}
		else
		{
			state = 0;
		}
	}

	//State 2: wall correction
	if (state == 2)
	{
		//Convert position to wall_id for general actions for wall
		if (robot->position.y < 20)			wall_id = 1; //N
		else if (robot->position.x > 1020)	wall_id = 2; //E
		else if (robot->position.y > 794)	wall_id = 3; //S
		else if (robot->position.x < 20)	wall_id = 4; //W

		//Switch for responses to respective walls
		switch (wall_id)
		{
		case 1:
			dxw = target.x - robot->position.x;
			//Manage dxw = 0
			if (dxw == 0)	theta_w = 90;
			else
			{
				if (dxw > 0)	theta_w = 45;
				else			theta_w = 135;
			}
			break;

		case 2:
			dyw = 407 - robot->position.y;
			if (dyw < 0)	theta_w = 135;
			else			theta_w = -135;
			break;

		case 3:
			dxw = target.x - robot->position.x;
			//Manage dxw = 0
			if (dxw == 0)	theta_w = -90;
			else
			{
				if (dxw > 0)	theta_w = -45;
				else			theta_w = -135;
			}
			break;

		case 4:
			dyw = 407 - robot->position.y;
			if (dyw < 0)	theta_w = 45;
			else			theta_w = -45;
			break;

		default:
			break;
		}

		interim_w.x = robot->position.x + 20 * cos(theta_w * PI/180);
		interim_w.y = robot->position.y + 20 * sin(theta_w * PI / 180);

		Position(which, interim_w, 127);

		if (!(robot->position.y < 20 || robot->position.y > 794 || robot->position.x < 20 || robot->position.x > 1020))		state = 0;
	}

	return 0;
}


Positional CStrategySystem::reportEnemy(int which)
{
	Positional positional;
	
	Robot2 *robot;
	switch (which) {
	case HOME1:
		robot = &home1;
		break;
	case HOME2:
		robot = &home2;
		break;
	case HOME3:
		robot = &home3;
		break;
	case HOME4:
		robot = &home4;
		break;
	case HOME5:
		robot = &home5;
		break;
	case HOME6:
		robot = &home6;
		break;
	case HOME7:
		robot = &home7;
		break;
	case HOME8:
		robot = &home8;
		break;
	case HOME9:
		robot = &home9;
		break;
	case HOME10:
		robot = &home10;
		break;
	case HGOALIE:
		robot = &hgoalie;
		break;
	}
	// TODO: Add your implementation code here.
	//Manage player collision
	CPoint enemy_position[11];
	int enemy_distance[11], i;
	positional.distance = 1041;

	enemy_position[0] = opponent.position1;
	enemy_position[1] = opponent.position2;
	enemy_position[2] = opponent.position3;
	enemy_position[3] = opponent.position4;
	enemy_position[4] = opponent.position5;
	enemy_position[5] = opponent.position6;
	enemy_position[6] = opponent.position7;
	enemy_position[7] = opponent.position8;
	enemy_position[8] = opponent.position9;
	enemy_position[9] = opponent.position10;
	enemy_position[10] = opponent.position11;

	for (i = 0; i < 11; ++i)
		enemy_distance[i] = computeDistance(robot->position, enemy_position[i]);

	for (i = 0; i < 11; ++i)
	{
		if (enemy_distance[i] < positional.distance)
		{
			positional.distance = enemy_distance[i];
			positional.coor2 = positional.coor1;
			positional.coor1 = enemy_position[i];
		}
	}

	return positional;
}
