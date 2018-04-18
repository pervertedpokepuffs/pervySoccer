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
	static int state_switch = 0;
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
			Defense1();
		}
	}
}


void CStrategySystem::KickOff()
{
	Velocity(HOME2, -5, -5);
	Position(HOME4, { 0,0 });
	Position(HOME5, { 0,0 });
	Position(HOME6, { 0,0 });
}

void CStrategySystem::Attack1()
{
	Stop(HOME2);
	kickBall(HOME1, -100);
}


void CStrategySystem::Defense1()
{
	Velocity(HOME3, 127, -127);
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

	theta_e = desired_angle - robot->angle;
	while(theta_e > 180)
		theta_e -= 360;
	while(theta_e < -180)
		theta_e += 360;                                      
	if(theta_e < -90)  
		theta_e += 180; 
	else if(theta_e > 90)  
		theta_e -= 180;		
	vL = (int)(60.0/90.0*theta_e);  
	vR = (int)(-60.0/90.0*theta_e);
	Velocity(which, vL, vR);
}


void CStrategySystem::Velocity(int which, int vL, int vR)
{
	if(vL < -127)	vL = -127;
	if(vL > 127)	vL = 127;

	if(vR < -127)	vR = -127;
	if(vR > 127)	vR = 127;

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


void CStrategySystem::Position(int which, CPoint point)
{
	Robot2 *robot;    
	double distance_e;                 
	int dx, dy, desired_angle, theta_e, vL, vR;
	
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
        
	dx = point.x - robot->position.x;
	dy = point.y - robot->position.y;
	                                            
	distance_e = sqrt(1.0*dx*dx+1.0*dy*dy);	                                            

    if(dx == 0 && dy == 0)
		desired_angle = 90;
	else
		desired_angle = (int)(180.0/M_PI*atan2((double)(dy), (double)(dx)));
	
	theta_e = desired_angle - robot->angle;

	while(theta_e > 180)
		theta_e -= 360;
	while(theta_e < -180)
		theta_e += 360;                                      

	if(theta_e < -90){  
		theta_e += 180; 
		distance_e = -distance_e;		
	}
	else if(theta_e > 90){  
		theta_e -= 180;		
		distance_e = -distance_e;
	}

	vL = (int)(5.*(100.0/1000.0*distance_e+40.0/90.0*theta_e));
	vR = (int)(5.*(100.0/1000.0*distance_e-40.0/90.0*theta_e));

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
	if(m_nGameArea==GAME_RIGHT)
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
		ball.position = bal.position;
		//ball.oldPosition = bal.oldPosition;

		home1.position = ho1.position;
		home1.angle = ho1.angle;

		home2.position = ho2.position;
		home2.angle = ho2.angle;

		home3.position = ho3.position;
		home3.angle = ho3.angle;

		home4.position = ho4.position;
		home4.angle = ho4.angle;

		home5.position = ho5.position;
		home5.angle = ho5.angle;

		home6.position = ho6.position;
		home6.angle = ho6.angle;

		home7.position = ho7.position;
		home7.angle = ho7.angle;

		home8.position = ho8.position;
		home8.angle = ho8.angle;

		home9.position = ho9.position;
		home9.angle = ho9.angle;

		home10.position = ho10.position;
		home10.angle = ho10.angle;

		hgoalie.position = hgo.position;
		hgoalie.angle = hgo.angle;

		opponent.position1 = opp.position1;
		opponent.position2 = opp.position2;
		opponent.position3 = opp.position3;
		opponent.position4 = opp.position4;
		opponent.position5 = opp.position5;
		opponent.position6 = opp.position6;
		opponent.position7 = opp.position7;
		opponent.position8 = opp.position8;
		opponent.position9 = opp.position9;
		opponent.position10 = opp.position10;
		opponent.position11 = opp.position11;
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
		Position(which, CPoint(target.x,target.y));
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

	if (desired_angle != robot->angle)
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
			Position(which, ball.position);
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



int CStrategySystem::dribbleTo(int which, CPoint target)
{
	// TODO: Add your implementation code here.
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
	int range = 10;

	//If ball outside range
	if (state == 0)
	{
		if (computeDistance(robot->position, ball.position) > range)
		{
			if (robot->angle == computeAngle(robot->position, ball.position))
			{
				Velocity(which, -127, -127);
			}
			else
			{
				faceBall(which);
			}
		}
		else
		{
			Stop(which);
			state = 1;
		}
	}

	//If in correct quadrant
	if (state == 1)
	{
		
	}

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
