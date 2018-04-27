#include "general.h"
#include <algorithm>

#ifndef _INSIDE_VISUAL_CPP_STRATEGYSYSTEM
#define _INSIDE_VISUAL_CPP_STRATEGYSYSTEM

class Positional {
public:
	double distance;
	CPoint coor1;
	CPoint coor2;


};

class CStrategySystem : public CObject
{
    DECLARE_DYNAMIC(CStrategySystem)
public:
	void Action();
    ~CStrategySystem();
	CStrategySystem(int id);
    void ReceiveData(Robot1 bal,Robot2 ho1,Robot2 ho2,Robot2 ho3,Robot2 ho4,
		             Robot2 ho5,Robot2 ho6,Robot2 ho7,Robot2 ho8,Robot2 ho9,
					 Robot2 ho10,Robot2 hgo,Robot3 opp);

#ifdef _DEBUG
#endif // _DEBUG
private:
	void Stop(int which);
	void Position(int which, CPoint point, int max_v);
	void KickOff();
	void Velocity(int which, int vL, int vR);
	void Angle(int which, int desired_angle);
	void Goalie(int which);
	CRect       boundRect;     
	int m_nStrategy;
	int         m_OurTeam;  
	int         m_nGameArea;    
	double      ShootLen;                  
	int         nShoot;                    
	int			nSweep;
	int			nKick2;
	CPoint      ShootVar;                  
	CPoint      KickVar; 
	double      KickLen;  
	CCommand C_Home3;
	CCommand C_Home2;
	CCommand C_Home1;
	CCommand C_Home4;
	CCommand C_Home5;
	CCommand C_Home6;
	CCommand C_Home7;
	CCommand C_Home8;
	CCommand C_Home9;
	CCommand C_Home10;
	CCommand C_Home11;
	Robot1 ball;
	Robot2 home1,home2,home3,home4,home5,home6,home7,home8,home9,home10,hgoalie;
	Robot3 opponent;
	void Think();
	void NormalGame4();
	void NormalGame3();
	void Defense1();
	void Attack1();
public:
	int command[35]; 
	
private:
	void faceBall(int which);
	// Kick the ball towards a certain point.
	void kickBall(int which, int velocity);
	int computeDistance(CPoint source, CPoint target);
	int computeAngle(CPoint source, CPoint target);
	// Move the ball to a specific location.
	//int dribbleBall(int which, CPoint destination);
	// //Return relative direction as per definition
	//int relativeDirection(CPoint source, CPoint target);
	//int facePoint(int which, CPoint target);
	int faceToPoint(int which, CPoint target);
	int reportQuadrant(CPoint source, CPoint centre);
	// Positioning with collision and wall avoidance
	int smartPosition(int which, CPoint target, int max_velocity);
	Positional reportEnemy(int which);
};

#endif // _INSIDE_VISUAL_CPP_STRATEGYSYSTEM
