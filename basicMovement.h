#pragma once
#include "StrategySystem.h"
class basicMovement :
	private CStrategySystem
{

public:
	basicMovement();
	~basicMovement();
private:
	// Make which face the ball.
	int faceBall(int which);
};

