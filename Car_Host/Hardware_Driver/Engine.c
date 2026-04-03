#include "Engine.h"

void Engine_Init(void)
{
	PWM_Init();
}
void Engine_SetAngle(float Angle)
{
	PWM_SetCompare7(Angle / 180 * 2000 + 500);
}
