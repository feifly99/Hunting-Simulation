#include "HuntingSimulation.h"
#pragma warning(disable:6011)
//Initializing Functions:
void initializeTargetUSV(OUTPTR PTUSV* Tusv, IN Point position, IN double normalVelocity, IN double escapingVelocity, IN double alertingRadius, IN double Sradius, IN double movableAngleRange, IN double headDirection)
{
	*Tusv = (PTUSV)malloc(sizeof(TUSV));
	(*Tusv)->pos.x = position.x;
	(*Tusv)->pos.y = position.y;
	(*Tusv)->normalVelocity = normalVelocity;
	(*Tusv)->escapingVelocity = escapingVelocity;
	(*Tusv)->alertingRadius = alertingRadius;
	(*Tusv)->safetyRadius = Sradius;
	(*Tusv)->movableAngleRange = movableAngleRange;
	(*Tusv)->headDirection = headDirection;
	return;
}
void initializeHunterUsv(OUTPTR PHUSV* Husv, IN Point position, IN double velocity, IN double movableAngleRange, IN double headDirection)
{
	*Husv = (PHUSV)malloc(sizeof(HUSV));
	(*Husv)->pos.x = position.x;
	(*Husv)->pos.y = position.y;
	(*Husv)->velocity = velocity;
	(*Husv)->movableAngleRange = movableAngleRange;
	(*Husv)->headDirection = headDirection;
	(*Husv)->evaluationIndicator.nearAngleDiffer = 0.0;
	(*Husv)->evaluationIndicator.idealDistanceDiffer = 0.0;
	(*Husv)->evaluationIndicator.headDirectionDiffer = 0.0;
	(*Husv)->evaluationIndicator.idealAngleDiffer = 0.0;
	(*Husv)->hunterListEntry.Flink = NULL;
	(*Husv)->hunterListEntry.Blink = NULL;
	return;
}
void buildCycleList(IN_OUT PHUSV* hunter)
{
	for (size_t j = 0; j < huntersNum - 1; j++)
	{
		hunter[j]->hunterListEntry.Flink = &hunter[j + 1]->hunterListEntry;
	}
	hunter[huntersNum - 1]->hunterListEntry.Flink = &hunter[0]->hunterListEntry;
	for (size_t j = huntersNum - 1; j > 0; j--)
	{
		hunter[j]->hunterListEntry.Blink = &hunter[j - 1]->hunterListEntry;
	}
	hunter[0]->hunterListEntry.Blink = &hunter[huntersNum - 1]->hunterListEntry;
	return;
}
void showTarget(IN PTUSV target)
{
	printf("Target has position: (%.3f, %.3f), normalVelocity: %.3f, escapingVelocity: %.3f, dangerousRadius: %.3f, SafetyRadius: %.3f\n", target->pos.x, target->pos.y, target->normalVelocity, target->escapingVelocity, target->alertingRadius, target->safetyRadius);
	return;
}
void showHunter(IN PHUSV hunter)
{
	printf("This->hunter has position: (%.3f, %.3f) with velocity: %.3f\n", hunter->pos.x, hunter->pos.y, hunter->velocity);
	return;
}
void checkCycleListF(IN PPHUSV hunter)
{
	if (hunter)
	{
		PLIST_ENTRY temp = &((*hunter)->hunterListEntry);
		showHunter(CONTAINING_RECORD(temp, HUSV, hunterListEntry));
		while (temp->Flink != &((*hunter)->hunterListEntry))
		{
			temp = temp->Flink;
			showHunter(CONTAINING_RECORD(temp, HUSV, hunterListEntry));
		}
	}
}
void checkCycleListB(IN PPHUSV hunter)
{
	if (hunter)
	{
		PLIST_ENTRY temp = &((*hunter)->hunterListEntry);
		showHunter(CONTAINING_RECORD(temp, HUSV, hunterListEntry));
		while (temp->Blink != &((*hunter)->hunterListEntry))
		{
			temp = temp->Blink;
			showHunter(CONTAINING_RECORD(temp, HUSV, hunterListEntry));
		}
	}
}
void checkCycleList(IN PPHUSV hunter)
{
	checkCycleListF(hunter);
}
void checkEI(IN EI pEvaluationIndicator)
{
	//ckFloatValue(pevaluationIndicator.angleDiffer); 此成员隐藏，不对外暴露
	ckFloatValue(pEvaluationIndicator.idealDistanceDiffer);
	ckFloatValue(pEvaluationIndicator.headDirectionDiffer);
	ckFloatValue(pEvaluationIndicator.idealAngleDiffer);
	return;
}
//Basic Mathematical Functions:
double mabs(IN double x)
{
	return x >= 0 ? x : -x;
}
double msign(IN double x)
{
	return x >= 0 ? 1 : -1;
}
double mmin(IN double x, IN double y)
{
	return x >= y ? y : x;
}
double mmax(IN double x, IN double y)
{
	return x >= y ? x : y;
}
double cosa(IN double x)
{
	return cos(cvtAngle2Rad(x));
}
double sina(IN double x)
{
	return sin(cvtAngle2Rad(x));
}
double closeBetter(IN double indicator, IN double numberCloseTo)
{
	const double epsilon = 1e-6;
	return 1.0 / (1.0 + mabs(indicator - numberCloseTo) + epsilon);
}
BOOL floatSame10(IN double x, IN double y)
{
	if (mabs(x - y) < 0.01)
		return 1;
	else
		return 0;
}
BOOL floatSame100(IN double x, IN double y)
{
	if (mabs(x - y) < 0.001)
		return 1;
	else
		return 0;
}
BOOL floatSame1000(IN double x, IN double y)
{
	if (mabs(x - y) < 0.0001)
		return 1;
	else
		return 0;
}
//Points and Vectors Functions:
double cvtAngle2Rad(IN double x)
{
	return x * pi / 180;
}
double cvtRad2Angle(IN double x)
{
	return x * 180 / pi;
}
double calculate_V_model(IN PV v)
{
	return sqrt(v->a * v->a + v->b * v->b);
}
double calculate_V$V_mul(IN PV v1, IN PV v2)
{
	return v1->a * v2->a + v1->b * v2->b;
}
double calculate_V$V_rad_abs(IN PV v1, IN PV v2)
{
	return acos(mabs(calculate_V$V_mul(v1, v2) / (calculate_V_model(v1) * calculate_V_model(v2))));
}
double calculate_V$V_angle_abs(IN PV v1, IN PV v2)
{
	return cvtRad2Angle(calculate_V$V_rad_abs(v1, v2));
}
double calculate_OX$V_signedAngle(IN PV v)
{
	V OX = { 1, 0 };
	if (v->a == 0 && v->b == 0)
	{
		printf("Fatal Error: zero vector input, program exiting...\n");
		exit(0);
		return 0;
	}
	else
	{
		if (v->a == 0)
		{
			if (v->b > 0)
			{
				return 90;
			}
			else
			{
				return -90;
			}
		}
		else if (v->b == 0)
		{
			if (v->a > 0)
			{
				return 0;
			}
			else
			{
				return 180;
			}
		}
		else
		{
			if (v->a > 0 && v->b > 0)
			{
				return calculate_V$V_angle_abs(&OX, v);
			}
			else if (v->a > 0 && v->b < 0)
			{
				return calculate_V$V_angle_abs(&OX, v) * (-1);
			}
			else if (v->a < 0 && v->b > 0)
			{
				return 180 - calculate_V$V_angle_abs(&OX, v);
			}
			else
			{
				return calculate_V$V_angle_abs(&OX, v) - 180;
			}
		}
	}
}
double calculate_OX$V_unsignedAngle(IN PV v)
{
	return calculate_OX$V_signedAngle(v) < 0 ? 360 - mabs(calculate_OX$V_signedAngle(v)) : calculate_OX$V_signedAngle(v);
}
void zoomV(OUT V* v, IN double factor)
{
	(*v).a *= factor;
	(*v).b *= factor;
	return;
}
void calculateTwoVertical_V(IN V v, OUT V* temp1, OUT V* temp2)
{
	(*temp1).a = -v.b;
	(*temp1).b = v.a;
	(*temp2).a = -(*temp1).a;
	(*temp2).b = -(*temp1).b;
	return;
}
void normalizeV(OUT V* v)
{
	zoomV(v, (1 / calculate_V_model(v)));
}
double calculate_P$P$P_unsignedAngle(IN Point A, IN Point B, IN Point C)
{
	V BA = { A.x - B.x, A.y - B.y };
	V BC = { C.x - B.x, C.y - B.y };
	double XBC = calculate_OX$V_unsignedAngle(&BC);
	double XBA = calculate_OX$V_unsignedAngle(&BA);
	if (XBC == XBA)
	{
		return 0;
	}
	else
	{
		if (XBC > XBA)
		{
			return XBC - XBA;
		}
		else
		{
			return 360 - (XBA - XBC);
		}
	}
} 
//calculate_P$P$P_unsignedAngle: 若输入A, B ,C，那么返回从A开始逆时针旋转到BC边的角度.
double calculate_P$P_distance(IN Point A, IN Point B)
{
	V v = { A.x - B.x, A.y - B.y };
	return calculate_V_model(&v);
}
//Updating Functions
void updateTargetPosByNewPoint(IN_OUT PTUSV* target, IN Point newPoint)
{
	V aheadVector = { newPoint.x - (*target)->pos.x, newPoint.y - (*target)->pos.y };
	(*target)->pos.x = newPoint.x;
	(*target)->pos.y = newPoint.y;
	(*target)->headDirection = calculate_OX$V_unsignedAngle(&aheadVector);
	return;
}
void updateHunterPosByNewPoint(IN_OUT PHUSV* hunter, IN Point newPoint)
{
	(*hunter)->pos.x = newPoint.x;
	(*hunter)->pos.y = newPoint.y;
	return;
}
void updateTargetPosByMovingDistanceAndUnsignedAngle(IN_OUT PTUSV* target, IN double movingDistance, IN double angle)
{
	(*target)->pos.x += movingDistance * cos(cvtAngle2Rad(angle));
	(*target)->pos.y += movingDistance * sin(cvtAngle2Rad(angle));
	return;
}
void updateHunterPosByMovingDistanceAndUnsignedAngle(IN_OUT PHUSV* hunter, IN double movingDistance, IN double angle)
{
	(*hunter)->pos.x += movingDistance * cos(cvtAngle2Rad(angle));
	(*hunter)->pos.y += movingDistance * sin(cvtAngle2Rad(angle));
	return;
}
void movingNormalTargetRandomly(PTUSV* target)
{
	int flag = rand() % 2;
	int angle_x = (flag == 0) ? (rand() % 70 - rand() % 44) : (-1) * (rand() % 70 - rand() % 44);
	int angle_y = (flag == 0) ? (rand() % 110 - rand() % 34) : (-1) * (rand() % 110 - rand() % 34);
	(*target)->pos.x += targetNormalVelocity * flag * cosa(angle_x);
	(*target)->pos.y += targetNormalVelocity * flag * sina(angle_y);
	return;
}
//Geometry Functions:
void getTwoCuttingPointOnCircle(IN Point center, IN double radius, IN Point externals, OUT Point* res1, OUT Point* res2)
{
	V temp = { center.x - externals.x, center.y - externals.y };
	double step = 0.001; Point point = { 0 };
	if (calculate_V_model(&temp) < radius)
	{
		printf("Fatal Error: External point in this circle, can't get cutting point.\n");
		return;
	}
	else if (calculate_V_model(&temp) == radius)
	{
		printf("Warning: External point at the edge of this circle, returns two same points.\n");
		(*res1).x = externals.x;
		(*res1).y = externals.y;
		(*res2).x = (*res1).x;
		(*res2).y = (*res1).y;
		return;
	}
	else
	{
		for (size_t j = 0; j < (size_t)(360 / step); j++)
		{
			point.x = center.x + radius * cos(cvtAngle2Rad(j * step));
			point.y = center.y + radius * sin(cvtAngle2Rad(j * step));
			if (floatSame100(calculate_P$P$P_unsignedAngle(center, point, externals), 90) == 1)
			{
				(*res1).x = point.x;
				(*res1).y = point.y;
				break;
			}
		}
		for (size_t j = 0; j < (size_t)(360 / step); j++)
		{
			point.x = center.x + radius * cos(cvtAngle2Rad(j * step));
			point.y = center.y + radius * sin(cvtAngle2Rad(j * step));
			if (floatSame100(calculate_P$P$P_unsignedAngle(center, point, externals), 270) == 1)
			{
				(*res2).x = point.x;
				(*res2).y = point.y;
				break;
			}
		}
	}
}
BOOL checkPointIsSafe(IN Point loc, IN PTUSV target)
{
	V temp = { loc.x - target->pos.x, loc.y - target->pos.y };
	return calculate_V_model(&temp) - RtN > target->safetyRadius + RtN + Rh;
}
//Evaluation Functions:
void getHuntersFourEIByHuntersLocAndHeadDirection(IN PHUSV hunter, IN PTUSV target)
{
	PHUSV curr = hunter;
	PHUSV next = CONTAINING_RECORD(curr->hunterListEntry.Flink, HUSV, hunterListEntry);
	PHUSV prev = CONTAINING_RECORD(curr->hunterListEntry.Blink, HUSV, hunterListEntry);
	hunter->evaluationIndicator.nearAngleDiffer = calculate_P$P$P_unsignedAngle(curr->pos, target->pos, next->pos) - calculate_P$P$P_unsignedAngle(prev->pos, target->pos, curr->pos);
	hunter->evaluationIndicator.headDirectionDiffer = mabs(curr->headDirection - target->headDirection);
	hunter->evaluationIndicator.idealAngleDiffer = mabs(calculate_P$P$P_unsignedAngle(curr->pos, target->pos, next->pos)) - ((double)360 / (double)huntersNum);
	hunter->evaluationIndicator.idealDistanceDiffer = calculate_P$P_distance(curr->pos, target->pos) - target->safetyRadius - 2 * RtN;
	return;
}
double getRewardPointsForHunterByHuntersEI(IN PHUSV hunter)
{
	EI abs_hunterEI = { 0 };

	abs_hunterEI.idealAngleDiffer = mabs(hunter->evaluationIndicator.idealAngleDiffer);
	abs_hunterEI.idealDistanceDiffer = mabs(hunter->evaluationIndicator.idealDistanceDiffer);

	double J = 0.0;

	if (hunter->evaluationIndicator.idealDistanceDiffer <= 0)
	{
		J = -1000;
	}
	else
	{
		J = w_idealAngleDiffer * closeBetter(abs_hunterEI.idealAngleDiffer, 2) + w_distanceDiffer * closeBetter(abs_hunterEI.idealDistanceDiffer, 0.5);
	}

	return J;
}
void changingHunterInfomationByRewardFunction(IN_OUT PHUSV* hunter, IN PTUSV target)
{
	Point _NEVER_USED_ITEM_nextHunterIdealLoc = { 0 };
	PHUSV nextHunterIdealInfo = NULL;
	initializeHunterUsv(&nextHunterIdealInfo, _NEVER_USED_ITEM_nextHunterIdealLoc, 0, 0, 0);
	memcpy(nextHunterIdealInfo, *hunter, sizeof(HUSV));

	double newMaxRewardPoints = -99999999.0;

	int global_j = 0;
	int global_i = 0;

	for (int j = 0; j < 180; j++)
	{
		for (int i = 0; i < 20; i++)
		{
			nextHunterIdealInfo->pos.x = (*hunter)->pos.x + (i * ((double)huntersVelocity / (double)20.0)) * cosa(j * 2);
			nextHunterIdealInfo->pos.y = (*hunter)->pos.y + (i * ((double)huntersVelocity / (double)20.0)) * sina(j * 2);
			getHuntersFourEIByHuntersLocAndHeadDirection(nextHunterIdealInfo, target);
			if (getRewardPointsForHunterByHuntersEI(nextHunterIdealInfo) > newMaxRewardPoints)
			{
				newMaxRewardPoints = getRewardPointsForHunterByHuntersEI(nextHunterIdealInfo);
				global_j = j;
				global_i = i;
			}
		}
	}
	(*hunter)->pos.x += (global_i * ((double)huntersVelocity / (double)20.0)) * cosa(global_j * 2);
	(*hunter)->pos.y += (global_i * ((double)huntersVelocity / (double)20.0)) * sina(global_j * 2);
	getHuntersFourEIByHuntersLocAndHeadDirection(*hunter, target);
	free(nextHunterIdealInfo);
	nextHunterIdealInfo = NULL;
	return;
}
BOOLEAN isSurroundingSuccess(PHUSV hunter[huntersNum], PTUSV target)
{
	double d[huntersNum] = { 0 };
	double a[huntersNum] = { 0 };
	for (size_t j = 0; j < huntersNum; j++)
	{
		d[j] = hunter[j]->evaluationIndicator.idealDistanceDiffer;
		a[j] = hunter[j]->evaluationIndicator.idealAngleDiffer;
	}
	for (size_t j = 0; j < huntersNum; j++)
	{
		if (d[j] <= 0 || d[j] >= 3)
		{
			return 0;
		}
		if (mabs(a[j]) >= 2)
		{
			return 0;
		}
	}
	
	return 1;
	
}
//C-Python Interaction Functions:
void makePythonFile(FILE* fp, PHUSV hunter[3], PTUSV target) 
{
	Point h[3] = { {0} };

	for (int j = 0; j < huntersNum; j++)
	{
		h[j].x = hunter[j]->pos.x;
		h[j].y = hunter[j]->pos.y;
	}

	Point t = { 0 };

	t.x = target->pos.x;
	t.y = target->pos.y;

	if (fp == NULL) 
	{
		fprintf(stderr, "File pointer is NULL.\n");
		return;
	}

	for (int i = 0; i < 3; i++) 
	{
		fprintf(fp, "hunter %d: x = %.3f, hunter %d: y = %.3f\n", i, h[i].x, i, h[i].y);
	}

	fprintf(fp, "target: x = %.3f, target: y = %.3f\n", t.x, t.y);

	fprintf(fp, "QAQ\n");

	return;
}