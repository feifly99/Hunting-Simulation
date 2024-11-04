#include "HuntingSimulation.h"
#pragma warning(disable: 6011)
#pragma warning(disable: 6387)
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
void initializeHunterUsv(OUTPTR PHUSV* Husv, IN Point position, IN double velocity, IN double movableAngleRange, IN double headDirection, IN size_t originalIndex)
{
	*Husv = (PHUSV)malloc(sizeof(HUSV));
	(*Husv)->pos.x = position.x;
	(*Husv)->pos.y = position.y;
	(*Husv)->originalIndex = originalIndex;
	(*Husv)->currentIndex = 0;
	(*Husv)->velocity = velocity;
	(*Husv)->movableAngleRange = movableAngleRange;
	(*Husv)->headDirection = headDirection;
	(*Husv)->isInsightByTarget = 0;
	(*Husv)->evaluationIndicator.nearAngleDiffer = 0.0;
	(*Husv)->evaluationIndicator.idealDistanceDiffer = 0.0;
	(*Husv)->evaluationIndicator.headDirectionDiffer = 0.0;
	(*Husv)->evaluationIndicator.idealAngleDiffer = 0.0;
	(*Husv)->hunterListEntry.Flink = NULL;
	(*Husv)->hunterListEntry.Blink = NULL;
	return;
}
void buildCycleListByHuntersOriginalIndex(IN_OUT PHUSV* hunter)
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
void checkHuntersCycleListF(IN PPHUSV hunter)
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
void checkHuntersCycleListB(IN PPHUSV hunter)
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
void checkHuntersCycleList(IN PPHUSV hunter)
{
	checkHuntersCycleListF(hunter);
}
void checkHuntersEI(IN EI pEvaluationIndicator)
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
void sortArray(IN double* array, IN size_t size)
{
	for (size_t i = 0; i < size - 1; i++)
	{
		for (size_t j = 0; j < size - 1 - i; j++)
		{
			if (array[j] > array[j + 1])
			{
				double temp = array[j];
				array[j] = array[j + 1];
				array[j + 1] = temp;
			}
		}
	}
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
//calculate_OX$V_signedAngle: 根据输入向量V指向的区域判断：如果V指向第一、第二象限（上面），那么返回OX轴逆时针到V的角度，为正数【0, 180】; 如果V指向第三、第四象限（下面），那么返回OX轴顺时针到V的角度，为负数【-180,0】.
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
//calculate_OX$V_unsignedAngle: 返回OX轴逆时针旋转到V的角度，为正数【0, 360】.
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
void mergeVector(IN PV vectorArray, IN size_t vectorArraySize, OUT V* vectorOut)
{
	fors(
		vectorArraySize,
		(*vectorOut).a += vectorArray[j].a;
		(*vectorOut).b += vectorArray[j].b;
	);
	return;
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
void movingNormalTargetRandomly(OUT PTUSV* target)
{
	int flag = rand() % 2;
	int angle_x = (flag == 0) ? (rand() % 70 - rand() % 24) : (-1) * (rand() % 70 - rand() % 24);
	int angle_y = (flag == 0) ? (rand() % 110 - rand() % 14) : (-1) * (rand() % 110 - rand() % 14);
	if(flag == 0)
	{
		(*target)->pos.x += RtN * (-1) * cosa((double)angle_x);
		(*target)->pos.y += RtN * (-1) * sina((double)angle_y);
	}
	else
	{
		(*target)->pos.x += RtN * cosa((double)angle_x);
		(*target)->pos.y += RtN * sina((double)angle_y);
	}
	/*(*target)->pos.x += RtN * cosa((double)240.0);
	(*target)->pos.y += RtN * sina((double)240.0);*/
	return;
}
void movingNormalTargetStably(OUT PTUSV* target)
{
	double angle1 = 60.0;
	double angle2 = 120.0;
	double angle3 = 240.0;
	double angle4 = 330.0;
	double angle = angle1;
	(*target)->pos.x += RtN * cosa((double)angle);
	(*target)->pos.y += RtN * sina((double)angle);
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
//Surroundding Simulation Functions:
void _SURROUNDING_getHuntersFourEIByHuntersLocAndHeadDirection(IN_OUT PHUSV* hunter, IN PTUSV target)
{
	//mostSafetyRadius是目标的安全圈半径加上一个采样时间内目标能够移动的长度；
	//RtN和Rh的系数必须大于等于1才能确保最糟糕的情况下（捕食者和目标相向而行）目标也不会发现捕食者.
	double mostSafetyRadius = target->safetyRadius + 1 * RtN + 1 * Rh;
	PHUSV curr = *hunter;
	PHUSV next = CONTAINING_RECORD(curr->hunterListEntry.Flink, HUSV, hunterListEntry);
	PHUSV prev = CONTAINING_RECORD(curr->hunterListEntry.Blink, HUSV, hunterListEntry);
	(*hunter)->evaluationIndicator.nearAngleDiffer = calculate_P$P$P_unsignedAngle(curr->pos, target->pos, next->pos) - calculate_P$P$P_unsignedAngle(prev->pos, target->pos, curr->pos);
	(*hunter)->evaluationIndicator.headDirectionDiffer = mabs(curr->headDirection - target->headDirection);
	(*hunter)->evaluationIndicator.idealAngleDiffer = mabs(calculate_P$P$P_unsignedAngle(curr->pos, target->pos, next->pos)) - ((double)360 / (double)huntersNum);
	(*hunter)->evaluationIndicator.idealDistanceDiffer = calculate_P$P_distance(curr->pos, target->pos) - mostSafetyRadius;
	return;
}
double _SURROUNDING_getRewardPointsForHunterByHuntersEI(IN PHUSV hunter)
{
	EI abs_hunterEI = { 0 };

	abs_hunterEI.idealAngleDiffer = mabs(hunter->evaluationIndicator.idealAngleDiffer);
	abs_hunterEI.idealDistanceDiffer = mabs(hunter->evaluationIndicator.idealDistanceDiffer);

	double J = 0.0;
	//误差线episilon_angle:以目标为中心的合围图形中心角越接近【360 / n + episilon_angle】越好
	double episilon_angle = 3.0;
	//误差线episilon，捕食者和目标最大安全圈的距离（mostSafetyRadius）越接近0.1越好；
	//注：这里只能接受mostSafetyRadius + 0.1, 如果捕食者进入了最大安全圈内部直接拒绝进入.
	double episilon_distance = 0.8;

	if (hunter->evaluationIndicator.idealDistanceDiffer <= 0)
	{
		J = -100000.0;
	}
	else
	{
		J = w_idealAngleDiffer * closeBetter(abs_hunterEI.idealAngleDiffer, episilon_angle) + w_distanceDiffer * closeBetter(abs_hunterEI.idealDistanceDiffer, episilon_distance);
	}
	
	return J;
}
void _SURROUNDING_changingHunterInfomationByRewardFunction(IN_OUT PHUSV* hunter, IN PTUSV target)
{
	double currentHunterValidRegionAngle = calculate_P$P$P_unsignedAngle(CONTAINING_RECORD((*hunter)->hunterListEntry.Blink, HUSV, hunterListEntry)->pos, target->pos, CONTAINING_RECORD((*hunter)->hunterListEntry.Flink, HUSV, hunterListEntry)->pos);
	Point nextHunterIdealLoc = { 0 };
	PHUSV nextHunterIdealInfo = NULL;
	nextHunterIdealInfo = (PHUSV)malloc(sizeof(HUSV));
	memcpy(nextHunterIdealInfo, *hunter, sizeof(HUSV));
	double newMaxRewardPoints = -9999.0;
	typedef struct _globalIndex
	{
		int i;
		int j;
	}GI, *PGI;
	typedef struct _globalInfoStack
	{
		GI* globalIndex;
		PGI stackTopPointer;
		PGI stackButtomPointer;
	}GIS, *PGIS;
	PGIS stack = (PGIS)malloc(sizeof(GIS));
	stack->globalIndex = (GI*)malloc(STACK_MAX_SIZE * sizeof(GI));
	stack->stackTopPointer = stack->globalIndex;
	stack->stackButtomPointer = stack->globalIndex;
	for (int j = 0; j < 180; j++)
	{
		for (int i = 0; i <= 20; i++) //注意i从0开始，可能保持不动
		{
			nextHunterIdealInfo->pos.x = (*hunter)->pos.x + (i * ((double)huntersVelocity / (double)20.0)) * cosa(j * 2);
			nextHunterIdealInfo->pos.y = (*hunter)->pos.y + (i * ((double)huntersVelocity / (double)20.0)) * sina(j * 2);
			_SURROUNDING_getHuntersFourEIByHuntersLocAndHeadDirection(&nextHunterIdealInfo, target);
			if (_SURROUNDING_getRewardPointsForHunterByHuntersEI(nextHunterIdealInfo) > newMaxRewardPoints)
			{
				newMaxRewardPoints = _SURROUNDING_getRewardPointsForHunterByHuntersEI(nextHunterIdealInfo);
				stack->stackTopPointer->i = i;
				stack->stackTopPointer->j = j;
				stack->stackTopPointer++;
			}
		}
	}
	double neighborDistanceDifferOfNextLoc[2] = { 0.0 };
	double huntersSafetyNeighborDistance = 1.1 * Rh;
	stack->stackTopPointer--;
	while(stack->stackTopPointer != stack->stackButtomPointer)
	{
		nextHunterIdealLoc.x = (*hunter)->pos.x + ((double)(stack->stackTopPointer->i) * ((double)huntersVelocity / (double)20.0)) * cosa((double)(stack->stackTopPointer->j * 2.0));
		nextHunterIdealLoc.y = (*hunter)->pos.y + ((double)(stack->stackTopPointer->i) * ((double)huntersVelocity / (double)20.0)) * sina((double)(stack->stackTopPointer->j * 2.0));
		nextHunterIdealInfo->pos.x = nextHunterIdealLoc.x;
		nextHunterIdealInfo->pos.y = nextHunterIdealLoc.y;
		neighborDistanceDifferOfNextLoc[0] = calculate_P$P_distance(CONTAINING_RECORD(nextHunterIdealInfo->hunterListEntry.Flink, HUSV, hunterListEntry)->pos, nextHunterIdealInfo->pos);
		neighborDistanceDifferOfNextLoc[1] = calculate_P$P_distance(CONTAINING_RECORD(nextHunterIdealInfo->hunterListEntry.Blink, HUSV, hunterListEntry)->pos, nextHunterIdealInfo->pos);
		if 
		(
			(neighborDistanceDifferOfNextLoc[0] <= huntersSafetyNeighborDistance || neighborDistanceDifferOfNextLoc[1] <= huntersSafetyNeighborDistance)
			||
			(
				calculate_P$P$P_unsignedAngle(CONTAINING_RECORD((*hunter)->hunterListEntry.Blink, HUSV, hunterListEntry)->pos, target->pos, nextHunterIdealInfo->pos)
				>=
				currentHunterValidRegionAngle
			)
			||
			(
				calculate_P$P$P_unsignedAngle(nextHunterIdealInfo->pos, target->pos, CONTAINING_RECORD((*hunter)->hunterListEntry.Flink, HUSV, hunterListEntry)->pos)
				>=
				currentHunterValidRegionAngle
			)
		)
		{
			stack->stackTopPointer--;
			if (stack->stackTopPointer == stack->stackButtomPointer)
			{
				printf("警告：最优全局i_j已经到达栈底.\n");
				nextHunterIdealLoc.x = (*hunter)->pos.x;
				nextHunterIdealLoc.y = (*hunter)->pos.y;
				break;
			}
			return;
		}
		else
		{
			break;
		}
	}
	updateHunterPosByNewPoint(hunter, nextHunterIdealLoc);
	_SURROUNDING_getHuntersFourEIByHuntersLocAndHeadDirection(hunter, target);
	free(stack->globalIndex);
	stack->globalIndex = NULL;
	free(stack);
	stack = NULL;
	free(nextHunterIdealInfo);
	nextHunterIdealInfo = NULL;
	return;
}
void _SURROUNDING_remakeHuntersCycleListByRelativeLocCompareToTarget(IN_OUT PHUSV (*hunter)[huntersNum], IN PTUSV target)
{
	double relativeAngles[huntersNum] = { 0.0 };
	V relativeVectors[huntersNum] = { {0, 0} };
	size_t k[huntersNum] = { 0 };
	fors(
		huntersNum,
		relativeVectors[j].a = ((*hunter)[j])->pos.x - target->pos.x;
		relativeVectors[j].b = ((*hunter)[j])->pos.y - target->pos.y;
	);
	fors(
		huntersNum,
		relativeAngles[j] = calculate_OX$V_unsignedAngle(&relativeVectors[j]);
	);
	//debug
	fors(
		huntersNum,
		ckFloatValue(relativeAngles[j]);
	);
	for (size_t j = 0; j < huntersNum - 1; j++)
	{
		for (size_t i = 0; i < huntersNum - 1 - j; i++)
		{
			if (relativeAngles[i] > relativeAngles[i + 1])
			{
				double temp = relativeAngles[i];
				relativeAngles[i] = relativeAngles[i + 1];
				relativeAngles[i + 1] = temp;
			}
		}
	}
	for (size_t j = 0; j < huntersNum; j++)
	{
		for (size_t i = 0; i < huntersNum; i++)
		{
			if (floatSame10(calculate_OX$V_unsignedAngle(&relativeVectors[i]), relativeAngles[j]))
			{
				k[j] = i;
				break;
			}
		}
	}
	fors(
		huntersNum,
		((*hunter)[j])->currentIndex = k[j];
	);
	buildCycleListByHuntersOriginalIndex(&(*hunter)[0]);
	//system("pause");
	return;
}
BOOLEAN isSurroundingSuccess(IN PHUSV hunter[huntersNum], IN PTUSV target)
{
	//maxTolerance这个值是临界值；
	//如果有任何一个捕食者到目标的距离大于mostSafetyRadius加上这个maxTolerance值，那么就视为不收敛；
	//maxTolerance值必须大于_SURROUNDING_getRewardPointsForHunterByHuntersEI中定义的episilon_distance；
	//否则永不收敛

	double maxAngleTolerance = 4.50;
	double maxDistanceTolerance = 1.20;
	

	double d[huntersNum] = { 0 };
	double a[huntersNum] = { 0 };

	for (size_t j = 0; j < huntersNum; j++)
	{
		d[j] = hunter[j]->evaluationIndicator.idealDistanceDiffer;
		a[j] = hunter[j]->evaluationIndicator.idealAngleDiffer;
	}
	for (size_t j = 0; j < huntersNum; j++)
	{
		if (d[j] <= 0.0 || d[j] >= maxDistanceTolerance)
		{
			return 0;
		}
		if (mabs(a[j]) >= maxAngleTolerance)
		{
			return 0;
		}
	}
	
	return 1;
	
}
//Hunters Hunting Method Simulation Functions:
void _HUNTER_goTowardsTargetDirectly(IN_OUT PHUSV* hunter, IN PTUSV target)
{
	double neighborSafeyDistance = 1.2 * Rh;
	Point currUsvPos = (*hunter)->pos;
	Point prevUsvPos = (CONTAINING_RECORD((*hunter)->hunterListEntry.Blink, HUSV, hunterListEntry))->pos;
	Point nextUsvPos = (CONTAINING_RECORD((*hunter)->hunterListEntry.Flink, HUSV, hunterListEntry))->pos;
	V nextIdealDirection = { target->pos.x - currUsvPos.x, target->pos.y - currUsvPos.y };
	double nextIdealMovingAngle = calculate_OX$V_unsignedAngle(&nextIdealDirection);
	Point nextIdealPoint = { currUsvPos.x + Rh * cosa(nextIdealMovingAngle),currUsvPos.y + Rh * sina(nextIdealMovingAngle) };
	if (calculate_P$P_distance(nextIdealPoint, prevUsvPos) >= neighborSafeyDistance && calculate_P$P_distance(nextIdealPoint, nextUsvPos) >= neighborSafeyDistance)
	{
		updateHunterPosByNewPoint(hunter, nextIdealPoint);
	}
	else
	{
		for (size_t j = 0; j < 360; j++)
		{
			for (size_t i = 1; i <= 20; i++)
			{
				nextIdealPoint.x = currUsvPos.x + i * ((double)(Rh) / (double)20.0) * cosa(nextIdealMovingAngle);
				nextIdealPoint.y = currUsvPos.y + i * ((double)(Rh) / (double)20.0) * sina(nextIdealMovingAngle);
				if (calculate_P$P_distance(nextIdealPoint, target->pos) >= calculate_P$P_distance(currUsvPos, target->pos) && calculate_P$P_distance(nextIdealPoint, prevUsvPos) >= neighborSafeyDistance && calculate_P$P_distance(nextIdealPoint, nextUsvPos) >= neighborSafeyDistance)
				{
					updateHunterPosByNewPoint(hunter, nextIdealPoint);
					break;
				}
				else
				{
					continue;
				}
			}
		}
	}	
	return;
}
//Target Escaping Simulation Functions:
void _TARGET_escapingAngleByVectorMethod(IN PHUSV hunter[huntersNum], IN PTUSV* target)
{
	//记录每一个目标-捕食者分支向量：
	V movingVectorEachDirection[huntersNum] = { {0} };
	//记录最终要移动的方向：
	V ultimateMovingDirection = { 0 };
	//【静态变量】：记录【上一次】移动的方向角度：
	//只要曾经触发过快速逃逸，那么记忆上一次的逃逸角度；
	//如果还没挣脱开包围，那么更新此记忆；
	//如果挣脱了包围（所有isInsightByTarget都是零），那么此值为上一次记忆的角度：
	static double ultimateMovingAngle = 0.0;
	//【静态变量】：记录是否被捕食者惊动【过】：
	//只要曾经有一个捕食者惊动过目标，此值在以后就恒为1.
	static BOOLEAN isAlreadyAlerted = 0;
	//零初始化
	fors(
		huntersNum,
		movingVectorEachDirection[j].a = 0.0;
		movingVectorEachDirection[j].b = 0.0;
	);
	//底层层面：记录movingVectorEachDirection是否全为0, 在后面用strncmp验证这个指标来看是否触发了向量逃逸：
	PVOID temp = malloc(huntersNum * sizeof(V));
	memcpy(temp, movingVectorEachDirection, huntersNum * sizeof(V));
	//循环判断各个捕食者和目标的距离：
	fors(
		huntersNum,
		if (calculate_P$P_distance(hunter[j]->pos, (*target)->pos) < (*target)->alertingRadius)
		{
			//如果某个捕食者进入了目标的警戒距离，那么把捕食者标志位置1；
			hunter[j]->isInsightByTarget = 1;
			//并且把【静态变量】isAlreadyAlerted【永远地】置为1，表达目标已经收到惊动，往后永远以高速行动.
			isAlreadyAlerted = 1;			
		}
		else
		{
			hunter[j]->isInsightByTarget = 0;
		}
	);
	if (!isAlreadyAlerted)
	{
		//如果没有受惊，那么无事发生，随便溜达：
		movingNormalTargetRandomly(target);
	}
	else
	{
		//此时受惊了，目标将会恒定以高速移动：
		fors(
			huntersNum,
			if (hunter[j]->isInsightByTarget == 1)
			{
				//只记录那些在受惊距离内的捕食者的向量，因为不在受惊距离内目标探测不到捕食者：
				movingVectorEachDirection[j].a = (*target)->pos.x - hunter[j]->pos.x;
				movingVectorEachDirection[j].b = (*target)->pos.y - hunter[j]->pos.y;
			}
		);
		if (strncmp((CONST CHAR*)temp, (CONST CHAR*)movingVectorEachDirection, huntersNum * sizeof(V)) != 0)
		{
			//字节比较，如果某一个向量非零，那么触发了向量逃逸方法，记录最终移动的方向和角度：
			fors(
				huntersNum,
				ultimateMovingDirection.a += movingVectorEachDirection[j].a;
				ultimateMovingDirection.b += movingVectorEachDirection[j].b;
			);
			//确定最终逃逸角度：
			ultimateMovingAngle = calculate_OX$V_unsignedAngle(&ultimateMovingDirection);
			updateTargetPosByMovingDistanceAndUnsignedAngle(target, RtE, ultimateMovingAngle);
			
		}
		else
		{
			//如果目标高速逃逸后远远甩开了捕食者，那么由于已经受惊，则：
			//1. 目标速度保持为高速；
			//2. 按照上次记忆的方向移动：
			updateTargetPosByMovingDistanceAndUnsignedAngle(target, RtE, ultimateMovingAngle);
			//printf("围捕失败!\n");
			
		}
	}
	//释放内存，清理悬挂指针：
	free(temp);
	temp = NULL;
	//返回
	return;
}
static double getRewardPointsForTargetByExtremeAngleDifferAndClosestHunterDistance(IN PHUSV hunter[huntersNum], IN PTUSV target)
{
	double J = 0.0;
	double closestDistance = 0.0;
	double extremeAngleDiffer = 0.0;
	double distance[huntersNum] = { 0.0 };
	V v[huntersNum] = { {0, 0} };
	double angle[huntersNum] = { 0.0 };
	double angle_differ[huntersNum] = { 0.0 };

	fors(
		huntersNum,
		distance[j] = calculate_P$P_distance(hunter[j]->pos, target->pos);
	);
	sortArray(distance, huntersNum);
	closestDistance = distance[0];

	fors(
		huntersNum,
		v[j].a = hunter[j]->pos.x - target->pos.x;
		v[j].b = hunter[j]->pos.y - target->pos.y;
	);

	fors(
		huntersNum,
		angle[j] = calculate_OX$V_unsignedAngle(&v[j]);
	);
	sortArray(angle, huntersNum);

	fors(
		huntersNum,
		if (j == 0)
		{
			angle_differ[j] = 360 - (angle[huntersNum - 1] - angle[0]);
		}
		else
		{
			angle_differ[j] = angle[j - 1] - angle[j];
		}
	);
	sortArray(angle_differ, huntersNum);
	extremeAngleDiffer = angle_differ[huntersNum - 1] - angle_differ[0];

	J = targetEscapingWeights_angle * closeBetter((1 / (extremeAngleDiffer + 0.00001)), 0.0) + targetEscapingWeights_distance * closeBetter((1 / (closestDistance + 0.00001)), 0.0);
	
	return J;
}
void _TARGET_escapingAngleByLearningMethod(IN PHUSV hunter[huntersNum], IN PTUSV* target)
{
	PTUSV tempTarget = (PTUSV)malloc(sizeof(TUSV));
	memcpy(tempTarget, *target, sizeof(TUSV));
	double currPoints = -999999.0;
	size_t global_i = 0;
	size_t global_j = 0;
	Point newPoint = { 0, 0 };

	for (size_t j = 0; j < 360; j++)
	{
		for (size_t i = 1; i <= 20; i++)
		{
			tempTarget->pos.x = (*target)->pos.x + ((double)i) * ((double)RtE / 20.0) * cosa((double)j);
			tempTarget->pos.y = (*target)->pos.y + ((double)i) * ((double)RtE / 20.0) * sina((double)j);
			if (getRewardPointsForTargetByExtremeAngleDifferAndClosestHunterDistance(hunter, tempTarget) >= currPoints)
			{
				currPoints = getRewardPointsForTargetByExtremeAngleDifferAndClosestHunterDistance(hunter, tempTarget);
				global_i = i;
				global_j = j;
			}
		}
	}

	newPoint.x = (*target)->pos.x + ((double)global_i) * ((double)RtE / 30.0) * cosa((double)global_j);
	newPoint.y = (*target)->pos.y + ((double)global_i) * ((double)RtE / 30.0) * sina((double)global_j);

	updateTargetPosByNewPoint(target, newPoint);

	free(tempTarget);
	tempTarget = NULL;
	return;
}
//Recycling Outcomes Judge Functions:
BOOLEAN isRecyclingSuccess(IN PHUSV hunter[huntersNum], IN PTUSV target)
{
	double distance[huntersNum] = { 0.0 };
	fors(
		huntersNum,
		distance[j] = calculate_P$P_distance(hunter[j]->pos, target->pos);
	);
	size_t idealRecyclingHuntersNum = 0x0;
	fors(
		huntersNum,
		if (distance[j] <= huntersRecyclingRadius)
		{
			idealRecyclingHuntersNum++;
		}
	);
	if (idealRecyclingHuntersNum >= (size_t)((huntersNum - 1) / 2) + 1)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
BOOLEAN isTerminatingRecycling(IN PHUSV hunter[huntersNum], IN PTUSV target)
{
	double maxTerminatingToleranceLimit = 5.0;
	static int remainTime = 8;
	V O_Target = { target->pos.x, target->pos.y };
	V v[huntersNum] = { {0, 0} };
	fors(
		huntersNum,
		v[j].a = target->pos.x - hunter[j]->pos.x;
		v[j].b = target->pos.y - hunter[j]->pos.y;
	);
	double angle[huntersNum] = { 0.0 };
	fors(
		huntersNum,
		angle[j] = calculate_OX$V_unsignedAngle(&v[j]);
	);
	sortArray(angle, huntersNum);
	fors(
		huntersNum,
		if (angle[j] - calculate_OX$V_unsignedAngle(&O_Target) > 10)
		{
			return 0;
		}
		else
		{
			continue;
		}
	);
	fors(
		remainTime--,
		return 0;
	);
	if (angle[huntersNum - 1] - angle[0] >= maxTerminatingToleranceLimit)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
//C-Python Interaction Functions:
void makePythonFile(IN FILE* fp, IN PHUSV hunter[huntersNum], IN PTUSV target)
{
	Point h[huntersNum] = { {0} };

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

	for (int i = 0; i < huntersNum; i++)
	{
		fprintf(fp, "hunter %d: x = %.3f, hunter %d: y = %.3f\n", i, h[i].x, i, h[i].y);
	}

	fprintf(fp, "target: x = %.3f, target: y = %.3f\n", t.x, t.y);

	fprintf(fp, "QAQ\n");

	return;
}
