#include "HuntingSimulation.h"
#pragma warning(disable: 6011)
#pragma warning(disable: 6387)
#define getFlink(j) CONTAINING_RECORD(hunter[j]->hunterListEntry.Flink, HUSV, hunterListEntry)
#define getBlink(j) CONTAINING_RECORD(hunter[j]->hunterListEntry.Blink, HUSV, hunterListEntry)
//Initializing Functions:
void initializeTargetUSV(OUTPTR PTUSV* Tusv, IN Point position, IN double normalVelocity, IN double safetyRadius)
{
	*Tusv = (PTUSV)malloc(sizeof(TUSV));
	(*Tusv)->pos.x = position.x;
	(*Tusv)->pos.y = position.y;
	(*Tusv)->normalVelocity = normalVelocity;
	(*Tusv)->safetyRadius = safetyRadius;
	return;
}
void initializeHunterUsv(OUTPTR PHUSV* Husv, IN Point position, IN double velocity, IN size_t hunterListIndex)
{
	*Husv = (PHUSV)malloc(sizeof(HUSV));
	(*Husv)->pos.x = position.x;
	(*Husv)->pos.y = position.y;
	(*Husv)->velocity = velocity;
	(*Husv)->hunterListIndex = hunterListIndex;
	(*Husv)->hunterListEntry.Flink = NULL;
	(*Husv)->hunterListEntry.Blink = NULL;
	return;
}
void initializeObstacles(OUTPTR POBSTACLE* obstacle, IN Point center, IN double size, IN double velocity, IN double movingDirection)
{
	*obstacle = (POBSTACLE)malloc(sizeof(OBSTACLE));
	RtlZeroMemory(*obstacle, sizeof(OBSTACLE));
	(*obstacle)->center.x = center.x;
	(*obstacle)->center.y = center.y;
	(*obstacle)->size = size;
	(*obstacle)->v = velocity;
	(*obstacle)->movingDirection = movingDirection;
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
	printf("Target has position: (%.3lf, %.3lf), normalVelocity: %.3lf, SafetyRadius: %.3lf\n", target->pos.x, target->pos.y, target->normalVelocity, target->safetyRadius);
	return;
}
void showHunter(IN PHUSV hunter)
{
	printf("This->hunter has position: (%.3lf, %.3lf) with velocity: %.3lf\n", hunter->pos.x, hunter->pos.y, hunter->velocity);
	return;
}
void showObstacle(IN POBSTACLE obstacle)
{
	printf("This->obstacle has position :(%.3lf, %.3lf) with velocity: %.3lf, direction: %.3lf, size: %.3lf\n", obstacle->center.x, obstacle->center.y, obstacle->v, obstacle->movingDirection, obstacle->size);
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
//Basic Mathematical Functions:
size_t selectUnsignedRandomNumberInRegion(size_t low, size_t high)
{
	if (low > high)
	{
		printf_red("Error: low must be less than or equal to high.\n");
		exit(EXIT_FAILURE);
	}
	return low + (rand() % (high - low + 1));
}
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
	return 1.0 / (mabs(indicator - numberCloseTo) + epsilon);
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
//calculate_P$P$P_unsignedAngle: 若输入A, B ,C，那么返回从A开始逆时针旋转到BC边的角度.
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
double calculate_P$P_distance(IN Point A, IN Point B)
{
	V v = { A.x - B.x, A.y - B.y };
	return calculate_V_model(&v);
}
//Updating Functions
void updateTargetPosByNewPoint(IN_OUT PTUSV* target, IN Point newPoint)
{
	(*target)->pos.x = newPoint.x;
	(*target)->pos.y = newPoint.y;
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
	if (flag == 0)
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
	double angle = angle3;
	(*target)->pos.x += RtN * cosa((double)angle);
	(*target)->pos.y += RtN * sina((double)angle);
	return;
}
void movingObstaclesRandomly(OUT POBSTACLE** obstacles)
{
	fors(
		obstaclesNum,
		(*obstacles)[j]->center.x += (*obstacles)[j]->v * sina((*obstacles)[j]->movingDirection);
		(*obstacles)[j]->center.y += (*obstacles)[j]->v * cosa((*obstacles)[j]->movingDirection);
	);
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
static double _SURROUNDING_getRewardPointsForHunter(IN PHUSV hunter, IN PTUSV target, IN POBSTACLE obstacles[obstaclesNum], IN int obstacleOneHotMark[obstaclesNum])
{
	size_t insightObstaclesCount = 0;
	fors(
		obstaclesNum,
		if (obstacleOneHotMark[j] == 1)
		{
			insightObstaclesCount++;
		}
	);
	if (insightObstaclesCount != 0)
	{
		double distanceNearby = 2.50;

		double angleNearby = 0.75;
		double obstacleDistanceNearby = 999999.0;

		double distanceDiffer = calculate_P$P_distance(hunter->pos, target->pos) - 1.1 * Rh - targetSafetyRadius;
		double angleDiffer = calculate_P$P$P_unsignedAngle(hunter->pos, target->pos, CONTAINING_RECORD(hunter->hunterListEntry.Flink, HUSV, hunterListEntry)->pos) - PHASE_ANGLE;
		double obstacleAverageDiffer = 0.0;

		double obstaclesDistanceSum = 0.0;
		fors(
			obstaclesNum,
			if (obstacleOneHotMark[j] == 1)
			{
				obstaclesDistanceSum += calculate_P$P_distance(hunter->pos, obstacles[j]->center);
			}
		);
		obstacleAverageDiffer = obstaclesDistanceSum * (double)insightObstaclesCount * (1 + closeBetter(calculate_P$P_distance(hunter->pos, target->pos), 999999.0));

		if (distanceDiffer <= 0.0)
		{
			return -10000.0;
		}

		double cbDistance = closeBetter(distanceDiffer, distanceNearby);
		double cbAngle = closeBetter(angleDiffer, angleNearby);
		double cbObstacleDistance = closeBetter(obstacleAverageDiffer, obstacleDistanceNearby);
		double cbSum = cbDistance + cbAngle + cbObstacleDistance;

		return (cbDistance * cbAngle + cbAngle * cbObstacleDistance + cbObstacleDistance * cbDistance) / cbSum;
	}
	else
	{
		double distanceNearby = 1.50;
		double angleNearby = 0.75;

		double distanceDiffer = calculate_P$P_distance(hunter->pos, target->pos) - 1.1 * Rh - targetSafetyRadius;
		double angleDiffer = calculate_P$P$P_unsignedAngle(hunter->pos, target->pos, CONTAINING_RECORD(hunter->hunterListEntry.Flink, HUSV, hunterListEntry)->pos) - PHASE_ANGLE;

		if (distanceDiffer <= 0.0)
		{
			return -10000.0;
		}

		double cbDistance = closeBetter(distanceDiffer, distanceNearby);
		double cbAngle = closeBetter(angleDiffer, angleNearby);

		return (cbDistance * cbAngle) / (cbDistance + cbAngle);
	}
}
void _SURROUNDING_changingHunterInfomationByRewardFunctionAndEnvironment(IN_OUT PHUSV* hunter, IN PTUSV target, IN POBSTACLE obstacles[obstaclesNum])
{
	int randoms = rand() % 100;
	static double alpha_0 = 0.0;
	static double alpha = 0.0;
	if ((*hunter)->hunterListIndex == 0 && alpha_0 == 0.0)
	{
		alpha_0 = 360.0 - calculate_P$P$P_unsignedAngle((*hunter)->pos, target->pos, getBlink((*hunter)->hunterListIndex)->pos);
#ifdef _DEBUG
		printf_red("sssssssssssssssssssssssssssssssssssssssssssssss\n");
		ckFloatValue(alpha_0);
#endif
	}
	if ((*hunter)->hunterListIndex == 0)
	{
		alpha = 360.0 - calculate_P$P$P_unsignedAngle((*hunter)->pos, target->pos, getBlink((*hunter)->hunterListIndex)->pos);
#ifdef _DEBUG
		printf_green("ttttttttttttttttttttttttttttttttttttttttttttt\n");
		ckFloatValue(alpha);
#endif
	}
	double wanderingFactor = 0.0;
	if (alpha >= 1.50 * ((360.0) / (double)huntersNum))
	{
		wanderingFactor = 15.0 * exp((double)(alpha / alpha_0));
	}
	else
	{
		wanderingFactor = 0.0;
	}
	int isMutatedThisTime = 0;
	if (randoms <= (int)wanderingFactor)
	{
		isMutatedThisTime = 1;
	}
	//B = Blink, F = Flink, T = target, Next = nextHunterIdealInfo || nextHunterIdealLoc
	//定义当前捕食者最大移动的角度，不能越过链表顺序(∠B_T_F)：
	double currentHunterValidRegionAngle = calculate_P$P$P_unsignedAngle(CONTAINING_RECORD((*hunter)->hunterListEntry.Blink, HUSV, hunterListEntry)->pos, target->pos, CONTAINING_RECORD((*hunter)->hunterListEntry.Flink, HUSV, hunterListEntry)->pos);
	//当前捕食者的下一个位置参数：
	Point nextHunterAvaliableLoc = { 0 };
	PHUSV nextHunterAvaliableInfo = NULL;
	nextHunterAvaliableInfo = (PHUSV)malloc(sizeof(HUSV));
	memcpy(nextHunterAvaliableInfo, *hunter, sizeof(HUSV));
	//决策数组：存储所有可移动位置的偏移：
	struct _offset
	{
		int distanceOffset;
		int angleOffset;
	}*OFFSETS;
	OFFSETS = (struct _offset*)malloc(OFFSETS_MAX_SIZE * sizeof(struct _offset));
	RtlZeroMemory(OFFSETS, OFFSETS_MAX_SIZE * sizeof(struct _offset));
	//决策角标，指向当前的决策
	size_t offsetIndex = 0;
	//定义捕食者视野内的障碍物的标记，一共有obstaclesNum个标记，对应位置为1表达该障碍物在捕食者视野内：
	int* obstacleInsightOneHotMark = (int*)malloc(obstaclesNum * sizeof(int));
	RtlZeroMemory(obstacleInsightOneHotMark, obstaclesNum * sizeof(int));
	//定义捕食者视野内的障碍物个数计数：
	size_t currentHunterInsightObstaclesCount = 0;
	//计算当前捕食者视野内的障碍物个数计数
	fors(
		obstaclesNum,
		if (calculate_P$P_distance((*hunter)->pos, obstacles[j]->center) <= 6.0 * obstacles[0]->size + 15.0 * Rh)
		{
			obstacleInsightOneHotMark[j] = 1;
			currentHunterInsightObstaclesCount++;
		}
	);
	//如果视野内有障碍物，那么分配清零内存；否则不分配：
	double* obstacleDistanceDifferOfNextLoc = NULL;
	if (currentHunterInsightObstaclesCount != 0)
	{
		obstacleDistanceDifferOfNextLoc = (double*)malloc(currentHunterInsightObstaclesCount * sizeof(double));
		RtlZeroMemory(obstacleDistanceDifferOfNextLoc, currentHunterInsightObstaclesCount * sizeof(double));
	}
	//填充决策数组：
	//遍历当前捕猎者的下一个所有可能位置；
	//只要此位置合法（避碰/链表/不被发现），那么就将此位置入决策数组；否则忽略.
	for (int j = 0; j < ANGLE_REGION_SIZE; j++)
	{
		for (int i = 1; i < DIS_REGION_SIZE; i++)
		{
			nextHunterAvaliableInfo->pos.x = (*hunter)->pos.x + (i * ((double)huntersVelocity / (double)DIS_REGION_SIZE)) * cosa((double)j * (360.0) / (double)ANGLE_REGION_SIZE);
			nextHunterAvaliableInfo->pos.y = (*hunter)->pos.y + (i * ((double)huntersVelocity / (double)DIS_REGION_SIZE)) * sina((double)j * (360.0) / (double)ANGLE_REGION_SIZE);
			int shouldPush = TRUE;
			if (
				(
					calculate_P$P$P_unsignedAngle(CONTAINING_RECORD((*hunter)->hunterListEntry.Blink, HUSV, hunterListEntry)->pos, target->pos, nextHunterAvaliableInfo->pos)
					>=
					currentHunterValidRegionAngle
					)
				||
				(
					calculate_P$P$P_unsignedAngle(nextHunterAvaliableInfo->pos, target->pos, CONTAINING_RECORD((*hunter)->hunterListEntry.Flink, HUSV, hunterListEntry)->pos)
					>=
					currentHunterValidRegionAngle
					)
				)
			{
				//∠(A-1)_T_A.want > currentPhaseAngle || ∠A.want_T_(A+1) > currentPhaseAngle，破坏了链表顺序
				shouldPush = FALSE;
			}
			if (
				calculate_P$P_distance(nextHunterAvaliableInfo->pos, target->pos) <= targetSafetyRadius + 1.1 * Rh
				)
			{
				//进入了目标的探测半径，破坏了隐蔽性
				shouldPush = FALSE;
			}
			if (
				(calculate_P$P_distance(nextHunterAvaliableInfo->pos, CONTAINING_RECORD((*hunter)->hunterListEntry.Flink, HUSV, hunterListEntry)->pos) <= huntersSafetyRadius)
				||
				(calculate_P$P_distance(nextHunterAvaliableInfo->pos, CONTAINING_RECORD((*hunter)->hunterListEntry.Blink, HUSV, hunterListEntry)->pos) <= huntersSafetyRadius)
				)
			{
				//进入了其他捕食者的安全半径，破坏了船间安全性
				shouldPush = FALSE;
			}
			int tempIndex = 0;
			if (obstacleDistanceDifferOfNextLoc != NULL)
			{
				for (size_t jj = 0; jj < obstaclesNum; jj++)
				{
					if (obstacleInsightOneHotMark[jj] == 1)
					{
						obstacleDistanceDifferOfNextLoc[tempIndex++] = calculate_P$P_distance(nextHunterAvaliableInfo->pos, obstacles[jj]->center);
					}
				}
				for (size_t jj = 0; jj < currentHunterInsightObstaclesCount; jj++)
				{
					if (obstacleDistanceDifferOfNextLoc[jj] <= 1.2 * obstacles[jj]->size)
					{
						shouldPush = FALSE;
						break;
					}
					else
					{
						continue;
					}
				}
			}
			if (shouldPush == TRUE)
			{
				OFFSETS[offsetIndex].distanceOffset = i;
				OFFSETS[offsetIndex].angleOffset = j;
				offsetIndex++;
			}
		}
	}
	//获得当前决策数组的有效位置数目：
	size_t stackAvaliableMovesCount = offsetIndex;
	//定义重复刷新的最大值栈区变量：
	double newMaxRewardScore = -9999.0;
	//记录最优距离偏移maxDisScoreIndex和角度偏移maxAngleScoreIndex：
	int maxDisScoreIndex = 0;
	int maxAngleScoreIndex = 0;
	for (size_t jt = 0; jt < stackAvaliableMovesCount; jt++)
	{
		//反推当前栈顶指针指向的位置:
		nextHunterAvaliableLoc.x = (*hunter)->pos.x + ((double)(OFFSETS[jt].distanceOffset) * ((double)huntersVelocity / (double)DIS_REGION_SIZE)) * cosa((double)(OFFSETS[jt].angleOffset * 360.0 / (double)ANGLE_REGION_SIZE));
		nextHunterAvaliableLoc.y = (*hunter)->pos.y + ((double)(OFFSETS[jt].distanceOffset) * ((double)huntersVelocity / (double)DIS_REGION_SIZE)) * sina((double)(OFFSETS[jt].angleOffset * 360.0 / (double)ANGLE_REGION_SIZE));
		nextHunterAvaliableInfo->pos.x = nextHunterAvaliableLoc.x;
		nextHunterAvaliableInfo->pos.y = nextHunterAvaliableLoc.y;
		//根据评分公式来对决策数组内的每一个位置打分，最高者出列：
		if (_SURROUNDING_getRewardPointsForHunter(nextHunterAvaliableInfo, target, obstacles, obstacleInsightOneHotMark) >= newMaxRewardScore)
		{
			//如果是当前最大值，刷新最大值并赋值maxDisScoreIndex和maxAngleScoreIndex：
			newMaxRewardScore = _SURROUNDING_getRewardPointsForHunter(nextHunterAvaliableInfo, target, obstacles, obstacleInsightOneHotMark);
			maxDisScoreIndex = OFFSETS[jt].distanceOffset;
			maxAngleScoreIndex = OFFSETS[jt].angleOffset;
		}
	}
	//根据最优偏移来计算理想位置：
	nextHunterAvaliableLoc.x = (*hunter)->pos.x + (maxDisScoreIndex * ((double)huntersVelocity / (double)DIS_REGION_SIZE)) * cosa((double)maxAngleScoreIndex * (360.0) / (double)ANGLE_REGION_SIZE);
	nextHunterAvaliableLoc.y = (*hunter)->pos.y + (maxDisScoreIndex * ((double)huntersVelocity / (double)DIS_REGION_SIZE)) * sina((double)maxAngleScoreIndex * (360.0) / (double)ANGLE_REGION_SIZE);
	if (calculate_P$P_distance(nextHunterAvaliableLoc, (*hunter)->pos) < 4.0 * (double)huntersVelocity / (double)DIS_REGION_SIZE)
	{
		(*hunter)->deadCorner++;
	}
	else
	{
		(*hunter)->deadCorner = 0;
	}
	//更新位置：
	updateHunterPosByNewPoint(hunter, nextHunterAvaliableLoc);
	//释放内存：
	free(nextHunterAvaliableInfo);
	nextHunterAvaliableInfo = NULL;
	if (obstacleDistanceDifferOfNextLoc != NULL);
	{
		free(obstacleDistanceDifferOfNextLoc);
		obstacleDistanceDifferOfNextLoc = NULL;
	}
	free(obstacleInsightOneHotMark);
	obstacleInsightOneHotMark = NULL;
	free(OFFSETS);
	OFFSETS = NULL;
	return;
}
BOOLEAN isSurroundingSuccess(IN PHUSV hunter[huntersNum], IN PTUSV target, IN POBSTACLE* obstacles)
{
	//maxTolerance这个值是临界值；
	//如果有任何一个捕食者到目标的距离大于mostSafetyRadius加上这个maxTolerance值，那么就视为不收敛；
	//maxTolerance值必须大于_SURROUNDING_getRewardPointsForHunterByHuntersEI中定义的episilon_distance；
	//否则永不收敛

	double maxDistanceTolerance = 8.00 + 2.0 * obstacles[0]->size + 0.1;
	double maxAngleTolerance = 8.00;

	double d[huntersNum] = { 0 };
	double a[huntersNum] = { 0 };

	for (size_t j = 0; j < huntersNum; j++)
	{
		d[j] = calculate_P$P_distance(hunter[j]->pos, target->pos) - targetSafetyRadius - 1.1 * Rh;
		a[j] = calculate_P$P$P_unsignedAngle(hunter[j]->pos, target->pos, CONTAINING_RECORD(hunter[j]->hunterListEntry.Flink, HUSV, hunterListEntry)->pos) - PHASE_ANGLE;
	}
	for (size_t j = 0; j < huntersNum; j++)
	{
		if (d[j] <= 0.0 || d[j] >= maxDistanceTolerance)
		{
			return FALSE;
		}
	}
	size_t subQueueNum = (size_t)((double)huntersNum * 0.8);
	for (size_t j = 0; j < subQueueNum; j++)
	{
		if (mabs(a[j]) >= maxAngleTolerance)
		{
			return FALSE;
		}
	}

	return TRUE;
}
//C-Python Interaction Functions:
void makePythonFile(IN FILE* fp, IN PHUSV hunter[huntersNum], IN PTUSV target, IN POBSTACLE obstacles[obstaclesNum])
{
	Point* h = (Point*)malloc(huntersNum * sizeof(Point));

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

	//fprintf(fp, "hunterNum = %zu\n", huntersNumRealTime);

	for (int i = 0; i < huntersNum; i++)
	{
		if (i == huntersNum - 1)
		{
			fprintf(fp, "hunter %d: x = %.3lf, y = %.3lf, distance: %lf, angle differ: %lf\n", i, h[i].x, h[i].y, calculate_P$P_distance(h[i], t), mabs(calculate_P$P$P_unsignedAngle(h[huntersNum - 1], t, h[0]) - (double)(360.0 / (double)huntersNum)));
		}
		else
		{
			fprintf(fp, "hunter %d: x = %.3lf, y = %.3lf, distance: %lf, angle differ: %lf\n", i, h[i].x, h[i].y, calculate_P$P_distance(h[i], t), mabs(calculate_P$P$P_unsignedAngle(h[i], t, h[i + 1]) - (double)(360.0 / (double)huntersNum)));
		}
	}

	fprintf(fp, "target: x = %.3f, target: y = %.3f\n", t.x, t.y);

	for (int i = 0; i < obstaclesNum; i++)
	{
		fprintf(fp, "obstacle %d: x = %.3f, y = %.3f, size: %lf\n", i, obstacles[i]->center.x, obstacles[i]->center.y, obstacles[i]->size);
	}

	fprintf(fp, "QAQ\n");

	ExFreeMemoryToNULL((PVOID*)&h);

	return;
}
