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
	(*Husv)->evaluationIndicator.idealDistanceDiffer = 0.0;
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
	printf("Target has position: (%.3f, %.3f), normalVelocity: %.3f, SafetyRadius: %.3f\n", target->pos.x, target->pos.y, target->normalVelocity, target->safetyRadius);
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
	ckFloatValue(pEvaluationIndicator.idealDistanceDiffer);
	ckFloatValue(pEvaluationIndicator.idealAngleDiffer);
	return;
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
void _SURROUNDING_getHuntersTwoEIByHuntersLocAndHeadDirection(IN_OUT PHUSV* hunter, IN PTUSV target)
{
	//mostSafetyRadius是目标的安全圈半径加上一个采样时间内目标能够移动的长度；
	//RtN和Rh的系数必须大于等于1才能确保最糟糕的情况下（捕食者和目标相向而行）目标也不会发现捕食者.
	double mostSafetyRadius = target->safetyRadius + 1 * RtN + 1 * Rh;
	PHUSV curr = *hunter;
	PHUSV next = CONTAINING_RECORD(curr->hunterListEntry.Flink, HUSV, hunterListEntry);
	PHUSV prev = CONTAINING_RECORD(curr->hunterListEntry.Blink, HUSV, hunterListEntry);
	(*hunter)->evaluationIndicator.idealAngleDiffer = mabs(calculate_P$P$P_unsignedAngle(curr->pos, target->pos, next->pos)) - ((double)360 / (double)huntersNum);
	(*hunter)->evaluationIndicator.idealDistanceDiffer = calculate_P$P_distance(curr->pos, target->pos) - mostSafetyRadius;
	return;
}
double _SURROUNDING_getRewardPointsForHunterByHuntersEI(IN PHUSV hunter)
{
	double angleDiff = mabs(hunter->evaluationIndicator.idealAngleDiffer);
	double disDiff = mabs(hunter->evaluationIndicator.idealDistanceDiffer);

	double J = 0.0;
	// 误差线 episilon_angle: 以目标为中心的合围图形中心角越接近【360 / n + episilon_angle】越好
	double episilon_angle = 2.0;
	// 误差线 episilon_distance: 捕食者和目标最大安全圈的距离越接近 0.8 越好；
	double episilon_distance = 0.8;

	//为了好证明：
	if (hunter->evaluationIndicator.idealDistanceDiffer - episilon_distance <= 0)
	{
		J = -100000.0;
		return J;
	}
	double cbangle = closeBetter(angleDiff, episilon_angle);
	double cbdis = closeBetter(disDiff, episilon_distance);
	double cbsum = cbangle + cbdis;
	J = (cbdis / cbsum) * cbangle; //+ (cbangle / cbsum) * cbdis;
	return J;
}
void _SURROUNDING_changingHunterInfomationByRewardFunction(IN_OUT PHUSV* hunter, IN PTUSV target)
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
	Point initialLoc = { (*hunter)->pos.x , (*hunter)->pos.y };
	//B = Blink, F = Flink, T = target, Next = nextHunterIdealInfo || nextHunterIdealLoc
	//定义当前捕食者最大移动的角度，不能越过链表顺序(∠B_T_F)：
	double currentHunterValidRegionAngle = calculate_P$P$P_unsignedAngle(CONTAINING_RECORD((*hunter)->hunterListEntry.Blink, HUSV, hunterListEntry)->pos, target->pos, CONTAINING_RECORD((*hunter)->hunterListEntry.Flink, HUSV, hunterListEntry)->pos);
	//当前捕食者的下一个位置参数：
	Point nextHunterIdealLoc = { 0 };
	PHUSV nextHunterIdealInfo = NULL;
	nextHunterIdealInfo = (PHUSV)malloc(sizeof(HUSV));
	memcpy(nextHunterIdealInfo, *hunter, sizeof(HUSV));
	//重复刷新的最大捕食者评分分数：
	double newMaxRewardPoints = -9999.0;
	typedef struct _globalIndex
	{
		int i;
		int j;
	}GI, * PGI;
	typedef struct _globalInfoStack
	{
		GI* globalIndex;
		ULONG64 stackTopPointer;
		ULONG64 stackButtomPointer;
	}GIS, * PGIS;
	//优先栈：评分最大的位置在栈顶，最大评分不符合条件出栈即可：
	PGIS stack = (PGIS)malloc(sizeof(GIS));
	stack->globalIndex = (GI*)malloc(STACK_MAX_SIZE * sizeof(GI));
	stack->stackTopPointer = (ULONG64)stack->globalIndex;
	stack->stackButtomPointer = (ULONG64)stack->globalIndex;
	RtlZeroMemory(stack->globalIndex, STACK_MAX_SIZE * sizeof(GI));
	//填充优先栈：
	//遍历当前捕猎者的下一个所有可能位置依次评分；
	//每刷新一次最大值就把此时刻的位置入栈.
	for (int j = 0; j < 180; j++)
	{
		for (int i = 0; i <= 20; i++) //注意i从0开始，可能保持不动
		{
			nextHunterIdealInfo->pos.x = (*hunter)->pos.x + (i * ((double)huntersVelocity / (double)20.0)) * cosa(j * 2);
			nextHunterIdealInfo->pos.y = (*hunter)->pos.y + (i * ((double)huntersVelocity / (double)20.0)) * sina(j * 2);
			_SURROUNDING_getHuntersTwoEIByHuntersLocAndHeadDirection(&nextHunterIdealInfo, target);
			if (_SURROUNDING_getRewardPointsForHunterByHuntersEI(nextHunterIdealInfo) > newMaxRewardPoints)
			{
				if (nextHunterIdealInfo->evaluationIndicator.idealAngleDiffer < 0)
				{
					newMaxRewardPoints = _SURROUNDING_getRewardPointsForHunterByHuntersEI(nextHunterIdealInfo);
					stack->stackTopPointer += sizeof(PGI);
					((PGI)stack->stackTopPointer)->i = i;
					((PGI)stack->stackTopPointer)->j = j;
				}
				else
				{
					if 
					(
						calculate_P$P$P_unsignedAngle(nextHunterIdealInfo->pos, target->pos, CONTAINING_RECORD((*hunter)->hunterListEntry.Flink, HUSV, hunterListEntry)->pos)
						>
						360.0 / (double)huntersNum
					)
					{
						//如果∠A_T_A.next > phase, 那么此语句保证A船移动的时候不会越过相角
						//即保证当A船在此轮的移动终点A*满足，∠(A*)_T_A.next > ∠(virtualPoint)_T_A.next，其中∠(virtualPoint)_T_A.next == 360.0 / (double)huntersNum
						//目的是保证sgn(∠(A*)_T_A.next - phase) == sgn(∠(A_T_A.next) - phase)
						newMaxRewardPoints = _SURROUNDING_getRewardPointsForHunterByHuntersEI(nextHunterIdealInfo);
						stack->stackTopPointer += sizeof(PGI);
						((PGI)stack->stackTopPointer)->i = i;
						((PGI)stack->stackTopPointer)->j = j;
					}
				}
			}
		}
	}
	//保存和当前捕食者为邻居的船间距离：
	double neighborDistanceDifferOfNextLoc[2] = { 0.0 };
	//定义的安全距离。如果下一个位置距离某个邻居小于此界限，那么视为不合适：
	double huntersSafetyNeighborDistance = 1.1 * Rh;
	//遍历栈
	while (stack->stackTopPointer != stack->stackButtomPointer)
	{
		//循环计算当前捕食者的下一个位置：
		nextHunterIdealLoc.x = (*hunter)->pos.x + ((double)(((PGI)stack->stackTopPointer)->i) * ((double)huntersVelocity / (double)20.0)) * cosa((double)(((PGI)stack->stackTopPointer)->j * 2.0));
		nextHunterIdealLoc.y = (*hunter)->pos.y + ((double)(((PGI)stack->stackTopPointer)->i) * ((double)huntersVelocity / (double)20.0)) * sina((double)(((PGI)stack->stackTopPointer)->j * 2.0));
		nextHunterIdealInfo->pos.x = nextHunterIdealLoc.x;
		nextHunterIdealInfo->pos.y = nextHunterIdealLoc.y;
		//计算当前捕食者的下一个位置和邻居的距离：
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
			//如果下一个位置离邻居太近，或者越过了链表顺序（即：∠B_T_Next >= ∠B_T_F || ∠Next_T_F >= ∠B_T_F）
			//那么退栈，选取次优位置：
			stack->stackTopPointer -= sizeof(PGI);
#ifdef _DEBUG
			if (stack->stackTopPointer != stack->stackButtomPointer)
			{
				printf_red("非栈顶\n");
			}
#endif
			if (stack->stackTopPointer == stack->stackButtomPointer)
			{
				//如果到达栈底，那么保持不动：
				printf_red("警告：最优全局i_j已经到达栈底.\n");
				free(stack->globalIndex);
				stack->globalIndex = NULL;
				free(stack);
				stack = NULL;
				free(nextHunterIdealInfo);
				nextHunterIdealInfo = NULL;
				//返回：
				return;
			}
		}
		else
		{
			//如果当前捕食者的下一个位置是合理的（不满足上面三个条件）
			if(!isMutatedThisTime)
			{
				//如果没有发生变异，直接选择栈顶：
				break;
			}
			else
			{
				//如果发生变异，那么从栈底到当前有效栈顶随机选择一个位置出栈！
				//记录当前可选择的栈空间位置所占机器字节：
				size_t currStackBytesSize = stack->stackTopPointer - stack->stackButtomPointer;
#ifdef _DEBUG
				printf_red("变异！\n");				
#endif
				if ((LONG64)currStackBytesSize - 2 * (LONG64)sizeof(ULONG64) <= 0)
				{
					printf("虽然变异，但是也只有栈顶可用了.\n");
					break;
				}
				else
				{
#ifdef _DEBUG
					printf_pink("正常变异！\n");
#endif
					printf("当前发生变异的捕食者编号：%zu\n", (*hunter)->hunterListIndex);
					//栈中的所有合法位置个数：
					size_t locsCount = currStackBytesSize >> 3;
					//记录下文的随机位置是否已经被访问过的标志数组：
					UCHAR* locsAlreadyVistedFlag = (UCHAR*)malloc(locsCount * sizeof(UCHAR));
					RtlZeroMemory(locsAlreadyVistedFlag, locsCount * sizeof(UCHAR));					
				H: 
					1;
					//已经访问过的随机位置计数：
					size_t locsVisitedCount = 0;
					//即将处理的随机位置：
					size_t nextLoc = (size_t)rand() % locsCount;
					//更新已经访问过的随机位置计数：
					fors(
						locsCount,
						if (locsAlreadyVistedFlag[j] == 0xFF)
						{
							locsVisitedCount++;
						}
					);
					//如果已经访问过的随机位置计数和栈中的所有合法位置个数相等；
					//那么出栈最原始的栈顶作为下一个位置：
					if (locsVisitedCount == locsCount)
					{
#ifdef _DEBUG
						printf_green("所有栈元素已经遍历.\n");
#endif
						stack->stackTopPointer = stack->stackButtomPointer + locsCount * sizeof(ULONG64);
						nextHunterIdealInfo->pos.x = (*hunter)->pos.x + ((double)(((PGI)stack->stackTopPointer)->i) * ((double)huntersVelocity / (double)20.0)) * cosa((double)(((PGI)stack->stackTopPointer)->j * 2.0));
						nextHunterIdealInfo->pos.y = (*hunter)->pos.y + ((double)(((PGI)stack->stackTopPointer)->i) * ((double)huntersVelocity / (double)20.0)) * sina((double)(((PGI)stack->stackTopPointer)->j * 2.0));
						free(locsAlreadyVistedFlag);
						locsAlreadyVistedFlag = NULL;
						break;
					}
					//如果新的随机位置之前已经被访问过，那么重新选择随机位置：
					if (locsAlreadyVistedFlag[nextLoc] == 0xFF)
					{
						goto H;
					}
					//把当前栈顶设置为未被访问过的随机位置：
					stack->stackTopPointer = stack->stackButtomPointer + (nextLoc + !nextLoc) * sizeof(ULONG64);
					nextHunterIdealInfo->pos.x = (*hunter)->pos.x + ((double)(((PGI)stack->stackTopPointer)->i) * ((double)huntersVelocity / (double)20.0)) * cosa((double)(((PGI)stack->stackTopPointer)->j * 2.0));
					nextHunterIdealInfo->pos.y = (*hunter)->pos.y + ((double)(((PGI)stack->stackTopPointer)->i) * ((double)huntersVelocity / (double)20.0)) * sina((double)(((PGI)stack->stackTopPointer)->j * 2.0));
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
						//变异的随机位置违背了三条原则！
						//先设置此位置已经被访问过，标志设置为0xFF；
						locsAlreadyVistedFlag[nextLoc] = 0xFF;
						//继续重新选点：
						goto H;
					}
					else
					{
						//变异的随机位置合法！
						//释放内存：
						free(locsAlreadyVistedFlag);
						locsAlreadyVistedFlag = NULL;
						//出栈！
						break;
					}
				}
			}
		}
	}
	//更新此捕食者的坐标：
	updateHunterPosByNewPoint(hunter, nextHunterIdealLoc);
	//调整此捕食者的EI指标。此函数会对hunter造成副作用：
	_SURROUNDING_getHuntersTwoEIByHuntersLocAndHeadDirection(hunter, target);
	//释放内存：
	free(stack->globalIndex);
	stack->globalIndex = NULL;
	free(stack);
	stack = NULL;
	free(nextHunterIdealInfo);
	nextHunterIdealInfo = NULL;
	//返回：
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
//C-Python Interaction Functions:
void makePythonFile(IN FILE* fp, IN PHUSV hunter[huntersNum], IN PTUSV target)
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
			fprintf(fp, "hunter %d: x = %.3f, hunter %d: y = %.3f, distance: %lf, angle differ: %lf\n", i, h[i].x, i, h[i].y, calculate_P$P_distance(h[i], t), mabs(calculate_P$P$P_unsignedAngle(h[huntersNum - 1], t, h[0]) - (double)(360.0 / (double)huntersNum)));
		}
		else
		{
			fprintf(fp, "hunter %d: x = %.3f, hunter %d: y = %.3f, distance: %lf, angle differ: %lf\n", i, h[i].x, i, h[i].y, calculate_P$P_distance(h[i], t), mabs(calculate_P$P$P_unsignedAngle(h[i], t, h[i + 1]) - (double)(360.0 / (double)huntersNum)));
		}
	}

	fprintf(fp, "target: x = %.3f, target: y = %.3f\n", t.x, t.y);

	fprintf(fp, "QAQ\n");

	ExFreeMemoryToNULL((PVOID*)&h);

	return;
}
